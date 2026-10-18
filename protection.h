// ============================================================
//  protection.h - Anti-islanding protection + reconnection
//                state machine. Part of GridSync v2.
// ------------------------------------------------------------
//  TRIP DETECTION (carried over from v1, same windows):
//    * voltage outside 0.88 - 1.10 pu of the nominal bus
//      (evaluated once the PLL is locked, as in v1)
//    * frequency outside 47.5 - 51.5 Hz
//    * |ROCOF| above 2.5 Hz/s (rate of change of frequency,
//      measured over >= 200 ms windows, as in v1)
//  plus the ACTIVE frequency shift v1 computed: a small bias
//  proportional to the frequency error, which walks an
//  islanded bus away from nominal until a passive window
//  catches it (exposed as afdShift for the control loop).
//
//  V2 ADDITION - RECONNECTION STATE MACHINE:
//  v1 let an operator reset restart the inverter instantly.
//  Real grid codes (and common sense) demand a disciplined
//  reconnection. The machine below enforces it:
//
//    RUN ----(window violation)----> TRIPPED
//    TRIPPED --(operator reset)----> WAIT
//    WAIT ----(reconnectDelayMs)--> VERIFY
//    VERIFY --(V & f in windows AND PLL locked,
//              continuously for verifyTimeMs)--> RAMP
//    VERIFY --(any condition lost)--> VERIFY (timer restarts)
//    RAMP ----(current ref ramped 0 -> set-point
//              over rampTimeMs)----> RUN (trip latch clears)
//    VERIFY / RAMP --(grid-tied monitoring lost)--> TRIPPED
//
//  Defaults: WAIT 30 s, VERIFY 5 s, RAMP 3 s - all
//  configurable fields. outputAllowed() is true only in RAMP
//  and RUN; rampScale (0 in TRIPPED/WAIT/VERIFY, 0..1 during
//  RAMP, 1 in RUN) scales the current reference so the
//  inverter soft-starts instead of stepping onto the grid.
//
//  The module is time-source agnostic: the caller passes a
//  millisecond clock (millis()) into update(), which also
//  makes the machine unit-testable off-target.
//
//  Header-only module: instantiate one IslandProtection.
// ============================================================

#ifndef GRIDSYNC_PROTECTION_H
#define GRIDSYNC_PROTECTION_H

#include <math.h>
#include <stdio.h>
#include <string.h>

enum ProtState {
  PROT_RUN = 0,     // normal operation, protection armed
  PROT_TRIPPED,     // fault latched, waiting for operator reset
  PROT_WAIT,        // post-reset enforced delay
  PROT_VERIFY,      // proving the grid is healthy again
  PROT_RAMP         // resynchronised, ramping current back up
};

struct IslandProtection {
  // ---- configuration (windows as v1) -------------------------
  float vNom     = 24.0f;    // nominal bus voltage, V rms
  float vMinPu   = 0.88f;    // voltage window, per-unit
  float vMaxPu   = 1.10f;
  float fNom     = 50.0f;    // nominal frequency, Hz
  float fMin     = 47.5f;    // frequency window, Hz
  float fMax     = 51.5f;
  float rocofMax = 2.5f;     // ROCOF trip threshold, Hz/s
  float afdGain  = 0.002f;   // active frequency shift gain (v1)

  unsigned long reconnectDelayMs = 30000;  // WAIT duration
  unsigned long verifyTimeMs     = 5000;   // VERIFY hold time
  unsigned long rampTimeMs       = 3000;   // RAMP duration

  // ---- outputs -------------------------------------------------
  ProtState state   = PROT_RUN;
  bool  tripped     = false;   // fault latch (clears at RUN)
  float rocof       = 0.0f;    // latest measured ROCOF, Hz/s
  float afdShift    = 0.0f;    // active frequency shift bias
  float rampScale   = 1.0f;    // current-reference scale 0..1
  char  tripReason[128] = "";  // detail of the last trip
  char  verifyStatus[64] = ""; // live VERIFY diagnostic (dashboard)
  unsigned int tripCount = 0;  // total trips since boot

  // ---- event reporting (one-deep queue for the logger) --------
  char eventMsg[160] = "";
  bool eventPending = false;

  // ---- internal state ------------------------------------------
  float lastFreq = 50.0f;
  unsigned long lastRocofMs = 0;
  unsigned long stateStartMs = 0;
  bool verifyTiming = false;
  unsigned long verifyStartMs = 0;

  void setEvent(const char* msg) {
    strncpy(eventMsg, msg, sizeof(eventMsg) - 1);
    eventMsg[sizeof(eventMsg) - 1] = '\0';
    eventPending = true;
  }

  // Fetch and clear a pending log event. Returns true if one
  // was waiting; the message is copied into out (NUL-safe).
  bool pollEvent(char* out, unsigned int len) {
    if (!eventPending) return false;
    strncpy(out, eventMsg, len - 1);
    out[len - 1] = '\0';
    eventPending = false;
    return true;
  }

  const char* stateName() const {
    switch (state) {
      case PROT_RUN:    return "RUN";
      case PROT_TRIPPED: return "TRIPPED";
      case PROT_WAIT:   return "WAIT";
      case PROT_VERIFY: return "VERIFY";
      case PROT_RAMP:   return "RESYNC/RAMP";
      default:          return "?";
    }
  }

  // PWM may be driven (in grid-tied mode) only in RAMP / RUN.
  bool outputAllowed() const {
    return state == PROT_RUN || state == PROT_RAMP;
  }

  bool voltageInWindow(float vrms) const {
    float vpu = (vNom > 0) ? vrms / vNom : 0.0f;
    return vpu >= vMinPu && vpu <= vMaxPu;
  }
  bool frequencyInWindow(float freq) const {
    return freq >= fMin && freq <= fMax;
  }

  // The full VERIFY condition, in one place: voltage and
  // frequency inside their windows AND the PLL locked.
  bool gridHealthy(float vrms, float freq, bool pllLocked) const {
    return voltageInWindow(vrms) && frequencyInWindow(freq) && pllLocked;
  }

  // Seconds left in the current timed phase, for the dashboard
  // countdown (WAIT: until verify; VERIFY: until ramp, once the
  // grid is holding good; RAMP / others: 0).
  unsigned long countdownSec(unsigned long nowMs) const {
    if (state == PROT_WAIT) {
      unsigned long el = nowMs - stateStartMs;
      if (el >= reconnectDelayMs) return 0;
      return (reconnectDelayMs - el + 999) / 1000;
    }
    if (state == PROT_VERIFY) {
      if (!verifyTiming) return verifyTimeMs / 1000;
      unsigned long el = nowMs - verifyStartMs;
      if (el >= verifyTimeMs) return 0;
      return (verifyTimeMs - el + 999) / 1000;
    }
    return 0;
  }

  // Trip percent complete during RAMP (0..100), else 0 or 100.
  int rampPercent() const {
    if (state == PROT_RAMP) return (int)(rampScale * 100.0f + 0.5f);
    return (state == PROT_RUN) ? 100 : 0;
  }

  void trip(const char* cause, float vrms, float freq) {
    state = PROT_TRIPPED;
    tripped = true;
    tripCount++;
    rampScale = 0.0f;
    verifyTiming = false;
    verifyStatus[0] = '\0';
    snprintf(tripReason, sizeof(tripReason),
             "%s: V=%.1fV f=%.2fHz ROCOF=%.2fHz/s",
             cause, vrms, freq, rocof);
    char buf[192];
    snprintf(buf, sizeof(buf), "ANTI-ISLANDING TRIP: %s - inverter disconnected",
             tripReason);
    setEvent(buf);
  }

  // Operator reset: NOT an instant restart (the v2 change) -
  // it starts the WAIT -> VERIFY -> RAMP reconnection sequence.
  void requestReset(unsigned long nowMs) {
    if (state != PROT_TRIPPED) return;
    state = PROT_WAIT;
    stateStartMs = nowMs;
    verifyTiming = false;
    rampScale = 0.0f;
    char buf[96];
    snprintf(buf, sizeof(buf),
             "Trip reset by operator: reconnection sequence started (%lus wait)",
             (unsigned long)(reconnectDelayMs / 1000));
    setEvent(buf);
  }

  // Main update. Call at a steady cadence (the sketch uses the
  // v1 100 ms slot).
  //   vrms, freq : latest measurements
  //   pllLocked  : PLL lock detector output
  //   monitoring : true while grid-tied mode is selected
  //   nowMs      : millisecond clock
  void update(float vrms, float freq, bool pllLocked,
              bool monitoring, unsigned long nowMs) {
    // ---- ROCOF over >= 200 ms windows (as v1) ----------------
    if (nowMs - lastRocofMs >= 200) {
      float dtSec = (nowMs - lastRocofMs) / 1000.0f;
      if (dtSec > 0) rocof = fabsf(freq - lastFreq) / dtSec;
      lastFreq = freq;
      lastRocofMs = nowMs;
    }

    // ---- active frequency shift bias (v1 formula) ------------
    afdShift = afdGain * (freq - fNom);

    // ---- losing grid-tied monitoring mid-reconnection --------
    if (!monitoring) {
      if (state == PROT_VERIFY || state == PROT_RAMP) {
        trip("RECONNECT ABORTED (mode changed)", vrms, freq);
      }
      if (state == PROT_RUN) rampScale = 1.0f;
      return;
    }

    switch (state) {
      case PROT_RUN: {
        rampScale = 1.0f;
        // Voltage window is judged on a locked PLL, as v1 did;
        // frequency and ROCOF are judged unconditionally.
        const char* cause = NULL;
        if (pllLocked && !voltageInWindow(vrms))   cause = "VOLTAGE OUT OF WINDOW";
        else if (!frequencyInWindow(freq))         cause = "FREQUENCY OUT OF WINDOW";
        else if (rocof > rocofMax)                 cause = "ROCOF LIMIT EXCEEDED";
        if (cause) trip(cause, vrms, freq);
        break;
      }

      case PROT_TRIPPED:
        rampScale = 0.0f;
        break;  // waits for requestReset()

      case PROT_WAIT:
        rampScale = 0.0f;
        if (nowMs - stateStartMs >= reconnectDelayMs) {
          state = PROT_VERIFY;
          stateStartMs = nowMs;
          verifyTiming = false;
          setEvent("Reconnection: verifying grid health (V, f, PLL lock)");
        }
        break;

      case PROT_VERIFY: {
        rampScale = 0.0f;
        bool vOk = voltageInWindow(vrms);
        bool fOk = frequencyInWindow(freq);
        bool ok  = gridHealthy(vrms, freq, pllLocked);
        if (ok) {
          if (!verifyTiming) { verifyTiming = true; verifyStartMs = nowMs; }
          snprintf(verifyStatus, sizeof(verifyStatus),
                   "grid healthy, holding %.1fs",
                   (nowMs - verifyStartMs) / 1000.0f);
          if (nowMs - verifyStartMs >= verifyTimeMs) {
            state = PROT_RAMP;
            stateStartMs = nowMs;
            verifyStatus[0] = '\0';
            setEvent("Reconnection: grid verified - resynchronising, ramping current");
          }
        } else {
          verifyTiming = false;  // continuity broken: restart the 5 s proof
          // Name the failing condition so the dashboard (and
          // the operator) can see WHY reconnection is waiting.
          if      (!vOk)       strncpy(verifyStatus, "waiting: voltage out of window", sizeof(verifyStatus) - 1);
          else if (!fOk)       strncpy(verifyStatus, "waiting: frequency out of window", sizeof(verifyStatus) - 1);
          else /* !pllLocked */ strncpy(verifyStatus, "waiting: PLL not locked", sizeof(verifyStatus) - 1);
          verifyStatus[sizeof(verifyStatus) - 1] = '\0';
        }
        break;
      }

      case PROT_RAMP: {
        unsigned long el = nowMs - stateStartMs;
        rampScale = (rampTimeMs > 0)
            ? (float)el / (float)rampTimeMs : 1.0f;
        if (rampScale >= 1.0f) {
          rampScale = 1.0f;
          state = PROT_RUN;
          tripped = false;       // latch clears only here
          setEvent("Reconnected to grid: full current restored");
        }
        break;
      }
    }
  }
};

#endif  // GRIDSYNC_PROTECTION_H

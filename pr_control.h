// ============================================================
//  pr_control.h - Proportional-Resonant current controller
//  Part of GridSync v2 (see GridSync_Inverter.ino).
// ------------------------------------------------------------
//  A PR controller tracks a sinusoidal reference with zero
//  steady-state error at its resonant frequency - something a
//  PI controller in the stationary frame cannot do - which is
//  why it is the standard current regulator in grid inverters:
//
//      G(s) = Kp + 2*Kr*wc*s / (s^2 + 2*wc*s + w0^2)
//
//  where wc is the resonant bandwidth (passes a few Hz around
//  w0 so the loop tolerates small frequency error).
//
//  V2 UPGRADE - frequency-adaptive + true Tustin form:
//  v1 integrated a state-space resonator with forward Euler
//  and two heuristic scale factors. v2 discretises the
//  resonant term with the bilinear (Tustin) transform, giving
//  a biquad whose coefficients are recomputed from the w0
//  handed in on EVERY call - the main sketch passes the PLL's
//  tracked angular frequency, so the resonant peak follows the
//  real grid frequency (47-53 Hz excursions included) instead
//  of sitting fixed at 50 Hz:
//
//      den = w0^2*Ts^2 + 4*wc*Ts + 4
//      a1  = (2*w0^2*Ts^2 - 8) / den
//      a2  = (w0^2*Ts^2 - 4*wc*Ts + 4) / den
//      b0  = 4*Kr*wc*Ts / den,  b1 = 0,  b2 = -b0
//
//  ANTI-WINDUP: the total output is clamped to +/-outMax. If
//  the unclamped output exceeds the clamp, the resonator state
//  update for that step is rolled back (conditional
//  integration), so the resonator cannot wind itself up while
//  the bridge is saturated.
//
//  An optional second resonant bank at the 5th harmonic (the
//  standard harmonic compensator of production inverters) is
//  included and shares the same Tustin machinery.
//
//  Header-only module: instantiate one PR controller.
// ============================================================

#ifndef GRIDSYNC_PR_CONTROL_H
#define GRIDSYNC_PR_CONTROL_H

#include <math.h>

struct PR {
  // ---- configuration (gains carried over from v1) -----------
  float kp     = 0.6f;       // proportional gain
  float kr     = 38.0f;      // resonant gain (fundamental)
  float wc     = 5.0f;       // resonant bandwidth, rad/s
  float outMax = 48.0f;      // output clamp (set to DC-bus volts)

  // Optional harmonic compensator: a second resonant bank at
  // harmonic*w0 (default the 5th). Grid current references are
  // never pure sine once the grid voltage itself carries the
  // usual 5th-harmonic distortion; a resonant term there keeps
  // the injected current clean. Set harmonicOn = false to run
  // the plain fundamental-only controller (v1 behaviour).
  bool  harmonicOn = true;
  int   harmonic   = 5;      // which harmonic to compensate
  float krH        = 12.0f;  // its (lower) resonant gain

  // ---- internal state ----------------------------------------
  float e1 = 0, e2 = 0;      // error history (shared input)
  float y1 = 0, y2 = 0;      // fundamental resonator history
  float h1 = 0, h2 = 0;      // harmonic resonator history

  void reset() { e1 = e2 = y1 = y2 = h1 = h2 = 0.0f; }

  static float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
  }

  // Tustin biquad coefficients for one resonant term with
  // resonant frequency wr, gain krx and bandwidth wc.
  void coeffs(float wr, float krx, float dt,
              float& a1, float& a2, float& b0) const {
    const float wrTs   = wr * dt;
    const float wr2Ts2 = wrTs * wrTs;
    const float den    = wr2Ts2 + 4.0f * wc * dt + 4.0f;
    a1 = (2.0f * wr2Ts2 - 8.0f) / den;
    a2 = (wr2Ts2 - 4.0f * wc * dt + 4.0f) / den;
    b0 = (4.0f * krx * wc * dt) / den;
  }

  // Run one control step.
  //   err : current error (reference - measured), amps
  //   w0  : resonant angular frequency, rad/s - pass the PLL's
  //         tracked frequency for frequency-adaptive operation
  //   dt  : sample period Ts, seconds
  // Returns the controller output (volts of bridge command).
  float run(float err, float w0, float dt) {
    // ---- fundamental resonator (coefficients at tracked w0) --
    float a1, a2, b0;
    coeffs(w0, kr, dt, a1, a2, b0);
    float yNew = b0 * err - b0 * e2 - a1 * y1 - a2 * y2;

    // ---- harmonic resonator (coefficients at harmonic * w0) --
    float hNew = 0.0f;
    if (harmonicOn) {
      float ha1, ha2, hb0;
      coeffs(w0 * harmonic, krH, dt, ha1, ha2, hb0);
      hNew = hb0 * err - hb0 * e2 - ha1 * h1 - ha2 * h2;
    }

    float out = kp * err + yNew + hNew;

    if (out > outMax || out < -outMax) {
      // Saturated: roll BOTH resonator states back (conditional
      // integration) and recompute with the previous states so
      // the returned value and the stored states stay consistent.
      out = kp * err + y1 + h1;
      out = clampf(out, -outMax, outMax);
      // still shift the error history so the b0*e2 terms stay
      // meaningful on the next step
      e2 = e1; e1 = err;
      return out;
    }

    // ---- commit state -----------------------------------------
    y2 = y1; y1 = yNew;
    h2 = h1; h1 = hNew;
    e2 = e1; e1 = err;
    return out;
  }
};

#endif  // GRIDSYNC_PR_CONTROL_H

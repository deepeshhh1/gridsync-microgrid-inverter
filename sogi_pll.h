// ============================================================
//  sogi_pll.h - SOGI-PLL with proper Tustin discretisation
//  Part of GridSync v2 (see GridSync_Inverter.ino).
// ------------------------------------------------------------
//  A Second-Order Generalized Integrator (SOGI) turns the
//  single-phase grid voltage into an orthogonal alpha/beta
//  pair:
//
//      D(s) = k*w*s   / (s^2 + k*w*s + w^2)     (in-phase)
//      Q(s) = k*w^2   / (s^2 + k*w*s + w^2)     (quadrature)
//
//  At resonance D has unity gain / 0 deg and Q has unity gain
//  with output v_beta = V*sin(theta_grid), so a Park transform
//  on (v_alpha, v_beta) yields
//
//      vq = V * sin(theta_grid - theta)
//
//  and a PI loop filter driving vq -> 0 locks theta onto the
//  grid angle and w onto the grid frequency. This is the
//  standard SOGI-PLL used in production single-phase
//  inverters (Ciobotaru / Teodorescu structure).
//
//  DISCRETISATION (the v2 upgrade over v1's Euler states):
//  both SOGI paths are realised as biquads obtained from the
//  bilinear (Tustin) transform, with coefficients recomputed
//  EVERY step from the currently tracked angular frequency w
//  and the sample period Ts, so the resonators stay centred
//  exactly on the tracked frequency as the grid drifts:
//
//      den  = w^2*Ts^2 + 2*k*w*Ts + 4
//      a1   = (2*w^2*Ts^2 - 8) / den
//      a2   = (w^2*Ts^2 - 2*k*w*Ts + 4) / den
//      b0d  = 2*k*w*Ts / den,   b1d = 0,  b2d = -b0d
//      b0q  = k*w^2*Ts^2 / den, b1q = 2*b0q, b2q = b0q
//
//  Header-only module: no globals, instantiate one SogiPll.
// ============================================================

#ifndef GRIDSYNC_SOGI_PLL_H
#define GRIDSYNC_SOGI_PLL_H

#include <math.h>

struct SogiPll {
  // ---- configuration (defaults carried over from v1) -------
  float k      = 1.4142f;    // SOGI gain (sqrt(2): critical damping)
  float piKp   = 0.9f;       // loop-filter PI gains (as v1)
  float piKi   = 55.0f;
  float f0     = 50.0f;      // nominal grid frequency, Hz
  float fMin   = 40.0f;      // tracked-frequency clamp, Hz
  float fMax   = 60.0f;
  float lockVMagMin = 5.0f;  // lock detector thresholds (as v1)
  float lockFreqTol = 1.5f;  // |f - f0| must be inside this, Hz
  float lockTimeSec = 0.10f; // condition must hold this long

  // DC-offset rejection: the ZMPT101B's bias point drifts with
  // temperature and supply, and even ~20 mV of DC in the sensed
  // voltage makes the SOGI outputs wobble at line frequency
  // and corrupts the Park phase-error signal. A slow first-
  // order estimator tracks the input offset and subtracts it
  // before the resonators. dcTrack is the per-step estimator
  // gain (0.0005 -> ~0.2 s time constant at 10 kHz).
  bool  dcReject = true;
  float dcTrack  = 0.0005f;
  float dcEst    = 0.0f;     // estimated input DC offset, volts

  // ---- outputs ----------------------------------------------
  float w          = 2.0f * PI * 50.0f;  // tracked angular freq, rad/s
  float theta      = 0.0f;              // tracked angle, rad [0, 2PI)
  float vAlpha     = 0.0f;              // in-phase component (peak V)
  float vBeta      = 0.0f;              // quadrature component
  float vMag       = 0.0f;              // sqrt(alpha^2 + beta^2), peak V
  float vd         = 0.0f;              // Park d (in-phase amplitude)
  float vq         = 0.0f;              // Park q (phase error signal)
  bool  locked     = false;             // lock detector output

  // ---- internal state ----------------------------------------
  float u1 = 0, u2 = 0;        // input history u[n-1], u[n-2]
  float d1 = 0, d2 = 0;        // D-path output history
  float q1 = 0, q2 = 0;        // Q-path output history
  float piInt = 0.0f;          // loop-filter integrator
  float lockTimer = 0.0f;      // how long lock conditions have held

  static float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
  }

  void reset() {
    w = 2.0f * PI * f0;
    theta = 0.0f;
    vAlpha = vBeta = vMag = vd = vq = 0.0f;
    locked = false;
    u1 = u2 = d1 = d2 = q1 = q2 = 0.0f;
    piInt = 0.0f;
    lockTimer = 0.0f;
    dcEst = 0.0f;
  }

  // Advance the PLL by one sample.
  //   vGrid : instantaneous grid voltage sample (volts)
  //   dt    : sample period Ts (seconds)
  void step(float vGridRaw, float dt) {
    // ---- DC-offset rejection --------------------------------
    if (dcReject) dcEst += dcTrack * (vGridRaw - dcEst);
    const float vGrid = vGridRaw - dcEst;

    // ---- SOGI biquad coefficients at the tracked frequency --
    const float wTs  = w * dt;
    const float w2Ts2 = wTs * wTs;
    const float den  = w2Ts2 + 2.0f * k * wTs + 4.0f;
    const float a1   = (2.0f * w2Ts2 - 8.0f) / den;
    const float a2   = (w2Ts2 - 2.0f * k * wTs + 4.0f) / den;
    const float b0d  = (2.0f * k * wTs) / den;
    const float b0q  = (k * w2Ts2) / den;

    // ---- run both resonators (shared denominator) -----------
    float dOut = b0d * vGrid - b0d * u2
               - a1 * d1 - a2 * d2;
    float qOut = b0q * vGrid + 2.0f * b0q * u1 + b0q * u2
               - a1 * q1 - a2 * q2;

    // shift histories
    u2 = u1;  u1 = vGrid;
    d2 = d1;  d1 = dOut;
    q2 = q1;  q1 = qOut;

    vAlpha = dOut;
    vBeta  = qOut;
    vMag   = sqrtf(vAlpha * vAlpha + vBeta * vBeta);

    // ---- Park transform --------------------------------------
    const float s = sinf(theta), c = cosf(theta);
    vd =  vAlpha * c + vBeta * s;
    vq = -vAlpha * s + vBeta * c;

    // ---- PI loop filter: drive vq -> 0 -----------------------
    piInt += vq * dt;
    float wCorr = piKp * vq + piKi * piInt;
    w = 2.0f * PI * f0 + wCorr;
    w = clampf(w, 2.0f * PI * fMin, 2.0f * PI * fMax);

    // ---- integrate angle --------------------------------------
    theta += w * dt;
    while (theta >= 2.0f * PI) theta -= 2.0f * PI;
    while (theta < 0.0f)       theta += 2.0f * PI;

    // ---- lock detector ----------------------------------------
    // Same windows the v1 sketch applied externally (signal
    // present + frequency close to nominal), plus a sustain
    // timer so a single good cycle cannot declare lock.
    float freq = w / (2.0f * PI);
    bool cond = (vMag > lockVMagMin) && (fabsf(freq - f0) < lockFreqTol);
    if (cond) {
      lockTimer += dt;
      if (lockTimer >= lockTimeSec) locked = true;
    } else {
      lockTimer = 0.0f;
      locked = false;
    }
  }

  float frequency() const { return w / (2.0f * PI); }
};

#endif  // GRIDSYNC_SOGI_PLL_H

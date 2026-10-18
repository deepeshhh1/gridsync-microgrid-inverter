// ============================================================
//  GRIDSYNC v2 - GRID-TIED INVERTER WITH SOGI-PLL,
//                ANTI-ISLANDING & MICROGRID DROOP CONTROL
//  ESP32 + Full-Bridge Inverter Stage + ZMPT101B / ACS712
// ------------------------------------------------------------
//  The control core that every solar/grid inverter runs on:
//
//  1) SOGI-PLL - a Second-Order Generalized Integrator makes
//     an orthogonal (alpha/beta) pair from the single-phase
//     grid voltage; a Park transform + PI loop filter locks
//     the phase and frequency even on a distorted grid.
//
//  2) PR CURRENT CONTROL - a Proportional-Resonant controller
//     tracks the sinusoidal current reference with zero
//     steady-state error at grid frequency, pushing power
//     into the grid at the P / Q set-points.
//
//  3) ANTI-ISLANDING - passive windows (voltage 0.88-1.10 pu,
//     frequency 47.5-51.5 Hz, ROCOF) PLUS an active frequency-
//     shift term. If the grid disappears, the inverter trips
//     and stops energising the island - a hard safety rule for
//     anything that feeds a grid.
//
//  4) DROOP CONTROL (microgrid mode) - with the grid absent
//     and a second droop unit present, frequency is drooped
//     against active power (f = f0 - m*(P - P0)) and voltage
//     against reactive power (V = V0 - n*(Q - Q0)), so two
//     inverters share load WITHOUT any communication -
//     exactly how real microgrids parallel.
//
//  FILE STRUCTURE (new in v2 - the sketch is multi-file):
//     GridSync_Inverter.ino - this file: sensing, modes,
//                             10 kHz control step, dashboard
//     sogi_pll.h            - SOGI-PLL, Tustin-discretised
//                             biquads recomputed each step
//                             from the tracked frequency
//     pr_control.h          - frequency-adaptive PR current
//                             controller (resonant peak follows
//                             the PLL), Tustin form, anti-windup
//     protection.h          - anti-islanding windows + ROCOF +
//                             active frequency shift, and the
//                             reconnection state machine:
//                             TRIPPED -> WAIT 30 s -> VERIFY
//                             (grid healthy 5 s + PLL lock) ->
//                             RESYNC/RAMP (current ramped over
//                             3 s) -> RUN. A trip reset now
//                             STARTS this sequence - it no
//                             longer restarts the inverter
//                             instantly as v1 did.
//  All v1 features, endpoints (/data, /cmd), set-points and
//  the droop laws are preserved.
//
//  DEMO SCALE & SAFETY: this firmware is written for a
//  low-voltage AC bus (24 V class transformer "grid"). Real
//  grid connection requires certified hardware, isolation and
//  utility approval - this project teaches and demonstrates
//  the control, it is NOT a licence to back-feed a home.
//
//  Author : Deepesh Chauhan, B.Tech Electrical Engineering
//           RGGEC Kangra
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

#include "sogi_pll.h"
#include "pr_control.h"
#include "protection.h"

// ---------------- WI-FI (replace with your credentials) ------
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ---------------- PINS ---------------------------------------
#define GRID_V_PIN   34      // ZMPT101B output
#define INV_I_PIN    35      // ACS712 on inverter output
#define PWM_H1_PIN   25      // full bridge leg 1 high
#define PWM_L1_PIN   26      // full bridge leg 1 low
#define PWM_H2_PIN   27      // full bridge leg 2 high
#define PWM_L2_PIN   14      // full bridge leg 2 low
#define RELAY_PIN    13      // grid contactor (demo bus)
#define VBUS_PIN     32      // DC bus divider

// ---------------- SAMPLING / CONTROL --------------------------
const float TS = 0.0001f;            // 10 kHz control step
const float GRID_F0 = 50.0f;
const float VBUS_NOM = 48.0f;        // DC bus the bridge draws from

// ZMPT101B: ADC volts -> grid volts (calibrate with multimeter)
const float VGRID_CAL = 118.0f;
const float IGRID_CAL = 5.4f;        // ACS712: A per ADC volt from centre
const float ADC_MID   = 1.65f;       // sensor DC bias (half of 3.3V)

// ---------------- MODULE INSTANCES -----------------------------
SogiPll pll;                          // sogi_pll.h
PR prCtrl;                            // pr_control.h
IslandProtection prot;                // protection.h

// ---------------- STATE ---------------------------------------
enum InvMode { MODE_OFF, MODE_GRID_TIED, MODE_DROOP };
InvMode mode = MODE_OFF;
bool  pllLocked = false;              // mirror of pll.locked
float pRef_W = 200.0f;                // dashboard set-points
float qRef_VAR = 0.0f;
float gridVrms = 0, gridFreq = 50.0f, invIrms = 0;
float pOut_W = 0, qOut_VAR = 0;
float rocof = 0;                      // mirror of prot.rocof
String eventLog = "Boot\n";

// =============================================================
//  MEASUREMENTS
// =============================================================
float readGridV() {
  int raw = analogRead(GRID_V_PIN);
  return ((raw / 4095.0f) * 3.3f - ADC_MID) * VGRID_CAL;
}
float readInvI() {
  int raw = analogRead(INV_I_PIN);
  return ((raw / 4095.0f) * 3.3f - ADC_MID) * IGRID_CAL;
}
float readVbus() {
  int raw = analogRead(VBUS_PIN);
  return (raw / 4095.0f) * 3.3f * 16.0f;      // divider ratio
}

// One-cycle RMS + P/Q accumulation (called at control rate)
float accV2 = 0, accI2 = 0, accP = 0, accQ = 0;
int   accN = 0;
void accumulate(float v, float i, float theta) {
  accV2 += v * v; accI2 += i * i;
  accP  += v * i;
  accQ  += pll.vBeta * i;   // quadrature voltage x current -> Q
  accN++;
  if (accN >= 200) {                     // ~ one 50 Hz cycle @10 kHz
    gridVrms = sqrt(accV2 / accN);
    invIrms  = sqrt(accI2 / accN);
    pOut_W   = accP / accN;
    qOut_VAR = accQ / accN;
    accV2 = accI2 = accP = accQ = 0; accN = 0;
  }
}

// =============================================================
//  DROOP CONTROL (microgrid mode)
//    f = f0 - m * (P - P0) ;  V = V0 - n * (Q - Q0)
// =============================================================
const float DROOP_M = 0.0025f;           // Hz per W
const float DROOP_N = 0.004f;            // V per VAR
float droopFreqRef() { return GRID_F0 - DROOP_M * (pOut_W - pRef_W); }
float droopVoltRef() { return 24.0f   - DROOP_N * (qOut_VAR - qRef_VAR); }

// =============================================================
//  MAIN CONTROL STEP (10 kHz schedule from loop)
// =============================================================
void controlStep() {
  float v = readGridV();
  float i = readInvI();
  pll.step(v, TS);
  gridFreq  = pll.frequency();
  pllLocked = pll.locked;
  accumulate(v, i, pll.theta);

  // Output gating: OFF never drives the bridge. GRID_TIED is
  // additionally gated by the protection state machine - no
  // PWM while TRIPPED / WAIT / VERIFY, and a scaled current
  // reference while the reconnection RAMP is running.
  bool gridBlocked = (mode == MODE_GRID_TIED) && !prot.outputAllowed();
  if (mode == MODE_OFF || gridBlocked) {
    ledcWrite(0, 0); ledcWrite(1, 0);
    prCtrl.reset();
    return;
  }

  // Current reference: in grid mode, i* in phase with grid for
  // P plus a quadrature term for Q. In droop mode the unit
  // FORMS the bus: voltage reference from droop laws instead.
  float scale = (mode == MODE_GRID_TIED) ? prot.rampScale : 1.0f;
  float vInst = pll.vAlpha;                       // in-phase comp
  float vqInst = pll.vBeta;                       // quadrature comp
  float vNorm = max(pll.vMag, 1.0f);
  float iPeak = (gridVrms > 1) ? (sqrt(2.0f) * pRef_W   / gridVrms) : 0;
  float qPeak = (gridVrms > 1) ? (sqrt(2.0f) * qRef_VAR / gridVrms) : 0;
  float iRef = scale * (iPeak * (vInst / vNorm) + qPeak * (vqInst / vNorm));

  float err = iRef - i;
  // Frequency-adaptive PR: resonant peak = PLL tracked w.
  float vCmd = prCtrl.run(err, pll.w, TS);
  float vFeed = vInst;                             // feed-forward
  float vOut = constrain(vFeed + vCmd, -VBUS_NOM, VBUS_NOM);

  // Unipolar SPWM onto the full bridge (LEDC ch 0..1; the
  // gate-driver board makes the complementary signals).
  float duty = constrain(0.5f + 0.5f * (vOut / VBUS_NOM), 0.03f, 0.97f);
  uint32_t d = (uint32_t)(duty * 1023);
  ledcWrite(0, d);            // leg1 high (leg1 low = driver /IN)
  ledcWrite(1, 1023 - d);     // leg2 (opposite phase)
}

// =============================================================
//  PROTECTION TASK (100 ms slot, as v1's islanding check)
// =============================================================
void protectionTask() {
  prot.update(gridVrms, gridFreq, pllLocked,
              mode == MODE_GRID_TIED, millis());
  rocof = prot.rocof;

  // Contactor: closed only while the state machine is in
  // RESYNC/RAMP or RUN on the grid. (Droop mode forms its own
  // bus with the contactor open, as in v1.)
  bool closeRelay = (mode == MODE_GRID_TIED) && prot.outputAllowed();
  digitalWrite(RELAY_PIN, closeRelay ? HIGH : LOW);

  // Bench diagnostics: log PLL lock transitions once, not at
  // the 100 ms cadence.
  static bool prevLocked = false;
  if (pllLocked != prevLocked) {
    prevLocked = pllLocked;
    logEvent(pllLocked ? "PLL locked on grid" : "PLL lost lock");
  }

  // Forward protection log events (trips, sequence steps).
  char msg[192];
  while (prot.pollEvent(msg, sizeof(msg))) logEvent(String(msg));
}

void logEvent(const String& msg) {
  eventLog += msg + "\n";
  if (eventLog.length() > 1600) eventLog = eventLog.substring(500);
  Serial.println(msg);
}

// =============================================================
//  DASHBOARD
// =============================================================
WebServer server(80);

const char DASHBOARD[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>GridSync - Grid-Tied Inverter</title>
<style>
 body{font-family:'Segoe UI',Arial;background:#0b1120;color:#e2e8f0;margin:0;padding:16px}
 h1{color:#38bdf8;text-align:center;margin:4px}
 .sub{text-align:center;color:#94a3b8;margin-bottom:14px}
 .wrap{max-width:980px;margin:auto}
 .cards{display:flex;gap:12px;flex-wrap:wrap;justify-content:center}
 .card{background:#1e293b;border-radius:12px;padding:14px 18px;min-width:140px;text-align:center}
 .card b{font-size:1.6em;color:#f8fafc}.card span{color:#94a3b8;font-size:.85em}
 .ok{color:#4ade80}.warn{color:#fbbf24}.bad{color:#f87171}
 button{background:#0284c7;color:#fff;border:0;border-radius:8px;padding:10px 18px;font-size:1em;cursor:pointer;margin:4px}
 button.stop{background:#b91c1c}button.droop{background:#7c3aed}
 canvas{background:#0f172a;border:1px solid #334155;border-radius:12px;width:100%}
 pre{background:#0f172a;border-radius:8px;padding:10px;white-space:pre-wrap;max-height:150px;overflow:auto}
</style></head><body><div class="wrap">
<h1>⚡ GridSync — Grid-Tied Inverter</h1>
<div class="sub">SOGI-PLL · PR Current Control · Anti-Islanding · Droop Microgrid Mode</div>
<div class="cards">
 <div class="card"><span>GRID VOLTAGE</span><br><b id="gv">0</b><br><span>V rms</span></div>
 <div class="card"><span>GRID FREQ</span><br><b id="gf">0</b><br><span>Hz</span></div>
 <div class="card"><span>PLL</span><br><b id="pll">--</b><br><span>SOGI lock</span></div>
 <div class="card"><span>POWER OUT</span><br><b id="po">0</b><br><span>W</span></div>
 <div class="card"><span>REACTIVE</span><br><b id="qo">0</b><br><span>VAR</span></div>
 <div class="card"><span>DC BUS</span><br><b id="busv">0</b><br><span>V</span></div>
 <div class="card"><span>MODE</span><br><b id="md">OFF</b><br><span id="trip"></span></div>
 <div class="card"><span>PROTECTION</span><br><b id="prot">RUN</b><br><span id="pdet">reconnection state</span></div>
</div>
<p style="text-align:center">
 P set-point: <input type="range" id="pr" min="0" max="500" value="200" step="10"> <b id="prv">200</b> W<br>
 <button onclick="cmd('grid')">Grid-Tied Mode</button>
 <button class="droop" onclick="cmd('droop')">Droop / Microgrid Mode</button>
 <button class="stop" onclick="cmd('off')">STOP</button>
 <button onclick="cmd('reset')">Reset Trip / Reconnect</button>
</p>
<h3>Frequency trend</h3>
<canvas id="tr" width="940" height="200"></canvas>
<h3>Event Log</h3><pre id="log"></pre>
<script>
let hist=[];
const pr=document.getElementById('pr');
pr.oninput=()=>document.getElementById('prv').textContent=pr.value;
pr.onchange=()=>fetch('/cmd?c=p&v='+pr.value);
function cmd(c){fetch('/cmd?c='+c);}
function draw(){
 const cv=document.getElementById('tr'),x=cv.getContext('2d');
 x.clearRect(0,0,cv.width,cv.height);
 x.strokeStyle='#334155';x.beginPath();x.moveTo(40,10);x.lineTo(40,170);x.lineTo(920,170);x.stroke();
 x.strokeStyle='#475569';x.setLineDash([4,4]);x.beginPath();
 const y50=170-((50-47)/6)*150;x.moveTo(40,y50);x.lineTo(920,y50);x.stroke();x.setLineDash([]);
 if(hist.length<2)return;
 x.strokeStyle='#38bdf8';x.lineWidth=2;x.beginPath();
 hist.forEach((v,i)=>{const px=40+i*(880/119),py=170-((v-47)/6)*150;i?x.lineTo(px,py):x.moveTo(px,py);});
 x.stroke();
}
async function tick(){
 const d=await(await fetch('/data')).json();
 document.getElementById('gv').textContent=d.gv.toFixed(1);
 document.getElementById('gf').textContent=d.gf.toFixed(2);
 const p=document.getElementById('pll');
 p.textContent=d.locked?'LOCKED':'SEARCHING';p.className=d.locked?'ok':'warn';
 document.getElementById('po').textContent=d.po.toFixed(0);
 document.getElementById('qo').textContent=d.qo.toFixed(0);
 document.getElementById('busv').textContent=d.vb.toFixed(1);
 document.getElementById('md').textContent=d.mode;
 document.getElementById('trip').textContent=d.tripped?'⚠ ISLAND TRIPPED':'';
 document.getElementById('trip').className='bad';
 const ps=document.getElementById('prot'),pd=document.getElementById('pdet');
 ps.textContent=d.prot;
 ps.className=(d.prot==='RUN')?'ok':((d.prot==='TRIPPED')?'bad':'warn');
 if(d.prot==='WAIT')pd.textContent='reconnect in '+d.cd+' s';
 else if(d.prot==='VERIFY')pd.textContent=(d.vstat?d.vstat:'proving grid')+' · '+d.cd+' s';
 else if(d.prot==='RESYNC/RAMP')pd.textContent='ramping current: '+d.ramp+'%';
 else if(d.prot==='TRIPPED')pd.textContent=d.reason;
 else pd.textContent='protection armed · trips: '+d.trips;
 document.getElementById('log').textContent=d.log;
 hist.push(d.gf);if(hist.length>120)hist.shift();draw();
}
setInterval(tick,500);tick();
</script></div></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html", DASHBOARD); }

void handleData() {
  String j = "{";
  j += "\"gv\":" + String(gridVrms, 2);
  j += ",\"gf\":" + String(gridFreq, 3);
  j += ",\"locked\":" + String(pllLocked ? "true" : "false");
  j += ",\"po\":" + String(pOut_W, 1);
  j += ",\"qo\":" + String(qOut_VAR, 1);
  j += ",\"mode\":\"" + String(mode == MODE_GRID_TIED ? "GRID-TIED" :
                              (mode == MODE_DROOP ? "DROOP" : "OFF")) + "\"";
  j += ",\"tripped\":" + String(prot.tripped ? "true" : "false");
  // v2 protection fields: state-machine state, countdown (s),
  // ramp percent, active-frequency-shift bias, last trip reason
  j += ",\"prot\":\"" + String(prot.stateName()) + "\"";
  j += ",\"cd\":" + String(prot.countdownSec(millis()));
  j += ",\"ramp\":" + String(prot.rampPercent());
  j += ",\"afd\":" + String(prot.afdShift, 4);
  j += ",\"vb\":" + String(readVbus(), 1);
  j += ",\"trips\":" + String(prot.tripCount);
  j += ",\"vstat\":\"" + String(prot.verifyStatus) + "\"";
  String reason = String(prot.tripReason); reason.replace("\"", "'");
  j += ",\"reason\":\"" + reason + "\"";
  j += ",\"log\":\"";
  String esc = eventLog; esc.replace("\n", "\\n"); esc.replace("\"", "'");
  j += esc + "\"}";
  server.send(200, "application/json", j);
}

void handleCmd() {
  String c = server.arg("c");
  if (c == "grid") {
    mode = MODE_GRID_TIED;
    if (prot.tripped) {
      logEvent("GRID-TIED selected but protection is TRIPPED - "
               "press Reset to run the reconnection sequence");
    } else {
      logEvent("GRID-TIED mode: PLL sync + PR current control");
    }
  }
  if (c == "droop") {
    mode = MODE_DROOP; digitalWrite(RELAY_PIN, LOW);
    logEvent("DROOP mode: f-P / V-Q droop active (microgrid forming)");
  }
  if (c == "off") { mode = MODE_OFF; digitalWrite(RELAY_PIN, LOW); logEvent("Inverter OFF"); }
  if (c == "reset") {
    // v2: reset enters WAIT -> VERIFY -> RAMP, never instant RUN
    prot.requestReset(millis());
  }
  if (c == "p") { pRef_W = server.arg("v").toFloat(); logEvent("P ref = " + String(pRef_W, 0) + " W"); }
  server.send(200, "text/plain", "ok");
}

// =============================================================
//  SETUP & LOOP
// =============================================================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  prCtrl.outMax = VBUS_NOM;      // PR anti-windup clamp = bus volts

  // Protection configuration (struct defaults restated here so
  // the operating point is visible in one place; windows are
  // the v1 values, timings are the reconnection sequence).
  prot.vNom            = 24.0f;   // demo bus nominal, V rms
  prot.reconnectDelayMs = 30000;  // WAIT before re-verifying
  prot.verifyTimeMs     = 5000;   // grid must hold healthy
  prot.rampTimeMs       = 3000;   // current ramp on reconnect

  // Full-bridge legs: two LEDC channels; the gate-driver board
  // generates complementary signals + dead time in hardware.
  ledcSetup(0, 20000, 10); ledcAttachPin(PWM_H1_PIN, 0);
  ledcSetup(1, 20000, 10); ledcAttachPin(PWM_H2_PIN, 1);
  ledcWrite(0, 0); ledcWrite(1, 0);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(300);
  if (WiFi.status() == WL_CONNECTED) {
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/cmd", handleCmd);
    server.begin();
    logEvent("Dashboard: http://" + WiFi.localIP().toString());
  }
  logEvent("GridSync v2 ready. DEMO BUS ONLY - low-voltage AC.");
}

unsigned long lastStep = 0, lastProt = 0;
void loop() {
  server.handleClient();
  unsigned long now = micros();
  if (now - lastStep >= 100) {           // 10 kHz control
    lastStep = now;
    controlStep();
  }
  if (millis() - lastProt >= 100) {      // 100 ms protection slot
    lastProt = millis();
    protectionTask();
  }
  handleSerial();
}

// Minimal serial command line (same actions as the dashboard,
// for bench work without Wi-Fi): g=grid-tied, d=droop, o=off,
// r=reset/reconnect, p<watts> set P (e.g. "p250"), ?=status.
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;
  char c = line.charAt(0);
  if (c == 'g') {
    mode = MODE_GRID_TIED;
    logEvent(prot.tripped
        ? "GRID-TIED selected but TRIPPED - send 'r' to reconnect"
        : "GRID-TIED mode (serial)");
  }
  else if (c == 'd') { mode = MODE_DROOP; digitalWrite(RELAY_PIN, LOW); logEvent("DROOP mode (serial)"); }
  else if (c == 'o') { mode = MODE_OFF;   digitalWrite(RELAY_PIN, LOW); logEvent("Inverter OFF (serial)"); }
  else if (c == 'r') { prot.requestReset(millis()); }
  else if (c == 'p') {
    pRef_W = line.substring(1).toFloat();
    logEvent("P ref = " + String(pRef_W, 0) + " W (serial)");
  }
  else if (c == '?') {
    Serial.printf("V=%.1fV f=%.2fHz PLL=%s P=%.0fW Q=%.0fVAR mode=%d prot=%s trips=%u\n",
                  gridVrms, gridFreq, pllLocked ? "LOCK" : "----",
                  pOut_W, qOut_VAR, (int)mode, prot.stateName(),
                  prot.tripCount);
  }
}

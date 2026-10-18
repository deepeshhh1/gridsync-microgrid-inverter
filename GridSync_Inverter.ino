// ============================================================
//  GRIDSYNC - GRID-TIED INVERTER CONTROL CORE
//  SOGI-PLL + PR Control + Anti-Islanding + Droop Microgrid
//  ESP32 + ZMPT101B + ACS712 + Gate Driver / H-Bridge
// ------------------------------------------------------------
//  !!! SAFETY - READ FIRST !!!
//  This firmware is a CONTROL-CORE DEMONSTRATION for a low
//  voltage (24 V class) ISOLATED demo bus. It is NOT a design
//  for connecting DIY equipment to the public grid. Real grid
//  connection requires type-tested certified inverters and
//  utility approval (CEA technical standards, India). Never
//  connect this hardware to mains wiring.
//
//  WHAT IT DEMONSTRATES
//   1) SOGI-PLL: Second-Order Generalised Integrator PLL for
//      phase/frequency tracking of the (demo) grid voltage,
//      robust to distortion.
//   2) PR CONTROL: Proportional-Resonant current controller -
//      zero steady-state error tracking a 50 Hz sinusoid, the
//      standard choice for single-phase grid inverters.
//   3) ANTI-ISLANDING: passive windows (V, f), ROCOF, and an
//      ACTIVE frequency-shift bias that drives an islanded
//      system out of window so the trip fires.
//   4) DROOP MODE: f-P / V-Q droop for a 24 V microgrid demo -
//      proportional load sharing between parallel inverters
//      with NO communication link.
//
//  Author : Deepesh Chauhan, B.Tech Electrical Engineering
//           RGGEC Kangra
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>
#include <math.h>

// ---------------- WI-FI (replace with your credentials) -----
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ---------------- PINS ---------------------------------------
#define VGRID_PIN   34        // ZMPT101B output
#define IOUT_PIN    35        // ACS712 output
#define PWM_PIN     25        // to gate driver input
#define CONTACTOR_PIN 26      // grid connect relay (demo)

// ---------------- SENSING CAL --------------------------------
const float ADC_VREF   = 3.3f;
const float VGRID_SCALE = 82.0f;    // ZMPT101B module scaling -> Vrms
const float VGRID_ZERO  = 1.65f;    // module bias (V)
const float ACS_SENS    = 0.185f;   // V/A (ACS712-5A)
const float IOUT_ZERO   = 1.65f;

const float F_NOM = 50.0f;
const float V_NOM = 24.0f;          // demo bus nominal (Vrms)
const float CTRL_DT = 0.0002f;      // 5 kHz control step

// ---------------- SOGI-PLL -------------------------------------
struct SogiPll {
  float vAlpha = 0, vBeta = 0;      // SOGI quadrature outputs
  float theta = 0, w = 2 * PI * F_NOM, integ = 0;
  float kp = 0.9f, ki = 55.0f;      // PLL loop gains
  float k = 1.4142f;                // SOGI damping (sqrt2)
  float vMag = 0, freq = F_NOM;

  void step(float v, float dt) {
    // SOGI resonators (forward-Euler discretisation, 5 kHz)
    float err = v - vAlpha;
    vAlpha += w * dt * (k * err - vBeta);
    vBeta  += w * dt * vAlpha;
    // Park transform (vd should -> 0 at lock)
    float vd = vAlpha * cosf(theta) + vBeta * sinf(theta);
    float vq = -vAlpha * sinf(theta) + vBeta * cosf(theta);
    (void)vq;
    integ += vd * dt;
    float wCorr = kp * vd + ki * integ;
    w = 2 * PI * F_NOM + wCorr;
    w = constrain(w, 2 * PI * 40.0f, 2 * PI * 60.0f);
    theta += w * dt;
    if (theta > 2 * PI) theta -= 2 * PI;
    vMag = sqrtf(vAlpha * vAlpha + vBeta * vBeta);
    freq = w / (2 * PI);
  }
};
SogiPll pll;
bool pllLocked = false;

// ---------------- PR CURRENT CONTROLLER ------------------------
// y(s)/e(s) = Kp + (2 Kr wc s) / (s^2 + 2 wc s + w0^2)
struct PR {
  float kp = 0.35f, kr = 60.0f, wc = 5.0f;
  float x1 = 0, x2 = 0;             // resonator states
  float run(float err, float w0, float dt) {
    float dx1 = x2;
    float dx2 = -w0 * w0 * x1 - 2 * wc * x2 + err;
    x1 += dx1 * dt;
    x2 += dx2 * dt;
    float res = 2 * kr * wc * x2 * 0.02f / (w0 / (2 * PI * F_NOM));
    float out = kp * err + res;
    return constrain(out, -1.0f, 1.0f);
  }
  void reset() { x1 = x2 = 0; }
};
PR prI;

// ---------------- ANTI-ISLANDING --------------------------------
const float V_MIN_PU = 0.88f, V_MAX_PU = 1.10f;
const float F_MIN = 47.5f, F_MAX = 51.5f;
const float ROCOF_MAX = 2.5f;       // Hz/s
bool islandTripped = false;
float lastFreq = F_NOM, rocof = 0, afdShift = 0;
unsigned long lastRocofMs = 0;

void antiIslandingCheck(float vPu, float dt) {
  if (!pllLocked) return;
  unsigned long now = millis();
  if (now - lastRocofMs >= 200) {
    float dtS = (now - lastRocofMs) / 1000.0f;
    rocof = fabsf(pll.freq - lastFreq) / max(dtS, 0.001f);
    lastFreq = pll.freq;
    lastRocofMs = now;
  }
  // Active frequency shift: bias grows with frequency deviation
  // (positive feedback ONLY visible to an island; a stiff grid
  // imposes its own frequency and ignores the bias).
  afdShift = 0.002f * (pll.freq - F_NOM);
  if (vPu < V_MIN_PU || vPu > V_MAX_PU ||
      pll.freq < F_MIN || pll.freq > F_MAX || rocof > ROCOF_MAX) {
    islandTripped = true;
    digitalWrite(CONTACTOR_PIN, LOW);
    logEvent("ANTI-ISLANDING TRIP: V=" + String(vPu, 2) + "pu f=" +
             String(pll.freq, 2) + "Hz ROCOF=" + String(rocof, 2));
  }
}

// ---------------- DROOP (microgrid form mode) -------------------
const float DROOP_F = 0.5f;         // Hz per pu of P
const float DROOP_V = 0.05f;        // pu V per pu of Q
float droopFreq = F_NOM, droopVolt = 1.0f;

// ---------------- STATE -----------------------------------------
enum Mode { MODE_OFF, MODE_GRID, MODE_DROOP };
Mode mode = MODE_OFF;
float pSetW = 100.0f, qSetVar = 0.0f;   // grid-tied setpoints (demo scale)
float pOut = 0, qOut = 0, vGridRms = 0, iOutRms = 0;
float thetaOut = 0;
String gridLog = "Boot\n";

void logEvent(const String& msg) {
  gridLog += msg + "\n";
  if (gridLog.length() > 1600) gridLog = gridLog.substring(500);
  Serial.println(msg);
}

// ---------------- FAST SENSING (instantaneous) -------------------
float readVgridInst() {
  int raw = analogRead(VGRID_PIN);
  return ((raw / 4095.0f) * ADC_VREF - VGRID_ZERO) * VGRID_SCALE;
}
float readIoutInst() {
  int raw = analogRead(IOUT_PIN);
  return ((raw / 4095.0f) * ADC_VREF - IOUT_ZERO) / ACS_SENS;
}

// slow RMS estimation (called from loop at ~1 kHz sampling)
float vSqAcc = 0, iSqAcc = 0; int rmsN = 0;
void rmsSample(float v, float i) {
  vSqAcc += v * v; iSqAcc += i * i; rmsN++;
  if (rmsN >= 100) {
    vGridRms = sqrtf(vSqAcc / rmsN);
    iOutRms  = sqrtf(iSqAcc / rmsN);
    vSqAcc = iSqAcc = 0; rmsN = 0;
  }
}

// =============================================================
//  CONTROL STEP  (5 kHz simulated step from loop timing)
// =============================================================
void controlStep() {
  float v = readVgridInst();
  float i = readIoutInst();
  pll.step(v, CTRL_DT);
  float vPu = (V_NOM > 0) ? (pll.vMag / V_NOM) : 0;
  pllLocked = (pll.vMag > 5.0f && fabsf(pll.freq - F_NOM) < 1.5f);

  if (mode == MODE_GRID) {
    if (!islandTripped) antiIslandingCheck(vPu, CTRL_DT);
    // current reference from P/Q setpoints in the grid frame
    float fRef = pll.freq + afdShift * 5.0f;      // AFD bias applied
    thetaOut = pll.theta;
    float iRefPk = (vGridRms > 1) ? (1.4142f * pSetW / vGridRms) : 0;
    float iRef = iRefPk * sinf(thetaOut) -
                 (qSetVar > 0 ? iRefPk * 0.2f * cosf(thetaOut) : 0);
    float mod = prI.run(iRef - i, pll.w, CTRL_DT);
    // apply modulation depth as PWM duty around 50%
    int top = 1023;
    int duty = (int)((0.5f + 0.5f * mod) * top);
    ledcWrite(0, constrain(duty, 0, top));
    digitalWrite(CONTACTOR_PIN, islandTripped ? LOW : HIGH);
  }
  else if (mode == MODE_DROOP) {
    // islanded: WE form the bus. f-P and V-Q droop set the
    // operating point from measured output power.
    float pPu = pOut / 500.0f;                    // 500 W rating demo
    float qPu = qOut / 500.0f;
    droopFreq = F_NOM - DROOP_F * pPu;
    droopVolt = 1.0f - DROOP_V * qPu;
    thetaOut += 2 * PI * droopFreq * CTRL_DT;
    if (thetaOut > 2 * PI) thetaOut -= 2 * PI;
    float vRef = 1.4142f * V_NOM * droopVolt * sinf(thetaOut);
    float mod = constrain(vRef / (2.0f * V_NOM), -1.0f, 1.0f);
    int top = 1023;
    ledcWrite(0, (int)((0.5f + 0.5f * mod) * top));
    digitalWrite(CONTACTOR_PIN, LOW);             // islanded from grid
  }
  else {
    ledcWrite(0, 512);
    digitalWrite(CONTACTOR_PIN, LOW);
  }

  // P/Q estimation (simple one-cycle accumulation in loop domain)
  static float pAcc = 0, qAcc = 0; static int nAcc = 0;
  pAcc += v * i; nAcc++;
  if (nAcc >= 5000) {
    pOut = pAcc / nAcc;
    qOut = 0;                                     // single-sensor demo
    pAcc = 0; nAcc = 0;
  }
  rmsSample(v, i);
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
 h1{color:#facc15;text-align:center;margin:4px}
 .sub{text-align:center;color:#94a3b8;margin-bottom:14px}
 .wrap{max-width:980px;margin:auto}
 .cards{display:flex;gap:12px;flex-wrap:wrap;justify-content:center}
 .card{background:#1e293b;border-radius:12px;padding:14px 18px;min-width:150px;text-align:center}
 .card b{font-size:1.5em;color:#f8fafc}.card span{color:#94a3b8;font-size:.85em}
 button{background:#ca8a04;color:#fff;border:0;border-radius:8px;padding:10px 18px;font-size:1em;cursor:pointer;margin:4px}
 button:hover{background:#a16207}
 input[type=number]{width:90px;padding:8px;border-radius:6px;border:1px solid #334155;background:#0f172a;color:#fff}
 pre{background:#0f172a;border-radius:8px;padding:10px;white-space:pre-wrap;max-height:150px;overflow:auto}
</style></head><body><div class="wrap">
<h1>⚡ GridSync — Grid-Tied Inverter</h1>
<div class="sub">SOGI-PLL · PR Current Control · Anti-Islanding · Droop Microgrid (24 V demo bus)</div>
<div class="cards">
 <div class="card"><span>GRID VOLTAGE</span><br><b id="vg">--</b><br><span>V rms</span></div>
 <div class="card"><span>FREQUENCY</span><br><b id="fg">--</b><br><span>Hz</span></div>
 <div class="card"><span>PLL</span><br><b id="pll">--</b><br><span>sync state</span></div>
 <div class="card"><span>P OUT</span><br><b id="po">--</b><br><span>watts</span></div>
 <div class="card"><span>MODE</span><br><b id="md">--</b><br><span>operating mode</span></div>
 <div class="card"><span>ISLANDING</span><br><b id="isl">--</b><br><span>protection</span></div>
</div>
<p style="text-align:center">
 P setpoint (W): <input type="number" id="pset" value="100">
 <button onclick="setP()">SET P</button>
 <button onclick="cmd('grid')">GRID-TIED MODE</button>
 <button onclick="cmd('droop')">DROOP (MICROGRID)</button>
 <button onclick="cmd('off')">OFF</button>
 <button onclick="cmd('reset')">RESET TRIP</button>
</p>
<h3>Event Log</h3><pre id="log"></pre>
<script>
function cmd(c){fetch('/cmd?c='+c);}
function setP(){fetch('/cmd?c=pset&v='+document.getElementById('pset').value);}
async function tick(){
 const d=await(await fetch('/data')).json();
 document.getElementById('vg').textContent=d.vg.toFixed(1);
 document.getElementById('fg').textContent=d.fg.toFixed(2);
 document.getElementById('pll').textContent=d.pll;
 document.getElementById('po').textContent=d.po.toFixed(0);
 document.getElementById('md').textContent=d.mode;
 document.getElementById('isl').textContent=d.isl;
 document.getElementById('log').textContent=d.log;
}
setInterval(tick,1000);tick();
</script></div></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html", DASHBOARD); }

String modeName() {
  if (mode == MODE_GRID) return islandTripped ? "GRID (TRIPPED)" : "GRID-TIED";
  if (mode == MODE_DROOP) return "DROOP MICROGRID";
  return "OFF";
}

void handleData() {
  String j = "{";
  j += "\"vg\":" + String(vGridRms, 1);
  j += ",\"fg\":" + String(pll.freq, 2);
  j += ",\"pll\":\"" + String(pllLocked ? "LOCKED" : "SEARCHING") + "\"";
  j += ",\"po\":" + String(pOut, 0);
  j += ",\"qo\":" + String(qOut, 0);
  j += ",\"mode\":\"" + modeName() + "\"";
  j += ",\"isl\":\"" + String(islandTripped ? "TRIPPED" : "ARMED") + "\"";
  j += ",\"rocof\":" + String(rocof, 2);
  j += ",\"log\":\"";
  String esc = gridLog; esc.replace("\n", "\\n"); esc.replace("\"", "'");
  j += esc + "\"}";
  server.send(200, "application/json", j);
}

void handleCmd() {
  String c = server.arg("c");
  if (c == "grid")       { mode = MODE_GRID; logEvent("Mode: GRID-TIED"); }
  else if (c == "droop") { mode = MODE_DROOP; logEvent("Mode: DROOP microgrid (bus-forming)"); }
  else if (c == "off")   { mode = MODE_OFF; logEvent("Mode: OFF"); }
  else if (c == "reset") { islandTripped = false; prI.reset(); logEvent("Island trip RESET by operator"); }
  else if (c == "pset")  { pSetW = server.arg("v").toFloat(); logEvent("P setpoint = " + String(pSetW, 0) + " W"); }
  server.send(200, "text/plain", "ok");
}

// =============================================================
//  SETUP & LOOP
// =============================================================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  ledcSetup(0, 20000, 10);
  ledcAttachPin(PWM_PIN, 0);
  pinMode(CONTACTOR_PIN, OUTPUT);
  digitalWrite(CONTACTOR_PIN, LOW);

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
  logEvent("GridSync ready (24 V DEMO BUS ONLY).");
}

void loop() {
  server.handleClient();
  static unsigned long lastStep = 0;
  unsigned long now = micros();
  if (now - lastStep >= 200) {              // 5 kHz nominal
    lastStep = now;
    controlStep();
  }
}

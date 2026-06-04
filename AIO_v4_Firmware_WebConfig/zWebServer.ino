// ── Web server – port 80 ───────────────────────────────────────────────────────
// Non-blocking: handleWebClient() is called every loop() iteration.
// When no browser is connected the call returns in < 1 µs.

#include <stdarg.h>

EthernetServer webServer(80);

static bool          pendingRestart = false;
static elapsedMillis restartDelay   = 0;

// ── Serial-to-web log ring buffer ─────────────────────────────────────────────
#define LOG_BUF_SIZE 4096
static char     logBuf[LOG_BUF_SIZE];
static uint16_t logWrite   = 0;
static bool     logWrapped = false;

// ── GPS raw ring buffer (UM98x config tab) ────────────────────────────────────
#define GPS_RAW_BUF_SIZE 4096
static char     gpsRawBuf[GPS_RAW_BUF_SIZE];
static uint16_t gpsRawWrite   = 0;
static bool     gpsRawWrapped = false;

void gpsRawByte(uint8_t c)
{
    if (!gpsRawActive) return;
    gpsRawBuf[gpsRawWrite] = (char)c;
    if (++gpsRawWrite >= GPS_RAW_BUF_SIZE) { gpsRawWrite = 0; gpsRawWrapped = true; }
}

void webLog(const char* msg)
{
    if (!msg || !logActive) return;
    while (*msg) {
        logBuf[logWrite] = *msg++;
        if (++logWrite >= LOG_BUF_SIZE) { logWrite = 0; logWrapped = true; }
    }
    logBuf[logWrite] = '\n';
    if (++logWrite >= LOG_BUF_SIZE) { logWrite = 0; logWrapped = true; }
}

void webLogf(const char* fmt, ...)
{
    char tmp[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    webLog(tmp);
}

// Log autosteer disengage source. Gated by DBG_DISENGAGE flag.
// Rate-limits identical reasons (once/500ms) so continuous conditions don't flood.
// Forces the message into the web log buffer even if live capture is off.
void disengageLog(const char* reason)
{
    if (!DBG_DISENGAGE) return;
    static uint32_t lastMs  = 0;
    static char     lastMsg[48] = "";
    uint32_t now = millis();
    if ((now - lastMs) < 500 && strncmp(lastMsg, reason, sizeof(lastMsg)) == 0) return;
    lastMs = now;
    strncpy(lastMsg, reason, sizeof(lastMsg) - 1);
    Serial.print("DISENGAGE: "); Serial.println(reason);
    bool prev = logActive;
    logActive = true;            // force into buffer regardless of capture toggle
    webLogf("DISENGAGE: %s", reason);
    logActive = prev;
}

// ── HTML page stored in flash ──────────────────────────────────────────────────
static const char HTML_PAGE[] = R"AIOHTML(<!DOCTYPE html>
<html lang="hr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AIO v4 Config</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:monospace;background:#0f172a;color:#e2e8f0;padding:12px;font-size:15px}
h1{color:#38bdf8;margin-bottom:12px;font-size:1.1em;letter-spacing:1px}
.tabs{display:flex;gap:6px;margin-bottom:14px}
.tab{padding:6px 16px;background:#1e293b;border:1px solid #334155;color:#94a3b8;cursor:pointer;border-radius:4px;font-family:monospace;font-size:14px}
.tab.active{border-color:#38bdf8;color:#38bdf8;background:#0c1a2e}
.panel{display:none}.panel.active{display:block}
.card{background:#1e293b;border:1px solid #334155;border-radius:6px;padding:12px;margin-bottom:10px}
.card h2{color:#38bdf8;font-size:13px;text-transform:uppercase;letter-spacing:1px;margin-bottom:10px;padding-bottom:6px;border-bottom:1px solid #334155;display:flex;align-items:center;gap:8px;font-weight:bold}
.row{display:flex;justify-content:space-between;align-items:center;padding:4px 0;border-bottom:1px solid #1e293b}
.row:last-child{border-bottom:none}
.lbl{color:#cbd5e1;font-size:15px;font-weight:500}
.val{color:#e2e8f0;text-align:right}
.badge{padding:1px 8px;border-radius:3px;font-size:13px;font-weight:bold;min-width:50px;text-align:center;display:inline-block}
.ok{background:#052e16;color:#4ade80}
.fail{background:#1c0505;color:#f87171}
select{background:#0f172a;border:1px solid #334155;color:#e2e8f0;padding:4px 6px;border-radius:3px;font-family:monospace;font-size:14px;min-width:200px}
.btn{padding:7px 18px;background:#0284c7;color:#fff;border:none;border-radius:4px;cursor:pointer;font-family:monospace;font-size:14px;margin-top:10px}
.btn:hover{background:#0369a1}
.btn.green{background:#15803d}.btn.green:hover{background:#166534}
.btn.sm{padding:3px 10px;font-size:13px;margin-top:0;margin-left:auto}
.chk-row{display:flex;align-items:center;gap:8px;padding:5px 0;font-size:15px;color:#e2e8f0}
input[type=checkbox]{width:15px;height:15px;cursor:pointer;accent-color:#38bdf8}
#sb{font-size:13px;color:#64748b;margin-top:8px;padding-top:6px;border-top:1px solid #1e293b}
.section-row{display:flex;gap:10px;flex-wrap:wrap}
.section-row .card{flex:1;min-width:280px}
#log,#gpsraw{background:#050d1a;border:1px solid #1e3a5f;border-radius:3px;padding:8px;height:320px;overflow-y:scroll;font-size:13px;color:#7dd3fc;white-space:pre-wrap;word-break:break-all;margin-top:8px}
textarea.gps-ta{width:100%;height:110px;background:#050d1a;border:1px solid #334155;color:#7dd3fc;font-family:monospace;font-size:14px;padding:6px;border-radius:3px;resize:vertical}
.btn.red{background:#7f1d1d}.btn.red:hover{background:#991b1b}
.ninput{background:#0f172a;border:1px solid #334155;color:#e2e8f0;padding:4px 6px;border-radius:3px;font-family:monospace;font-size:14px;width:80px;text-align:right}
</style>
</head>
<body>
<h1>&#9881; AIO v4 | Web Config</h1>
<div class="tabs">
<button class="tab active" onclick="showTab('config',this)">Config</button>
<button class="tab" onclick="showTab('live',this)">Live</button>
<button class="tab" onclick="showTab('keya',this)">Keya</button>
<button class="tab" onclick="showTab('cansteer',this)">CAN Steer</button>
<button class="tab" onclick="showTab('debug',this)">Debug</button>
<button class="tab" onclick="showTab('um98x',this)">UM98x Config</button>
</div>

<!-- CONFIG TAB -->
<div id="config" class="panel active">

<div class="section-row">
<div class="card">
<h2>Detected devices</h2>
<style>
.grp{color:#38bdf8;font-size:13px;font-weight:bold;padding:6px 0 2px;border-bottom:1px solid #334155;margin-bottom:2px}
.sub{padding-left:18px;font-size:14px}
</style>
<div class="row"><span class="lbl grp">GPS</span><span id="d5" class="badge fail">?</span></div>
<div class="row sub"><span class="lbl">GGA</span><span id="d_gga" class="badge fail">?</span></div>
<div class="row sub"><span class="lbl">VTG</span><span id="d_vtg" class="badge fail">?</span></div>
<div class="row sub"><span class="lbl">HPR <small style="color:#64748b">(dual heading)</small></span><span id="d_hpr" class="badge fail">?</span></div>
<div class="row" style="padding-top:8px"><span class="lbl grp">IMU</span><span></span></div>
<div class="row sub"><span class="lbl">BNO085 I2C</span><span id="d1" class="badge fail">?</span></div>
<div class="row sub"><span class="lbl">TM171</span><span id="d2" class="badge fail">?</span></div>
<div class="row" style="padding-top:8px"><span class="lbl grp">WAS</span><span></span></div>
<div class="row sub"><span class="lbl">ADS1115</span><span id="d_ads" class="badge fail">?</span></div>
<div class="row sub"><span class="lbl">IMU as WAS (CAN)</span><span id="d_imuWas" class="badge fail">?</span></div>
<div class="row" style="padding-top:8px"><span class="lbl grp">Keya motor</span><span id="d3" class="badge fail">?</span></div>
</div>

<div class="card">
<h2>Data Source <span style="color:#64748b;font-weight:normal;font-size:11px">— restart required on change</span></h2>
<p style="color:#64748b;font-size:12px;margin-bottom:8px;line-height:1.4">Select which sensor or message provides each data channel sent to AgIO.</p>
<div class="lbl" style="margin:8px 0 3px">WAS sensor</div>
<select id="wasSource" style="width:100%">
<option value="0">ADS1115 — analog voltage sensor (default)</option>
<option value="1">Keya encoder — brushless motor position</option>
<option value="2">IMU via CAN — wheel-mounted IMU on CAN bus</option>
<option value="3">CAN valve — angle from V_Bus steer valve</option>
</select>
<p style="color:#94a3b8;font-size:12px;margin:3px 0 8px;line-height:1.3">Wheel angle source. For CAN options the corresponding CAN port must be configured below.</p>
<div class="lbl" style="margin:8px 0 3px">Roll source</div>
<select id="rollSource" style="width:100%">
<option value="0">IMU — onboard BNO085 or TM171 (default)</option>
<option value="1">HPR — $HPR NMEA from dual GPS (UM980 / UM982)</option>
</select>
<p style="color:#94a3b8;font-size:12px;margin:3px 0 8px;line-height:1.3">Roll / tilt angle in the NMEA sentence to AgIO. IMU: from onboard sensor. HPR: receiver computes roll from dual antennas and sends it in the $HPR sentence.</p>
<div class="lbl" style="margin:8px 0 3px">Heading source</div>
<select id="headingSource" style="width:100%">
<option value="0">IMU — onboard BNO085 or TM171 (default)</option>
<option value="1">HPR — $HPR NMEA from dual GPS (UM980 / UM982)</option>
<option value="2">RELPOS — UBX binary from dual u-blox F9P</option>
</select>
<p style="color:#94a3b8;font-size:12px;margin:3px 0 8px;line-height:1.3">Heading direction source. IMU: from compass in BNO085 or TM171. HPR: receiver with two antennas sends $HPR NMEA (single cable). RELPOS: two u-blox F9P receivers connected via UBX binary protocol (older dual setup).</p>
<div class="lbl" style="margin:8px 0 3px">NMEA sentence to AgIO</div>
<select id="nmeaType" style="width:100%">
<option value="0">$PANDA — single GPS or IMU heading (default)</option>
<option value="1">$PAOGI — dual GPS heading</option>
</select>
<p style="color:#94a3b8;font-size:12px;margin:3px 0 8px;line-height:1.3">Sentence name in the output to AgIO. Data fields are identical — AgIO uses the name to know if dual GPS heading is available. Select PAOGI when using HPR or RELPOS heading source.</p>
<button class="btn green" onclick="saveDataSource()" style="margin-top:8px">Save Data Source (restart)</button>
</div>

<div class="card">
<h2>Serial port assignment <span style="color:#64748b;font-weight:normal;font-size:11px">— restart required on change</span></h2>
<div class="lbl" style="margin:8px 0 3px">GPS receiver</div>
<div style="display:flex;gap:8px;margin-bottom:6px">
<select id="gpsSerial" style="min-width:0;width:38%">
<option value="1">Serial1</option><option value="2">Serial2</option><option value="3">Serial3</option>
<option value="4">Serial4</option><option value="5">Serial5</option><option value="6">Serial6</option>
<option value="7">Serial7</option><option value="8">Serial8</option>
</select>
<select id="gpsBaud2" style="min-width:0;width:38%">
<option value="9600">9600</option><option value="19200">19200</option><option value="38400">38400</option>
<option value="57600">57600</option><option value="115200">115200</option><option value="230400">230400</option>
<option value="460800">460800</option><option value="921600">921600</option>
</select></div>
<div class="lbl" style="margin:8px 0 3px">TM171 IMU</div>
<div style="display:flex;gap:8px;margin-bottom:6px">
<select id="tm171Serial" style="min-width:0;width:38%">
<option value="1">Serial1</option><option value="2">Serial2</option><option value="3">Serial3</option>
<option value="4">Serial4</option><option value="5">Serial5</option><option value="6">Serial6</option>
<option value="7">Serial7</option><option value="8">Serial8</option>
</select>
<select id="tm171Baud" style="min-width:0;width:38%">
<option value="9600">9600</option><option value="19200">19200</option><option value="38400">38400</option>
<option value="57600">57600</option><option value="115200">115200</option><option value="230400">230400</option>
<option value="460800">460800</option><option value="921600">921600</option>
</select></div>
<button class="btn green" onclick="saveSerial()" style="margin-top:8px">Save Serial assignment (restart)</button>
</div>

<div class="card">
<h2>CAN port assignment <span style="color:#64748b;font-weight:normal;font-size:11px">— restart required on change</span></h2>
<p style="color:#64748b;font-size:12px;margin-bottom:8px;line-height:1.4">Map each physical CAN port to its function and baud rate.</p>
<div class="row"><span class="lbl">CAN1</span>
<select id="can1Mode" style="flex:2">
<option value="0">Off</option>
<option value="1">Keya motor</option>
<option value="2">IMU as WAS</option>
<option value="3">V_Bus (steer valve)</option>
<option value="4">K_Bus (Fendt engage)</option>
<option value="5">ISO_Bus (engage/hitch)</option>
<option value="6">J1939/NMEA broadcast</option>
<option value="7">CANtest (loopback)</option>
<option value="8">Custom</option>
</select>
<select id="can1Baud" style="flex:1;max-width:90px;min-width:0;margin-left:6px">
<option value="125000">125k</option>
<option value="250000">250k</option>
<option value="500000">500k</option>
<option value="1000000">1M</option>
</select></div>
<div class="row"><span class="lbl">CAN2</span>
<select id="can2Mode" style="flex:2">
<option value="0">Off</option>
<option value="1">Keya motor</option>
<option value="2">IMU as WAS</option>
<option value="3">V_Bus (steer valve)</option>
<option value="4">K_Bus (Fendt engage)</option>
<option value="5">ISO_Bus (engage/hitch)</option>
<option value="6">J1939/NMEA broadcast</option>
<option value="7">CANtest (loopback)</option>
<option value="8">Custom</option>
</select>
<select id="can2Baud" style="flex:1;max-width:90px;min-width:0;margin-left:6px">
<option value="125000">125k</option>
<option value="250000">250k</option>
<option value="500000">500k</option>
<option value="1000000">1M</option>
</select></div>
<div class="row"><span class="lbl">CAN3</span>
<select id="can3Mode" style="flex:2">
<option value="0">Off</option>
<option value="1">Keya motor</option>
<option value="2">IMU as WAS</option>
<option value="3">V_Bus (steer valve)</option>
<option value="4">K_Bus (Fendt engage)</option>
<option value="5">ISO_Bus (engage/hitch)</option>
<option value="6">J1939/NMEA broadcast</option>
<option value="7">CANtest (loopback)</option>
<option value="8">Custom</option>
</select>
<select id="can3Baud" style="flex:1;max-width:90px;min-width:0;margin-left:6px">
<option value="125000">125k</option>
<option value="250000">250k</option>
<option value="500000">500k</option>
<option value="1000000">1M</option>
</select></div>
<button class="btn green" onclick="saveCanModes()" style="margin-top:8px">Save CAN assignment (restart)</button>
</div>

</div>

<div class="card">
<h2>Settings received from AgIO (read-only)</h2>
<div class="section-row" style="gap:0">
<div style="flex:1;min-width:200px">
<div class="row"><span class="lbl">Kp</span><span class="val" id="u0">-</span></div>
<div class="row"><span class="lbl">High PWM</span><span class="val" id="u1">-</span></div>
<div class="row"><span class="lbl">Low PWM</span><span class="val" id="u2">-</span></div>
<div class="row"><span class="lbl">Min PWM</span><span class="val" id="u3">-</span></div>
<div class="row"><span class="lbl">Sensor counts/deg</span><span class="val" id="u4">-</span></div>
<div class="row"><span class="lbl">WAS offset</span><span class="val" id="u5">-</span></div>
</div>
<div style="flex:1;min-width:200px;padding-left:10px">
<div class="row"><span class="lbl">Invert WAS</span><span class="val" id="u6">-</span></div>
<div class="row"><span class="lbl">Cytron driver</span><span class="val" id="u7">-</span></div>
<div class="row"><span class="lbl">Shaft encoder</span><span class="val" id="u8">-</span></div>
<div class="row"><span class="lbl">Pressure sensor</span><span class="val" id="u9">-</span></div>
<div class="row"><span class="lbl">Current sensor</span><span class="val" id="u10">-</span></div>
<div class="row"><span class="lbl">Danfoss</span><span class="val" id="u11">-</span></div>
</div>
</div>
</div>


<div class="card">
<h2>Motor (PWM) speed-direction disengage</h2>
<p style="color:#f59e0b;font-size:13px;margin-bottom:8px">&#9888; Current Sensor must be enabled in AOG</p>
<div class="row"><span class="lbl">Enable</span>
<input type="checkbox" id="md0" style="width:15px;height:15px;accent-color:#38bdf8;cursor:pointer"></div>
<div class="row"><span class="lbl">Angle error min (deg)</span>
<input type="number" id="md1" min="0" max="30" step="1" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Minimum steer angle error (degrees) before motor movement is considered a hand override. Higher = less sensitive.</p>
<div class="row"><span class="lbl">SpeedDiff timeout (ms)</span>
<input type="number" id="md2" min="0" max="5000" step="10" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">How long the motor must run in the wrong direction before autosteer is cut. Shorter = more sensitive to hand override.</p>
<button class="btn green" onclick="saveMotor()" style="margin-top:8px">Save Motor params</button>
</div>

<div class="card">
<h2>IMU as WAS</h2>
<p style="color:#64748b;font-size:13px;margin-bottom:8px">Wheel-mounted IMU sends yaw over CAN1 (ID 0x300, 250 kbps). Set WAS = IMUasWAS and CAN1 = IMUasWAS above. Requires chassis IMU active.</p>
<div class="row"><span class="lbl">Invert direction <small style="color:#64748b">(def OFF)</small></span>
<input type="checkbox" id="iw0" style="width:15px;height:15px;accent-color:#38bdf8;cursor:pointer"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Flip the sign of the measured wheel angle. Enable if turning right shows a negative angle.</p>
<div class="row"><span class="lbl">Sensitivity scale <small style="color:#64748b">(def 1.0)</small></span>
<input type="number" id="iw1" min="0.5" max="2.0" step="0.01" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Scales the raw angle delta between wheel and chassis IMU. Increase if steering angle reads too small, decrease if too large.</p>
<button class="btn" onclick="setImuWasZero()" style="margin-top:6px">Set Zero Now</button>
<p style="color:#94a3b8;font-size:10px;margin:3px 0 5px;line-height:1.3">Resets integrators so the current wheel position reads 0°. Use with wheels pointing straight ahead.</p>
<div style="border-top:1px solid #334155;margin:10px 0 8px"></div>
<div style="color:#38bdf8;font-size:12px;text-transform:uppercase;letter-spacing:1px;margin-bottom:8px">Auto-zero — GPS drift correction</div>
<div class="row"><span class="lbl">Enable <small style="color:#64748b">(def ON)</small></span>
<input type="checkbox" id="iw2" style="width:15px;height:15px;accent-color:#38bdf8;cursor:pointer"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Automatically corrects slow IMU drift by comparing measured angle to GPS-computed wheel angle while driving straight.</p>
<div class="row"><span class="lbl">Beta / correction fraction <small style="color:#64748b">(def 0.05)</small></span>
<input type="number" id="iw3" min="0.001" max="1" step="0.001" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">How aggressively auto-zero corrects drift each cycle. 0.05 = slow and stable, 0.3 = fast but may hunt.</p>
<div class="row"><span class="lbl">Min GPS speed km/h <small style="color:#64748b">(def 1.0)</small></span>
<input type="number" id="iw4" min="0" max="25" step="0.5" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Auto-zero only runs above this speed. Prevents false corrections when stationary or moving very slowly.</p>
<div class="row"><span class="lbl">Max chassis yaw rate deg/s <small style="color:#64748b">(def 0.8)</small></span>
<input type="number" id="iw5" min="0.1" max="10" step="0.1" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Maximum chassis rotation rate to consider the vehicle driving straight. Lower = stricter straight detection, fewer false corrections.</p>
<div class="row"><span class="lbl">Wheelbase (m) <small style="color:#64748b">(def 3.20, shared)</small></span>
<input type="number" id="iwwb" min="0.5" max="6" step="0.01" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Distance between front and rear axles. Used to compute theoretical steer angle from GPS yaw rate and speed (bicycle model). Shared with Keya WAS — changing here changes it there too.</p>
<button class="btn green" onclick="saveImuWas()" style="margin-top:8px">Save IMU WAS params</button>
</div>

</div><!-- /config -->

<!-- LIVE TAB -->
<div id="live" class="panel">
<style>
.lvtoggle{display:flex;gap:6px;margin-bottom:12px}
.subtab{padding:6px 16px;background:#1e293b;border:1px solid #334155;color:#94a3b8;cursor:pointer;border-radius:4px;font-family:monospace;font-size:14px}
.subtab.on{border-color:#38bdf8;color:#38bdf8;background:#0c1a2e}
.grpbtns{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:12px}
.gbtn{padding:6px 14px;background:#1e293b;border:1px solid #334155;color:#94a3b8;cursor:pointer;border-radius:4px;font-family:monospace;font-size:14px}
.gbtn.on{border-color:#4ade80;color:#4ade80;background:#052e16}
.lvgrp{color:#38bdf8;font-size:13px;font-weight:bold;padding:8px 0 2px;border-bottom:1px solid #334155;margin:6px 0 2px}
</style>
<div class="lvtoggle">
<button class="subtab on" id="lvValuesTab" onclick="lvShow('values')">Values</button>
<button class="subtab" id="lvGraphTab" onclick="lvShow('graph')">Graph</button>
</div>

<!-- VALUES sub-panel -->
<div id="lvValues">
<div class="grpbtns">
<button class="gbtn on" onclick="setGroup(0,this)">Live Off</button>
<button class="gbtn" onclick="setGroup(1,this)">Gr1 GPS</button>
<button class="gbtn" onclick="setGroup(2,this)">Gr2 IMU</button>
<button class="gbtn" onclick="setGroup(3,this)">Gr3 WAS</button>
<button class="gbtn" onclick="setGroup(4,this)">Gr4 Keya</button>
<button class="gbtn" onclick="setGroup(5,this)">Gr5 Steer</button>
<button class="gbtn" onclick="setGroup(6,this)">Gr6 CAN Steer</button>
</div>
<div class="card" id="lvCard" style="display:none">
<h2 id="lvHdr">Group</h2>
<div id="lvBody"></div>
</div>
<p id="lvOffMsg" style="color:#64748b;font-size:13px">Live Off — select a group above to start streaming values @ 0.5s.</p>
</div>

<!-- GRAPH sub-panel -->
<div id="lvGraph" style="display:none">
<div class="card">
<h2>Online Graph</h2>
<div id="gRows" style="font-size:13px"></div>
<div style="display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin-top:8px;padding-top:8px;border-top:1px solid #334155">
<span class="lbl">Time</span><input type="number" id="gTime" value="30" min="5" max="300" step="5" class="ninput" style="width:60px"> s
<span class="lbl">Freq</span>
<select id="gFreq" style="min-width:80px">
<option value="1">1 Hz</option>
<option value="2">2 Hz</option>
<option value="5" selected>5 Hz</option>
<option value="10">10 Hz</option>
</select>
<button class="btn green" onclick="gSet()">Set</button>
<label class="chk-row" style="padding:0"><input type="checkbox" id="gEnable" onchange="gToggle(this.checked)"> Enable logging</label>
<button class="btn" id="gPauseBtn" onclick="gPauseToggle()">Pause</button>
<button class="btn" onclick="gExportCsv()">⬇ Export CSV</button>
<button class="btn sm" onclick="gResetView()">Reset view</button>
</div>
<canvas id="gcanvas" width="720" height="340" style="width:100%;margin-top:10px;background:#050d1a;border:1px solid #1e3a5f;border-radius:3px;cursor:crosshair"></canvas>
<div id="gReadout" style="font-size:12px;color:#94a3b8;margin-top:6px;font-family:monospace">Paused: drag = pan, wheel = zoom, move = read</div>
</div>
</div>
</div><!-- /live -->

<!-- KEYA TAB -->
<div id="keya" class="panel">
<div class="lvtoggle">
<button class="subtab on" id="kyTuningTab" onclick="kyShow('tuning')">Tuning</button>
<button class="subtab" id="kyMotorTab" onclick="kyShow('motor')">Motor Config</button>
</div>

<div id="keyaTuning">
<div class="card">
<h2>Keya status</h2>
<div class="row"><span class="lbl">Detected</span><span id="k_det" class="badge fail">--</span></div>
<div class="row"><span class="lbl">WAS initial zero</span><span id="k_zero" class="badge fail">--</span></div>
<div class="row"><span class="lbl">Encoder (ticks)</span><span class="val" id="k_enc">-</span></div>
<div class="row"><span class="lbl">GPS drift offset (°)</span><span class="val" id="k_off">-</span></div>
<div class="row"><span class="lbl">Actual motor speed</span><span class="val" id="k_act">-</span></div>
<div class="row"><span class="lbl">Set motor speed</span><span class="val" id="k_set">-</span></div>
</div>

<div class="card">
<h2>Keya WAS — steering geometry</h2>
<p style="color:#64748b;font-size:13px;margin-bottom:8px">Compensates hydraulic backlash on direction reversal and unequal left/right steering. Tune manually here, or use auto-calibration (coming next) with a wheel-mounted reference IMU.</p>
<div class="row"><span class="lbl">Dead zone <small style="color:#64748b">(°, def 0)</small></span>
<input type="number" id="ksgDz" min="0" max="180" step="0.01" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Backlash on direction reversal — the WAS reading is frozen for this many degrees of free play, then resumes. On good tractors this is near zero. 0 = off. (Applied in WAS output degrees.)</p>
<div class="row"><span class="lbl">Ticks/deg left <small style="color:#64748b">(0 = same as base)</small></span>
<input type="number" id="ksgL" min="0" max="500" step="0.1" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Encoder ticks per degree when steering left. Leave 0 to use the base "Ticks per degree". Set only if left turns read differently than right (asymmetric cylinder). Measure: turn left a known angle, count tick change, divide. <b>Set Invert encoder first</b> — left/right follow the inverted direction.</p>
<div class="row"><span class="lbl">Ticks/deg right <small style="color:#64748b">(0 = same as base)</small></span>
<input type="number" id="ksgR" min="0" max="500" step="0.1" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Encoder ticks per degree when steering right. Leave 0 to use the base value.</p>
<button class="btn green" onclick="saveKeyaGeom()" style="margin-top:8px">Save geometry</button>
</div>

<div class="card">
<h2>Keya WAS — auto-calibration <span style="color:#64748b;font-weight:normal;font-size:11px">(reference IMU)</span></h2>
<p style="color:#64748b;font-size:13px;margin-bottom:8px">Uses a wheel-mounted reference IMU (TM171 on ESP32, via the laptop bridge app) to auto-measure dead zone and per-side ticks/deg. The Teensy turns the steering motor itself — vehicle must be stationary, stands clear.</p>
<div class="row"><span class="lbl">Reference IMU link</span><span id="calRefBadge" class="badge fail">--</span></div>
<div class="row"><span class="lbl">Reference angle</span><span class="val" id="calRefAngle">—</span></div>
<p style="color:#f59e0b;font-size:12px;margin:6px 0 8px;line-height:1.3">&#9888; The Teensy turns the steering motor by itself. Vehicle stationary, stand clear, hand ready on the wheel. Aborts on steer switch / motion / lost reference.</p>
<div class="row"><span class="lbl">Motor speed <small style="color:#64748b">(slow, def 25)</small></span>
<input type="number" id="calSpeed" min="5" max="80" step="1" value="25" class="ninput"></div>
<div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:8px">
<button class="btn green" onclick="calStartBtn()">Start calibration</button>
<button class="btn red" onclick="calAbortBtn()">Abort</button>
</div>
<div class="row" style="margin-top:8px"><span class="lbl">State</span><span class="val" id="calState">idle</span></div>
<div class="row"><span class="lbl">Measured dead zone</span><span class="val" id="calDz">—</span></div>
<div class="row"><span class="lbl">Measured ticks/deg L | R</span><span class="val" id="calLR">—</span></div>
<div class="row"><span class="lbl">Measured base ticks/deg</span><span class="val" id="calTpd">—</span></div>
<button class="btn" id="calApplyBtn" onclick="calApplyBtn()" style="margin-top:8px;display:none">Apply &amp; save results</button>
</div>

<div class="card">
<h2>Keya encoder as WAS — parameters</h2>
<p style="color:#64748b;font-size:13px;margin-bottom:8px">Activate by setting WAS source to "Keya encoder" above. Autosteer is locked until the first auto-zero completes (drive straight at &gt;2.5 km/h). ADS1115 settings in AgIO have no effect.</p>
<div class="row"><span class="lbl">Ticks per degree <small style="color:#64748b">(def 24)</small></span>
<input type="number" id="kw1" min="1" max="500" step="0.1" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Mechanical ratio: encoder ticks per degree of wheel steering. Turn wheel a known angle and count ticks to calibrate.</p>
<div class="row"><span class="lbl">Invert encoder <small style="color:#64748b">(def OFF)</small></span>
<input type="checkbox" id="kw2" style="width:15px;height:15px;accent-color:#38bdf8;cursor:pointer"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Flips the Keya encoder direction. Enable if turning right shows a negative angle. <b>Note:</b> AgOpenGPS "Invert WAS" has NO effect on Keya — use this instead. Acts on the raw sensor (before left/right ticks), so set this FIRST; if you change it after setting per-side ticks, swap Ticks/deg left and right.</p>
<div class="row"><span class="lbl">EMA filter alpha <small style="color:#64748b">(def 0.0 = off, 0.3 = medium)</small></span>
<input type="number" id="kwema" min="0" max="0.99" step="0.01" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Exponential smoothing on WAS output. 0 = off, 0.3 = medium, 0.8 = heavy. Higher values reduce noise but add lag.</p>
<div class="row"><span class="lbl">GPS drift offset (°)</span><span class="val" id="kw_off">-</span></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Active drift correction applied to encoder angle. Auto-zero adjusts this continuously. Should converge toward 0 during straight driving.</p>
<div style="border-top:1px solid #334155;margin:10px 0 8px"></div>
<div style="color:#38bdf8;font-size:12px;text-transform:uppercase;letter-spacing:1px;margin-bottom:8px">Auto-zero — GPS drift correction</div>
<div class="row"><span class="lbl">Enable <small style="color:#64748b">(def ON)</small></span>
<input type="checkbox" id="kw4" style="width:15px;height:15px;accent-color:#38bdf8;cursor:pointer"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Corrects encoder drift by comparing measured angle to GPS-computed wheel angle while driving straight. First correction unlocks autosteer on boot.</p>
<div class="row"><span class="lbl">Wheelbase (m) <small style="color:#64748b">(def 3.20, shared)</small></span>
<input type="number" id="kwwb" min="0.5" max="6" step="0.01" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Distance between front and rear axles. Used to compute the theoretical steer angle from GPS yaw rate and speed (bicycle model). The GPS-derived wheel angle is the reference the auto-zero corrects the WAS toward — set this to your tractor's real wheelbase. Shared with IMU as WAS.</p>
<div class="row"><span class="lbl">Beta / correction fraction <small style="color:#64748b">(def 0.05)</small></span>
<input type="number" id="kw5" min="0.001" max="1" step="0.001" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Soft correction speed — 5% of error per cycle. 0.05 = slow and stable. When autosteer is active, correction is automatically reduced 5x to avoid fighting the PID.</p>
<div class="row"><span class="lbl">Min speed km/h <small style="color:#64748b">(def 2.5)</small></span>
<input type="number" id="kw6" min="0" max="25" step="0.5" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Below this speed auto-zero is completely blocked. Prevents false corrections when stationary.</p>
<div class="row"><span class="lbl">Max yaw rate deg/s <small style="color:#64748b">(def 0.3)</small></span>
<input type="number" id="kw7" min="0.05" max="5" step="0.05" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Yaw rate threshold for "driving straight". Applies to GPS course rate, and to the chassis IMU rate if enabled below. Lower = stricter.</p>
<div class="chk-row" style="padding:2px 0">
<input type="checkbox" id="kwImu"><label for="kwImu" style="cursor:pointer">Also require chassis IMU yaw rate stable</label></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Adds the onboard IMU gyro as a second "straight" check (in addition to GPS course). Helps at low speed where GPS course is noisy. Requires an active IMU.</p>
<div class="row"><span class="lbl">Speed slow threshold km/h <small style="color:#64748b">(def 3)</small></span>
<input type="number" id="kw8" min="0" max="25" step="1" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Below this speed the slow straight-time applies. Longer wait at low speed reduces false corrections on headland turns.</p>
<div class="row"><span class="lbl">Speed fast threshold km/h <small style="color:#64748b">(def 12)</small></span>
<input type="number" id="kw9" min="0" max="30" step="1" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Above this speed the fast straight-time applies. Between slow and fast thresholds the time is linearly interpolated.</p>
<div class="row"><span class="lbl">Straight time at slow speed ms <small style="color:#64748b">(def 500)</small></span>
<input type="number" id="kw10" min="100" max="5000" step="50" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Time vehicle must drive straight below the slow threshold before a correction is applied. Longer = safer, corrections rarer.</p>
<div class="row"><span class="lbl">Straight time at fast speed ms <small style="color:#64748b">(def 200)</small></span>
<input type="number" id="kw11" min="50" max="2000" step="50" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Time vehicle must drive straight above the fast threshold. At speed the tractor naturally drives straighter so shorter time is sufficient.</p>
<button class="btn green" onclick="saveKeyaWas()" style="margin-top:8px">Save WAS params</button>
</div>

<div class="card">
<h2>Keya speed-direction disengage</h2>
<p style="color:#f59e0b;font-size:13px;margin-bottom:8px">&#9888; Current Sensor must be enabled in AOG</p>
<div class="row"><span class="lbl">Enable <small style="color:#64748b">(def OFF)</small></span>
<input type="checkbox" id="kd0" style="width:15px;height:15px;accent-color:#38bdf8;cursor:pointer"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Cuts autosteer if the motor runs in the opposite direction to the command — indicates hand override on the wheel.</p>
<div class="row"><span class="lbl">Set speed min threshold <small style="color:#64748b">(def 10)</small></span>
<input type="number" id="kd1" min="0" max="200" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Minimum commanded motor speed (absolute value) to activate detection. Ignores very small commands near centre.</p>
<div class="row"><span class="lbl">Act speed min threshold <small style="color:#64748b">(def 5)</small></span>
<input type="number" id="kd2" min="0" max="200" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">Minimum actual motor speed (absolute value) to activate detection. Ignores motor noise when nearly stationary.</p>
<div class="row"><span class="lbl">SpeedDiff timeout (ms) <small style="color:#64748b">(def 250)</small></span>
<input type="number" id="kd3" min="0" max="5000" step="10" class="ninput"></div>
<p style="color:#94a3b8;font-size:12px;margin:-2px 0 5px;line-height:1.3">How long set and actual motor speed must disagree in direction before autosteer is cut. Shorter = more sensitive.</p>
<button class="btn green" onclick="saveKeya()" style="margin-top:8px">Save Keya params</button>
</div>
</div><!-- /keyaTuning -->

<div id="keyaMotorCfg" style="display:none">
<div class="card">
<h2>Keya motor parameters <span style="color:#64748b;font-weight:normal;font-size:11px">— writes to the motor over CAN</span></h2>
<p style="color:#64748b;font-size:13px;margin-bottom:8px">Reads/writes parameters in the Keya drive. Requires a CAN port set to "Keya motor". Click <b>Read</b> first, edit a value, <b>Write</b> (to RAM), then <b>Store EEPROM</b> to make it permanent.</p>
<div style="display:flex;gap:8px;flex-wrap:wrap;margin-bottom:8px">
<button class="btn" onclick="kcCmd('enter')">Read params</button>
<button class="btn green" onclick="kcCmd('store')">Store EEPROM</button>
<button class="btn" onclick="kcCmd('exit')">Exit config</button>
<span id="kcMode" class="badge fail" style="align-self:center">OUT</span>
</div>
<table style="width:100%;border-collapse:collapse;font-size:13px">
<tr style="color:#64748b;font-size:11px"><th style="text-align:left;padding:3px 6px">Parameter</th><th style="padding:3px 6px">RAM</th><th style="padding:3px 6px">ROM</th><th style="padding:3px 6px">New</th><th></th></tr>
<tr style="border-top:1px solid #1e293b"><td style="padding:4px 6px">Max Current (A)</td><td style="text-align:center" id="kcRam0">-</td><td style="text-align:center" id="kcRom0">-</td><td style="text-align:center"><input type="number" id="kcNew0" min="1" max="60" class="ninput" style="width:60px"></td><td><button class="btn sm" onclick="kcWrite(3,'kcNew0')">Write</button></td></tr>
<tr style="border-top:1px solid #1e293b"><td style="padding:4px 6px">Speed Kp</td><td style="text-align:center" id="kcRam1">-</td><td style="text-align:center" id="kcRom1">-</td><td style="text-align:center"><input type="number" id="kcNew1" min="0" max="255" class="ninput" style="width:60px"></td><td><button class="btn sm" onclick="kcWrite(7,'kcNew1')">Write</button></td></tr>
<tr style="border-top:1px solid #1e293b"><td style="padding:4px 6px">Speed Ki</td><td style="text-align:center" id="kcRam2">-</td><td style="text-align:center" id="kcRom2">-</td><td style="text-align:center"><input type="number" id="kcNew2" min="0" max="255" class="ninput" style="width:60px"></td><td><button class="btn sm" onclick="kcWrite(8,'kcNew2')">Write</button></td></tr>
<tr style="border-top:1px solid #1e293b"><td style="padding:4px 6px">CAN bitrate</td><td style="text-align:center" id="kcRam3">-</td><td style="text-align:center" id="kcRom3">-</td><td style="text-align:center"><select id="kcNew3" style="min-width:0;width:80px"><option value="1">125k</option><option value="2">250k</option><option value="3">500k</option><option value="4">1M</option></select></td><td><button class="btn sm" onclick="kcWriteSel(21,'kcNew3')">Write</button></td></tr>
</table>
<p style="color:#f59e0b;font-size:12px;margin:8px 0 0;line-height:1.3">&#9888; CAN bitrate: after Store + motor reboot the drive talks at the new speed — set the matching baud in Config → CAN port assignment, or you lose comms.</p>
</div>

<div class="card">
<h2>Keya wheel test <span style="color:#64748b;font-weight:normal;font-size:11px">— stationary only</span></h2>
<p style="color:#f59e0b;font-size:12px;margin-bottom:8px;line-height:1.3">&#9888; Turns the wheel. Vehicle stationary, hand ready. <b>Hold</b> a button to turn, release to stop. Speed-controlled motor — the wheel keeps turning while held, so use a low speed.</p>
<div class="row"><span class="lbl">Speed <small style="color:#64748b">(0-250, slow=30-60)</small></span>
<input type="number" id="kcTestSpeed" min="0" max="250" step="10" value="50" class="ninput"></div>
<div style="display:flex;gap:8px;margin-top:8px">
<button class="btn" id="kcLeftBtn">◀ Left (hold)</button>
<button class="btn" id="kcRightBtn">Right ▶ (hold)</button>
<button class="btn red" onclick="kcStop()">Stop</button>
</div>
</div>
</div><!-- /keyaMotorCfg -->

</div><!-- /keya -->

<!-- CAN STEER TAB -->
<div id="cansteer" class="panel">

<div class="card">
<h2>Tractor brand</h2>
<p style="color:#64748b;font-size:13px;margin-bottom:10px">Brand determines CAN IDs and message format for V_Bus and ISO_Bus. CAN port assignment is set in the Config tab. Restart required.</p>
<div class="row"><span class="lbl">Brand</span>
<select id="csBrand">
<option value="0">Claas</option>
<option value="1">Valtra / Massey / McCormick / MF</option>
<option value="2">Case IH / New Holland</option>
<option value="3">Fendt</option>
<option value="4">JCB</option>
<option value="5">FendtOne</option>
<option value="6">Lindner</option>
<option value="7">AgOpenGPS</option>
<option value="8">Cat / Challenger MT (Late)</option>
<option value="9">Cat / Challenger MT (Early)</option>
</select></div>
<button class="btn green" onclick="saveCanBrand()" style="margin-top:8px">Save brand (restart)</button>
</div>

<div class="card">
<h2>Custom CAN engage</h2>
<div class="chk-row" style="margin-bottom:8px">
<input type="checkbox" id="ceEnable"><label for="ceEnable" style="cursor:pointer">Enable custom engage (mask + match on a CAN frame)</label>
</div>
<p style="color:#64748b;font-size:12px;margin-bottom:8px;line-height:1.4">For tractors not covered by a brand preset. Matches an engage button/switch in a CAN frame. Restart required when enabling or changing port/ID.</p>
<div class="lbl" style="margin:6px 0 3px">Listen on CAN</div>
<select id="cePort" style="width:62px;min-width:0">
<option value="1">CAN1</option><option value="2">CAN2</option><option value="3">CAN3</option>
</select>
<select id="ceExt" style="width:84px;min-width:0;margin-left:6px">
<option value="1">29-bit</option><option value="0">11-bit</option>
</select>
<div class="row" style="margin-top:6px"><span class="lbl">CAN ID (hex)</span>
<input type="text" id="ceId" placeholder="18EF1C32" class="ninput" style="width:110px;text-align:left"></div>
<div class="row"><span class="lbl">Engage mode</span>
<select id="ceMode" style="flex:1;max-width:220px">
<option value="0">Toggle — momentary button</option>
<option value="1">Level — latched switch</option>
</select></div>
<div style="border-top:1px solid #334155;margin:10px 0 6px"></div>
<div style="color:#38bdf8;font-size:12px;text-transform:uppercase;letter-spacing:1px;margin-bottom:6px">Learn (auto-fill)</div>
<p style="color:#94a3b8;font-size:12px;margin:0 0 6px;line-height:1.3">Set CAN ID and Save first. Then: capture with button released, then capture while holding engage. Changed bits become the mask automatically. You can edit below afterwards.</p>
<button class="btn" onclick="ceCaptureIdle()">1. Capture IDLE</button>
<button class="btn" onclick="ceCapturePressed()" style="margin-left:6px">2. Capture PRESSED</button>
<span id="ceLearnMsg" style="margin-left:8px;font-size:12px;color:#64748b"></span>
<div style="font-size:12px;color:#64748b;margin-top:6px">IDLE: <span id="ceIdleHex" style="color:#94a3b8;font-family:monospace">—</span> &nbsp; PRESSED: <span id="cePressHex" style="color:#94a3b8;font-family:monospace">—</span></div>
<div style="border-top:1px solid #334155;margin:10px 0 6px"></div>
<div style="color:#38bdf8;font-size:12px;text-transform:uppercase;letter-spacing:1px;margin-bottom:6px">Match pattern (hex per byte)</div>
<div style="display:flex;gap:3px;align-items:center;font-size:11px;color:#64748b;margin-bottom:2px"><span style="width:42px">Byte</span>
<span style="width:34px;text-align:center">0</span><span style="width:34px;text-align:center">1</span><span style="width:34px;text-align:center">2</span><span style="width:34px;text-align:center">3</span><span style="width:34px;text-align:center">4</span><span style="width:34px;text-align:center">5</span><span style="width:34px;text-align:center">6</span><span style="width:34px;text-align:center">7</span></div>
<div style="display:flex;gap:3px;align-items:center;margin-bottom:3px"><span style="width:42px;font-size:12px;color:#cbd5e1">Value</span><span id="ceValRow"></span></div>
<div style="display:flex;gap:3px;align-items:center"><span style="width:42px;font-size:12px;color:#cbd5e1">Mask</span><span id="ceMskRow"></span></div>
<p style="color:#94a3b8;font-size:11px;margin:4px 0 0;line-height:1.3">Mask FF = whole byte, 01 = only bit 0, 00 = ignore byte. Engage triggers when (frame &amp; mask) == (value &amp; mask) for every byte.</p>
<button class="btn green" onclick="ceSave()" style="margin-top:10px">Save Custom Engage (restart)</button>
<div style="border-top:1px solid #334155;margin:10px 0 6px"></div>
<div style="color:#38bdf8;font-size:12px;text-transform:uppercase;letter-spacing:1px;margin-bottom:6px">Live — AOG steer state</div>
<div class="row"><span class="lbl">Frame on ID</span><span class="val" id="ceFrameHex" style="font-family:monospace;font-size:12px">—</span></div>
<div class="row"><span class="lbl">Pattern match now</span><span id="ceMatchBadge" class="badge fail">--</span></div>
<div class="row"><span class="lbl">Engage toggle (steerSwitch)</span><span id="ceEngBadge" class="badge fail">OFF</span></div>
<div class="row"><span class="lbl">AOG guidance command</span><span id="ceGuidBadge" class="badge fail">OFF</span></div>
<div class="row"><span class="lbl">Autosteer active</span><span id="ceAutoBadge" class="badge fail">OFF</span></div>
</div>

<div class="card">
<h2>Live status</h2>
<div class="row"><span class="lbl">Valve ready state</span><span class="val" id="cs0">—</span></div>
<div class="row"><span class="lbl">estCurve (valve angle)</span><span class="val" id="cs1">—</span></div>
<div class="row"><span class="lbl">setCurve (command)</span><span class="val" id="cs2">—</span></div>
<div class="row"><span class="lbl">Rear hitch</span><span class="val" id="cs3">—</span></div>
<div class="row"><span class="lbl">Steering intend</span><span id="cs4" class="badge fail">OFF</span></div>
</div>

<div class="card">
<h2>CAN Scan <span style="color:#64748b;font-weight:normal;font-size:11px">— detect connected tractor</span></h2>
<p style="color:#64748b;font-size:13px;margin-bottom:8px">Listens on all active CAN ports for 5 s and identifies known tractor message IDs. CAN ports must be configured and connected.</p>
<button class="btn" id="scanBtn" onclick="startCanScan()">Start 5s scan</button>
<span id="scanTimer" style="margin-left:12px;font-size:13px;color:#64748b"></span>
<div id="scanResults" style="margin-top:10px"></div>
</div>

<div class="card">
<h2>CAN plot</h2>
<p style="color:#64748b;font-size:13px;margin-bottom:8px">Captures raw CAN frames on V_Bus (valve), ISO_Bus (engage/hitch) and K_Bus (Fendt). Toggle logging, then scroll below.</p>
<button class="btn" id="canPlotBtn" onclick="toggleCanPlot()">Enable CAN log</button>
<button class="btn" onclick="clearCanRaw()" style="margin-left:8px">Clear</button>
<pre id="canraw" style="background:#0f172a;color:#86efac;font-size:12px;height:180px;overflow-y:auto;padding:6px;margin-top:8px;border:1px solid #334155;white-space:pre-wrap;word-break:break-all">(CAN log empty — enable logging above)</pre>
</div>

<div class="card" id="pvedCard">
<h2>PVED tool <span style="color:#64748b;font-weight:normal;font-size:11px">(Danfoss PVED-CL/CLS — Claas / Valtra / CaseIH / JCB / Lindner)</span></h2>
<p style="color:#64748b;font-size:13px;margin-bottom:8px">Reads/writes Danfoss PVED valve parameters via ISO_Bus. Responses appear in CAN plot above (enable logging first).</p>
<button class="btn" onclick="pvedCmd('readall')">Read all params</button>
<button class="btn" onclick="pvedCmd('read64007')" style="margin-left:8px">Read param 64007</button>
<button class="btn" id="pvedWriteBtn" onclick="pvedCmd('write')" style="display:none;margin-left:8px">Write param 64007 (set controller addr)</button>
<button class="btn" onclick="pvedCmd('commit')" style="margin-left:8px">Save changes</button>
<div id="pvedInfo" style="display:none;margin-top:10px;padding:8px 12px;background:#1e293b;border:1px solid #334155;border-radius:6px;font-size:13px;color:#94a3b8;line-height:1.7"></div>
<button class="btn" id="pvedRestoreBtn" onclick="pvedCmd('restore')" style="display:none;margin-top:8px;background:#7f1d1d;border-color:#991b1b">Restore factory value</button>
</div>


<div class="card">
<h2>J1939 / NMEA 2000 GPS broadcast</h2>
<p style="color:#64748b;font-size:13px;margin-bottom:8px">Set a CAN port to "J1939/NMEA broadcast" in the Config tab to enable. Broadcasts GPS position received from AgIO onto that port.</p>
<div class="row"><span class="lbl">Source address <small style="color:#64748b">(hex, def 0x1E=30)</small></span>
<input type="number" id="j19Addr" min="0" max="254" class="ninput"></div>
<div style="border-top:1px solid #334155;margin:10px 0 8px"></div>
<div style="color:#38bdf8;font-size:12px;text-transform:uppercase;letter-spacing:1px;margin-bottom:8px">PGN 65267 / 65256 — position + direction (J1939)</div>
<div class="row"><span class="lbl">Enable <small style="color:#64748b">(def ON)</small></span>
<input type="checkbox" id="j19En65267" style="width:15px;height:15px;accent-color:#38bdf8;cursor:pointer"></div>
<div class="row"><span class="lbl">Send interval ms <small style="color:#64748b">(def 200 = 5 Hz)</small></span>
<input type="number" id="j19R65267" min="50" max="2000" step="50" class="ninput"></div>
<div style="border-top:1px solid #334155;margin:10px 0 8px"></div>
<div style="color:#38bdf8;font-size:12px;text-transform:uppercase;letter-spacing:1px;margin-bottom:8px">PGN 129029 — NMEA 2000 fast-packet (7 CAN frames)</div>
<div class="row"><span class="lbl">Enable <small style="color:#64748b">(def OFF)</small></span>
<input type="checkbox" id="j19En129029" style="width:15px;height:15px;accent-color:#38bdf8;cursor:pointer"></div>
<div class="row"><span class="lbl">Send interval ms <small style="color:#64748b">(def 1000 = 1 Hz)</small></span>
<input type="number" id="j19R129029" min="100" max="5000" step="100" class="ninput"></div>
<div style="border-top:1px solid #334155;margin:10px 0 8px"></div>
<div class="row"><span class="lbl">Last GPS: fix</span><span class="val" id="j19Fix">—</span></div>
<div class="row"><span class="lbl">Lat / Lon</span><span class="val" id="j19LatLon">—</span></div>
<button class="btn green" onclick="saveJ1939()" style="margin-top:8px">Save J1939 params</button>
</div>

</div><!-- /cansteer -->

<!-- DEBUG TAB -->
<div id="debug" class="panel">
<div class="card">
<h2>Debug flags</h2>
<p style="color:#64748b;font-size:13px;margin-bottom:10px">Prints to USB Serial and the log below. No restart needed.</p>
<div class="chk-row"><input type="checkbox" id="dbg0"><label for="dbg0">GPS</label></div>
<div class="chk-row"><input type="checkbox" id="dbg4"><label for="dbg4">CAN bus (Keya)</label></div>
<div class="chk-row"><input type="checkbox" id="dbg7"><label for="dbg7">Disengage trigger (logs what cut autosteer)</label></div>
<button class="btn green" onclick="saveDbg()">&#10003; Apply debug flags</button>
</div>
<div class="card">
<h2>Serial log <button class="btn sm" onclick="clearLog()">&#128465; Clear</button></h2>
<div class="chk-row" style="margin-bottom:8px">
<input type="checkbox" id="logActive" onchange="setLogActive(this.checked)">
<label for="logActive" style="cursor:pointer">Active — enable log capture (disable to save CPU when not debugging)</label>
</div>
<pre id="log">(log capture disabled — enable above to start)</pre>
</div>
</div><!-- /debug -->

<!-- UM98x CONFIG TAB -->
<div id="um98x" class="panel">
<div class="card">
<h2>Send GPS commands</h2>
<p style="color:#64748b;font-size:13px;margin-bottom:8px">Paste config lines (one per line). Upload sends each with 1 s delay. Responses appear in the raw log below.</p>
<textarea class="gps-ta" id="gpsLines" placeholder="e.g. CONFIG HEADING FIXINTERVAL 0.1"></textarea>
<div style="display:flex;gap:8px;margin-top:8px;flex-wrap:wrap;align-items:center">
<button class="btn green" onclick="uploadLines()">&#8679; Upload</button>
<button class="btn" onclick="gpsCmd('CONFIG')">&#128196; Get Config</button>
<button class="btn" onclick="gpsCmd('VERSION')">&#128196; Get Version</button>
<button class="btn" onclick="gpsCmd('SAVECONFIG')">&#128190; Save Config</button>
<button class="btn red" onclick="doFactoryReset()">&#9888; Factory Reset</button>
<span id="gpsStatus" style="color:#94a3b8;font-size:11px;margin-left:4px"></span>
</div>
</div>
<div class="card">
<h2>GPS raw serial <button class="btn sm" onclick="clearGpsRaw()">&#128465; Clear</button></h2>
<div class="chk-row" style="margin-bottom:8px">
<input type="checkbox" id="gpsActive" onchange="setGpsActive(this.checked)">
<label for="gpsActive" style="cursor:pointer">Active — enable GPS raw capture</label>
</div>
<pre id="gpsraw">(GPS raw capture disabled — enable above to start)</pre>
</div>
</div><!-- /um98x -->

<div id="sb">Connecting to Teensy...</div>

<script>
var loaded = false;
var configLoaded = false;
var activeTab = 'config';
var logFetching = false;
var tickTimer = null;
var activeGroup = 0;       // Live tab: 0=Off, 1=GPS, 2=IMU, 3=WAS, 4=Keya, 5=Steer, 6=CAN
var lvMode = 'values';     // Live tab sub-panel: 'values' | 'graph'

function lvShow(m) {
  lvMode = m;
  document.getElementById('lvValues').style.display = (m === 'values') ? '' : 'none';
  document.getElementById('lvGraph').style.display  = (m === 'graph')  ? '' : 'none';
  document.getElementById('lvValuesTab').classList.toggle('on', m === 'values');
  document.getElementById('lvGraphTab').classList.toggle('on', m === 'graph');
  if (m === 'graph' && typeof gDraw === 'function') gDraw();
}

function setGroup(n, btn) {
  activeGroup = n;
  document.querySelectorAll('.gbtn').forEach(function(e){ e.classList.remove('on'); });
  btn.classList.add('on');
  document.getElementById('lvCard').style.display   = (n > 0) ? '' : 'none';
  document.getElementById('lvOffMsg').style.display = (n > 0) ? 'none' : '';
  if (n === 4) fetch('/api/keyaposzero');   // zero relative position when entering Keya group
  if (n > 0) tick();   // immediate fetch
}

var lvQ = {0:'invalid',1:'GPS',2:'DGPS',4:'RTK',5:'Float RTK'};
function lvRow(lbl, val){ return '<div class="row"><span class="lbl">'+lbl+'</span><span class="val">'+val+'</span></div>'; }
function lvSub(t){ return '<div class="lvgrp">'+t+'</div>'; }

function renderGroup(d) {
  var h = '', hdr = '';
  if (activeGroup === 1) {
    hdr = 'Group 1 — GPS';
    h += lvSub('GGA');
    if (d.haveGGA) {
      h += lvRow('Msg interval', d.ggaMs + ' ms');
      h += lvRow('Fix time', d.fixT || '--');
      h += lvRow('Latitude', d.lat + ' ' + d.ns);
      h += lvRow('Longitude', d.lon + ' ' + d.ew);
      h += lvRow('Fix quality', (lvQ[d.fixQ] || d.fixQ) + ' (' + d.fixQ + ')');
      h += lvRow('Satellites', d.sats);
      h += lvRow('HDOP', d.hdop);
      h += lvRow('Altitude', d.alt + ' m');
      h += lvRow('DGPS age', d.age + ' s');
    } else { h += lvRow('GGA', '--'); }
    h += lvSub('VTG');
    if (d.haveVTG) {
      h += lvRow('Msg interval', d.vtgMs + ' ms');
      h += lvRow('Heading', d.vtgHdg.toFixed(1) + ' °');
      h += lvRow('Speed', d.spd.toFixed(1) + ' km/h');
    } else { h += lvRow('VTG', '--'); }
    h += lvSub('HPR');
    if (d.haveHPR) {
      var q = parseInt(d.hprQ);
      h += lvRow('Msg interval', d.hprMs + ' ms');
      h += lvRow('Heading', d.hprHdg + ' °');
      h += lvRow('Roll', d.hprRoll + ' °');
      h += lvRow('RTK quality', (lvQ[q] || d.hprQ) + ' (' + d.hprQ + ')');
    } else {
      h += lvRow('Heading', '--');
      h += lvRow('Roll', '--');
      h += lvRow('RTK quality', '--');
    }
  }
  else if (activeGroup === 2) {
    hdr = 'Group 2 — IMU';
    h += lvSub('BNO085');
    if (d.bno) {
      h += lvRow('Heading', d.bnoHdg.toFixed(1) + ' °');
      h += lvRow('Roll', d.bnoRoll.toFixed(1) + ' °');
      h += lvRow('Pitch', d.bnoPitch.toFixed(1) + ' °');
      h += lvRow('Yaw rate', d.bnoYaw + ' °/s');
    } else { h += lvRow('Heading','--')+lvRow('Roll','--')+lvRow('Pitch','--')+lvRow('Yaw rate','--'); }
    h += lvSub('TM171');
    if (d.tm) {
      h += lvRow('Heading', d.tmHdg + ' °');
      h += lvRow('Roll', d.tmRoll + ' °');
      h += lvRow('Pitch', d.tmPitch + ' °');
      h += lvRow('Yaw rate', d.tmYaw + ' °/s');
    } else { h += lvRow('Heading','--')+lvRow('Roll','--')+lvRow('Pitch','--')+lvRow('Yaw rate','--'); }
  }
  else if (activeGroup === 3) {
    hdr = 'Group 3 — WAS';
    var srcN = {0:'ADS1115',1:'Keya encoder',2:'IMU via CAN',3:'CAN valve'};
    h += lvRow('Active source', srcN[d.src] || d.src);
    h += lvSub('ADS1115');
    h += lvRow('Raw counts', d.adsRaw);
    h += lvSub('IMU as WAS');
    h += lvRow('Raw yaw', d.imuRaw.toFixed(2) + ' °');
    h += lvRow('After scale', d.imuScaled.toFixed(2) + ' °  (×' + d.imuScale.toFixed(2) + ')');
    h += lvSub('GPS reference (auto-zero)');
    h += lvRow('Yaw rate (GPS course)', d.yawRate.toFixed(2) + ' °/s');
    h += lvRow('Wheel angle (GPS)', d.wheelGps.toFixed(2) + ' °');
    h += lvSub('Final');
    h += lvRow('WAS angle actual', d.actual.toFixed(2) + ' °');
  }
  else if (activeGroup === 4) {
    hdr = 'Group 4 — Keya';
    h += lvRow('Detected', d.det ? 'YES' : 'NO');
    h += lvRow('Initial zero', d.zero ? 'DONE' : 'PENDING');
    h += lvRow('Encoder', d.enc + ' ticks');
    h += '<div class="row"><span class="lbl">Rel position <button class="btn sm" onclick="fetch(\'/api/keyaposzero\')">Zero</button></span><span class="val">' + d.relPos.toFixed(2) + ' ° wheel (' + d.relTicks + ' ticks)</span></div>';
    h += lvRow('Steering wheel pos', d.swPos.toFixed(1) + ' ° (1:1 motor)');
    h += lvRow('Zero offset', d.zTicks + ' ticks');
    h += lvRow('GPS drift offset', d.off.toFixed(3) + ' °');
    h += lvRow('Actual speed', d.act);
    h += lvRow('Set speed', d.set);
    h += lvRow('Final angle', d.actual.toFixed(2) + ' °');
  }
  else if (activeGroup === 5) {
    hdr = 'Group 5 — Steer';
    h += lvRow('Steer actual', d.actual.toFixed(2) + ' °');
    h += lvRow('Steer setpoint', d.setpt.toFixed(2) + ' °');
    h += lvRow('Steer error', d.err.toFixed(2) + ' °');
    h += lvRow('PWM', d.pwm);
    h += lvRow('Autosteer', d.on ? 'ACTIVE' : 'OFF');
    h += lvRow('Speed', d.spd.toFixed(1) + ' km/h');
    var sw = function(v){ return v === 0 ? 'PRESSED' : 'open'; };
    h += lvSub('AOG switches');
    h += lvRow('Steer button', sw(d.steerPin));
    h += lvRow('Work switch', sw(d.workPin));
    h += lvRow('Remote', sw(d.remotePin));
    h += lvSub('PCB sensors');
    h += lvRow('Current sensor (A17)', d.curRaw);
    h += lvRow('Pressure sensor (A10)', d.presRaw);
    h += lvRow('Sensor reading (to AOG)', d.sensorRd);
  }
  else if (activeGroup === 6) {
    hdr = 'Group 6 — CAN Steer';
    h += lvRow('Valve ready', d.vReady === 16 ? 'READY (16)' : d.vReady);
    h += lvRow('estCurve', d.eCurve + ' (' + (d.eCurve - 32128) + ')');
    h += lvRow('setCurve', d.sCurve + ' (' + (d.sCurve - 32128) + ')');
    h += lvRow('Rear hitch', Math.round(d.hitch / 2.5) + ' %');
    h += lvRow('Steering intend', d.intend ? 'STEERING' : 'IDLE');
  }
  document.getElementById('lvHdr').textContent = hdr;
  document.getElementById('lvBody').innerHTML = h;
  document.getElementById('sb').textContent = 'Updated: ' + new Date().toLocaleTimeString();
}

function showTab(t, el) {
  activeTab = t;
  document.querySelectorAll('.tab').forEach(function(e) { e.classList.remove('active'); });
  document.querySelectorAll('.panel').forEach(function(e) { e.classList.remove('active'); });
  document.getElementById(t).classList.add('active');
  el.classList.add('active');
  restartTick();
  if (t === 'debug') pollLog();
  if (t === 'um98x') pollGpsRaw();
}

function restartTick() {
  if (tickTimer) clearInterval(tickTimer);
  var rate = (activeTab === 'live') ? 500 : 2000;
  tickTimer = setInterval(tick, rate);
}

function badge(id, ok) {
  var el = document.getElementById(id);
  el.className = 'badge ' + (ok ? 'ok' : 'fail');
  el.textContent = ok ? 'OK' : '--';
}

function yn(v) { return v ? 'YES' : 'NO'; }

function upd(d) {
  badge('k_det', d.detected.keya);
  badge('k_zero', d.keya_was.initialZeroDone);
  document.getElementById('k_enc').textContent = d.keya_was.encoderRaw + ' ticks';
  document.getElementById('k_off').textContent = d.keya_was.gpsOffset.toFixed(3) + ' °';
  document.getElementById('kw_off').textContent = d.keya_was.gpsOffset.toFixed(3) + ' °';
  document.getElementById('k_act').textContent = d.live.keyaActSpeed;
  document.getElementById('k_set').textContent = d.live.keyaSetSpeed;

  badge('d5',       d.detected.gps);
  badge('d_gga',    d.detected.gga);
  badge('d_vtg',    d.detected.vtg);
  badge('d_hpr',    d.detected.hpr);
  badge('d1',       d.detected.bno_i2c);
  badge('d2',       d.detected.tm171);
  badge('d_ads',    d.detected.ads1115);
  badge('d_imuWas', d.detected.imuWas);
  badge('d3',       d.detected.keya);
  document.getElementById('logActive').checked = !!d.detected.logActive;

  if (d.can_steer) {
    document.getElementById('cs0').textContent = d.can_steer.valveReady === 16 ? 'READY (16)' : (d.can_steer.valveReady || 0);
    document.getElementById('cs1').textContent = d.can_steer.estCurve + ' (' + (d.can_steer.estCurve - 32128) + ')';
    document.getElementById('cs2').textContent = d.can_steer.setCurve + ' (' + (d.can_steer.setCurve - 32128) + ')';
    document.getElementById('cs3').textContent = Math.round(d.can_steer.hitch / 2.5) + ' %';
    var ci = document.getElementById('cs4');
    ci.className = 'badge ' + (d.can_steer.intend ? 'ok' : 'fail');
    ci.textContent = d.can_steer.intend ? 'STEERING' : 'IDLE';
    var pvedBrands = [0,1,2,4,6];
    var pc = document.getElementById('pvedCard');
    if (pc) pc.style.display = (pvedBrands.indexOf(d.cfg.steerBrand) >= 0) ? '' : 'none';
    if (d.pved) {
      var pi = document.getElementById('pvedInfo');
      var rb = document.getElementById('pvedRestoreBtn');
      if (pi) {
        var det = d.pved.detected;
        var last = d.pved.last64007;
        var fac = d.pved.factory64007;
        var toHex = function(v) { return '0x' + v.toString(16).toUpperCase().padStart(2,'0'); };
        var html = 'Valve: <b style="color:' + (det ? '#4ade80' : '#f87171') + '">' + (det ? 'detected' : 'not detected') + '</b>';
        if (last !== 65535) html += '&nbsp;&nbsp;&nbsp;Param 64007 (current): <b style="color:#e2e8f0">' + toHex(last) + '</b>';
        if (fac !== 65535) html += '&nbsp;&nbsp;&nbsp;Factory value (saved): <b style="color:#fbbf24">' + toHex(fac) + '</b>';
        pi.innerHTML = html;
        pi.style.display = (det || fac !== 65535) ? '' : 'none';
        var wb = document.getElementById('pvedWriteBtn');
        if (wb) wb.style.display = (fac !== 65535) ? '' : 'none';
        if (rb) rb.style.display = (fac !== 65535 && fac !== 30) ? '' : 'none';
      }
    }
    var pb = document.getElementById('canPlotBtn');
    if (pb && d.can_steer.showData) pb.textContent = 'Disable CAN log';
  }

  document.getElementById('u0').textContent  = d.udp.kp;
  document.getElementById('u1').textContent  = d.udp.highPWM;
  document.getElementById('u2').textContent  = d.udp.lowPWM;
  document.getElementById('u3').textContent  = d.udp.minPWM;
  document.getElementById('u4').textContent  = d.udp.counts;
  document.getElementById('u5').textContent  = d.udp.wasOffset;
  document.getElementById('u6').textContent  = yn(d.udp.invertWAS);
  document.getElementById('u7').textContent  = yn(d.udp.cytron);
  document.getElementById('u8').textContent  = yn(d.udp.shaftEncoder);
  document.getElementById('u9').textContent  = yn(d.udp.pressureSensor);
  document.getElementById('u10').textContent = yn(d.udp.currentSensor);
  document.getElementById('u11').textContent = yn(d.udp.danfoss);

  if (!loaded) {
    loaded = true;
    var f = d.cfg.debugFlags;
    document.getElementById('dbg0').checked = !!(f & 1);
    document.getElementById('dbg4').checked = !!(f & 16);
    document.getElementById('dbg7').checked = !!(f & 128);
    document.getElementById('kd0').checked = !!d.keya_dis.enable;
    document.getElementById('kd1').value   = d.keya_dis.setSpeedMin;
    document.getElementById('kd2').value   = d.keya_dis.actSpeedMin;
    document.getElementById('md0').checked = !!d.motor_dis.enable;
    document.getElementById('md1').value   = d.motor_dis.angleErrorMin;
    document.getElementById('kd3').value   = d.keya_dis.timeoutMs;
    document.getElementById('md2').value   = d.motor_dis.timeoutMs;
    document.getElementById('kw1').value   = d.keya_was.ticksPerDeg;
    document.getElementById('kw2').checked = !!d.keya_was.encInvert;
    document.getElementById('kwema').value  = d.keya_was.emaAlpha;
    document.getElementById('kw4').checked = !!d.keya_was.azEnable;
    document.getElementById('kw5').value   = d.keya_was.azBeta;
    document.getElementById('kw6').value   = d.keya_was.azSpeedMin;
    document.getElementById('kw7').value   = d.keya_was.azYawMax;
    document.getElementById('kw8').value   = d.keya_was.azSpeedSlow;
    document.getElementById('kw9').value   = d.keya_was.azSpeedFast;
    document.getElementById('kwImu').checked = !!d.keya_was.azUseImu;
    document.getElementById('kwwb').value  = d.keya_was.wheelBase;
    document.getElementById('ksgDz').value = d.keya_was.deadZone;
    document.getElementById('ksgL').value  = d.keya_was.ticksLeft;
    document.getElementById('ksgR').value  = d.keya_was.ticksRight;
    document.getElementById('kw10').value  = d.keya_was.azTimeSlowMs;
    document.getElementById('kw11').value  = d.keya_was.azTimeFastMs;
    document.getElementById('can1Mode').value = d.cfg.can1Mode || 0;
    document.getElementById('can2Mode').value = d.cfg.can2Mode || 0;
    document.getElementById('can3Mode').value = d.cfg.can3Mode || 0;
    document.getElementById('can1Baud').value = d.cfg.can1Baud || 250000;
    document.getElementById('can2Baud').value = d.cfg.can2Baud || 250000;
    document.getElementById('can3Baud').value = d.cfg.can3Baud || 250000;
    document.getElementById('wasSource').value     = d.cfg.wasSource     || 0;
    document.getElementById('rollSource').value    = d.cfg.rollSource    || 0;
    document.getElementById('headingSource').value = d.cfg.headingSource || 0;
    document.getElementById('nmeaType').value      = d.cfg.nmeaType      || 0;
    document.getElementById('gpsSerial').value     = d.cfg.gpsSerial     || 7;
    document.getElementById('gpsBaud2').value      = d.cfg.gpsBaud       || 115200;
    document.getElementById('tm171Serial').value   = d.cfg.tm171Serial   || 2;
    document.getElementById('tm171Baud').value     = d.cfg.tm171Baud     || 115200;
    if (document.getElementById('csBrand')) document.getElementById('csBrand').value = d.cfg.steerBrand || 0;
    if (d.custEng) {
      document.getElementById('ceEnable').checked = !!d.custEng.enable;
      document.getElementById('cePort').value = d.custEng.port || 1;
      document.getElementById('ceExt').value  = d.custEng.ext;
      document.getElementById('ceMode').value = d.custEng.mode;
      document.getElementById('ceId').value   = (d.custEng.id || 0).toString(16).toUpperCase();
      for (var ci = 0; ci < 8; ci++) {
        document.getElementById('ceVal'+ci).value = ceHex(d.custEng.match[ci]);
        document.getElementById('ceMsk'+ci).value = ceHex(d.custEng.mask[ci]);
      }
    }
    if (d.j1939) {
      document.getElementById('j19Addr').value      = d.j1939.srcAddr;
      document.getElementById('j19En65267').checked  = !!d.j1939.en65267;
      document.getElementById('j19R65267').value    = d.j1939.rate65267;
      document.getElementById('j19En129029').checked = !!d.j1939.en129029;
      document.getElementById('j19R129029').value   = d.j1939.rate129029;
      var fixNames = ['—','GPS','DGPS','PPS','RTK','Float RTK','Est','Manual','Sim'];
      document.getElementById('j19Fix').textContent = fixNames[d.j1939.fixType] || d.j1939.fixType;
      if (d.j1939.lat !== 0 || d.j1939.lon !== 0)
        document.getElementById('j19LatLon').textContent = d.j1939.lat.toFixed(7) + ' / ' + d.j1939.lon.toFixed(7);
    }
    document.getElementById('iw0').checked = !!d.imu_was.invert;
    document.getElementById('iw1').value   = d.imu_was.cpdScale;
    document.getElementById('iw2').checked = !!d.imu_was.azEnable;
    document.getElementById('iw3').value   = d.imu_was.azBeta;
    document.getElementById('iw4').value   = d.imu_was.azSpeedMin;
    document.getElementById('iw5').value   = d.imu_was.azYawMax;
    document.getElementById('iwwb').value  = d.imu_was.wheelBase;
  }

  document.getElementById('sb').textContent = 'Updated: ' + new Date().toLocaleTimeString();
}

function saveMotor() {
  var url = '/api/save?motorDis=' + (document.getElementById('md0').checked ? 1 : 0)
          + '&motorAngleMin=' + document.getElementById('md1').value
          + '&speedDiffTimeout=' + document.getElementById('md2').value;
  fetch(url).then(function(r) {
    document.getElementById('sb').textContent = r.ok ? 'Motor params saved.' : 'ERROR saving.';
  });
}

function saveKeyaGeom() {
  var url = '/api/save?keyaDeadZone=' + document.getElementById('ksgDz').value
          + '&keyaTicksLeft='  + document.getElementById('ksgL').value
          + '&keyaTicksRight=' + document.getElementById('ksgR').value;
  fetch(url).then(function(r) {
    document.getElementById('sb').textContent = r.ok ? 'Steering geometry saved.' : 'ERROR saving.';
  });
}

function kyShow(m) {
  document.getElementById('keyaTuning').style.display   = (m === 'tuning') ? '' : 'none';
  document.getElementById('keyaMotorCfg').style.display = (m === 'motor')  ? '' : 'none';
  document.getElementById('kyTuningTab').classList.toggle('on', m === 'tuning');
  document.getElementById('kyMotorTab').classList.toggle('on', m === 'motor');
}
function kcCmd(c) { fetch('/api/keyacfg?cmd=' + c); }
function kcWrite(id, fld) {
  var v = document.getElementById(fld).value;
  if (v === '') return;
  fetch('/api/keyacfg?cmd=write&id=' + id + '&val=' + v);
}
function kcWriteSel(id, fld) {
  fetch('/api/keyacfg?cmd=write&id=' + id + '&val=' + document.getElementById(fld).value);
}
var kcHoldTimer = null;
function kcHoldStart(dir) {
  kcHoldStop();
  var send = function(){ fetch('/api/keyacfg?cmd=test&dir=' + dir + '&speed=' + document.getElementById('kcTestSpeed').value); };
  send();
  kcHoldTimer = setInterval(send, 100);   // refresh so the Keya keeps moving while held
}
function kcHoldStop() {
  if (kcHoldTimer) { clearInterval(kcHoldTimer); kcHoldTimer = null; }
}
function kcStop() { kcHoldStop(); fetch('/api/keyacfg?cmd=disable'); }
function kcBindHold() {
  var l = document.getElementById('kcLeftBtn'), r = document.getElementById('kcRightBtn');
  if (!l || !r) return;
  [['L', l], ['R', r]].forEach(function(p) {
    var dir = p[0], btn = p[1];
    btn.addEventListener('mousedown',  function(e){ e.preventDefault(); kcHoldStart(dir); });
    btn.addEventListener('touchstart', function(e){ e.preventDefault(); kcHoldStart(dir); }, {passive:false});
    btn.addEventListener('mouseup',    function(){ kcStop(); });
    btn.addEventListener('mouseleave', function(){ kcStop(); });
    btn.addEventListener('touchend',   function(){ kcStop(); });
  });
}

function calStartBtn() {
  if (!confirm('The steering motor will turn by itself. Vehicle stationary, stand clear, hand on wheel. Start calibration?')) return;
  var sp = document.getElementById('calSpeed').value;
  fetch('/api/calib?speed=' + sp + '&start');
}
function calAbortBtn() { fetch('/api/calib?abort'); }
function calApplyBtn() { fetch('/api/calib?apply').then(function(){ configLoaded = false; }); }

function saveKeya() {
  var url = '/api/save?keyaDis=' + (document.getElementById('kd0').checked ? 1 : 0)
          + '&keyaSet=' + document.getElementById('kd1').value
          + '&keyaAct=' + document.getElementById('kd2').value
          + '&speedDiffTimeout=' + document.getElementById('kd3').value;
  fetch(url).then(function(r) {
    document.getElementById('sb').textContent = r.ok ? 'Keya params saved.' : 'ERROR saving.';
  });
}

function saveDbg() {
  var f = 0;
  if (document.getElementById('dbg0').checked) f |= 1;
  if (document.getElementById('dbg4').checked) f |= 16;
  if (document.getElementById('dbg7').checked) f |= 128;
  fetch('/api/save?debugFlags=' + f).then(function(r) {
    document.getElementById('sb').textContent = r.ok
      ? 'Debug flags saved (no restart).' : 'ERROR saving flags.';
  });
}

function pollLog() {
  if (logFetching) return;
  logFetching = true;
  var el = document.getElementById('log');
  var atBottom = el.scrollHeight - el.clientHeight <= el.scrollTop + 10;
  fetch('/api/log', { cache: 'no-store' })
    .then(function(r) { return r.text(); })
    .then(function(t) {
      logFetching = false;
      el.textContent = t || '(empty)';
      if (atBottom) el.scrollTop = el.scrollHeight;
    })
    .catch(function() { logFetching = false; });
}

function clearLog() {
  fetch('/api/log?clear=1').then(function() {
    document.getElementById('log').textContent = '(cleared)';
  });
}

function tick() {
  if (!configLoaded) {
    fetch('/api/status', { cache: 'no-store' })
      .then(function(r) { return r.json(); })
      .then(function(d) { upd(d); configLoaded = true; })
      .catch(function() { document.getElementById('sb').textContent = 'No connection to Teensy...'; });
    return;
  }
  // Live tab: poll only what's needed (minimal payload), or nothing if Live Off
  if (activeTab === 'live') {
    if (lvMode === 'values' && activeGroup > 0) {
      fetch('/api/grp?g=' + activeGroup, { cache: 'no-store' })
        .then(function(r) { return r.json(); })
        .then(function(d) { renderGroup(d); })
        .catch(function() { document.getElementById('sb').textContent = 'No connection to Teensy...'; });
    } else if (lvMode === 'graph') {
      gPoll();
    }
    return;
  }
  // Other tabs: lightweight full live update
  fetch('/api/live', { cache: 'no-store' })
    .then(function(r) { return r.json(); })
    .then(function(d) { updLive(d); })
    .catch(function() { document.getElementById('sb').textContent = 'No connection to Teensy...'; });
  if (activeTab === 'debug') pollLog();
  if (activeTab === 'um98x') pollGpsRaw();
  if (activeTab === 'cansteer') pollCanRaw();
}

function updLive(d) {
  badge('d5', d.gps); badge('d_gga', d.gga); badge('d_vtg', d.vtg); badge('d_hpr', d.hpr);
  badge('d1', d.bno); badge('d2', d.tm);
  badge('d_ads', d.ads); badge('d_imuWas', d.iWas); badge('d3', d.keya);

  badge('k_det',  d.keya); badge('k_zero', d.kZero);
  var rb = document.getElementById('calRefBadge');
  if (rb) { rb.className = 'badge ' + (d.refFresh ? 'ok' : 'fail'); rb.textContent = d.refFresh ? 'OK' : '--'; }
  var ra = document.getElementById('calRefAngle');
  if (ra) ra.textContent = d.refFresh ? (d.refAngle.toFixed(2) + ' °') : '—';
  var csv = document.getElementById('calState');
  if (csv && d.calMsg !== undefined) {
    csv.textContent = d.calMsg;
    csv.style.color = (d.calState === 7) ? '#f87171' : (d.calState === 6) ? '#4ade80' : '#e2e8f0';
    document.getElementById('calDz').textContent  = d.calDz > 0 ? d.calDz.toFixed(2) + ' °' : '—';
    document.getElementById('calLR').textContent  = (d.calTL > 0 || d.calTR > 0) ? (d.calTL.toFixed(1) + ' | ' + d.calTR.toFixed(1)) : '—';
    document.getElementById('calTpd').textContent = d.calTpd > 0 ? d.calTpd.toFixed(1) : '—';
    var ab = document.getElementById('calApplyBtn');
    if (ab) ab.style.display = (d.calState === 6) ? '' : 'none';
  }
  // Keya motor config
  if (d.kcRam) {
    var km = document.getElementById('kcMode');
    if (km) { km.className = 'badge ' + (d.kcMode ? 'ok' : 'fail'); km.textContent = d.kcMode ? 'CONFIG' : 'OUT'; }
    for (var ki = 0; ki < 4; ki++) {
      var er = document.getElementById('kcRam' + ki); if (er) er.textContent = (d.kcRam[ki] < 0) ? '-' : d.kcRam[ki];
      var eo = document.getElementById('kcRom' + ki); if (eo) eo.textContent = (d.kcRom[ki] < 0) ? '-' : d.kcRom[ki];
    }
  }
  document.getElementById('k_enc').textContent  = d.kEnc + ' ticks';
  document.getElementById('k_off').textContent  = d.kOff.toFixed(3) + ' °';
  document.getElementById('kw_off').textContent = d.kOff.toFixed(3) + ' °';
  document.getElementById('k_act').textContent  = d.kAct;
  document.getElementById('k_set').textContent  = d.kSet;

  document.getElementById('cs0').textContent = d.vReady === 16 ? 'READY (16)' : (d.vReady || 0);
  document.getElementById('cs1').textContent = d.eCurve + ' (' + (d.eCurve - 32128) + ')';
  document.getElementById('cs2').textContent = d.sCurve + ' (' + (d.sCurve - 32128) + ')';
  document.getElementById('cs3').textContent = Math.round(d.hitch / 2.5) + ' %';
  var ci = document.getElementById('cs4');
  ci.className = 'badge ' + (d.intend ? 'ok' : 'fail');
  ci.textContent = d.intend ? 'STEERING' : 'IDLE';
  var pb = document.getElementById('canPlotBtn');
  if (pb && d.showData) pb.textContent = 'Disable CAN log';

  // Custom engage live + AOG steer state
  if (d.ceBuf) {
    ceLastBuf = d.ceBuf;
    var fh = document.getElementById('ceFrameHex');
    if (fh) fh.textContent = d.ceSeen ? d.ceBuf.map(ceHex).join(' ') : '— (no frame on ID)';
    var mb = document.getElementById('ceMatchBadge');
    if (mb) { mb.className = 'badge ' + (d.ceMatch ? 'ok' : 'fail'); mb.textContent = d.ceMatch ? 'MATCH' : 'no'; }
    var eb = document.getElementById('ceEngBadge');
    if (eb) { eb.className = 'badge ' + (d.aogSteerSw === 0 ? 'ok' : 'fail'); eb.textContent = d.aogSteerSw === 0 ? 'ENGAGED' : 'OFF'; }
    var gb = document.getElementById('ceGuidBadge');
    if (gb) { gb.className = 'badge ' + (d.aogGuidance ? 'ok' : 'fail'); gb.textContent = d.aogGuidance ? 'ON' : 'OFF'; }
    var ab = document.getElementById('ceAutoBadge');
    if (ab) { ab.className = 'badge ' + (d.on ? 'ok' : 'fail'); ab.textContent = d.on ? 'ACTIVE' : 'OFF'; }
  }

  document.getElementById('u0').textContent  = d.kp;
  document.getElementById('u1').textContent  = d.hiPWM;
  document.getElementById('u2').textContent  = d.loPWM;
  document.getElementById('u3').textContent  = d.minPWM;
  document.getElementById('u4').textContent  = d.cnt;
  document.getElementById('u5').textContent  = d.wasOff;
  document.getElementById('u6').textContent  = yn(d.invWAS);
  document.getElementById('u7').textContent  = yn(d.cytron);
  document.getElementById('u8').textContent  = yn(d.shaftEnc);
  document.getElementById('u9').textContent  = yn(d.pres);
  document.getElementById('u10').textContent = yn(d.curr);
  document.getElementById('u11').textContent = yn(d.danf);

  var fixNames = ['—','GPS','DGPS','PPS','RTK','Float RTK','Est','Manual','Sim'];
  document.getElementById('j19Fix').textContent = fixNames[d.j19Fix] || d.j19Fix;
  if (d.j19Lat !== 0 || d.j19Lon !== 0)
    document.getElementById('j19LatLon').textContent = d.j19Lat.toFixed(7) + ' / ' + d.j19Lon.toFixed(7);

  var pi = document.getElementById('pvedInfo');
  var rb = document.getElementById('pvedRestoreBtn');
  if (pi) {
    var toHex = function(v) { return '0x' + v.toString(16).toUpperCase().padStart(2,'0'); };
    var html = 'Valve: <b style="color:' + (d.pvedDet ? '#4ade80' : '#f87171') + '">' + (d.pvedDet ? 'detected' : 'not detected') + '</b>';
    if (d.pvedLast !== 65535) html += '&nbsp;&nbsp;&nbsp;Param 64007 (current): <b style="color:#e2e8f0">' + toHex(d.pvedLast) + '</b>';
    if (d.pvedFac !== 65535) html += '&nbsp;&nbsp;&nbsp;Factory value (saved): <b style="color:#fbbf24">' + toHex(d.pvedFac) + '</b>';
    pi.innerHTML = html;
    pi.style.display = (d.pvedDet || d.pvedFac !== 65535) ? '' : 'none';
    var wb = document.getElementById('pvedWriteBtn');
    if (wb) wb.style.display = (d.pvedFac !== 65535) ? '' : 'none';
    if (rb) rb.style.display = (d.pvedFac !== 65535 && d.pvedFac !== 30) ? '' : 'none';
  }

  document.getElementById('logActive').checked = !!d.logOn;
  document.getElementById('sb').textContent = 'Updated: ' + new Date().toLocaleTimeString();
}

function setLogActive(on) {
  fetch('/api/log?active=' + (on ? '1' : '0'));
}

function setGpsActive(on) {
  fetch('/api/gpsraw?active=' + (on ? '1' : '0'));
}

function saveCanModes() {
  var url = '/api/save?can1Mode=' + document.getElementById('can1Mode').value
          + '&can2Mode='  + document.getElementById('can2Mode').value
          + '&can3Mode='  + document.getElementById('can3Mode').value
          + '&can1Baud='  + document.getElementById('can1Baud').value
          + '&can2Baud='  + document.getElementById('can2Baud').value
          + '&can3Baud='  + document.getElementById('can3Baud').value;
  fetch(url).then(function(r) {
    document.getElementById('sb').textContent = r.ok ? 'CAN assignment saved – restarting...' : 'Error saving CAN assignment.';
    configLoaded = false;
  });
}

function saveJ1939() {
  var url = '/api/save'
    + '?j19Addr='    + document.getElementById('j19Addr').value
    + '&j19En65267=' + (document.getElementById('j19En65267').checked ? 1 : 0)
    + '&j19R65267='  + document.getElementById('j19R65267').value
    + '&j19En129029='+ (document.getElementById('j19En129029').checked ? 1 : 0)
    + '&j19R129029=' + document.getElementById('j19R129029').value;
  fetch(url).then(function(r) {
    document.getElementById('sb').textContent = r.ok ? 'J1939 params saved.' : 'Error saving J1939 params.';
  });
}

function saveDataSource() {
  var url = '/api/save'
    + '?wasSource='     + document.getElementById('wasSource').value
    + '&rollSource='    + document.getElementById('rollSource').value
    + '&headingSource=' + document.getElementById('headingSource').value
    + '&nmeaType='      + document.getElementById('nmeaType').value;
  fetch(url).then(function(r) {
    document.getElementById('sb').textContent = r.ok ? 'Data source saved – restarting...' : 'Error saving data source.';
    configLoaded = false;
  });
}

function saveSerial() {
  var url = '/api/save'
    + '?gpsSerial='   + document.getElementById('gpsSerial').value
    + '&gpsBaudR='    + document.getElementById('gpsBaud2').value
    + '&tm171Serial=' + document.getElementById('tm171Serial').value
    + '&tm171Baud='   + document.getElementById('tm171Baud').value;
  fetch(url).then(function(r) {
    document.getElementById('sb').textContent = r.ok ? 'Serial assignment saved – restarting...' : 'Error saving serial assignment.';
    configLoaded = false;
  });
}

function setKeyaZero() {
  fetch('/api/keyazero', { cache: 'no-store' }).then(function(r) {
    document.getElementById('sb').textContent = r.ok ? 'Keya WAS zero set.' : 'Error setting zero.';
    loaded = false;
  });
}

function saveKeyaWas() {
  var url = '/api/save'
    + '?keyaTicksPD=' + document.getElementById('kw1').value
    + '&keyaEncInv='  + (document.getElementById('kw2').checked ? 1 : 0)
    + '&keyaEmaA='    + document.getElementById('kwema').value
    + '&keyaAzEn='    + (document.getElementById('kw4').checked ? 1 : 0)
    + '&keyaAzBeta='  + document.getElementById('kw5').value
    + '&keyaAzVmin='  + document.getElementById('kw6').value
    + '&keyaAzYawMax='+ document.getElementById('kw7').value
    + '&keyaAzVslow=' + document.getElementById('kw8').value
    + '&keyaAzVfast=' + document.getElementById('kw9').value
    + '&keyaAzUseImu=' + (document.getElementById('kwImu').checked ? 1 : 0)
    + '&keyaAzTslow=' + document.getElementById('kw10').value
    + '&keyaAzTfast=' + document.getElementById('kw11').value
    + '&wheelBase='   + document.getElementById('kwwb').value;
  fetch(url).then(function(r) {
    document.getElementById('sb').textContent = r.ok ? 'Keya WAS params saved.' : 'ERROR saving.';
  });
}


function setImuWasZero() {
  fetch('/api/imuwaszero', { cache: 'no-store' }).then(function(r) {
    document.getElementById('sb').textContent = r.ok ? 'IMU WAS zero set.' : 'Error setting zero.';
  });
}

function saveImuWas() {
  var url = '/api/save'
    + '?imuWasInv='  + (document.getElementById('iw0').checked ? 1 : 0)
    + '&imuWasCpd='  + document.getElementById('iw1').value
    + '&imuWasAzEn=' + (document.getElementById('iw2').checked ? 1 : 0)
    + '&imuWasAzB='  + document.getElementById('iw3').value
    + '&imuWasVmin=' + document.getElementById('iw4').value
    + '&imuWasYaw='  + document.getElementById('iw5').value
    + '&wheelBase='  + document.getElementById('iwwb').value;
  fetch(url).then(function(r) {
    document.getElementById('sb').textContent = r.ok ? 'IMU WAS params saved.' : 'ERROR saving.';
  });
}

var gpsRawFetching = false;

function pollGpsRaw() {
  if (gpsRawFetching) return;
  gpsRawFetching = true;
  var el = document.getElementById('gpsraw');
  var atBottom = el.scrollHeight - el.clientHeight <= el.scrollTop + 10;
  fetch('/api/gpsraw', { cache: 'no-store' })
    .then(function(r) { return r.text(); })
    .then(function(t) {
      gpsRawFetching = false;
      el.textContent = t || '(empty)';
      if (atBottom) el.scrollTop = el.scrollHeight;
    })
    .catch(function() { gpsRawFetching = false; });
}

function clearGpsRaw() {
  fetch('/api/gpsraw?clear=1').then(function() {
    document.getElementById('gpsraw').textContent = '(cleared)';
  });
}

function gpsCmd(cmd) {
  document.getElementById('gpsStatus').textContent = 'Sending: ' + cmd;
  fetch('/api/gpscmd?line=' + encodeURIComponent(cmd))
    .then(function() {
      document.getElementById('gpsStatus').textContent = 'Sent: ' + cmd;
      setTimeout(pollGpsRaw, 500);
    });
}

function doFactoryReset() {
  if (!confirm('Factory reset the GPS receiver? This will erase all saved config.')) return;
  gpsCmd('FRESET');
}

function saveCanBrand() {
  var v = document.getElementById('csBrand').value;
  fetch('/api/save?brand=' + v).then(function(r) {
    document.getElementById('sb').textContent = r.ok ? 'Brand saved – restarting...' : 'Error saving brand.';
  });
}

// ── Custom CAN engage ─────────────────────────────────────────────────────────
var ceLastBuf = [0,0,0,0,0,0,0,0];
var ceIdle = null, cePress = null;
function ceHex(v){ return ('0' + (v & 0xFF).toString(16).toUpperCase()).slice(-2); }
function ceMsg(t){ document.getElementById('ceLearnMsg').textContent = t; }

function ceBuildRows(){
  var st='width:34px;text-align:center;background:#0f172a;border:1px solid #334155;color:#e2e8f0;font-family:monospace;font-size:12px;padding:3px 0;border-radius:3px;margin-right:3px';
  var vr='', mr='';
  for(var i=0;i<8;i++){
    vr+='<input type="text" id="ceVal'+i+'" maxlength="2" value="00" style="'+st+'">';
    mr+='<input type="text" id="ceMsk'+i+'" maxlength="2" value="00" style="'+st+'">';
  }
  document.getElementById('ceValRow').innerHTML=vr;
  document.getElementById('ceMskRow').innerHTML=mr;
}

function ceCaptureIdle(){
  ceIdle = ceLastBuf.slice();
  document.getElementById('ceIdleHex').textContent = ceIdle.map(ceHex).join(' ');
  ceMsg('IDLE captured'); ceTryLearn();
}
function ceCapturePressed(){
  cePress = ceLastBuf.slice();
  document.getElementById('cePressHex').textContent = cePress.map(ceHex).join(' ');
  ceMsg('PRESSED captured'); ceTryLearn();
}
function ceTryLearn(){
  if(!ceIdle || !cePress) return;
  var any=0;
  for(var i=0;i<8;i++){
    var mask = ceIdle[i] ^ cePress[i];
    any |= mask;
    document.getElementById('ceMsk'+i).value = ceHex(mask);
    document.getElementById('ceVal'+i).value = ceHex(cePress[i] & mask);
  }
  ceMsg(any ? 'Auto-filled from changed bits — review and Save' : 'No bit changed — hold engage during PRESSED capture');
}

function ceSave(){
  var vals=[], msks=[];
  for(var i=0;i<8;i++){
    vals.push(document.getElementById('ceVal'+i).value || '0');
    msks.push(document.getElementById('ceMsk'+i).value || '0');
  }
  var url='/api/save?ceEnable='+(document.getElementById('ceEnable').checked?1:0)
    +'&cePort='+document.getElementById('cePort').value
    +'&ceExt='+document.getElementById('ceExt').value
    +'&ceMode='+document.getElementById('ceMode').value
    +'&ceId='+(document.getElementById('ceId').value||'0')
    +'&ceVal='+vals.join(',')
    +'&ceMsk='+msks.join(',');
  fetch(url).then(function(r){
    document.getElementById('sb').textContent = r.ok ? 'Custom engage saved – restarting...' : 'Error saving custom engage.';
    configLoaded = false;
  });
}

var canRawFetching = false;

function pollCanRaw() {
  if (canRawFetching) return;
  canRawFetching = true;
  var el = document.getElementById('canraw');
  var atBottom = el.scrollHeight - el.clientHeight <= el.scrollTop + 10;
  fetch('/api/canraw', { cache: 'no-store' })
    .then(function(r) { return r.text(); })
    .then(function(t) {
      canRawFetching = false;
      el.textContent = t || '(empty)';
      if (atBottom) el.scrollTop = el.scrollHeight;
    })
    .catch(function() { canRawFetching = false; });
}

function clearCanRaw() {
  fetch('/api/canraw?clear=1').then(function() {
    document.getElementById('canraw').textContent = '(cleared)';
  });
}

function toggleCanPlot() {
  var btn = document.getElementById('canPlotBtn');
  var on = btn.textContent === 'Enable CAN log' ? 1 : 0;
  fetch('/api/canraw?show=' + (on ? '1' : '0')).then(function(r) {
    if (r.ok) btn.textContent = on ? 'Disable CAN log' : 'Enable CAN log';
  });
}

var knownCanIds = {
  '0x0CAC1E13':'Claas — steering curve + valve state',
  '0x18EF1CD2':'Claas — engage',
  '0x1CFFE6D2':'Claas — CEBIS work mode',
  '0x0CAD131E':'Claas — TX (AIO → valve)',
  '0x0CAC1C13':'Valtra / McCormick / MF / AgOpenGPS — steering curve + valve state',
  '0x18EF1C32':'Valtra — engage',
  '0x18EF1CFC':'McCormick — engage',
  '0x18EF1C00':'Massey Ferguson — engage',
  '0x18FF8306':'McCormick — joystick engage',
  '0x0CAD131C':'Valtra / McC / MF / AgOpenGPS — TX (AIO → valve)',
  '0x0CACAA08':'CaseIH / New Holland — steering curve + valve state',
  '0x18FFBB03':'CaseIH — engage',
  '0x14FF7706':'CaseIH — K_Bus engage',
  '0x18FE4523':'CaseIH — K_Bus rear hitch',
  '0x0CAD08AA':'CaseIH / NH — TX (AIO → valve)',
  '0x0CEF2CF0':'Fendt / FendtOne — steering curve',
  '0x0CEFF02C':'Fendt / FendtOne — TX (AIO → valve)',
  '0x18EF2CF0':'Fendt — ISO engage',
  '0x0CFFD899':'FendtOne — K_Bus engage',
  '0x0CACAB13':'JCB — steering curve + valve state',
  '0x18EFAB27':'JCB — engage',
  '0x0CAD13AB':'JCB — TX (AIO → valve)',
  '0x0CACF013':'Lindner — steering curve + valve state',
  '0x0CEFF021':'Lindner — engage',
  '0x0CAD13F0':'Lindner — TX (AIO → valve)',
  '0x18EF1CF0':'Cat / Challenger MT Late — curve + valve + engage',
  '0x1CEFF01C':'Cat MT Late — TX (AIO → valve)',
  '0x0CEFFF76':'Cat / Challenger MT Early — curve + valve + engage',
  '0x0CEF762C':'Cat MT Early — TX (AIO → valve)',
};
function getIdLabel(id) {
  var k = knownCanIds[id];
  if (k) return k;
  var n = parseInt(id, 16);
  if ((n & 0x00FFFF00) === 0x00FEBB00) return 'ISO — rear hitch PGN 65093';
  if ((n & 0x00FF0000) === 0x00EA0000) return 'J1939 — parameter request (PVED tool)';
  if ((n & 0x00FF0000) === 0x00EF0000) return 'J1939 — proprietary peer-to-peer';
  return '';
}
function startCanScan() {
  var btn = document.getElementById('scanBtn');
  btn.disabled = true;
  document.getElementById('scanResults').innerHTML = '';
  document.getElementById('scanTimer').textContent = '';
  fetch('/api/canscan?start').then(function() {
    var t = 5;
    document.getElementById('scanTimer').textContent = t + 's remaining';
    var iv = setInterval(function() {
      t--;
      if (t > 0) {
        document.getElementById('scanTimer').textContent = t + 's remaining';
      } else {
        clearInterval(iv);
        document.getElementById('scanTimer').textContent = 'analysing...';
        fetch('/api/canscan?stop').then(function() {
          fetch('/api/canscan?data').then(function(r) { return r.json(); }).then(function(d) {
            btn.disabled = false;
            renderScanResults(d);
          });
        });
      }
    }, 1000);
  });
}
function renderScanResults(d) {
  var el = document.getElementById('scanTimer');
  if (d.count === 0) {
    el.textContent = 'No messages received';
    document.getElementById('scanResults').innerHTML = '<p style="color:#f87171;font-size:13px;margin-top:6px">Nothing detected. Check CAN port configuration and wiring.</p>';
    return;
  }
  el.textContent = d.count + ' unique ID' + (d.count > 1 ? 's' : '') + ' found';
  d.entries.sort(function(a,b){ return b.n - a.n; });
  var known = d.entries.filter(function(e){ return getIdLabel(e.id) !== ''; });
  var unknown = d.entries.filter(function(e){ return getIdLabel(e.id) === ''; });
  var html = '';
  if (known.length) {
    html += '<p style="color:#4ade80;font-size:12px;margin:6px 0 4px"><b>Known IDs detected:</b></p>';
    html += '<table style="width:100%;border-collapse:collapse;font-size:13px">';
    html += '<tr style="color:#64748b;font-size:11px"><th style="text-align:left;padding:3px 6px">Bus</th><th style="text-align:left;padding:3px 6px">ID</th><th style="text-align:right;padding:3px 6px">Cnt</th><th style="text-align:left;padding:3px 6px">Description</th></tr>';
    known.forEach(function(e) {
      html += '<tr style="border-top:1px solid #1e293b"><td style="padding:4px 6px;color:#94a3b8">' + e.bus + '</td>';
      html += '<td style="padding:4px 6px;font-family:monospace;color:#e2e8f0">' + e.id + '</td>';
      html += '<td style="padding:4px 6px;text-align:right;color:#64748b">' + e.n + '</td>';
      html += '<td style="padding:4px 6px;color:#4ade80">' + getIdLabel(e.id) + '</td></tr>';
    });
    html += '</table>';
  }
  if (unknown.length) {
    html += '<p style="color:#64748b;font-size:12px;margin:10px 0 4px">Unknown IDs (' + unknown.length + '):</p>';
    html += '<table style="width:100%;border-collapse:collapse;font-size:12px">';
    unknown.forEach(function(e) {
      html += '<tr style="border-top:1px solid #1e293b"><td style="padding:3px 6px;color:#475569;width:20px">' + e.bus + '</td>';
      html += '<td style="padding:3px 6px;font-family:monospace;color:#64748b">' + e.id + '</td>';
      html += '<td style="padding:3px 6px;text-align:right;color:#334155">' + e.n + '</td><td></td></tr>';
    });
    html += '</table>';
  }
  document.getElementById('scanResults').innerHTML = html;
}

function pvedCmd(cmd) {
  fetch('/api/pved?cmd=' + cmd).then(function(r) {
    document.getElementById('sb').textContent = r.ok ? 'PVED ' + cmd + ' sent. Enable CAN log to see responses.' : 'Error.';
    setTimeout(pollCanRaw, 300);
    if (cmd === 'read64007' || cmd === 'restore') setTimeout(tick, 800);
  });
}

function uploadLines() {
  var raw = document.getElementById('gpsLines').value;
  var lines = raw.split('\n').map(function(l) { return l.trim(); }).filter(function(l) { return l.length > 0; });
  if (!lines.length) { document.getElementById('gpsStatus').textContent = 'No lines to send.'; return; }
  var i = 0;
  var st = document.getElementById('gpsStatus');
  function sendNext() {
    if (i >= lines.length) {
      st.textContent = 'Done — ' + lines.length + ' line(s) sent.';
      setTimeout(pollGpsRaw, 300);
      return;
    }
    var line = lines[i];
    st.textContent = 'Sending ' + (i + 1) + '/' + lines.length + ': ' + line;
    fetch('/api/gpscmd?line=' + encodeURIComponent(line))
      .then(function() { i++; setTimeout(sendNext, 1000); })
      .catch(function() { st.textContent = 'Error on line ' + (i + 1); });
  }
  sendNext();
}

// ── Graph (Live tab → Graph) ─────────────────────────────────────────────────
var gSignals = [
 {id:0,n:'-- none --'},
 // ── Gr1 GPS ──
 {id:1,n:'Gr1 GGA fixQuality'},{id:2,n:'Gr1 GGA numSats'},{id:3,n:'Gr1 GGA HDOP'},{id:4,n:'Gr1 GGA altitude'},
 {id:5,n:'Gr1 VTG heading'},{id:6,n:'Gr1 VTG speed'},
 {id:7,n:'Gr1 HPR heading'},{id:8,n:'Gr1 HPR roll'},{id:9,n:'Gr1 HPR quality'},
 {id:38,n:'Gr1 GGA interval ms'},{id:39,n:'Gr1 VTG interval ms'},{id:40,n:'Gr1 HPR interval ms'},
 // ── Gr2 IMU ──
 {id:10,n:'Gr2 BNO heading'},{id:11,n:'Gr2 BNO roll'},{id:12,n:'Gr2 BNO pitch'},{id:13,n:'Gr2 BNO yawRate'},
 {id:14,n:'Gr2 TM171 heading'},{id:15,n:'Gr2 TM171 roll'},{id:16,n:'Gr2 TM171 pitch'},
 // ── Gr3 WAS ──
 {id:17,n:'Gr3 WAS ADS raw'},{id:18,n:'Gr3 WAS IMU raw'},{id:19,n:'Gr3 WAS IMU scaled'},
 {id:20,n:'Gr3 WAS chassis yawRate'},{id:21,n:'Gr3 WAS wheelAngleGPS'},{id:22,n:'Gr3 WAS actual'},
 // ── Gr4 Keya ──
 {id:23,n:'Gr4 Keya encoder'},{id:24,n:'Gr4 Keya gpsOffset'},{id:25,n:'Gr4 Keya actSpeed'},{id:26,n:'Gr4 Keya setSpeed'},
 {id:44,n:'Gr4 Keya wheel pos (deg)'},{id:45,n:'Gr4 Keya steering-wheel pos (deg)'},
 // ── Gr5 Steer ──
 {id:27,n:'Gr5 Steer actual'},{id:28,n:'Gr5 Steer setpoint'},{id:29,n:'Gr5 Steer error'},{id:30,n:'Gr5 PWM'},{id:31,n:'Gr5 speed'},
 {id:41,n:'Gr5 current sensor (A17)'},{id:42,n:'Gr5 pressure sensor (A10)'},{id:43,n:'Gr5 sensor reading'},
 // ── Gr6 CAN Steer ──
 {id:32,n:'Gr6 valveReady'},{id:33,n:'Gr6 estCurve'},{id:34,n:'Gr6 setCurve'},{id:35,n:'Gr6 hitch'},
 // ── Performance (no group) ──
 {id:36,n:'Perf loop time ms'},{id:37,n:'Perf loop max ms'}
];
var gCol  = ['#4ade80','#38bdf8','#fbbf24','#f87171'];
var gDef  = [27,28,22,11];
var gMin  = [-30,-30,-30,-10];
var gMax  = [30,30,30,10];
var gData = [[],[],[],[]];
var gTimes= [];
var gClock= 0, gRateMs = 200, gLogging = false, gPaused = false, gView = null;
var gCursorX = null, gDragging = false, gDragX = 0;

function gSigName(id){ for(var i=0;i<gSignals.length;i++) if(gSignals[i].id===id) return gSignals[i].n; return ''; }

// Current time window [start,end] in seconds. Fill from left until window full, then scroll.
function gWin(){
  if(gView) return gView;
  var win=parseFloat(document.getElementById('gTime').value);
  if(gClock<=win) return {start:0,end:win};        // fill phase: 0..win, pen moves left→right
  return {start:gClock-win,end:gClock};            // scroll phase
}

function gBuildRows(){
  var html='';
  for(var c=0;c<4;c++){
    var opts='';
    for(var s=0;s<gSignals.length;s++)
      opts+='<option value="'+gSignals[s].id+'"'+(gSignals[s].id===gDef[c]?' selected':'')+'>'+gSignals[s].n+'</option>';
    html+='<div style="display:flex;gap:6px;align-items:center;padding:2px 0;flex-wrap:wrap">'
      +'<span style="color:'+gCol[c]+';font-weight:bold;width:54px">Data '+(c+1)+'</span>'
      +'<select id="gsig'+c+'" style="flex:1;min-width:150px">'+opts+'</select>'
      +' min<input type="number" id="gmin'+c+'" value="'+gMin[c]+'" step="any" class="ninput" style="width:56px">'
      +' max<input type="number" id="gmax'+c+'" value="'+gMax[c]+'" step="any" class="ninput" style="width:56px">'
      +' <button class="btn sm" onclick="gAuto('+c+')">Auto</button>'
      +' <span style="color:#64748b;font-size:11px">'+(c<2?'← left':'→ right')+'</span></div>';
  }
  document.getElementById('gRows').innerHTML=html;
}

function gCfgUrl(stop){
  return '/api/graphcfg?d1='+gDef[0]+'&d2='+gDef[1]+'&d3='+gDef[2]+'&d4='+gDef[3]+'&rate='+gRateMs+(stop?'&stop':'');
}

function gSet(){
  for(var c=0;c<4;c++){
    gDef[c]=parseInt(document.getElementById('gsig'+c).value);
    gMin[c]=parseFloat(document.getElementById('gmin'+c).value);
    gMax[c]=parseFloat(document.getElementById('gmax'+c).value);
  }
  gRateMs=Math.round(1000/parseInt(document.getElementById('gFreq').value));
  // zero Keya relative position if any channel plots it (signal 44/45)
  if (gDef.indexOf(44) >= 0 || gDef.indexOf(45) >= 0) fetch('/api/keyaposzero');
  gData=[[],[],[],[]]; gTimes=[]; gClock=0; gView=null; gPaused=false;
  document.getElementById('gPauseBtn').textContent='Pause';
  fetch(gCfgUrl(!gLogging)).then(function(r){document.getElementById('sb').textContent=r.ok?'Graph set':'Graph error';});
  gDraw();
}

function gToggle(on){
  gLogging=on;
  if(on){gPaused=false;document.getElementById('gPauseBtn').textContent='Pause';gView=null;}
  fetch(gCfgUrl(!on));
}

function gPauseToggle(){
  gPaused=!gPaused;
  document.getElementById('gPauseBtn').textContent=gPaused?'Resume':'Pause';
  if(gPaused){var v=gWin();gView={start:v.start,end:v.end};}
  else gView=null;
  gDraw();
}

function gAuto(c){
  if(!gData[c].length)return;
  var mn=1e9,mx=-1e9;
  for(var i=0;i<gData[c].length;i++){var v=gData[c][i];if(v<mn)mn=v;if(v>mx)mx=v;}
  if(mn===mx){mn-=1;mx+=1;}
  var pad=(mx-mn)*0.1;
  gMin[c]=+(mn-pad).toFixed(2); gMax[c]=+(mx+pad).toFixed(2);
  document.getElementById('gmin'+c).value=gMin[c];
  document.getElementById('gmax'+c).value=gMax[c];
  gDraw();
}

function gResetView(){gView=null;if(gPaused){gPaused=false;document.getElementById('gPauseBtn').textContent='Pause';}gDraw();}

function gPoll(){
  if(!gLogging||gPaused)return;
  fetch('/api/graphdata',{cache:'no-store'}).then(function(r){return r.json();}).then(function(d){
    var dt=d.dt/1000;
    for(var i=0;i<d.n;i++){gClock+=dt;gTimes.push(gClock);for(var c=0;c<4;c++)gData[c].push(d.d[c][i]);}
    var win=parseFloat(document.getElementById('gTime').value);
    var cut=gClock-win-2;
    while(gTimes.length&&gTimes[0]<cut){gTimes.shift();for(var c=0;c<4;c++)gData[c].shift();}
    gDraw();
  }).catch(function(){});
}

function gDraw(){
  var cv=document.getElementById('gcanvas');if(!cv)return;
  var ctx=cv.getContext('2d'),W=cv.width,H=cv.height;
  ctx.clearRect(0,0,W,H);
  var L=46,R=W-46,T=10,B=H-22;
  var vw=gWin(), vStart=vw.start, vEnd=vw.end;
  if(vEnd-vStart<=0)vStart=vEnd-1;
  ctx.strokeStyle='#1e3a5f';ctx.font='10px monospace';ctx.lineWidth=1;
  for(var g=0;g<=5;g++){
    var y=T+(B-T)*g/5;
    ctx.beginPath();ctx.moveTo(L,y);ctx.lineTo(R,y);ctx.stroke();
    var lv=gMax[0]+(gMin[0]-gMax[0])*g/5, rv=gMax[2]+(gMin[2]-gMax[2])*g/5;
    ctx.fillStyle=gCol[0];ctx.textAlign='right';ctx.fillText(lv.toFixed(1),L-3,y+3);
    ctx.fillStyle=gCol[2];ctx.textAlign='left';ctx.fillText(rv.toFixed(1),R+3,y+3);
  }
  ctx.fillStyle='#64748b';ctx.textAlign='center';
  for(var t=0;t<=4;t++){var x=L+(R-L)*t/4,tv=vStart+(vEnd-vStart)*t/4;ctx.fillText((tv-vStart).toFixed(1)+'s',x,B+14);}
  function xt(tt){return L+(R-L)*(tt-vStart)/(vEnd-vStart);}
  for(var c=0;c<4;c++){
    var mn=gMin[c],mx=gMax[c];if(mx===mn)mx=mn+1;
    ctx.strokeStyle=gCol[c];ctx.lineWidth=1.5;ctx.beginPath();var st=false;
    for(var i=0;i<gTimes.length;i++){
      if(gTimes[i]<vStart||gTimes[i]>vEnd)continue;
      var x=xt(gTimes[i]),y=B-(B-T)*(gData[c][i]-mn)/(mx-mn);
      if(y<T)y=T;if(y>B)y=B;
      if(!st){ctx.moveTo(x,y);st=true;}else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }
  ctx.textAlign='left';ctx.font='11px monospace';
  for(var c=0;c<4;c++){
    var last=gData[c].length?gData[c][gData[c].length-1]:null;
    ctx.fillStyle=gCol[c];
    ctx.fillText('D'+(c+1)+' '+gSigName(gDef[c])+(last!==null?'  ='+last.toFixed(2):''),L+4,T+12+c*13);
  }
  if(gPaused&&gCursorX!==null){
    ctx.strokeStyle='#e2e8f0';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(gCursorX,T);ctx.lineTo(gCursorX,B);ctx.stroke();
    var tt=vStart+(vEnd-vStart)*(gCursorX-L)/(R-L);
    var idx=-1,best=1e9;
    for(var i=0;i<gTimes.length;i++){var dd=Math.abs(gTimes[i]-tt);if(dd<best){best=dd;idx=i;}}
    var txt=idx>=0?('t='+gTimes[idx].toFixed(2)+'s'):('t='+tt.toFixed(2)+'s');
    if(idx>=0)for(var c=0;c<4;c++)txt+='   D'+(c+1)+'='+gData[c][idx].toFixed(2);
    document.getElementById('gReadout').textContent=txt;
  }
}

function gCanvasInit(){
  var cv=document.getElementById('gcanvas');if(!cv)return;
  cv.addEventListener('mousemove',function(e){
    if(!gPaused)return;
    var rect=cv.getBoundingClientRect();
    gCursorX=(e.clientX-rect.left)*cv.width/rect.width;
    if(gDragging&&gView){var dx=(gCursorX-gDragX)/(cv.width-92)*(gView.end-gView.start);gView.start-=dx;gView.end-=dx;gDragX=gCursorX;}
    gDraw();
  });
  cv.addEventListener('mousedown',function(e){if(gPaused){gDragging=true;var rect=cv.getBoundingClientRect();gDragX=(e.clientX-rect.left)*cv.width/rect.width;}});
  cv.addEventListener('mouseup',function(){gDragging=false;});
  cv.addEventListener('mouseleave',function(){gDragging=false;gCursorX=null;gDraw();});
  cv.addEventListener('wheel',function(e){
    if(!gPaused||!gView)return;e.preventDefault();
    var rect=cv.getBoundingClientRect(),cx=(e.clientX-rect.left)*cv.width/rect.width,L=46,R=cv.width-46;
    var frac=(cx-L)/(R-L);if(frac<0)frac=0;if(frac>1)frac=1;
    var tAt=gView.start+(gView.end-gView.start)*frac,f=e.deltaY<0?0.8:1.25;
    gView.start=tAt-(tAt-gView.start)*f;gView.end=tAt+(gView.end-tAt)*f;
    gDraw();
  },{passive:false});
}

function gExportCsv(){
  var rows='time_s';
  for(var c=0;c<4;c++)rows+=','+gSigName(gDef[c]).replace(/[, ]/g,'_');
  rows+='\n';
  var vw=gWin(),vStart=vw.start,vEnd=vw.end;
  for(var i=0;i<gTimes.length;i++){
    if(gTimes[i]<vStart||gTimes[i]>vEnd)continue;
    rows+=gTimes[i].toFixed(3);
    for(var c=0;c<4;c++)rows+=','+gData[c][i].toFixed(3);
    rows+='\n';
  }
  var blob=new Blob([rows],{type:'text/csv'});
  var a=document.createElement('a');a.href=URL.createObjectURL(blob);
  a.download='aio_graph_'+Date.now()+'.csv';a.click();
}

gBuildRows();
gCanvasInit();
ceBuildRows();
kcBindHold();
tick();
restartTick();
</script>
</body>
</html>
)AIOHTML";

// ── Helper: send buffer in 512-byte chunks ────────────────────────────────────
void sendBuf(EthernetClient& client, const uint8_t* p, size_t len)
{
    while (len > 0 && client.connected()) {
        size_t chunk = (len > 512) ? 512 : len;
        size_t sent  = client.write(p, chunk);
        if (sent == 0) delay(5);
        else { p += sent; len -= sent; }
    }
}

// ── Public API ─────────────────────────────────────────────────────────────────

void webServerBegin()
{
    webServer.begin();
    Serial.print("Web server: http://");
    Serial.println(Ethernet.localIP());
    webLogf("Web server: http://%d.%d.%d.%d",
        Ethernet.localIP()[0], Ethernet.localIP()[1],
        Ethernet.localIP()[2], Ethernet.localIP()[3]);
}

void handleWebClient()
{
    if (pendingRestart && restartDelay > 600) {
        Serial.println("Restarting Teensy...");
        delay(50);
        SCB_AIRCR = 0x05FA0004;
    }

    EthernetClient client = webServer.available();
    if (!client) return;

    // ── Read first request line ── 50ms max (browser sends immediately on LAN)
    char reqLine[256];
    int  idx = 0;
    elapsedMillis t = 0;
    while (client.connected() && t < 50) {
        if (client.available()) {
            char c = client.read();
            if (c == '\n') break;
            if (c != '\r' && idx < 254) reqLine[idx++] = c;
        }
    }
    reqLine[idx] = '\0';

    if (DBG_STEER) { Serial.print("WEB: "); Serial.println(reqLine); }

    // ── Drain headers until blank line ── 50ms max
    {
        int hi = 0;
        t = 0;
        while (client.connected() && t < 50) {
            if (client.available()) {
                char c = client.read();
                if (c == '\n') { if (hi == 0) break; hi = 0; }
                else if (c != '\r') hi++;
            }
        }
    }

    // ── Route ─────────────────────────────────────────────────────────────────
    bool isApi = (strstr(reqLine, "/api/") != NULL);

    if      (strstr(reqLine, "/api/save")        != NULL) handleApiSave(client, reqLine);
    else if (strstr(reqLine, "/api/graphcfg")    != NULL) handleApiGraphCfg(client, reqLine);
    else if (strstr(reqLine, "/api/graphdata")   != NULL) handleApiGraphData(client);
    else if (strstr(reqLine, "/api/grp")         != NULL) handleApiGrp(client, reqLine);
    else if (strstr(reqLine, "/api/live")        != NULL) handleApiLive(client);
    else if (strstr(reqLine, "/api/log")         != NULL) handleApiLog(client, reqLine);
    else if (strstr(reqLine, "/api/status")      != NULL) handleApiStatus(client);
    else if (strstr(reqLine, "/api/gpsraw")      != NULL) handleApiGpsRaw(client, reqLine);
    else if (strstr(reqLine, "/api/gpscmd")      != NULL) handleApiGpsCmd(client, reqLine);
    else if (strstr(reqLine, "/api/gpsbaud")     != NULL) handleApiGpsBaud(client, reqLine);
    else if (strstr(reqLine, "/api/keyaposzero") != NULL) { keyaPosRef = keyaEncoderRaw; sendHeaders(client, "text/plain"); client.print(F("OK")); }
    else if (strstr(reqLine, "/api/keyazero")    != NULL) handleApiKeyaZero(client);
    else if (strstr(reqLine, "/api/imuwaszero")  != NULL) handleApiImuWasZero(client);
    else if (strstr(reqLine, "/api/canraw")      != NULL) handleApiCanRaw(client, reqLine);
    else if (strstr(reqLine, "/api/canscan")     != NULL) handleApiCanScan(client, reqLine);
    else if (strstr(reqLine, "/api/pved")        != NULL) handleApiPved(client, reqLine);
    else if (strstr(reqLine, "/api/calib")       != NULL) handleApiCalib(client, reqLine);
    else if (strstr(reqLine, "/api/keyacfg")     != NULL) handleApiKeyaCfg(client, reqLine);
    else if (Autosteer_running && watchdogTimer < WATCHDOG_THRESHOLD) {
        client.println(F("HTTP/1.1 503 Service Unavailable\r\n"
                         "Content-Type: text/plain\r\n"
                         "Connection: close\r\n"
                         "\r\n"
                         "Autosteer active - open page when not steering."));
    }
    else { isApi = false; handleRoot(client); }

    // ── Close ─────────────────────────────────────────────────────────────────
    if (isApi) {
        // API: TCP stack delivers buffered data after stop() — no blocking wait
        client.stop();
    } else {
        // HTML page (~7KB): short wait so browser receives all chunks
        client.flush();
        elapsedMillis closeTimer = 0;
        while (client.connected() && closeTimer < 100) {
            while (client.available()) client.read();
        }
        client.stop();
    }
}

// ── Route handlers ─────────────────────────────────────────────────────────────

void sendHeaders(EthernetClient& c, const char* ct)
{
    c.println(F("HTTP/1.1 200 OK"));
    c.print(F("Content-Type: ")); c.println(ct);
    c.println(F("Connection: close"));
    c.println(F("Cache-Control: no-cache"));
    c.println();
}

void handleRoot(EthernetClient& client)
{
    sendHeaders(client, "text/html");
    sendBuf(client, (const uint8_t*)HTML_PAGE, strlen(HTML_PAGE));
}

void handleApiStatus(EthernetClient& client)
{
    sendHeaders(client, "application/json");

    client.print(F("{\"detected\":{"));
    client.print(F("\"bno_rvc\":")); client.print(useBNO08xRVC  ? F("true") : F("false"));
    client.print(F(",\"bno_i2c\":")); client.print(useBNO08xI2C ? F("true") : F("false"));
    client.print(F(",\"tm171\":")); client.print(useTMxx_IMU    ? F("true") : F("false"));
    client.print(F(",\"keya\":")); client.print(keyaDetected    ? F("true") : F("false"));
    client.print(F(",\"ads1115\":")); client.print(adcConnected ? F("true") : F("false"));
    client.print(F(",\"imuWas\":")); client.print((imuWasReceived && imuWasTimeout < 500) ? F("true") : F("false"));
    client.print(F(",\"gps\":")); client.print(GGAReadyTime < 10000 ? F("true") : F("false"));
    client.print(F(",\"gga\":")); client.print(GGAReadyTime < 10000 ? F("true") : F("false"));
    client.print(F(",\"vtg\":")); client.print(VTGReadyTime < 10000 ? F("true") : F("false"));
    client.print(F(",\"hpr\":")); client.print(HPRReadyTime < 10000 ? F("true") : F("false"));
    client.print(F(",\"logActive\":")); client.print(logActive ? F("true") : F("false"));

    client.print(F("},\"udp\":{"));
    client.print(F("\"kp\":")); client.print(steerSettings.Kp);
    client.print(F(",\"highPWM\":")); client.print(steerSettings.highPWM);
    client.print(F(",\"lowPWM\":")); client.print(steerSettings.lowPWM);
    client.print(F(",\"minPWM\":")); client.print(steerSettings.minPWM);
    client.print(F(",\"counts\":")); client.print((int)steerSettings.steerSensorCounts);
    client.print(F(",\"wasOffset\":")); client.print(steerSettings.wasOffset);
    client.print(F(",\"invertWAS\":")); client.print(steerConfig.InvertWAS);
    client.print(F(",\"cytron\":")); client.print(steerConfig.CytronDriver);
    client.print(F(",\"shaftEncoder\":")); client.print(steerConfig.ShaftEncoder);
    client.print(F(",\"pressureSensor\":")); client.print(steerConfig.PressureSensor);
    client.print(F(",\"currentSensor\":")); client.print(steerConfig.CurrentSensor);
    client.print(F(",\"danfoss\":")); client.print(steerConfig.IsDanfoss);

    client.print(F("},\"live\":{"));
    client.print(F("\"actualAngle\":")); client.print(steerAngleActual, 2);
    client.print(F(",\"setpoint\":")); client.print(steerAngleSetPoint, 2);
    client.print(F(",\"gpsSpeed\":")); client.print(gpsSpeed, 1);
    client.print(F(",\"active\":")); client.print(
        (watchdogTimer < WATCHDOG_THRESHOLD) ? F("true") : F("false"));
    client.print(F(",\"keyaActSpeed\":")); client.print(keyaCurrentActualSpeed);
    client.print(F(",\"keyaSetSpeed\":")); client.print(keyaCurrentSetSpeed);

    client.print(F("},\"keya_dis\":{"));
    client.print(F("\"enable\":")); client.print(moduleConfig.keyaDisEnable);
    client.print(F(",\"setSpeedMin\":")); client.print(moduleConfig.keyaSetSpeedMin);
    client.print(F(",\"actSpeedMin\":")); client.print(moduleConfig.keyaActSpeedMin);
    client.print(F(",\"timeoutMs\":")); client.print(moduleConfig.speedDiffTimeout);

    client.print(F("},\"motor_dis\":{"));
    client.print(F("\"enable\":")); client.print(moduleConfig.motorDisEnable);
    client.print(F(",\"angleErrorMin\":")); client.print(moduleConfig.motorAngleErrorMin);
    client.print(F(",\"timeoutMs\":")); client.print(moduleConfig.speedDiffTimeout);

    client.print(F("},\"keya_was\":{"));
    client.print(F("\"ticksPerDeg\":")); client.print(moduleConfig.keyaTicksPerDeg, 2);
    client.print(F(",\"encInvert\":")); client.print(moduleConfig.keyaEncInvert);
    client.print(F(",\"zeroTicks\":")); client.print(moduleConfig.keyaZeroTicks);
    client.print(F(",\"azEnable\":")); client.print(moduleConfig.keyaAzEnable);
    client.print(F(",\"azBeta\":")); client.print(moduleConfig.keyaAzBeta, 4);
    client.print(F(",\"azSpeedMin\":")); client.print(moduleConfig.keyaAzSpeedMin, 1);
    client.print(F(",\"azYawMax\":")); client.print(moduleConfig.keyaAzYawMax, 2);
    client.print(F(",\"azSpeedSlow\":")); client.print(moduleConfig.keyaAzSpeedSlow);
    client.print(F(",\"azSpeedFast\":")); client.print(moduleConfig.keyaAzSpeedFast, 1);
    client.print(F(",\"azUseImu\":")); client.print(moduleConfig.keyaAzUseImu);
    client.print(F(",\"azTimeSlowMs\":")); client.print(moduleConfig.keyaAzTimeSlowMs);
    client.print(F(",\"azTimeFastMs\":")); client.print(moduleConfig.keyaAzTimeFastMs);
    client.print(F(",\"emaAlpha\":")); client.print(moduleConfig.keyaEmaAlpha, 3);
    client.print(F(",\"encoderRaw\":")); client.print(keyaEncoderRaw);
    client.print(F(",\"gpsOffset\":")); client.print(keyaGpsOffset, 3);
    client.print(F(",\"wheelBase\":")); client.print(moduleConfig.wheelBase, 2);
    client.print(F(",\"deadZone\":")); client.print(moduleConfig.keyaDeadZone, 2);
    client.print(F(",\"ticksLeft\":")); client.print(moduleConfig.keyaTicksLeft, 1);
    client.print(F(",\"ticksRight\":")); client.print(moduleConfig.keyaTicksRight, 1);
    client.print(F(",\"initialZeroDone\":")); client.print(keyaInitialZeroDone ? F("true") : F("false"));

    client.print(F("},\"imu_was\":{"));
    client.print(F("\"invert\":")); client.print(moduleConfig.imuWasInvert);
    client.print(F(",\"cpdScale\":")); client.print(moduleConfig.imuWasCpdScale, 3);
    client.print(F(",\"azEnable\":")); client.print(moduleConfig.imuWasAzEnable);
    client.print(F(",\"azBeta\":")); client.print(moduleConfig.imuWasAzBeta, 4);
    client.print(F(",\"azSpeedMin\":")); client.print(moduleConfig.imuWasSpeedMin, 1);
    client.print(F(",\"azYawMax\":")); client.print(moduleConfig.imuWasYawMax, 2);
    client.print(F(",\"wheelBase\":")); client.print(moduleConfig.wheelBase, 2);

    client.print(F("},\"j1939\":{"));
    client.print(F("\"srcAddr\":")); client.print(moduleConfig.j1939SrcAddr);
    client.print(F(",\"en65267\":")); client.print(moduleConfig.j1939En65267);
    client.print(F(",\"en129029\":")); client.print(moduleConfig.j1939En129029);
    client.print(F(",\"rate65267\":")); client.print(moduleConfig.j1939Rate65267);
    client.print(F(",\"rate129029\":")); client.print(moduleConfig.j1939Rate129029);
    client.print(F(",\"fixType\":")); client.print(j1939FixType);
    client.print(F(",\"lat\":")); client.print(j1939Lat, 7);
    client.print(F(",\"lon\":")); client.print(j1939Lon, 7);

    client.print(F("},\"can_steer\":{"));
    client.print(F("\"valveReady\":")); client.print(steeringValveReady);
    client.print(F(",\"estCurve\":")); client.print(estCurve);
    client.print(F(",\"setCurve\":")); client.print(setCurve);
    client.print(F(",\"hitch\":")); client.print(ISORearHitch);
    client.print(F(",\"intend\":")); client.print(canSteerIntend ? F("true") : F("false"));
    client.print(F(",\"showData\":")); client.print(showCANData ? F("true") : F("false"));

    client.print(F("},\"cfg\":{"));
    client.print(F("\"imuType\":")); client.print(moduleConfig.imuType);
    client.print(F(",\"can1Mode\":")); client.print(moduleConfig.can1Mode);
    client.print(F(",\"can2Mode\":")); client.print(moduleConfig.can2Mode);
    client.print(F(",\"can3Mode\":")); client.print(moduleConfig.can3Mode);
    client.print(F(",\"can1Baud\":")); client.print(moduleConfig.can1Baud);
    client.print(F(",\"can2Baud\":")); client.print(moduleConfig.can2Baud);
    client.print(F(",\"can3Baud\":")); client.print(moduleConfig.can3Baud);
    client.print(F(",\"wasSource\":")); client.print(moduleConfig.wasSource);
    client.print(F(",\"rollSource\":")); client.print(moduleConfig.rollSource);
    client.print(F(",\"headingSource\":")); client.print(moduleConfig.headingSource);
    client.print(F(",\"nmeaType\":")); client.print(moduleConfig.nmeaType);
    client.print(F(",\"gpsSerial\":")); client.print(moduleConfig.gpsSerial);
    client.print(F(",\"tm171Serial\":")); client.print(moduleConfig.tm171Serial);
    client.print(F(",\"tm171Baud\":")); client.print(moduleConfig.tm171Baud);
    client.print(F(",\"steerBrand\":")); client.print(moduleConfig.steerBrand);
    client.print(F(",\"disengageType\":")); client.print(moduleConfig.disengageType);
    client.print(F(",\"debugFlags\":")); client.print(moduleConfig.debugFlags);
    client.print(F(",\"gpsBaud\":")); client.print(moduleConfig.gpsBaud);
    client.print(F(",\"hasVbus\":")); client.print(hasFuncMode(CAN_MODE_VBUS) ? F("true") : F("false"));
    client.print(F(",\"hasImu\":")); client.print(hasFuncMode(CAN_MODE_IMU)  ? F("true") : F("false"));

    client.print(F("},\"pved\":{"));
    client.print(F("\"detected\":")); client.print(pvedValveDetected ? F("true") : F("false"));
    client.print(F(",\"last64007\":")); client.print(pvedLastRead64007);
    client.print(F(",\"factory64007\":")); client.print(moduleConfig.pvedParam64007Factory);

    client.print(F("},\"custEng\":{"));
    client.print(F("\"enable\":")); client.print(moduleConfig.customEngageEnable);
    client.print(F(",\"port\":")); client.print(moduleConfig.customEngageCanPort);
    client.print(F(",\"ext\":")); client.print(moduleConfig.customEngageExt);
    client.print(F(",\"mode\":")); client.print(moduleConfig.customEngageMode);
    client.print(F(",\"id\":")); client.print(moduleConfig.customEngageId);
    client.print(F(",\"match\":["));
    for (uint8_t i = 0; i < 8; i++) { if (i) client.print(','); client.print(moduleConfig.customEngageMatch[i]); }
    client.print(F("],\"mask\":["));
    for (uint8_t i = 0; i < 8; i++) { if (i) client.print(','); client.print(moduleConfig.customEngageMask[i]); }
    client.print(F("]}}"));
}

// ── Graph: signal value lookup (IDs must match JS dropdown) ──────────────────
float getSignalValue(uint8_t id)
{
    switch (id) {
    // GPS
    case 1:  return (float)mainFixQuality;
    case 2:  return atof(numSats);
    case 3:  return atof(HDOP);
    case 4:  return atof(altitude);
    case 5:  return (float)headingVTG;
    case 6:  return gpsSpeed;
    case 7:  return atof(umHeading);
    case 8:  return atof(umRoll);
    case 9:  return (float)solQualityHPR;
    // IMU
    case 10: return yaw / 10.0f;
    case 11: return roll / 10.0f;
    case 12: return pitch / 10.0f;
    case 13: return atof(imuYawRate);
    case 14: return useTMxx_IMU ? atof(TM171_IMU.getYawStr())   : 0;
    case 15: return useTMxx_IMU ? atof(TM171_IMU.getRollStr())  : 0;
    case 16: return useTMxx_IMU ? atof(TM171_IMU.getPitchStr()) : 0;
    // WAS
    case 17: return (float)steeringPosition;
    case 18: return imuWasRawYaw;
    case 19: return imuWasRawYaw * moduleConfig.imuWasCpdScale;
    case 20: return (float)headingRate;
    case 21: return wheelAngleGPS;
    case 22: return steerAngleActual;
    // Keya
    case 23: return (float)keyaEncoderRaw;
    case 24: return keyaGpsOffset;
    case 25: return (float)keyaCurrentActualSpeed;
    case 26: return (float)keyaCurrentSetSpeed;
    // Steer
    case 27: return steerAngleActual;
    case 28: return steerAngleSetPoint;
    case 29: return steerAngleError;
    case 30: return (float)pwmDisplay;
    case 31: return gpsSpeed;
    // CAN Steer
    case 32: return (float)steeringValveReady;
    case 33: return (float)estCurve;
    case 34: return (float)setCurve;
    case 35: return (float)ISORearHitch;
    // Performance / message rate
    case 36: return loopTimeMs;
    case 37: return loopTimeMax;
    case 38: return (float)ggaIntervalMs;
    case 39: return (float)vtgIntervalMs;
    case 40: return (float)hprIntervalMs;
    // PCB sensors
    case 41: return (float)analogRead(CURRENT_SENSOR_PIN);
    case 42: return (float)analogRead(PRESSURE_SENSOR_PIN);
    case 43: return sensorReading;
    // Keya relative position (since reference zeroed on view/graph start), wheel degrees
    case 44: {
        float tpd = (moduleConfig.keyaTicksPerDeg > 1.0f) ? moduleConfig.keyaTicksPerDeg : 24.0f;
        return (float)(keyaEncoderRaw - keyaPosRef) / tpd;
    }
    // Keya steering-wheel position (relative), degrees — 65536 ticks = 1 motor turn = 360°
    case 45: return (float)(keyaEncoderRaw - keyaPosRef) * 360.0f / 65536.0f;
    default: return 0;
    }
}

void graphSampleNow()
{
    if (graphBufCount >= GRAPH_BATCH_MAX) return;  // browser fell behind — drop until polled
    for (uint8_t ch = 0; ch < 4; ch++)
        graphBuf[ch][graphBufCount] = getSignalValue(graphSig[ch]);
    graphBufCount++;
}

void handleApiGraphCfg(EthernetClient& client, const char* req)
{
    const char* p;
    if ((p = strstr(req, "d1=")) != NULL) graphSig[0] = (uint8_t)atoi(p + 3);
    if ((p = strstr(req, "d2=")) != NULL) graphSig[1] = (uint8_t)atoi(p + 3);
    if ((p = strstr(req, "d3=")) != NULL) graphSig[2] = (uint8_t)atoi(p + 3);
    if ((p = strstr(req, "d4=")) != NULL) graphSig[3] = (uint8_t)atoi(p + 3);
    if ((p = strstr(req, "rate=")) != NULL) {
        uint16_t r = (uint16_t)atoi(p + 5);
        if (r < 50) r = 50; if (r > 2000) r = 2000;
        graphRateMs = r;
    }
    graphBufCount = 0;
    graphSampleTimer = 0;
    graphActive = (strstr(req, "stop") == NULL);
    sendHeaders(client, "text/plain");
    client.print(F("OK"));
}

void handleApiGraphData(EthernetClient& client)
{
    sendHeaders(client, "application/json");
    uint8_t n = graphBufCount;
    client.print(F("{\"n\":")); client.print(n);
    client.print(F(",\"dt\":")); client.print(graphRateMs);
    client.print(F(",\"d\":["));
    for (uint8_t ch = 0; ch < 4; ch++) {
        if (ch > 0) client.print(',');
        client.print('[');
        for (uint8_t i = 0; i < n; i++) {
            if (i > 0) client.print(',');
            client.print(graphBuf[ch][i], 3);
        }
        client.print(']');
    }
    client.print(F("]}"));
    graphBufCount = 0;  // consumed
}

// ── Live group data — compact per-group JSON for Live tab ────────────────────
void handleApiGrp(EthernetClient& client, const char* req)
{
    const char* p = strstr(req, "g=");
    uint8_t g = p ? (uint8_t)atoi(p + 2) : 0;
    sendHeaders(client, "application/json");

    switch (g) {
    case 1: { // GPS
        client.print(F("{\"haveGGA\":")); client.print(GGAReadyTime < 2000 ? F("true") : F("false"));
        client.print(F(",\"haveVTG\":")); client.print(VTGReadyTime < 2000 ? F("true") : F("false"));
        client.print(F(",\"haveHPR\":")); client.print(HPRReadyTime < 2000 ? F("true") : F("false"));
        client.print(F(",\"fixT\":\""));  client.print(fixTime);   client.print('"');
        client.print(F(",\"lat\":\""));   client.print(latitude);  client.print('"');
        client.print(F(",\"ns\":\""));    client.print(latNS);     client.print('"');
        client.print(F(",\"lon\":\""));   client.print(longitude); client.print('"');
        client.print(F(",\"ew\":\""));    client.print(lonEW);     client.print('"');
        client.print(F(",\"fixQ\":"));    client.print(mainFixQuality);
        client.print(F(",\"sats\":\""));  client.print(numSats);   client.print('"');
        client.print(F(",\"hdop\":\""));  client.print(HDOP);      client.print('"');
        client.print(F(",\"alt\":\""));   client.print(altitude);  client.print('"');
        client.print(F(",\"age\":\""));   client.print(ageDGPS);   client.print('"');
        client.print(F(",\"vtgHdg\":"));  client.print(headingVTG, 1);
        client.print(F(",\"spd\":"));     client.print(gpsSpeed, 1);
        client.print(F(",\"hprHdg\":\""));  client.print(umHeading);  client.print('"');
        client.print(F(",\"hprRoll\":\"")); client.print(umRoll);     client.print('"');
        client.print(F(",\"hprQ\":\""));    client.print(solQuality); client.print('"');
        client.print(F(",\"ggaMs\":")); client.print(ggaIntervalMs);
        client.print(F(",\"vtgMs\":")); client.print(vtgIntervalMs);
        client.print(F(",\"hprMs\":")); client.print(hprIntervalMs);
        client.print(F("}"));
        break;
    }
    case 2: { // IMU
        client.print(F("{\"bno\":"));  client.print((useBNO08xI2C || useBNO08xRVC) ? F("true") : F("false"));
        client.print(F(",\"tm\":"));   client.print(useTMxx_IMU ? F("true") : F("false"));
        client.print(F(",\"bnoHdg\":"));   client.print(yaw / 10.0, 1);
        client.print(F(",\"bnoRoll\":"));  client.print(roll / 10.0, 1);
        client.print(F(",\"bnoPitch\":")); client.print(pitch / 10.0, 1);
        client.print(F(",\"bnoYaw\":\""));   client.print(imuYawRate); client.print('"');
        client.print(F(",\"tmHdg\":\""));    client.print(useTMxx_IMU ? TM171_IMU.getYawStr()   : "0"); client.print('"');
        client.print(F(",\"tmRoll\":\""));   client.print(useTMxx_IMU ? TM171_IMU.getRollStr()  : "0"); client.print('"');
        client.print(F(",\"tmPitch\":\""));  client.print(useTMxx_IMU ? TM171_IMU.getPitchStr() : "0"); client.print('"');
        client.print(F(",\"tmYaw\":\""));    client.print(useTMxx_IMU ? imuYawRate : "0"); client.print('"');
        client.print(F("}"));
        break;
    }
    case 3: { // WAS
        client.print(F("{\"src\":")); client.print(moduleConfig.wasSource);
        client.print(F(",\"adsRaw\":")); client.print(steeringPosition);
        client.print(F(",\"imuRaw\":")); client.print(imuWasRawYaw, 2);
        client.print(F(",\"imuScale\":")); client.print(moduleConfig.imuWasCpdScale, 2);
        client.print(F(",\"imuScaled\":")); client.print(imuWasRawYaw * moduleConfig.imuWasCpdScale, 2);
        client.print(F(",\"yawRate\":")); client.print(headingRate, 2);
        client.print(F(",\"wheelGps\":")); client.print(wheelAngleGPS, 2);
        client.print(F(",\"actual\":")); client.print(steerAngleActual, 2);
        client.print(F("}"));
        break;
    }
    case 4: { // Keya
        client.print(F("{\"det\":")); client.print(keyaDetected ? F("true") : F("false"));
        client.print(F(",\"zero\":")); client.print(keyaInitialZeroDone ? F("true") : F("false"));
        client.print(F(",\"enc\":")); client.print(keyaEncoderRaw);
        client.print(F(",\"relTicks\":")); client.print(keyaEncoderRaw - keyaPosRef);
        client.print(F(",\"relPos\":")); client.print((keyaEncoderRaw - keyaPosRef) / ((moduleConfig.keyaTicksPerDeg > 1.0f) ? moduleConfig.keyaTicksPerDeg : 24.0f), 2);
        client.print(F(",\"swPos\":")); client.print((keyaEncoderRaw - keyaPosRef) * 360.0f / 65536.0f, 1);
        client.print(F(",\"zTicks\":")); client.print(moduleConfig.keyaZeroTicks);
        client.print(F(",\"off\":")); client.print(keyaGpsOffset, 3);
        client.print(F(",\"act\":")); client.print(keyaCurrentActualSpeed);
        client.print(F(",\"set\":")); client.print(keyaCurrentSetSpeed);
        client.print(F(",\"actual\":")); client.print(steerAngleActual, 2);
        client.print(F("}"));
        break;
    }
    case 5: { // Steer
        client.print(F("{\"actual\":")); client.print(steerAngleActual, 2);
        client.print(F(",\"setpt\":")); client.print(steerAngleSetPoint, 2);
        client.print(F(",\"err\":")); client.print(steerAngleError, 2);
        client.print(F(",\"pwm\":")); client.print(pwmDisplay);
        client.print(F(",\"on\":")); client.print((watchdogTimer < WATCHDOG_THRESHOLD) ? F("true") : F("false"));
        client.print(F(",\"spd\":")); client.print(gpsSpeed, 1);
        // AOG switches — raw pin reads (active-low: 0 = pressed/closed, 1 = open)
        client.print(F(",\"steerPin\":")); client.print(digitalRead(STEERSW_PIN));
        client.print(F(",\"workPin\":"));  client.print(digitalRead(WORKSW_PIN));
        client.print(F(",\"remotePin\":"));client.print(digitalRead(REMOTE_PIN));
        // On-PCB analog sensors
        client.print(F(",\"curRaw\":"));  client.print(analogRead(CURRENT_SENSOR_PIN));
        client.print(F(",\"presRaw\":")); client.print(analogRead(PRESSURE_SENSOR_PIN));
        client.print(F(",\"sensorRd\":")); client.print(sensorReading, 0);
        client.print(F("}"));
        break;
    }
    case 6: { // CAN Steer
        client.print(F("{\"vReady\":")); client.print(steeringValveReady);
        client.print(F(",\"eCurve\":")); client.print(estCurve);
        client.print(F(",\"sCurve\":")); client.print(setCurve);
        client.print(F(",\"hitch\":")); client.print(ISORearHitch);
        client.print(F(",\"intend\":")); client.print(canSteerIntend ? F("true") : F("false"));
        client.print(F("}"));
        break;
    }
    default:
        client.print(F("{}"));
        break;
    }
}

void handleApiLive(EthernetClient& client)
{
    sendHeaders(client, "application/json");
    client.print(F("{"));
    client.print(F("\"angle\":")); client.print(steerAngleActual, 2);
    client.print(F(",\"setpt\":")); client.print(steerAngleSetPoint, 2);
    client.print(F(",\"spd\":")); client.print(gpsSpeed, 1);
    client.print(F(",\"on\":")); client.print((watchdogTimer < WATCHDOG_THRESHOLD) ? F("true") : F("false"));
    client.print(F(",\"kAct\":")); client.print(keyaCurrentActualSpeed);
    client.print(F(",\"kSet\":")); client.print(keyaCurrentSetSpeed);
    client.print(F(",\"gps\":")); client.print(GGAReadyTime < 10000 ? F("true") : F("false"));
    client.print(F(",\"gga\":")); client.print(GGAReadyTime < 10000 ? F("true") : F("false"));
    client.print(F(",\"vtg\":")); client.print(VTGReadyTime < 10000 ? F("true") : F("false"));
    client.print(F(",\"hpr\":")); client.print(HPRReadyTime < 10000 ? F("true") : F("false"));
    client.print(F(",\"bno\":")); client.print(useBNO08xI2C ? F("true") : F("false"));
    client.print(F(",\"tm\":")); client.print(useTMxx_IMU ? F("true") : F("false"));
    client.print(F(",\"keya\":")); client.print(keyaDetected ? F("true") : F("false"));
    client.print(F(",\"kZero\":")); client.print(keyaInitialZeroDone ? F("true") : F("false"));
    client.print(F(",\"kEnc\":")); client.print(keyaEncoderRaw);
    client.print(F(",\"kRelTicks\":")); client.print(keyaEncoderRaw - keyaPosRef);
    client.print(F(",\"kRelPos\":")); client.print((keyaEncoderRaw - keyaPosRef) / ((moduleConfig.keyaTicksPerDeg > 1.0f) ? moduleConfig.keyaTicksPerDeg : 24.0f), 2);
    client.print(F(",\"kOff\":")); client.print(keyaGpsOffset, 3);
    client.print(F(",\"ads\":")); client.print(adcConnected ? F("true") : F("false"));
    client.print(F(",\"iWas\":")); client.print((imuWasReceived && imuWasTimeout < 500) ? F("true") : F("false"));
    client.print(F(",\"vReady\":")); client.print(steeringValveReady);
    client.print(F(",\"eCurve\":")); client.print(estCurve);
    client.print(F(",\"sCurve\":")); client.print(setCurve);
    client.print(F(",\"hitch\":")); client.print(ISORearHitch);
    client.print(F(",\"intend\":")); client.print(canSteerIntend ? F("true") : F("false"));
    client.print(F(",\"showData\":")); client.print(showCANData ? F("true") : F("false"));
    // Custom engage live + AOG steer state
    client.print(F(",\"ceMatch\":")); client.print(customEngMatch ? F("true") : F("false"));
    client.print(F(",\"ceSeen\":")); client.print(customEngSeen ? F("true") : F("false"));
    client.print(F(",\"ceBuf\":["));
    for (uint8_t i = 0; i < 8; i++) { if (i) client.print(','); client.print(customEngLastBuf[i]); }
    client.print(F("]"));
    client.print(F(",\"aogGuidance\":")); client.print(guidanceStatus);
    client.print(F(",\"aogSteerSw\":")); client.print(steerSwitch);
    // Reference IMU (calib bridge)
    client.print(F(",\"refAngle\":")); client.print(refWheelAngle, 2);
    client.print(F(",\"refFresh\":")); client.print((refAngleTime < 1000 && refAngleValid) ? F("true") : F("false"));
    client.print(F(",\"calState\":")); client.print(calState);
    client.print(F(",\"calMsg\":\"")); client.print(calMsg); client.print('"');
    client.print(F(",\"calSpeed\":")); client.print(calSpeed);
    client.print(F(",\"calDz\":")); client.print(calResDz, 2);
    client.print(F(",\"calTL\":")); client.print(calResTL, 1);
    client.print(F(",\"calTR\":")); client.print(calResTR, 1);
    client.print(F(",\"calTpd\":")); client.print(calResTpd, 1);
    client.print(F(",\"kcMode\":")); client.print(keyaCfgMode ? F("true") : F("false"));
    client.print(F(",\"kcRam\":["));
    for (uint8_t i = 0; i < 4; i++) { if (i) client.print(','); client.print(keyaCfgRam[i]); }
    client.print(F("],\"kcRom\":["));
    for (uint8_t i = 0; i < 4; i++) { if (i) client.print(','); client.print(keyaCfgRom[i]); }
    client.print(F("]"));
    client.print(F(",\"kp\":")); client.print(steerSettings.Kp);
    client.print(F(",\"hiPWM\":")); client.print(steerSettings.highPWM);
    client.print(F(",\"loPWM\":")); client.print(steerSettings.lowPWM);
    client.print(F(",\"minPWM\":")); client.print(steerSettings.minPWM);
    client.print(F(",\"cnt\":")); client.print((int)steerSettings.steerSensorCounts);
    client.print(F(",\"wasOff\":")); client.print(steerSettings.wasOffset);
    client.print(F(",\"invWAS\":")); client.print(steerConfig.InvertWAS);
    client.print(F(",\"cytron\":")); client.print(steerConfig.CytronDriver);
    client.print(F(",\"shaftEnc\":")); client.print(steerConfig.ShaftEncoder);
    client.print(F(",\"pres\":")); client.print(steerConfig.PressureSensor);
    client.print(F(",\"curr\":")); client.print(steerConfig.CurrentSensor);
    client.print(F(",\"danf\":")); client.print(steerConfig.IsDanfoss);
    client.print(F(",\"j19Fix\":")); client.print(j1939FixType);
    client.print(F(",\"j19Lat\":")); client.print(j1939Lat, 7);
    client.print(F(",\"j19Lon\":")); client.print(j1939Lon, 7);
    client.print(F(",\"pvedDet\":")); client.print(pvedValveDetected ? F("true") : F("false"));
    client.print(F(",\"pvedLast\":")); client.print(pvedLastRead64007);
    client.print(F(",\"pvedFac\":")); client.print(moduleConfig.pvedParam64007Factory);
    client.print(F(",\"logOn\":")); client.print(logActive ? F("true") : F("false"));
    client.print(F("}"));
}

void handleApiLog(EthernetClient& client, const char* req)
{
    if (strstr(req, "active=1") != NULL) {
        logActive = true;
        sendHeaders(client, "text/plain");
        client.print(F("OK"));
        return;
    }
    if (strstr(req, "active=0") != NULL) {
        logActive = false;
        sendHeaders(client, "text/plain");
        client.print(F("OK"));
        return;
    }
    if (strstr(req, "clear=1") != NULL) {
        logWrite   = 0;
        logWrapped = false;
        sendHeaders(client, "text/plain");
        client.print(F("OK"));
        return;
    }
    sendHeaders(client, "text/plain; charset=utf-8");
    if (logWrapped) {
        // oldest data first: from logWrite to end, then 0 to logWrite-1
        sendBuf(client, (const uint8_t*)logBuf + logWrite, LOG_BUF_SIZE - logWrite);
        sendBuf(client, (const uint8_t*)logBuf, logWrite);
    } else if (logWrite > 0) {
        sendBuf(client, (const uint8_t*)logBuf, logWrite);
    }
}

void handleApiKeyaZero(EthernetClient& client)
{
    moduleConfig.keyaZeroTicks = keyaEncoderRaw;
    moduleConfigSave();
    Serial.print("Keya WAS zero set at "); Serial.println(keyaEncoderRaw);
    webLogf("Keya WAS zero set at %ld ticks", (long)keyaEncoderRaw);
    sendHeaders(client, "text/plain");
    client.print(F("OK"));
}

void handleApiImuWasZero(EthernetClient& client)
{
    imuWasZeroRequest = true;
    Serial.println("IMU WAS: zero request");
    webLog("IMU WAS: zero request");
    sendHeaders(client, "text/plain");
    client.print(F("OK"));
}

void handleApiGpsRaw(EthernetClient& client, const char* req)
{
    if (strstr(req, "active=1") != NULL) {
        gpsRawActive = true;
        sendHeaders(client, "text/plain");
        client.print(F("OK"));
        return;
    }
    if (strstr(req, "active=0") != NULL) {
        gpsRawActive = false;
        sendHeaders(client, "text/plain");
        client.print(F("OK"));
        return;
    }
    if (strstr(req, "clear=1") != NULL) {
        gpsRawWrite   = 0;
        gpsRawWrapped = false;
        sendHeaders(client, "text/plain");
        client.print(F("OK"));
        return;
    }
    sendHeaders(client, "text/plain; charset=utf-8");
    if (gpsRawWrapped) {
        sendBuf(client, (const uint8_t*)gpsRawBuf + gpsRawWrite, GPS_RAW_BUF_SIZE - gpsRawWrite);
        sendBuf(client, (const uint8_t*)gpsRawBuf, gpsRawWrite);
    } else if (gpsRawWrite > 0) {
        sendBuf(client, (const uint8_t*)gpsRawBuf, gpsRawWrite);
    }
}

static uint8_t hexNib(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

static void urlDecode(char* dst, const char* src, size_t maxLen)
{
    size_t i = 0;
    while (*src && i < maxLen - 1) {
        if (*src == '%' && *(src+1) && *(src+2)) {
            *dst++ = (char)((hexNib(*(src+1)) << 4) | hexNib(*(src+2)));
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
        i++;
    }
    *dst = '\0';
}

void handleApiGpsBaud(EthernetClient& client, const char* req)
{
    static const uint32_t validBauds[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
    const char* p = strstr(req, "baud=");
    if (p) {
        uint32_t nb = (uint32_t)atol(p + 5);
        bool ok = false;
        for (uint8_t i = 0; i < 8; i++) if (validBauds[i] == nb) { ok = true; break; }
        if (ok) {
            moduleConfig.gpsBaud = nb;
            moduleConfigSave();
            SerialGPS->begin(nb);
            Serial.print("GPS baud: "); Serial.println(nb);
            webLogf("GPS baud changed to %lu", nb);
        }
    }
    sendHeaders(client, "text/plain");
    client.print(F("OK"));
}

void handleApiGpsCmd(EthernetClient& client, const char* req)
{
    const char* p = strstr(req, "line=");
    if (p) {
        p += 5;
        char raw[160];
        int i = 0;
        while (*p && *p != ' ' && *p != '\r' && i < 159) raw[i++] = *p++;
        raw[i] = '\0';
        char decoded[160];
        urlDecode(decoded, raw, sizeof(decoded));
        SerialGPS->print(decoded);
        SerialGPS->print("\r\n");
        Serial.print("GPS cmd: "); Serial.println(decoded);
        webLogf("GPS cmd: %s", decoded);
    }
    sendHeaders(client, "text/plain");
    client.print(F("OK"));
}

void handleApiCanRaw(EthernetClient& client, const char* req)
{
    if (strstr(req, "show=1") != NULL) {
        showCANData = true;
        sendHeaders(client, "text/plain");
        client.print(F("OK"));
        return;
    }
    if (strstr(req, "show=0") != NULL) {
        showCANData = false;
        sendHeaders(client, "text/plain");
        client.print(F("OK"));
        return;
    }
    if (strstr(req, "clear=1") != NULL) {
        canRawWrite   = 0;
        canRawWrapped = false;
        sendHeaders(client, "text/plain");
        client.print(F("OK"));
        return;
    }
    sendHeaders(client, "text/plain; charset=utf-8");
    if (canRawWrapped) {
        sendBuf(client, (const uint8_t*)canRawBuf + canRawWrite, CAN_RAW_BUF_SIZE - canRawWrite);
        sendBuf(client, (const uint8_t*)canRawBuf, canRawWrite);
    } else if (canRawWrite > 0) {
        sendBuf(client, (const uint8_t*)canRawBuf, canRawWrite);
    }
}

void handleApiCanScan(EthernetClient& client, const char* req)
{
    if (strstr(req, "start") != NULL) {
        memset(canScanBuf, 0, sizeof(canScanBuf));
        canScanCount = 0;
        canScanActive = true;
        sendHeaders(client, "text/plain");
        client.print(F("OK"));
        return;
    }
    if (strstr(req, "stop") != NULL) {
        canScanActive = false;
        sendHeaders(client, "text/plain");
        client.print(F("OK"));
        return;
    }
    // data
    sendHeaders(client, "application/json");
    client.print(F("{\"active\":"));
    client.print(canScanActive ? F("true") : F("false"));
    client.print(F(",\"count\":"));
    client.print(canScanCount);
    client.print(F(",\"entries\":["));
    for (uint8_t i = 0; i < canScanCount; i++) {
        if (i > 0) client.print(',');
        char hex[12];
        snprintf(hex, sizeof(hex), "0x%08lX", (unsigned long)canScanBuf[i].id);
        client.print(F("{\"id\":\""));  client.print(hex);
        client.print(F("\",\"bus\":\""));client.print(canScanBuf[i].bus);
        client.print(F("\",\"n\":"));   client.print(canScanBuf[i].count);
        client.print('}');
    }
    client.print(F("]}"));
}

void handleApiPved(EthernetClient& client, const char* req)
{
    if (strstr(req, "cmd=readall") != NULL) {
        pvedReadAll();
        showCANData = true;
    } else if (strstr(req, "cmd=read64007") != NULL) {
        pvedReadParam(64007);
        showCANData = true;
    } else if (strstr(req, "cmd=write") != NULL) {
        pvedWriteParam();
    } else if (strstr(req, "cmd=commit") != NULL) {
        pvedCommit();
    } else if (strstr(req, "cmd=restore") != NULL) {
        pvedRestoreParam64007();
    }
    sendHeaders(client, "text/plain");
    client.print(F("OK"));
}

void handleApiCalib(EthernetClient& client, const char* req)
{
    const char* p;
    if ((p = strstr(req, "speed=")) != NULL) {
        uint8_t s = (uint8_t)atoi(p + 6);
        if (s < 5) s = 5; if (s > 80) s = 80;
        calSpeed = s;
    }
    if      (strstr(req, "start") != NULL) calStart();
    else if (strstr(req, "abort") != NULL) calAbort();
    else if (strstr(req, "apply") != NULL) calApply();
    sendHeaders(client, "text/plain");
    client.print(F("OK"));
}

void handleApiKeyaCfg(EthernetClient& client, const char* req)
{
    const char* p;
    if      (strstr(req, "cmd=enter")   != NULL) keyaCfgEnter();
    else if (strstr(req, "cmd=exit")    != NULL) keyaCfgExit();
    else if (strstr(req, "cmd=store")   != NULL) keyaCfgStore();
    else if (strstr(req, "cmd=disable") != NULL) keyaCfgTestStop();
    else if (strstr(req, "cmd=write")   != NULL) {
        uint8_t id = 0, val = 0;
        if ((p = strstr(req, "id="))  != NULL) id  = (uint8_t)atoi(p + 3);
        if ((p = strstr(req, "val=")) != NULL) val = (uint8_t)atoi(p + 4);
        if (id) keyaCfgWrite(id, val);
    }
    else if (strstr(req, "cmd=test")    != NULL) {
        int8_t dir = (strstr(req, "dir=L") != NULL) ? -1 : +1;
        int16_t sp = 0;
        if ((p = strstr(req, "speed=")) != NULL) sp = (int16_t)atoi(p + 6);
        keyaCfgTest(dir, sp);
    }
    sendHeaders(client, "text/plain");
    client.print(F("OK"));
}

void handleApiSave(EthernetClient& client, const char* req)
{
    bool needRestart = false;
    const char* p;

    if ((p = strstr(req, "imuType=")) != NULL) {
        moduleConfig.imuType = (uint8_t)atoi(p + 8);
        needRestart = true;
    }
    if ((p = strstr(req, "can1Mode=")) != NULL) {
        moduleConfig.can1Mode = (uint8_t)atoi(p + 9);
        needRestart = true;
    }
    if ((p = strstr(req, "can2Mode=")) != NULL) {
        moduleConfig.can2Mode = (uint8_t)atoi(p + 9);
        needRestart = true;
    }
    if ((p = strstr(req, "can3Mode=")) != NULL) {
        moduleConfig.can3Mode = (uint8_t)atoi(p + 9);
        needRestart = true;
    }
    if ((p = strstr(req, "brand=")) != NULL) {
        moduleConfig.steerBrand = (uint8_t)atoi(p + 6);
        needRestart = true;
    }
    if ((p = strstr(req, "disengage=")) != NULL) {
        moduleConfig.disengageType = (uint8_t)atoi(p + 10);
        needRestart = true;
    }
    if ((p = strstr(req, "debugFlags=")) != NULL) {
        moduleConfig.debugFlags = (uint8_t)atoi(p + 11);
    }
    if ((p = strstr(req, "keyaDis=")) != NULL) {
        moduleConfig.keyaDisEnable = (uint8_t)atoi(p + 8);
    }
    if ((p = strstr(req, "keyaSet=")) != NULL) {
        moduleConfig.keyaSetSpeedMin = (uint8_t)atoi(p + 8);
    }
    if ((p = strstr(req, "keyaAct=")) != NULL) {
        moduleConfig.keyaActSpeedMin = (uint8_t)atoi(p + 8);
    }
    if ((p = strstr(req, "motorDis=")) != NULL) {
        moduleConfig.motorDisEnable = (uint8_t)atoi(p + 9);
    }
    if ((p = strstr(req, "motorAngleMin=")) != NULL) {
        moduleConfig.motorAngleErrorMin = (uint8_t)atoi(p + 14);
    }
    if ((p = strstr(req, "speedDiffTimeout=")) != NULL)
        moduleConfig.speedDiffTimeout = (uint16_t)atoi(p + 17);
    if ((p = strstr(req, "keyaTicksPD="))  != NULL) moduleConfig.keyaTicksPerDeg = atof(p + 12);
    if ((p = strstr(req, "keyaEncInv="))   != NULL) moduleConfig.keyaEncInvert   = (uint8_t)atoi(p + 11);
    if ((p = strstr(req, "keyaAzEn="))     != NULL) moduleConfig.keyaAzEnable    = (uint8_t)atoi(p + 9);
    if ((p = strstr(req, "keyaAzBeta="))   != NULL) moduleConfig.keyaAzBeta      = atof(p + 11);
    if ((p = strstr(req, "keyaAzVmin="))   != NULL) moduleConfig.keyaAzSpeedMin  = atof(p + 11);
    if ((p = strstr(req, "keyaAzYawMax=")) != NULL) moduleConfig.keyaAzYawMax    = atof(p + 13);
    if ((p = strstr(req, "keyaAzVslow="))  != NULL) moduleConfig.keyaAzSpeedSlow = (uint8_t)atoi(p + 12);
    if ((p = strstr(req, "keyaAzVfast="))  != NULL) moduleConfig.keyaAzSpeedFast = atof(p + 12);
    if ((p = strstr(req, "keyaAzUseImu=")) != NULL) moduleConfig.keyaAzUseImu    = (uint8_t)atoi(p + 13);
    if ((p = strstr(req, "keyaAzTslow="))  != NULL) moduleConfig.keyaAzTimeSlowMs = (uint16_t)atoi(p + 12);
    if ((p = strstr(req, "keyaAzTfast="))  != NULL) moduleConfig.keyaAzTimeFastMs = (uint16_t)atoi(p + 12);
    if ((p = strstr(req, "keyaEmaA="))     != NULL) moduleConfig.keyaEmaAlpha      = atof(p + 9);
    if ((p = strstr(req, "wheelBase="))    != NULL) moduleConfig.wheelBase         = atof(p + 10);
    if ((p = strstr(req, "keyaDeadZone=")) != NULL) moduleConfig.keyaDeadZone     = atof(p + 13);
    if ((p = strstr(req, "keyaTicksLeft=")) != NULL) moduleConfig.keyaTicksLeft    = atof(p + 14);
    if ((p = strstr(req, "keyaTicksRight=")) != NULL) moduleConfig.keyaTicksRight  = atof(p + 15);
    if ((p = strstr(req, "can2Baud="))     != NULL) {
        moduleConfig.can2Baud = (uint32_t)atol(p + 9);
        needRestart = true;
    }
    if ((p = strstr(req, "can3Baud="))     != NULL) {
        moduleConfig.can3Baud = (uint32_t)atol(p + 9);
        needRestart = true;
    }
    if ((p = strstr(req, "wasSource="))     != NULL) { moduleConfig.wasSource      = (uint8_t)atoi(p + 10); needRestart = true; }
    if ((p = strstr(req, "rollSource="))    != NULL) { moduleConfig.rollSource     = (uint8_t)atoi(p + 11); needRestart = true; }
    if ((p = strstr(req, "headingSource=")) != NULL) { moduleConfig.headingSource  = (uint8_t)atoi(p + 14); needRestart = true; }
    if ((p = strstr(req, "nmeaType="))      != NULL) { moduleConfig.nmeaType       = (uint8_t)atoi(p + 9);  needRestart = true; }
    if ((p = strstr(req, "gpsSerial="))     != NULL) { moduleConfig.gpsSerial      = (uint8_t)atoi(p + 10); needRestart = true; }
    if ((p = strstr(req, "tm171Serial="))   != NULL) { moduleConfig.tm171Serial    = (uint8_t)atoi(p + 12); needRestart = true; }
    if ((p = strstr(req, "tm171Baud="))     != NULL) { moduleConfig.tm171Baud      = (uint32_t)atol(p + 10); needRestart = true; }
    if ((p = strstr(req, "gpsBaudR="))      != NULL) { moduleConfig.gpsBaud        = (uint32_t)atol(p + 9);  needRestart = true; }
    if ((p = strstr(req, "ceEnable="))      != NULL) { moduleConfig.customEngageEnable  = (uint8_t)atoi(p + 9);  needRestart = true; }
    if ((p = strstr(req, "cePort="))        != NULL) { moduleConfig.customEngageCanPort = (uint8_t)atoi(p + 7);  needRestart = true; }
    if ((p = strstr(req, "ceExt="))         != NULL) moduleConfig.customEngageExt  = (uint8_t)atoi(p + 6);
    if ((p = strstr(req, "ceMode="))        != NULL) moduleConfig.customEngageMode = (uint8_t)atoi(p + 7);
    if ((p = strstr(req, "ceId="))          != NULL) moduleConfig.customEngageId   = (uint32_t)strtoul(p + 5, NULL, 16);
    if ((p = strstr(req, "ceVal=")) != NULL) {        // 8 hex bytes, comma-separated
        const char* q = p + 6;
        for (uint8_t i = 0; i < 8 && q && *q; i++) {
            moduleConfig.customEngageMatch[i] = (uint8_t)strtoul(q, NULL, 16);
            q = strchr(q, ','); if (q) q++;
        }
    }
    if ((p = strstr(req, "ceMsk=")) != NULL) {
        const char* q = p + 6;
        for (uint8_t i = 0; i < 8 && q && *q; i++) {
            moduleConfig.customEngageMask[i] = (uint8_t)strtoul(q, NULL, 16);
            q = strchr(q, ','); if (q) q++;
        }
    }
    if ((p = strstr(req, "j19Addr="))     != NULL) moduleConfig.j1939SrcAddr    = (uint8_t)atoi(p + 9);
    if ((p = strstr(req, "j19En65267="))  != NULL) moduleConfig.j1939En65267    = (uint8_t)atoi(p + 12);
    if ((p = strstr(req, "j19R65267="))   != NULL) moduleConfig.j1939Rate65267  = (uint16_t)atoi(p + 11);
    if ((p = strstr(req, "j19En129029=")) != NULL) moduleConfig.j1939En129029   = (uint8_t)atoi(p + 13);
    if ((p = strstr(req, "j19R129029="))  != NULL) moduleConfig.j1939Rate129029 = (uint16_t)atoi(p + 12);
    if ((p = strstr(req, "imuWasInv="))   != NULL) moduleConfig.imuWasInvert    = (uint8_t)atoi(p + 10);
    if ((p = strstr(req, "imuWasCpd="))   != NULL) moduleConfig.imuWasCpdScale = atof(p + 10);
    if ((p = strstr(req, "imuWasAzEn="))  != NULL) moduleConfig.imuWasAzEnable = (uint8_t)atoi(p + 11);
    if ((p = strstr(req, "imuWasAzB="))   != NULL) moduleConfig.imuWasAzBeta   = atof(p + 10);
    if ((p = strstr(req, "imuWasVmin="))  != NULL) moduleConfig.imuWasSpeedMin = atof(p + 11);
    if ((p = strstr(req, "imuWasYaw="))   != NULL) moduleConfig.imuWasYawMax   = atof(p + 10);

    moduleConfigSave();

    sendHeaders(client, "text/plain");
    client.print(F("OK"));

    if (needRestart) {
        Serial.println("Config saved – restart pending...");
        webLog("Config saved - restarting...");
        pendingRestart = true;
        restartDelay   = 0;
    } else {
        Serial.print("Debug flags: 0x");
        Serial.println(moduleConfig.debugFlags, HEX);
        webLogf("Debug flags: 0x%02X", moduleConfig.debugFlags);
    }
}

#include <M5Unified.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include "secrets.h"

#define ROVERC_I2C_ADDR 0x38
#define CMD_TIMEOUT_MS  500

class RoverC {
public:
  void begin() {
    delay(10);
    Wire.beginTransmission(ROVERC_I2C_ADDR);
    Wire.endTransmission();
  }
  bool setMotor(uint8_t m, int8_t s) {
    if (m < 1 || m > 4) return false;
    Wire.beginTransmission(ROVERC_I2C_ADDR);
    Wire.write(m - 1);
    Wire.write(s);
    return Wire.endTransmission() == 0;
  }
  void stop() { for (int i = 1; i <= 4; i++) setMotor(i, 0); }

  void setSpeed(int8_t x, int8_t y, int8_t z) {
    int16_t m1 = (int16_t)y + x + z;
    int16_t m2 = (int16_t)y - x - z;
    int16_t m3 = (int16_t)y - x + z;
    int16_t m4 = (int16_t)y + x - z;
    int16_t mabs = max(max(abs(m1), abs(m2)), max(abs(m3), abs(m4)));
    if (mabs > 127) {
      float s = 127.0f / mabs;
      m1 = round(m1 * s); m2 = round(m2 * s);
      m3 = round(m3 * s); m4 = round(m4 * s);
    }
    setMotor(1, m1); setMotor(2, m2);
    setMotor(3, m3); setMotor(4, m4);
  }
};

RoverC rover;
WebServer server(80);

String apSSID;
String apURL;
String staIP;
String staSSID;
bool useApMode = true;

int8_t cmdX = 0, cmdY = 0, cmdZ = 0;
unsigned long lastCmdMs = 0;
unsigned long lastPrintMs = 0;
unsigned long lastMotorMs = 0;
int8_t lastSentX = 127, lastSentY = 127, lastSentZ = 127;

static const char PROGMEM PAGE_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<title>RoverC</title>
<style>
*{margin:0;padding:0;box-sizing:border-box;touch-action:none;-webkit-user-select:none;user-select:none;-webkit-tap-highlight-color:transparent;-webkit-text-size-adjust:100%}
html,body{height:100dvh;width:100dvw;background:#0b0b10;color:#fff;font-family:system-ui,-apple-system,BlinkMacSystemFont,sans-serif;overflow:hidden}
#portrait{display:none;height:100%;width:100%;flex-direction:column;align-items:center;justify-content:center;text-align:center;padding:20px}
#portrait .icon{font-size:48px;margin-bottom:12px;animation:rot 1.5s ease-in-out infinite}
@keyframes rot{0%{transform:rotate(0)}50%{transform:rotate(90deg)}100%{transform:rotate(90deg)}}
#portrait h2{font-size:20px;margin-bottom:8px;color:#ff2d55}
#portrait p{font-size:14px;color:#8e8e93;line-height:1.5}
#landscape{height:100%;width:100%;display:none;flex-direction:column}
#h{height:44px;display:flex;align-items:center;justify-content:space-between;padding:0 18px;background:rgba(255,255,255,0.03);border-bottom:1px solid rgba(255,255,255,0.08);flex-shrink:0}
#t{font-weight:700;color:#ff2d55;font-size:16px;letter-spacing:1px}
#st{font-size:12px;color:#00e676;display:flex;align-items:center;gap:6px}
#st::before{content:"";display:inline-block;width:8px;height:8px;border-radius:50%;background:#00e676;box-shadow:0 0 8px #00e676}
#m{flex:1;display:flex;align-items:center;justify-content:space-around;min-height:0;padding:12px 16px;gap:16px}
.pad{display:flex;flex-direction:column;align-items:center;gap:10px;flex:1;max-width:260px}
.lbl{font-size:11px;color:#8e8e93;text-transform:uppercase;letter-spacing:1.5px;font-weight:600}
.joy{width:34vmin;height:34vmin;max-width:170px;max-height:170px;border-radius:50%;background:radial-gradient(circle at 50% 50%,#1c1c26 0%,#111118 100%);border:2px solid rgba(255,255,255,0.12);position:relative;box-shadow:inset 0 0 24px rgba(0,0,0,0.5),0 8px 32px rgba(0,0,0,0.35)}
.joy::before{content:"";position:absolute;top:50%;left:10%;width:80%;height:1px;background:rgba(255,255,255,0.08);transform:translateY(-50%)}
.joy::after{content:"";position:absolute;top:10%;left:50%;width:1px;height:80%;background:rgba(255,255,255,0.08);transform:translateX(-50%)}
.knob{width:34%;height:34%;border-radius:50%;background:linear-gradient(145deg,#ff2d55,#ff5e3a);box-shadow:0 4px 20px rgba(255,45,85,0.45);position:absolute;top:33%;left:33%;pointer-events:none}
#vals{font-family:monospace;font-size:12px;color:#8e8e93;text-align:center;padding:8px 0 10px;background:rgba(255,255,255,0.02);flex-shrink:0;letter-spacing:0.5px}
@media (orientation:portrait){#portrait{display:flex}#landscape{display:none}}
@media (orientation:landscape){#portrait{display:none}#landscape{display:flex}}
</style>
</head>
<body>
<div id="portrait">
  <div class="icon">📱</div>
  <h2>Rotate to Landscape</h2>
  <p>Please hold your phone horizontally<br>for the dual joystick controller.</p>
</div>
<div id="landscape">
  <div id="h"><span id="t">ROVER C</span><span id="st">LINK</span></div>
  <div id="m">
    <div class="pad"><div class="lbl">Move</div><div class="joy" id="joyL"><div class="knob"></div></div></div>
    <div class="pad"><div class="lbl">Turn</div><div class="joy" id="joyR"><div class="knob"></div></div></div>
  </div>
  <div id="vals">x:0 y:0 z:0</div>
</div>
<script>
(function(){'use strict';
var ax=0,ay=0,az=0,seq=0;
var st=document.getElementById('st'),vals=document.getElementById('vals');
function send(){
  var url='/cmd?x='+ax+'&y='+ay+'&z='+az+'&s='+(++seq);
  vals.textContent='x:'+ax+' y:'+ay+' z:'+az;
  fetch(url,{cache:'no-store'}).catch(function(e){console.log('fetch err',e);});
}
function bind(id,hAxis,vAxis){
  var el=document.getElementById(id),knob=el.querySelector('.knob');
  function move(dx,dy){
    var r=el.offsetWidth/2,max=r*0.62,d=Math.sqrt(dx*dx+dy*dy),l=Math.min(d,max)/max,a=Math.atan2(dy,dx);
    var px=Math.cos(a)*l*max,py=Math.sin(a)*l*max;
    knob.style.left=(r+px-knob.offsetWidth/2)+'px';
    knob.style.top=(r+py-knob.offsetHeight/2)+'px';
    var vx=Math.round(Math.cos(a)*l*100)||0,vy=Math.round(-Math.sin(a)*l*100)||0;
    if(hAxis==='x')ax=vx;if(hAxis==='z')az=vx;if(vAxis==='y')ay=vy;
    send();
  }
  function h(e){e.preventDefault();var t=e.touches[0],r=el.getBoundingClientRect();move(t.clientX-(r.left+r.width/2),t.clientY-(r.top+r.height/2));}
  function reset(){st.style.color='#00e676';move(0,0);}
  el.addEventListener('touchstart',function(e){st.style.color='#ff2d55';h(e);});
  el.addEventListener('touchmove',h,{passive:false});
  el.addEventListener('touchend',reset);
  el.addEventListener('touchcancel',reset);
}
bind('joyL','x','y');
bind('joyR','z',null);
setInterval(function(){send();},100);
send();
})();
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", PAGE_HTML);
}

void handleCmd() {
  lastCmdMs = millis();
  if (server.hasArg("x")) cmdX = constrain(server.arg("x").toInt(), -100, 100);
  if (server.hasArg("y")) cmdY = constrain(server.arg("y").toInt(), -100, 100);
  if (server.hasArg("z")) cmdZ = constrain(server.arg("z").toInt(), -100, 100);
  unsigned long seq = server.hasArg("s") ? server.arg("s").toInt() : 0;
  Serial.printf("[CMD] seq:%lu x:%d y:%d z:%d\n", seq, cmdX, cmdY, cmdZ);
  server.send(200, "text/plain", "OK");
}

void handleNotFound() {
  server.send(200, "text/html", PAGE_HTML);
}

void updateDisplay() {
  auto& d = M5.Display;
  d.fillScreen(WHITE);
  d.setTextSize(1);
  d.setTextColor(BLACK, WHITE);
  // Title row
  d.setCursor(4, 2);
  d.print("RoverC");
  d.setTextColor(ORANGE, WHITE);
  d.drawString("[AP]", 70, 2);
  // AP SSID
  d.setTextColor(BLACK, WHITE);
  d.setCursor(4, 12);
  d.print("SSID: ");
  d.print(apSSID);
  // STA IP (if connected)
  if (staIP.length()) {
    d.setCursor(172, 2);
    d.setTextColor(GREEN, WHITE);
    d.drawString(staIP, 172, 2);
  }
  // URL at bottom
  d.setCursor(4, 128);
  d.setTextColor(BLUE, WHITE);
  d.print(apURL);
  d.qrcode(apURL.c_str(), -1, 16, 112, 1, true);
}

void setup() {
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  Serial.begin(115200);
  delay(200);
  Serial.println("\n\n=== RoverC WiFi ===");

  auto cfg = M5.config();
  cfg.internal_imu = false;
  cfg.internal_rtc = false;
  cfg.internal_spk = false;
  cfg.internal_mic = false;
  M5.begin(cfg);

  M5.Power.setExtOutput(true);
  delay(200);

  auto& d = M5.Display;
  d.setRotation(1);
  d.fillScreen(BLACK);
  d.setTextSize(2);
  d.setTextColor(WHITE, BLACK);
  d.println("RoverC WiFi");
  d.setTextSize(1);

  Wire.begin((int)0, (int)26, (uint32_t)400000);
  delay(50);
  Wire.beginTransmission(ROVERC_I2C_ADDR);
  bool roverOk = (Wire.endTransmission() == 0);
  if (roverOk) {
    rover.begin();
    d.setTextColor(GREEN, BLACK);
    d.println("RoverC: OK");
  } else {
    d.setTextColor(RED, BLACK);
    d.println("RoverC: FAIL");
  }
  d.setTextColor(WHITE, BLACK);
  delay(1500);

  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  apSSID = "RoverC-" + String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);
  apSSID.toUpperCase();

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(
    IPAddress(192, 168, 4, 1),
    IPAddress(192, 168, 4, 1),
    IPAddress(255, 255, 255, 0)
  );
  WiFi.softAP(apSSID.c_str(), NULL, 6, 0, 1);
  apURL = "http://192.168.4.1/";
  useApMode = true;
  Serial.printf("\nAP SSID: %s\n", apSSID.c_str());

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("STA: connecting to %s", WIFI_SSID);
  int staWait = 0;
  while (WiFi.status() != WL_CONNECTED && staWait < 50) {
    delay(100);
    Serial.print(".");
    staWait++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    staIP = WiFi.localIP().toString();
    staSSID = WiFi.SSID();
    Serial.printf("\nSTA Connected: %s (%s)\n", staSSID.c_str(), staIP.c_str());
  } else {
    Serial.println("\nSTA: not connected (AP only)");
  }

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.onNotFound(handleNotFound);
  server.begin();

  updateDisplay();
}

void loop() {
  server.handleClient();
  M5.update();

  unsigned long now = millis();

  if (now - lastPrintMs >= 100) {
    lastPrintMs = now;
    Serial.printf("[LOOP] cmdX:%d cmdY:%d cmdZ:%d lastCmd:%lu\n",
                  cmdX, cmdY, cmdZ, lastCmdMs);
  }

  if (lastCmdMs && (now - lastCmdMs > CMD_TIMEOUT_MS)) {
    cmdX = cmdY = cmdZ = 0;
    lastCmdMs = 0;
  }

  bool changed = (cmdX != lastSentX || cmdY != lastSentY || cmdZ != lastSentZ);
  if (changed || (now - lastMotorMs >= 20)) {
    lastMotorMs = now;
    lastSentX = cmdX; lastSentY = cmdY; lastSentZ = cmdZ;
    if (cmdX || cmdY || cmdZ) {
      rover.setSpeed(cmdX, cmdY, cmdZ);
    } else {
      rover.stop();
    }
  }

  if (M5.BtnB.wasPressed()) {
    cmdX = cmdY = cmdZ = 0;
    lastCmdMs = 0;
    lastSentX = 127; lastSentY = 127; lastSentZ = 127;
    rover.stop();
  }

  if (M5.BtnA.wasPressed()) {
    updateDisplay();
  }
}

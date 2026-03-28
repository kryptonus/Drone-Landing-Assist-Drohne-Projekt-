/*
 * Drone Landing Assist System
 * 
 * Sensor fusion: HC-SR04 ultrasonic + LD2410C 24GHz radar
 * Output: SSD1315 OLED display + WiFi web dashboard
 * 
 * Author: Vaishnav Vinod
 * Hardware: ESP32 NodeMCU (BerryBase)
 * 
 * Pin mapping:
 *   GPIO5  -> HC-SR04 TRIG
 *   GPIO18 -> HC-SR04 ECHO (via 1k/2k voltage divider)
 *   GPIO4  -> LD2410C OUT
 *   GPIO16 -> LD2410C TX (UART)
 *   GPIO21 -> SSD1315 SDA (I2C)
 *   GPIO22 -> SSD1315 SCL (I2C)
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>

#define TRIG_PIN 5
#define ECHO_PIN 18
#define RADAR_OUT 4
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);

float readings[128];
int readIdx = 0;
float currentDist = 0;
bool humanDetected = false;

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return duration * 0.034 / 2.0;
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Drone Landing Assist</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0a0e17;color:#e0e0e0;font-family:'Segoe UI',sans-serif;overflow-x:hidden}
.header{background:linear-gradient(135deg,#0d1b2a,#1b2d4f);padding:20px;text-align:center;border-bottom:2px solid #00d4ff}
h1{font-size:24px;color:#00d4ff;letter-spacing:2px}
.subtitle{color:#7a8ba0;font-size:13px;margin-top:4px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:16px;padding:16px;max-width:800px;margin:0 auto}
.card{background:#111827;border:1px solid #1e3a5f;border-radius:12px;padding:20px}
.card-full{grid-column:1/3}
.label{color:#7a8ba0;font-size:12px;text-transform:uppercase;letter-spacing:1px}
.value{font-size:42px;font-weight:700;color:#00d4ff;margin:8px 0}
.unit{font-size:18px;color:#4a6a8a}
.status{font-size:20px;font-weight:600;padding:8px 16px;border-radius:8px;display:inline-block;margin-top:8px}
.safe{background:#064e3b;color:#34d399;border:1px solid #34d399}
.danger{background:#7f1d1d;color:#f87171;border:1px solid #f87171}
.cruise{background:#1e3a5f;color:#60a5fa;border:1px solid #60a5fa}
.nosig{background:#374151;color:#9ca3af;border:1px solid #9ca3af}
canvas{width:100%;height:200px;border-radius:8px;margin-top:12px}
.radar-dot{width:16px;height:16px;border-radius:50%;display:inline-block;margin-right:8px;vertical-align:middle}
.dot-on{background:#f87171;animation:pulse 1s infinite}
.dot-off{background:#34d399}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:0.4}}
.bar-container{background:#1e293b;border-radius:6px;height:24px;margin-top:8px;overflow:hidden}
.bar-fill{height:100%;border-radius:6px;transition:width 0.3s;background:linear-gradient(90deg,#34d399,#00d4ff,#f87171)}
.stats{display:flex;justify-content:space-between;margin-top:12px}
.stat{text-align:center}
.stat-val{font-size:18px;font-weight:600;color:#00d4ff}
.stat-label{font-size:11px;color:#7a8ba0}
</style>
</head>
<body>
<div class='header'>
<h1>DRONE LANDING ASSIST</h1>
<div class='subtitle'>ESP32 Sensor Fusion Dashboard</div>
</div>
<div class='grid'>
<div class='card'>
<div class='label'>Altitude (Ultrasonic)</div>
<div class='value' id='dist'>--<span class='unit'> cm</span></div>
<div class='bar-container'><div class='bar-fill' id='bar' style='width:0%'></div></div>
</div>
<div class='card'>
<div class='label'>24GHz Radar</div>
<div class='value' style='font-size:28px'><span class='radar-dot' id='rdot'></span><span id='rstatus'>--</span></div>
<div class='status' id='landing'>--</div>
</div>
<div class='card card-full'>
<div class='label'>Altitude History</div>
<canvas id='chart'></canvas>
<div class='stats'>
<div class='stat'><div class='stat-val' id='mn'>--</div><div class='stat-label'>MIN</div></div>
<div class='stat'><div class='stat-val' id='av'>--</div><div class='stat-label'>AVG</div></div>
<div class='stat'><div class='stat-val' id='mx'>--</div><div class='stat-label'>MAX</div></div>
<div class='stat'><div class='stat-val' id='ct'>0</div><div class='stat-label'>SAMPLES</div></div>
</div>
</div>
</div>
<script>
const hist=[];
let count=0,mn=9999,mx=0,sum=0;
const canvas=document.getElementById('chart');
const ctx=canvas.getContext('2d');

function resize(){canvas.width=canvas.clientWidth;canvas.height=canvas.clientHeight}
window.addEventListener('resize',resize);
resize();

function drawChart(){
  const w=canvas.width,h=canvas.height;
  ctx.clearRect(0,0,w,h);
  ctx.fillStyle='#111827';
  ctx.fillRect(0,0,w,h);
  if(hist.length<2)return;
  const maxV=Math.max(...hist,50);
  ctx.strokeStyle='#1e3a5f';
  ctx.lineWidth=0.5;
  for(let i=0;i<5;i++){
    const y=h*i/4;
    ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();
  }
  ctx.beginPath();
  ctx.strokeStyle='#00d4ff';
  ctx.lineWidth=2;
  ctx.shadowColor='#00d4ff';
  ctx.shadowBlur=6;
  for(let i=0;i<hist.length;i++){
    const x=i*w/(hist.length-1);
    const y=h-(hist[i]/maxV)*h*0.9-h*0.05;
    i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
  }
  ctx.stroke();
  ctx.shadowBlur=0;
  ctx.lineTo(w,h);ctx.lineTo(0,h);ctx.closePath();
  const grd=ctx.createLinearGradient(0,0,0,h);
  grd.addColorStop(0,'rgba(0,212,255,0.15)');
  grd.addColorStop(1,'rgba(0,212,255,0)');
  ctx.fillStyle=grd;
  ctx.fill();
}

function update(){
  fetch('/data').then(r=>r.json()).then(d=>{
    const dist=d.distance;
    const human=d.human;
    count++;
    if(dist>0){
      if(dist<mn)mn=dist;
      if(dist>mx)mx=dist;
      sum+=dist;
    }
    hist.push(dist);
    if(hist.length>200)hist.shift();
    document.getElementById('dist').innerHTML=dist.toFixed(1)+'<span class="unit"> cm</span>';
    document.getElementById('bar').style.width=Math.min(dist/200*100,100)+'%';
    document.getElementById('rdot').className=human?'radar-dot dot-on':'radar-dot dot-off';
    document.getElementById('rstatus').textContent=human?'HUMAN DETECTED':'CLEAR';
    const el=document.getElementById('landing');
    if(dist>0&&dist<30&&!human){el.textContent='SAFE TO LAND';el.className='status safe'}
    else if(human){el.textContent='ABORT LANDING';el.className='status danger'}
    else if(dist===0){el.textContent='NO SIGNAL';el.className='status nosig'}
    else{el.textContent='CRUISING';el.className='status cruise'}
    document.getElementById('mn').textContent=mn<9999?mn.toFixed(1):'--';
    document.getElementById('av').textContent=count>0?(sum/count).toFixed(1):'--';
    document.getElementById('mx').textContent=mx>0?mx.toFixed(1):'--';
    document.getElementById('ct').textContent=count;
    drawChart();
  }).catch(e=>{});
}
setInterval(update,200);
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleData() {
  String json = "{\"distance\":" + String(currentDist, 1) +
                ",\"human\":" + (humanDetected ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(RADAR_OUT, INPUT);
  Wire.begin(21, 22);

  for (int i = 0; i < 128; i++) readings[i] = 0;

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed!");
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 15);
  display.println("Starting WiFi...");
  display.display();

  WiFi.softAP("DroneAssist", "12345678");
  IPAddress ip = WiFi.softAPIP();

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("LANDING ASSIST v2");
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);
  display.setCursor(0, 15);
  display.print("WiFi: DroneAssist");
  display.setCursor(0, 25);
  display.print("Pass: 12345678");
  display.setCursor(0, 40);
  display.print("Open: http://");
  display.setCursor(0, 50);
  display.print(ip);
  display.display();
  delay(5000);
}

void loop() {
  server.handleClient();
  currentDist = getDistance();
  humanDetected = digitalRead(RADAR_OUT) == HIGH;

  readings[readIdx % 128] = currentDist;
  readIdx++;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("LANDING ASSIST");
  display.drawLine(0, 9, 128, 9, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 12);
  if (currentDist == 0) {
    display.println("NO ECHO");
  } else {
    display.print(currentDist, 1);
    display.println("cm");
  }

  display.setTextSize(1);
  display.setCursor(0, 34);
  display.print("ALT");
  int barWidth = map(constrain((int)currentDist, 0, 200), 0, 200, 0, 100);
  display.drawRect(22, 33, 100, 10, SSD1306_WHITE);
  display.fillRect(22, 33, barWidth, 10, SSD1306_WHITE);

  display.drawLine(0, 46, 128, 46, SSD1306_WHITE);
  display.setCursor(0, 50);
  display.print(humanDetected ? "HUMAN " : "CLEAR ");
  if (currentDist > 0 && currentDist < 30 && !humanDetected)
    display.println("LAND OK");
  else if (humanDetected)
    display.println("ABORT!");
  else if (currentDist == 0)
    display.println("NO SIG");
  else
    display.println("CRUISE");

  display.display();
  delay(100);
}

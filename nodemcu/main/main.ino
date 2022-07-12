#include <Ticker.h>
#include <SPI.h>
#include <Wire.h>
#include <DHT.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <Arduino_JSON.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
MAX30105 particleSensor;

// Replace with your network credentials
const char* ssid = "init0xyz";
const char* password = "deepdarkfantasy";
//const char* ssid = "409的爸爸们";
//const char* password = "4409ydlc";

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif
int8_t pp=0;
int8_t flag=0;
int32_t process=0;
int32_t process2=0;
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int32_t spo; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read
const int detection = 15;
//BMP
float pres;
Adafruit_BMP280 bme; 
//PULUSE
const int WIDTH=128;
const int HEIGHT=64;
const int LENGTH=WIDTH;
volatile int rate[10];       // 保留最後 10個 IBI 值
volatile unsigned long sampleCounter = 0; 
volatile unsigned long lastBeatTime = 0;  
volatile int P =512;            
volatile int T = 512;    
volatile int thresh = 512;         
volatile int amp = 100;                  
volatile boolean firstBeat = true;       
volatile boolean secondBeat = false;  
volatile int BPM;       // 存放心律變數
volatile int Signal;    // 保留讀進來的原始資料
volatile int IBI = 600;             
volatile boolean Pulse = false;    
volatile boolean QS = false;   
Ticker flipper;

TwoWire wire;
uint8_t i1;
uint8_t j;
uint8_t k;

byte sp = 0;
int x;
int y[LENGTH];

void clearY(){
  for(int i=0; i<=LENGTH; i++){
    y[i] = -1;
  }
}

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;  
unsigned long timerDelay = 3000;

static const uint8_t image_data_Saraarray[128] = {
0xFF,0xFF,0x3F,0x1F,0x0F,0x07,0x03,0x03,0x03,0x03,0x07,0x07,0x0F,0x1F,0x3F,0x7F,0x3F,0x1F,0x0F,0x07,0x07,0x03,0x03,0x03,
0x03,0x07,0x07,0x0F,0x0F,0x3F,0xFF,0xFF,0xFF,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xFF,0xFF,0xFF,0xFE,0xFC,0xF8,0xF0,0xE0,0xC0,
0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xC0,0xE0,0xF0,0xF8,0xFE,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFC,0xF8,0xF0,0xE0,0xC0,0x80,0xC0,0xE0,0xF0,0xF8,0xFC,0xFE,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};
static const uint8_t image_data[512] = {
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x7F,0xBF,0xBF,0xDF,0xDF,0xEF,0xEF,0xEF,0xEF,0xF7,0x77,0x77,0x77,0x77,0x77,
0x77,0x77,0x77,0x77,0x77,0x77,0x77,0xF7,0xEF,0xEF,0xEF,0xDF,0xDF,0xFF,0xBF,0xBF,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0xBF,0xDF,0xEF,0xF7,0x7B,0xBD,0xDD,0xDE,
0xEF,0xEF,0x1F,0xFF,0xFB,0xE5,0xDD,0x7E,0xFE,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFE,0xFE,0xBF,0xED,0xF1,0xFF,0x7F,0xDF,0xEF,0xEE,0xDE,0xBD,0x7B,0xFB,0xF7,0xEF,0xDF,0x3F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0x9F,0xEF,0xFB,0xFC,0x3F,0x0F,0xD7,0xEB,0xEE,0xEF,0xDF,0xFF,0xEF,0xEF,0xFF,0x9F,0xFE,0xF9,0xF7,0xDF,0x3F,
0xFC,0xF3,0xEF,0xBF,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x3F,0xCF,0xF7,0xF9,0x7E,0xBF,0xEF,0xF3,0xFC,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFD,0xF3,0xEF,0x9F,0x7E,0xFD,0xF7,0xDF,0x7F,0xFF,0x3F,0xC7,0xFC,0xFF,0x7F,0x87,0xF8,0xFF,
0xFF,0xFD,0xFB,0xF7,0xEF,0xDF,0xFF,0xEF,0xF7,0xE3,0xDC,0xBD,0x7B,0xFB,0xFB,0xFB,0x3C,0x33,0xFF,0xFF,0xFE,0xFD,0xF7,0x77,
0x77,0x77,0x77,0xFB,0xFC,0xFF,0xCF,0x37,0xF9,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFC,0xE3,0x1F,0xFF,0xF0,0x0F,0xE0,0xEF,0xDF,0xDF,0xDC,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,
0xDD,0xDD,0xDD,0xDD,0xDD,0xDC,0xDD,0xDF,0xDF,0x1F,0xFF,0xFF,0xE3,0xDC,0xBE,0x7F,0x7F,0x7F,0xBF,0xBE,0xC1,0xFF,0xFF,0x7F,
0x9C,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDD,0xDC,0xDF,0xDF,0xE0,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFE,0xFD,0xFB,0xFB,0xF7,0xF7,0xF7,0xF7,0xFB,0xFB,0xFB,0xFD,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF

};
void initBME(){
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
}

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


#define DHTPIN 14     // Digital pin connected to the DHT sensor

#define DHTTYPE    DHT11     // DHT 11

DHT dht(DHTPIN, DHTTYPE);


// Initialize LittleFS
void initFS() {
 /*
  if (!LittleFS.begin()) {
    Serial.println("An error has occurred while mounting LittleFS");
  }
  Serial.println("LittleFS mounted successfully");
  */
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting the filesystem");
  }
  else{
    Serial.println("Filesystem mounted successfully");
  }
}

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

// Get Sensor Readings and return JSON object
String getSensorReadings(){
  readings["temperature"] = String(bme.readTemperature());
  readings["humidity"] =  String(dht.readHumidity());
  readings["pressure"] =  String(bme.readPressure()/1000);
  readings["BPM"] =  String(BPM);
  readings["spO2"] =  String(spo2);
  
  String jsonString = JSON.stringify(readings);
  return jsonString;
}

String readBMP280Temperature() {
  // Read temperature as Celsius (the default)
  float t = bme.readTemperature();
  // Convert temperature to Fahrenheit
  //t = 1.8 * t + 32;
  if (isnan(t)) {    
    Serial.println("Failed to read from BMP280 sensor!");
    return "";
  }
  else {
    Serial.println(t);
    return String(t);
  }
}

String readDHT11Humidity() {   
  float h = bme.readTemperature()+random(20,22);
  if (isnan(h)) {
    Serial.println("Failed to read from DHT11 sensor!");
    return "";
  }
  else {
    Serial.println(h);
    return String(h);
  }
}

String readBMP280Pressure() {
  float p = bme.readPressure() / 100.0F;
  if (isnan(p)) {
    Serial.println("Failed to read from BMP280 sensor!");
    return "";
  }
  else {
    Serial.println(p);
    return String(p);
  }
}

void setup()
{
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  initWiFi();
  initFS();
    pinMode(detection, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(detection), detectsMovement, RISING);
  wire.begin(4,5);
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
  dht.begin();
  //初始化BMP
 // TCA9548A(5);
  initBME();
  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  wire.begin(12,13);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(1000);
  display.clearDisplay();
  display.setTextColor(WHITE);

clearY();
  interruptSetup(); // 設定每 2mS讀取 Pulse Sensor 的訊號

  wire.begin(12,13);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(20,20);
  display.print("INIT...");
  display.display(); 

  
    wire.begin(4,5);
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps


    for (byte i = 25; i < 100; i++)
  {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }
  
  //take 25 sets of samples before calculating the heart rate.
  for (byte i = 75; i < 100; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data
  
    digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
  
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample 
  }
   maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate); 


   // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");
  
  // Request for the latest sensor readings
  server.on("/readings", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = getSensorReadings();
    request->send(200, "application/json", json);
    json = String();
  });

  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html");
  });
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readBMP280Temperature().c_str()); 
  });
  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readDHT11Humidity().c_str());   
  });
  server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readBMP280Pressure().c_str());
  });

  // Start server
  server.begin();
  
}


int ByteOpera(uint8_t num,uint8_t dat){
  uint8_t opop[8] = {0x01,0x02,0x04,0x8,0x10,0x20,0x40,0x80};
  if (dat&opop[num])
    return 1;
  else
    return 0;
}


void loop()
{   
 
  wire.begin(4,5);
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  pres = bme.readPressure()/100.0F;
  Serial.print(" T = ");
  Serial.print(t,DEC);
  Serial.print(" H = ");
  Serial.print(h,DEC);
  Serial.print(" PRES = ");
  Serial.print(pres,DEC);
  Serial.print(" BPM = ");
  Serial.print(BPM,DEC);
  Serial.print(" SPO2 = ");
  Serial.println(spo2,DEC);
    
  for(sp=0;sp<99;sp++){
    redBuffer[sp] = redBuffer[sp+1];
    irBuffer[sp] = irBuffer[sp+1];
  }
  redBuffer[99] = particleSensor.getRed();
  irBuffer[99] = particleSensor.getIR();
  if(redBuffer[99]<5000 || irBuffer[99]<5000){
    process = 0;
  }
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);  
     
  if ((millis() - lastTime) > timerDelay) {
    // Send Events to the client with the Sensor Readings Every 30 seconds
    events.send("ping",NULL,millis());
    events.send(getSensorReadings().c_str(),"new_readings" ,millis());
    lastTime = millis();
  }
  
if(pp==3){
     wire.begin(12,13);
     display.clearDisplay();
  if(process<130){
    process++;
  }
  display.setTextSize(2);
  display.setCursor(70,0);
  display.print("spo");
  display.setTextSize(1);
  display.setCursor(108,8);
  display.print("2");
    display.drawRect(0, 52, process, 6, WHITE);
    display.fillRect(0, 52, process, 6, WHITE);
    
 // if(process>125){  
  display.setTextSize(3);
  display.setCursor(56,25);
  display.print(spo2);
  display.print("%");
  spo=spo2;
  flag=1;
//}
 for(i1=0;i1<4;i1++){
  for(j=0;j<32;j++){
    for(k=0;k<8;k++){
       if(ByteOpera(k,image_data_Saraarray[i1*32+j])==0){
          display.drawPixel(15+j,10+i1*8+k,WHITE);
       }
    }
  }
 }
    display.display(); 
    //After gathering 25 new samples recalculate HR and SP02
 //maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
//  }
}

if(pp==0){
  wire.begin(12,13); 
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextSize(2);
  display.setCursor(20, 0);
  display.print(pres); 
  display.setTextSize(1);
  display.setCursor(105, 8);
  display.print("hpa"); 
 
   for(i1=0;i1<8;i1++){
  for(j=0;j<64;j++){
    for(k=0;k<8;k++){
       if(ByteOpera(k,image_data[i1*64+j])==0){
          display.drawPixel(30+j,10+i1*8+k,WHITE);
       }
    }
  }
 }
  display.display(); 
}

if(pp==1){  
  wire.begin(12,13);
  y[x] = map(Signal, 0, 1023, HEIGHT-14, 0); 
    drawY();
  x++;
  if(x >= WIDTH/2){
        display.clearDisplay();
        display.drawLine(0, 51, 127, 51, WHITE);
        display.drawLine(0, 63, 127, 63, WHITE);
        display.setTextSize(0);    //0
        display.setTextColor(WHITE);
        display.setCursor(0,54);
        display.print(" BPM = ");
        display.print(BPM);
        display.print("  IBI = ");
        display.print(IBI);
        display.print("  ");
    x = 0;
    clearY();
  }

  sendDataToProcessing('S', Signal);   
  if (QS == true){                      
        // 送出心律，前面帶一個'B'                  
        sendDataToProcessing('B',BPM);
        // 送出心跳間的時間，前面帶一個'Q'   
        sendDataToProcessing('Q',IBI);   
        QS = false;       // 重新設定   

     }
     
  display.display();   
   //delay(5);      
}

if(pp==2){
  
  wire.begin(12,13);
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
  }
  // clear display
  display.clearDisplay();
  // display temperature
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("Temperature: ");
  display.setTextSize(2);
  display.setCursor(0,10);
  display.print(t);
  display.print(" ");
  display.setTextSize(1);
  display.cp437(true);
  display.write(167);
  display.setTextSize(2);
  display.print("C");
  
  // display humidity
  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print("Humidity: ");
  display.setTextSize(2);
  display.setCursor(0, 45);
  display.print(h);
  display.print(" %"); 
  
  display.display(); 
}
}

void drawY(){
  display.drawPixel(0, y[0], WHITE);
  for(int i=1; i<=LENGTH/2; i++){
    if(y[i]!=-1){
      display.drawLine(2*i-2, y[i-1], 2*i-1, y[i], WHITE);
      display.drawLine(2*i-1, y[i-1], 2*i, y[i], WHITE);
   }else{
      break;
    }
  }
}

void sendDataToProcessing(char symbol, int data ){
    Serial.print(symbol); // 前置符號告知進來的資料為何種型態
    Serial.println(data);                
  }


void interruptSetup(){     
  // 初始化Ticker 讓 flipper 每 2mS 執行 ISR 
  flipper.attach_ms(2, ISRTr);     
} 


// 以下是 TICKER 中斷服務的歷程
// Ticker確認每2 miliseconds 讀一次
void ISRTr(){                          
  cli();                          // 停止中斷
  Signal = analogRead(A0)*3.3;        // 讀取 Pulse Sensor 
  sampleCounter += 2;    
  // 兩個 Beat 間的時間差，避免雜訊
               
  int N = sampleCounter - lastBeatTime;  

  // 找到脈衝波的最高值
  // 等待 3/5 of last IBI
  if(Signal < thresh && N > (IBI/5)*3){   
    if (Signal < T){                        
      T = Signal;                        
    }
  }

  if(Signal > thresh && Signal > P){  
    P = Signal;       // 保留最高的脈搏振幅
  }                                                 

  //  開始進行心跳偵測
  if (N > 250){               // 避免高頻率的雜訊
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){        
      Pulse = true;                               
      IBI = sampleCounter - lastBeatTime;         
      lastBeatTime = sampleCounter;              

      if(secondBeat){          // 假使是第二個心跳, secondBeat == TRUE
        secondBeat = false;                  
        for(int i=0; i<=9; i++){          
          rate[i] = IBI;                      
        }
      }

      if(firstBeat){          // 第一次的心跳, firstBeat == TRUE
        firstBeat = false;                   
        secondBeat = true;                   
        sei();                // 再次啟動interrupts
        return;                              
      }   


      // 保留最後 10個 IBI 值的加總
      word runningTotal = 0;                      

      for(int i=0; i<=8; i++){              
        rate[i] = rate[i+1];                  
        runningTotal += rate[i];             
      }

      rate[9] = IBI;                          
      runningTotal += rate[9];                
      runningTotal /= 10;                     
      BPM = 60000/runningTotal;               
      QS = true;                              
      
    }                       
  }

  if (Signal < thresh && Pulse == true){   
    Pulse = false;                         
    amp = P - T;                           
    thresh = amp/2 + T;       // 設定 thresh 為振幅的 50% 
    P = thresh;                            
    T = thresh;
  }

  if (N > 2500){             // 假使 2.5 秒沒有脈搏跳動
    thresh = 512;            // 將變數設為預設值
    P = 512;                               
    T = 512;                               
    lastBeatTime = sampleCounter;                
    firstBeat = true;                      
    secondBeat = false;                    
  }

  sei();                 // 結束時啟動 interrupts
} 

ICACHE_RAM_ATTR void detectsMovement() {
  Serial.println("MOTION DETECTED!!!");
  if(pp<3) pp++;
  else pp=0;
  Serial.println(pp,DEC);
  display.clearDisplay();
}

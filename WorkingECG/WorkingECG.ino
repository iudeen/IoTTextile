#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <FirebaseArduino.h>
#include <Wire.h>
#include <MPU6050.h>
#define MAX_BUFFER 100
#include "DHT.h"
#define DHTPIN 14
#define DHTTYPE DHT11 

// Set these to run example.
#define FIREBASE_HOST "textileant.firebaseio.com"
#define FIREBASE_AUTH "nzBDqU9ZTkxz4PH6zUDlLH1t1MwOdOeZ0g6t7oCQ"
#define WIFI_SSID "Android"
#define WIFI_PASSWORD "rgvp4335"
MPU6050 mpu;

boolean ledState = false;
boolean freefallDetected = false;
int freefallBlinkCount = 0;

uint32_t prevData[MAX_BUFFER];
uint32_t sumData=0;
uint32_t maxData=0;
uint32_t avgData=0;
uint32_t roundrobin=0;
uint32_t countData=0;
uint32_t period=0;
uint32_t lastperiod=0;
uint32_t millistimer=millis();
double frequency;
double beatspermin=0;
uint32_t newData;
float h;
float t;
DHT dht(DHTPIN, DHTTYPE);


void doInt()
{
  freefallBlinkCount = 0;
  freefallDetected = true;  
}

void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:                ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Motion Interrupt:     ");
  Serial.println(mpu.getIntMotionEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Zero Motion Interrupt:     ");
  Serial.println(mpu.getIntZeroMotionEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Free Fall Interrupt:       ");
  Serial.println(mpu.getIntFreeFallEnabled() ? "Enabled" : "Disabled");

  Serial.print(" * Free Fal Threshold:          ");
  Serial.println(mpu.getFreeFallDetectionThreshold());

  Serial.print(" * Free FallDuration:           ");
  Serial.println(mpu.getFreeFallDetectionDuration());
  
  Serial.print(" * Clock Source:              ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:             ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets:     ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());

  Serial.print(" * Accelerometer power delay: ");
  switch(mpu.getAccelPowerOnDelay())
  {
    case MPU6050_DELAY_3MS:            Serial.println("3ms"); break;
    case MPU6050_DELAY_2MS:            Serial.println("2ms"); break;
    case MPU6050_DELAY_1MS:            Serial.println("1ms"); break;
    case MPU6050_NO_DELAY:             Serial.println("0ms"); break;
  }  
  
  Serial.println();
}
/*
 * This is just a homebrew function.
 * Don't take this for critical measurements !!! 
 * Do your own research on frequencydetection for arbitrary waveforms.
 */
void freqDetec() {
  if (countData==MAX_BUFFER) {
   if (prevData[roundrobin] < avgData*1.5 && newData >= avgData*1.5){ // increasing and crossing last midpoint
    period = millis()-millistimer;//get period from current timer value
    millistimer = millis();//reset timer
    maxData = 0;
   }
  }
 roundrobin++;
 if (roundrobin >= MAX_BUFFER) {
    roundrobin=0;
 }
 if (countData<MAX_BUFFER) {
    countData++;
    sumData+=newData;
 } else {
    sumData+=newData-prevData[roundrobin];
 }
 avgData = sumData/countData;
 if (newData>maxData) {
  maxData = newData;
 }

 /* Ask your Ask your cardiologist
 * how to place the electrodes and read the data!
 */
#ifdef PLOTT_DATA
  Serial.print(newData);
 Serial.print("\t");
 Serial.print(avgData);
 Serial.print("\t");
 Serial.print(avgData*1.5);
 Serial.print("\t");
 Serial.print(maxData);
 Serial.print("\t");
 Serial.println(beatspermin);
#endif
 prevData[roundrobin] = newData;//store previous value
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
    // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
  
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);
  
  mpu.setIntFreeFallEnabled(true);
  mpu.setIntZeroMotionEnabled(false);
  mpu.setIntMotionEnabled(false);
  
  mpu.setDHPFMode(MPU6050_DHPF_5HZ);

  mpu.setFreeFallDetectionThreshold(17);
  mpu.setFreeFallDetectionDuration(2);  
  
  checkSettings();

 dht.begin();
 digitalWrite(LED_BUILTIN, LOW);
  
 attachInterrupt(0, doInt, RISING);
}

void loop() {
  newData = analogRead(A0);
  freqDetec();
  if (period!=lastperiod) {
     frequency = 1000/(double)period;//timer rate/period
     if (frequency*60 > 20 && frequency*60 < 200) { // supress unrealistic Data
      beatspermin=frequency*60;
#ifndef PLOTT_DATA
        Serial.print(frequency);
        Serial.print(" hz");
        Serial.print(" ");
        Serial.print(beatspermin);
        Firebase.setFloat("textileant/pulse", beatspermin);
        Firebase.setFloat("textileant/freq", frequency);
        Serial.println(" bpm");
#endif
        lastperiod=period;
     }
  }
  delay(5);
  Vector rawAccel = mpu.readRawAccel();
  Activites act = mpu.readActivites();

 // Serial.print(act.isFreeFall);
 // Serial.print("\n");
  if (freefallDetected)
  {
    //ledState = !ledState;

    digitalWrite( LED_BUILTIN, ledState);

    freefallBlinkCount++;

    if (freefallBlinkCount == 20)
    {
      freefallDetected = false;
      ledState = false;
      digitalWrite(LED_BUILTIN, ledState);
    }
  }
  Serial.println(freefallBlinkCount);
  
  delay(100);
  readTempHum();
  Firebase.setFloat("textileant/temp", t);
  Firebase.setFloat("textileant/hum", h);
}

void readTempHum()
{
  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();
  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
}

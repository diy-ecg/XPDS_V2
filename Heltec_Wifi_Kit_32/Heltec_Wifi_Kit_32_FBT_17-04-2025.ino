#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <ADS1115_WE.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"

#define SDA_ADS 26
#define SCL_ADS 33
#define ADS_ADDRESS 0x48
#define ADS_FREQ 400000

TaskHandle_t displayTaskHandle;

TwoWire I2C_ads = TwoWire(1);
ADS1115_WE adc = ADS1115_WE(&I2C_ads, ADS_ADDRESS);

#define ADC_BATTERY_PIN 1
#define ADC_U2_PIN 5  // GPIO5 = ADC1_CH4
#define ADC_DRL_PIN 7  // GPIO7 = ADC1_CH6

const char* ssid = "Raspi-Wifi-2.4";
const char* password = "raspi01-wifi";
WiFiUDP Udp;
IPAddress unicastIP(10, 42, 0, 1);
constexpr uint16_t PORT = 8266;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ 21, /* clock=*/ 18, /* data=*/ 17);


hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

#define faADC 200
#define packetSize 20

int packetCounter = 0;
char packetBuffer[64 * packetSize];
volatile bool readADC = false;  // Flag aus ISR

unsigned long lastBatteryCheck = 0;
const unsigned long batteryCheckInterval = 10000;

char runtimeStats[1024];
float core0Usage = -1;
float core1Usage = -1;

char voltageStr[16], vgStr[24], coreStr[24], memStr[24];
int batValue, vgValue;

unsigned long TimeTotal = 0;
unsigned long activeTimeTotal = 0;
unsigned long idleCount = 0;
unsigned long activeCount = 0;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  readADC = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(115200);
  I2C_ads.begin(SDA_ADS, SCL_ADS, ADS_FREQ);

  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "Power on");
  u8g2.drawStr(0, 20, "Searching ...");
  u8g2.sendBuffer();

  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(500);
  }

  Serial.println("\nVerbunden mit: " + WiFi.SSID());
  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "WiFi connected");

  if (!adc.init()) {
    Serial.println("ADS1115 NICHT gefunden");
    u8g2.drawStr(0, 20, "ADS1115 NOT found");
  } else {
    Serial.println("ADS1115 verbunden");
    u8g2.drawStr(0, 20, "ADS1115 connected");
    adc.setVoltageRange_mV(ADS1115_RANGE_0256);
    adc.setCompareChannels(ADS1115_COMP_0_1);
    adc.setConvRate(ADS1115_250_SPS);
    adc.setMeasureMode(ADS1115_CONTINUOUS);
  }
  delay(3000);

  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000000 / faADC, true, 0);
  
  // Display-Task starten (nachdem alles initialisiert ist)
  xTaskCreatePinnedToCore(
    displayTask,
    "DisplayTask",
    4096,
    NULL,
    1,
    &displayTaskHandle,
    0  // Core 0
  );
}


void loop() {  
  unsigned long loopStart = micros();
  if (readADC) {
    unsigned long activeStart = micros();
    
    portENTER_CRITICAL(&timerMux);
    readADC = false;
    portEXIT_CRITICAL(&timerMux);

    int adcValue = adc.getRawResult();
    int drlValue = analogRead(ADC_DRL_PIN);         // interner ESP32 ADC

    char mString1[32];
    char mString2[32];
    int timestamp = millis();
    sprintf(mString1, "FBT:%i,t:%lu\r\n", adcValue, timestamp);
    sprintf(mString2, "DRL:%i,t:%lu\r\n", drlValue, timestamp);

    strcat(packetBuffer, mString1);
    strcat(packetBuffer, mString2);
    packetCounter++;
    // UDP-Paket senden fuehrt zu einem erhoehten Leistungs- bzw. Strombedarf des ESP32
    // Daher kommen die beobachtbaren Pegelschwankungen auf der Versorgungsleitung
    if (packetCounter > packetSize - 1) {
      Udp.beginPacket(unicastIP, PORT);
      Udp.write((uint8_t*)packetBuffer, strlen(packetBuffer));
      Udp.endPacket();
      packetCounter = 0;
      packetBuffer[0] = '\0';
    }
    activeTimeTotal += micros() - activeStart;
    activeCount++;
  }
  else {
    idleCount++;
  }
  TimeTotal += micros() - loopStart; 
} 

void displayTask(void* parameter) {
  for (;;) {
    batValue = analogRead(ADC_BATTERY_PIN);
    vgValue  = analogRead(ADC_U2_PIN);
    float voltage = (batValue / 4095.0) * 3.3;
    float vg_volt = (vgValue / 4095.0) * 3.3;
    // Core-Last messen
    vTaskGetRunTimeStats(runtimeStats);
    float idle0 = -1, idle1 = -1;
    char* line = strtok(runtimeStats, "\n");
    while (line != nullptr) {
      if (strstr(line, "IDLE0")) sscanf(line, "IDLE0 %*u %f", &idle0);
      else if (strstr(line, "IDLE1")) sscanf(line, "IDLE1 %*u %f", &idle1);
      line = strtok(nullptr, "\n");
    }
    //char voltageStr[16], vgStr[24], coreStr[24], memStr[24];
    if (voltage != 0) {
       snprintf(voltageStr, sizeof(voltageStr), "ESP-Bat:%.2fV", voltage);
    }
    if (vg_volt != 0) {
        snprintf(vgStr, sizeof(vgStr), "VG=%d (%.2fV)", vgValue, vg_volt);
    }
    if (idle0 >= 0 && idle1 >= 0) {
      core0Usage = 100.0 - idle0;
      core1Usage = 100.0 - idle1;
      snprintf(coreStr, sizeof(coreStr), "C0:%.1f%%|C1:%.1f%%", core0Usage, core1Usage);
    } else {
      snprintf(coreStr, sizeof(coreStr), "No IDLE-Task");
    }

    snprintf(memStr, sizeof(memStr), "Heap: %lu", ESP.getFreeHeap());
    
    // Zeitmessung in der loop
    snprintf(memStr, sizeof(memStr), "A:%lu/%lu",activeTimeTotal,TimeTotal);
    TimeTotal = 0;
    activeTimeTotal = 0;
    idleCount = 0;
    activeCount = 0;
    
    // OLED aktualisieren
    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "WiFi connected");
    u8g2.drawStr(0, 20, "ADS1115 connected");
    u8g2.drawStr(0, 30, voltageStr);
    u8g2.drawStr(0, 40, vgStr);
    u8g2.drawStr(0, 50, coreStr);
    u8g2.drawStr(0, 60, memStr);
    u8g2.sendBuffer();

    vTaskDelay(pdMS_TO_TICKS(5000));  // alle 2 Sekunden aktualisieren
  }
}


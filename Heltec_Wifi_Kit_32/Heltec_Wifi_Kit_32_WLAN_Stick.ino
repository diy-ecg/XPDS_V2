// Optimierter Code für ESP32-S3 mit sicherem Timer-Handling
// UDP-Paketversand wird aus ISR herausgenommen, um WDT zu vermeiden

#include <Wire.h>
#include <U8g2lib.h>
#include <ADS1115_WE.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define SDA_ADS 26
#define SCL_ADS 33
#define ADS_ADDRESS 0x48
#define ADS_FREQ 400000

TwoWire I2C_ads = TwoWire(1);
ADS1115_WE adc = ADS1115_WE(&I2C_ads, ADS_ADDRESS);

#define ADC_BATTERY_PIN 1
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
char packetBuffer[25 * packetSize];
volatile bool readADC = false;  // Flag aus ISR

unsigned long lastBatteryCheck = 0;
const unsigned long batteryCheckInterval = 60000;

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

  u8g2.sendBuffer();
  delay(3000);

  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000000 / faADC, true, 0);
}

void loop() {
  if (readADC) {
    portENTER_CRITICAL(&timerMux);
    readADC = false;
    portEXIT_CRITICAL(&timerMux);

    int adcValue = adc.getRawResult();
    char mString[25];
    sprintf(mString, "FBT:%i,t:%lu\r\n", adcValue, millis());
    strcat(packetBuffer, mString);
    packetCounter++;

    if (packetCounter > packetSize - 1) {
      Udp.beginPacket(unicastIP, PORT);
      Udp.write((uint8_t*)packetBuffer, strlen(packetBuffer));
      Udp.endPacket();
      packetCounter = 0;
      packetBuffer[0] = '\0';
    }
  }

  unsigned long currentMillis = millis();
  if (currentMillis - lastBatteryCheck >= batteryCheckInterval) {
    lastBatteryCheck = currentMillis;
    int adcValue = analogRead(ADC_BATTERY_PIN);
    float voltage = (adcValue / 4095.0) * 3.3;

    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "WiFi connected");
    u8g2.drawStr(0, 20, "ADS1115 connected");
    char voltageStr[16];
    snprintf(voltageStr, sizeof(voltageStr), "%.2fV", voltage);
    u8g2.drawStr(0, 30, voltageStr);
    u8g2.sendBuffer();
  }
} 

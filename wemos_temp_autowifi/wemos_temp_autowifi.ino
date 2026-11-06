#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <ESP8266HTTPClient.h>

// use this uploader:
// install venv esptool
// venv/bin/esptool.py --port /dev/ttyUSB0 --baud 9600 flash-id
// venv/bin/esptool.py --port /dev/ttyUSB0 --baud 115200 write-flash 0x0 /home/dna/.cache/arduino/sketches/9D74FE9317C537A5E57668367612682E/wemos_temp_autowifi.ino.bin

// board NodeMCU 1.0 esp-12e
// flash size 4M
// 80MHz
// upload speed 115000
// avrisp mkii

// for wemos 12e mini
//#define SETUP_PIN D7
//#define SDA_SCL D3,D4

// ADC_MODE(ADC_VCC);

// for esp8266 3m

// old version
#define SETUP_PIN D7
// #define SDA_SCL D3, D4
#define SDA_SCL D2, D1
#define SETUP_GND_PIN 0

// new version
//#define SETUP_PIN 5
//#define SETUP_GND_PIN 0
//#define SDA_SCL 12, 13

// RTC memory layout (each slot = 4 bytes):
//   [0..9]  RtcTrimData  (40 bytes)
//   [10..16] RtcWifiData (28 bytes)

#define RTC_TRIM_MAGIC 0xBEEF1234
struct RtcTrimData {
    uint32_t magic;
    uint16_t T1; int16_t T2, T3;
    uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
    int8_t H1, H3, H6; uint8_t _h_pad;
    int16_t H2, H4, H5; uint8_t _end_pad[2];
};

#define RTC_WIFI_MAGIC  0xCAFE5678
#define RTC_WIFI_OFFSET 10
struct RtcWifiData {
    uint32_t magic;
    uint32_t ip;
    uint32_t gateway;
    uint32_t subnet;
    uint32_t dns;
    uint8_t  bssid[6];
    uint8_t  channel;
    uint8_t  _pad;
};

namespace bmp280 {
    void init();
    void read();
    bool readTrim();
    extern bool trimFromRtc;
    extern unsigned long int temp_raw, pres_raw;
    void readVals(double&, double&, double&);
    void readData();
    signed long int calibration_T(signed long int adc_T);
    unsigned long int calibration_P(signed long int adc_T);
    unsigned long int calibration_H(signed long int adc_T);
    void writeReg(uint8_t reg_address, uint8_t data);
    extern uint8_t chipId;
}
const int sensorId = ESP.getChipId();

// Set by setup() based on whether a serial terminal is detected at boot.
// Opening the serial monitor resets the ESP via DTR; press any key within
// 200ms of boot to enable debug logging for that run.
bool debugMode = true;

void wifiSetup() {

    WiFiManager wifiManager;
    wifiManager.setTimeout(240);
    // Give the STA connect a real timeout and a few retries.
    wifiManager.setConnectTimeout(30);
    wifiManager.setConnectRetries(3);
    wifiManager.setCleanConnect(true);
    wifiManager.setDebugOutput(true);
    // Return from the portal after Save even if the connect failed, so the
    // failure reason below gets printed.
    wifiManager.setBreakAfterConfig(true);
    String apName = String("wifi.therm.") + String(sensorId);

    if (!wifiManager.startConfigPortal(apName.c_str())) {
        if (debugMode) Serial.println("failed to connect and hit timeout");
    }
    // Always log the connect result in setup mode (wrong password, no SSID, ...).
    Serial.print("connect result: ");
    Serial.println(wifiManager.getWLStatusString(wifiManager.getLastConxResult()));
    Serial.print("ssid:[");
    Serial.print(WiFi.SSID());
    Serial.print("] psk length:");
    Serial.println(WiFi.psk().length());
    if (debugMode) {
        Serial.print("ssid:");
        Serial.print(WiFi.SSID());
        Serial.print(" passwd:");
        Serial.println(WiFi.psk());
    }

    // reset after 5 secs
    while (1) {
        ESP.deepSleep(5 * 1000000);
    }
}

void setup()
{
    Serial.begin(74880);

    // Detect serial terminal: opening the monitor resets the chip via DTR (~300ms
    // boot ROM). Press any key during that window and it lands in the FIFO before
    // we get here — zero wait cost. Battery-powered with nothing on RX → silent.

    //debugMode = Serial.available() > 0;
    //while (Serial.available()) Serial.read();
    debugMode = false;
    if (debugMode) {
        Serial.println("entering setup");
    }

    unsigned long setupStart = millis();

    pinMode(SETUP_PIN, INPUT_PULLUP);

    // gnd for switch
    pinMode(SETUP_GND_PIN, OUTPUT);
    digitalWrite(SETUP_GND_PIN, LOW);

    delay(10);
    if (digitalRead(SETUP_PIN) == LOW) {
        if (debugMode) Serial.println("Entering setup mode");
        wifiSetup();
    }

    Wire.begin(SDA_SCL);
    bmp280::init();

    if (debugMode) {
        Serial.print("setup: "); Serial.print(millis() - setupStart); Serial.println("ms");
    }
}

// Save current connection details into RTC for the next wake cycle.
static void saveWifiCache() {
    RtcWifiData w;
    w.magic   = RTC_WIFI_MAGIC;
    w.ip      = (uint32_t)WiFi.localIP();
    w.gateway = (uint32_t)WiFi.gatewayIP();
    w.subnet  = (uint32_t)WiFi.subnetMask();
    w.dns     = (uint32_t)WiFi.dnsIP();
    memcpy(w.bssid, WiFi.BSSID(), 6);
    w.channel = WiFi.channel();
    w._pad    = 0;
    ESP.rtcUserMemoryWrite(RTC_WIFI_OFFSET, (uint32_t*)&w, sizeof(w));
}

// Invalidate the WiFi cache so the next wake does a full connect.
static void invalidateWifiCache() {
    RtcWifiData w = {};
    ESP.rtcUserMemoryWrite(RTC_WIFI_OFFSET, (uint32_t*)&w, sizeof(w));
}

int sendData(double temperature, double humidity, double pressure, double voltage) {
    RtcWifiData w;
    bool hasCached = ESP.rtcUserMemoryRead(RTC_WIFI_OFFSET, (uint32_t*)&w, sizeof(w))
                     && w.magic == RTC_WIFI_MAGIC;

    unsigned long t0 = millis();

    // Don't let the SDK re-write credentials to flash on every connect.
    WiFi.persistent(false);

    if (hasCached) {
        // Skip DHCP by reusing last known IP, skip scan by targeting BSSID+channel directly.
        WiFi.config(IPAddress(w.ip), IPAddress(w.gateway), IPAddress(w.subnet), IPAddress(w.dns));
        WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str(), w.channel, w.bssid, true);
        if (debugMode) Serial.print("fast connect..");
    } else {
        WiFi.begin();
        if (debugMode) Serial.print("full connect..");
    }

    unsigned long deadline = millis() + 4000;
    while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
        delay(10);
        if (debugMode) Serial.print("#1 delay 10");
    }

    if (WiFi.status() != WL_CONNECTED && hasCached) {
        // Cached details stale (router rebooted, channel changed, IP conflict, etc.)
        if (debugMode) Serial.print("\ncache miss, full connect..");
        WiFi.disconnect();
        delay(100);
        if (debugMode) Serial.print("#2 delay 100");
        // Restore DHCP and do a full connect.
        WiFi.config(IPAddress(0U, 0U, 0U, 0U), IPAddress(0U, 0U, 0U, 0U), IPAddress(0U, 0U, 0U, 0U));
        WiFi.begin();
        deadline = millis() + 10000;
        while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
            delay(10);
            if (debugMode) Serial.print("#3 delay 10");
        }
    }

    if (WiFi.status() != WL_CONNECTED) {
        invalidateWifiCache();
        return -1;
    }

    unsigned long connMs = millis() - t0;

    if (debugMode) {
        Serial.print("\nwifi connect: "); Serial.print(connMs); Serial.println("ms");
        if (hasCached) {
            Serial.print("ip: "); Serial.println(WiFi.localIP());
        }
    }

    saveWifiCache();

    HTTPClient http;
    WiFiClient client;
    char url[160];
    snprintf(url, sizeof(url), "http://dna1.cz/report.py?id=%d&temp=%.2lf&hum=%.2lf&pres=%.2lf&v=%.2lf&conn=%lu&chip=0x%02X",
        sensorId, temperature, humidity, pressure, voltage, connMs, bmp280::chipId);

    t0 = millis();
    http.begin(client, url);
    int httpCode = http.GET();
    if (debugMode) {
        Serial.print("http GET: "); Serial.print(millis() - t0); Serial.println("ms");
    }

    if (httpCode > 0) {
        String payload = http.getString();
        if (debugMode) Serial.println(payload);
    } else {
        http.end();
        return -1;
    }

    http.end();
    return 0;
}

void loop()
{
    if (debugMode) {
        Serial.println("entering loop");
    }

    unsigned long loopStart = millis();

    double temperature = 0.0, pressure = 0.0, humidity = 0.0;
    unsigned long t0 = millis();
    bmp280::readVals(temperature, pressure, humidity);
    // Always log the reading (before WiFi connects) so the sensor can be checked
    // without debugMode. chip 0x58 = BMP280 (no humidity), 0x60 = BME280.
    Serial.print("\nsensor chip:0x"); Serial.print(bmp280::chipId, HEX);
    Serial.print(" temp:"); Serial.print(temperature);
    Serial.print("C press:"); Serial.print(pressure);
    Serial.print("hPa hum:"); Serial.print(humidity);
    Serial.print("% read:"); Serial.print(millis() - t0); Serial.println("ms");
    Serial.print("trim from:"); Serial.print(bmp280::trimFromRtc ? "rtc" : "sensor");
    Serial.print(" raw t:"); Serial.print(bmp280::temp_raw);
    Serial.print(" p:"); Serial.println(bmp280::pres_raw);

#if 1
    pinMode(A0, INPUT);
    double voltage = double(analogRead(A0)) / 1024.;
#endif

#if 0
    double voltage = ((float)ESP.getVcc()) / 1024.;
#endif

    long sleepTime = 600L;
    t0 = millis();
    if (sendData(temperature, humidity, pressure, voltage) != 0) {
        sleepTime = 600L;
    }
    if (debugMode) {
        Serial.print("sendData: "); Serial.print(millis() - t0); Serial.println("ms");
        Serial.print("loop: "); Serial.print(millis() - loopStart); Serial.println("ms");
    }

    // sleep & reset
    ESP.deepSleep(sleepTime * 1000000);
}

/* ********************************************************************************* */
/* ********************************************************************************* */
/* ********************************************************************************* */

namespace bmp280 {

#define BME280_ADDRESS 0x76
unsigned long int hum_raw,temp_raw,pres_raw;
signed long int t_fine;
uint8_t chipId;
bool trimFromRtc = false;
uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
int8_t  dig_H1;
int16_t dig_H2;
int8_t  dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t  dig_H6;

void readVals(double &temp_act, double &press_act, double &hum_act) {
    // osrs_t=1, osrs_p=1, mode=forced(01) — triggers one measurement then sensor sleeps
    writeReg(0xF4, (1 << 5) | (1 << 2) | 1);
    delay(15); // OSR x1 measurement completes in ~10ms
    signed long int temp_cal;
    unsigned long int press_cal,hum_cal;
    readData();
    temp_cal = calibration_T(temp_raw);
    press_cal = calibration_P(pres_raw);
    hum_cal = calibration_H(hum_raw);
    temp_act = (double)temp_cal / 100.0;
    press_act = (double)press_cal / 100.0;
    hum_act = (double)hum_cal / 1024.0;
}

void init()
{
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xD0);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS, 1);
    chipId = Wire.read();

    uint8_t osrs_h = 1;             //Humidity oversampling x 1
    uint8_t t_sb = 5;               //Tstandby 1000ms
    uint8_t filter = 0;             //Filter off
    uint8_t spi3w_en = 0;           //3-wire SPI Disable
    uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;

    writeReg(0xF2, osrs_h);
    writeReg(0xF5, config_reg);
    // ctrl_meas (0xF4) written in readVals() to trigger each forced measurement

    RtcTrimData rtc;
    if (ESP.rtcUserMemoryRead(0, (uint32_t*)&rtc, sizeof(rtc)) && rtc.magic == RTC_TRIM_MAGIC) {
        dig_T1 = rtc.T1; dig_T2 = rtc.T2; dig_T3 = rtc.T3;
        dig_P1 = rtc.P1; dig_P2 = rtc.P2; dig_P3 = rtc.P3;
        dig_P4 = rtc.P4; dig_P5 = rtc.P5; dig_P6 = rtc.P6;
        dig_P7 = rtc.P7; dig_P8 = rtc.P8; dig_P9 = rtc.P9;
        dig_H1 = rtc.H1; dig_H2 = rtc.H2; dig_H3 = rtc.H3;
        dig_H4 = rtc.H4; dig_H5 = rtc.H5; dig_H6 = rtc.H6;
        trimFromRtc = true;
    } else {
        readTrim();
    }
}


// Returns false (and doesn't cache) if the sensor didn't return every byte,
// so a glitched read isn't kept in RTC memory across deep sleeps.
bool readTrim()
{
    uint8_t data[32],i=0;
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,24);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }

    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xA1);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,1);
    data[i] = Wire.read();
    i++;
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xE1);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,7);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }

    if (i != 32) return false;

    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];
    dig_H1 = data[24];
    dig_H2 = (data[26]<< 8) | data[25];
    dig_H3 = data[27];
    dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
    dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
    dig_H6 = data[31];

    RtcTrimData rtc;
    rtc.magic = RTC_TRIM_MAGIC;
    rtc.T1 = dig_T1; rtc.T2 = dig_T2; rtc.T3 = dig_T3;
    rtc.P1 = dig_P1; rtc.P2 = dig_P2; rtc.P3 = dig_P3;
    rtc.P4 = dig_P4; rtc.P5 = dig_P5; rtc.P6 = dig_P6;
    rtc.P7 = dig_P7; rtc.P8 = dig_P8; rtc.P9 = dig_P9;
    rtc.H1 = dig_H1; rtc.H2 = dig_H2; rtc.H3 = dig_H3;
    rtc.H4 = dig_H4; rtc.H5 = dig_H5; rtc.H6 = dig_H6;
    ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtc, sizeof(rtc));
    return true;
}

void writeReg(uint8_t reg_address, uint8_t data)
{
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(reg_address);
    Wire.write(data);
    Wire.endTransmission();
}

void readData()
{
    int i = 0;
    uint32_t data[8];
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xF7);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,8);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }

    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    hum_raw  = (data[6] << 8) | data[7];
}

signed long int calibration_T(signed long int adc_T)
{
    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

unsigned long int calibration_P(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);

    if (var1 == 0)
    {
        return 0;
    }

    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000)
    {
        P = (P << 1) / ((unsigned long int) var1);
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;
    }

    var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
    signed long int v_x1;
    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) +
    ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) *
    (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
    ((signed long int) dig_H2) + 8192) >> 14));
    v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
    v_x1 = (v_x1 < 0 ? 0 : v_x1);
    v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
    return (unsigned long int)(v_x1 >> 12);
}

} // namespace bmp280

#include "WisLTEBG96MQTT.h"
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <SparkFun_External_EEPROM.h>
//#include "SoftwareSerial.h"

#define DSerial Serial
#define ATSerial Serial2
// #define AT_TX_PIN  11
// #define AT_RX_PIN  10
// SoftwareSerial  DSerial(AT_RX_PIN, AT_TX_PIN);


//#ifndef CONFIG_ARDUINO_LOOP_STACK_SIZE
// default next line:
//#define CONFIG_ARDUINO_LOOP_STACK_SIZE 8192
// changed to:
//#define CONFIG_ARDUINO_LOOP_STACK_SIZE 8192*4
//#endif

//#define USE_CERT
#define USE_CERT
/* Fill your certificate.pem.root CA wiht LINE ENDING */
const char  pem_pkey[] PROGMEM = {"-----BEGIN PRIVATE KEY-----\r\n"
                                  "MIIEwAIBADANBgkqhkiG9w0BAQEFAASCBKowggSmAgEAAoIBAQDq1FT42/MdGCfX\r\n"
                                  "xZtECXQ2wkXPfMA9SGEbLtU+M8cFLkAlTcmt2gZiOchzItBjrkVo157wSG/8po6W\r\n"
                                  "IfZksEy1HOIk1ApJ0zmODnIqUm541KUKxoUV5jYR+FAutDJxq4/rIgSINl7UsZxN\r\n"
                                  "XnkmVkUX+C3WzoWT5ernX0tpuO0fuNbVwu0XoLfoS56tM0HqhQDomPMtJV9+utCH\r\n"
                                  "TatSoPMrZ0mKXrzrr7kVEHka7oOBfriu4YvehFdSjw4cVug68s7FoeBNH+Y05eRj\r\n"
                                  "X5QcbF3omumAMYW2pkT1WTiAcayF9U3KR9WfMhEzH1/GotEydEihIjvyhaIKcQRu\r\n"
                                  "yhTRwQfTAgMBAAECggEBANN1xtqd/ctaQC6eJBXdSYgx4vlXhijpL4Bx6NZte68f\r\n"
                                  "meW9qXnjFdKH5AhB9lN8z8X8PIR1RlrFhSqOhLjgxQJmcwHk6yrViUNxoL6Zoh8u\r\n"
                                  "pTwRzjANMO13pISaAb64Vg3PgTNDepufIHfPNdse3nMaKfT/3etzo9cbyeX2/5iK\r\n"
                                  "eVCHHnvGiBjvHZwa5Dds66hrHmr8ziD9Mm0lZA7DTcAQUewUP1yhovLdqr0G5T9I\r\n"
                                  "+tZWaa0jmuX6kggKIbWd85WNaqavo2YvDjTdY16O8t1eC/r29rQzcNdn119SnD1I\r\n"
                                  "blUpyuKwa3xws6+spxg8Sd6U1GRdfzLXO9Msrvvkx6ECgYEA+RS/SINaEyei+wOw\r\n"
                                  "ijGLztXOW7mdd4Ay7J91mdsZBY69v9bO9vi0VaFb+iExuMNSeiYTFaDH800lJASJ\r\n"
                                  "VhxDzifjeZ+pA1iKfF5sK11DVoakls+i4hdWUo8qdsMxNcXh8RDXQsYkejPHFc1j\r\n"
                                  "U4SV+av9fLOcI2K8ZqwMr2BS37ECgYEA8Vo9QjEz+mx2wo9Wh9bzsMJ9UB+BUz2V\r\n"
                                  "t7W3XTOIey+OOyLmuESbnluzbjuOxkhhjDCTx2r4WCnGsWYJNYQQz+L3t3Pr0VUI\r\n"
                                  "7+vParC48MWy+dUd+QsK/MapriYQUdOkEjtR1c9D9/9Uv6rX/FvYmW0yGEaK+5+l\r\n"
                                  "8VHL7FKX5MMCgYEA0iDmnk3xl2dA33JlZLAXug41YLxeU4oba/O6SmnD2iRpf4XA\r\n"
                                  "ZCm8OOE/ruyB/dUJp3Kndv+Er4TNaG1LTxHZQfn0tY40D1aKvhEKQCsVu9Eq2O+i\r\n"
                                  "AgqeEFgnArn4sdyWl8AVWYe/DjjQI1x6BuM7gr3Nw33ioxeQyS0sJmekgoECgYEA\r\n"
                                  "mIAOaP6x+zABslnjsR1vCWc6yx+9FZ26NkRJwkhHmp6n3lHlOtWPBvamX6aeRkeB\r\n"
                                  "65TeK4k5tmtfMsRoWTpDbTfakj5a6QA0D3UOsDlOAkTJG/c/YxdJMANPP8ypTyrN\r\n"
                                  "Zv+4a8L1DYR2Rk1q//gA2qEWoCQiuW9c6ShoE8D0XQUCgYEA24G0kWv7S1WvSg0w\r\n"
                                  "rFTwqj8ibMLYVc7bDPpk33+xkN903t55EKAQqsiMoWHjKMM+tJUXUBAzPmxwMDeW\r\n"
                                  "VAM/XGLaztIe8SlbSaJCuAAlmG6cY2U/BoS2+bZpNjhz4kc97ffq1h/i2FCuZr46\r\n"
                                  "xrtrOFDK1RCzlk7A0LIDL4dTTMw=\r\n"
                                  "-----END PRIVATE KEY-----\r\n"
                                 };

/* Fill your certificate.pem.crt wiht LINE ENDING */
const char pem_cert[] PROGMEM = {
                                  "-----BEGIN CERTIFICATE-----\r\n"
                                 "MIIGnTCCBYWgAwIBAgIIaU9QKMWHGRAwDQYJKoZIhvcNAQELBQAwgbQxCzAJBgNV\r\n"
                                 "BAYTAlVTMRAwDgYDVQQIEwdBcml6b25hMRMwEQYDVQQHEwpTY290dHNkYWxlMRow\r\n"
                                 "GAYDVQQKExFHb0RhZGR5LmNvbSwgSW5jLjEtMCsGA1UECxMkaHR0cDovL2NlcnRz\r\n"
                                 "LmdvZGFkZHkuY29tL3JlcG9zaXRvcnkvMTMwMQYDVQQDEypHbyBEYWRkeSBTZWN1\r\n"
                                 "cmUgQ2VydGlmaWNhdGUgQXV0aG9yaXR5IC0gRzIwHhcNMjIxMTA0MDcxNjIwWhcN\r\n"
                                 "MjMxMTA0MDY1OTAwWjAeMRwwGgYDVQQDDBMqLnRyYWNjY2l2aWwuY29tLmF1MIIB\r\n"
                                 "IjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEA3jI32mfzH7yoitVm5R+KlpJX\r\n"
                                 "BS6jjVJtjT70Za+ZZIh/6xc8puva+6lizdNJKt51oLt5Hax8xIXhAzs33g/M6PjV\r\n"
                                 "IiCa0FANovcw/RNHt4uopJGLLJDaBlPLTViPuUUKwlI620+kZDtjsrlCTXbX7iPJ\r\n"
                                 "l9j4Ho8DCpEfSp3G9H2cjHgWqfYGtvIVuzvHhnIbNwgsiYYqe6r79yvEKNHW2nyk\r\n"
                                 "FV1G17FpTk9rIIuXPoACnEqXKV8WGdnXsUOq7gTcsIrZ3+8HmYeBTDZX4cGKfPX8\r\n"
                                 "sM4oBAmwo+xmwkBGnwWIoSCSpahOA9tU9rQlvm61/QViArzNKyYHDWdhYVgU4QID\r\n"
                                 "AQABo4IDRjCCA0IwDAYDVR0TAQH/BAIwADAdBgNVHSUEFjAUBggrBgEFBQcDAQYI\r\n"
                                 "KwYBBQUHAwIwDgYDVR0PAQH/BAQDAgWgMDgGA1UdHwQxMC8wLaAroCmGJ2h0dHA6\r\n"
                                 "Ly9jcmwuZ29kYWRkeS5jb20vZ2RpZzJzMS00NzI5LmNybDBdBgNVHSAEVjBUMEgG\r\n"
                                 "C2CGSAGG/W0BBxcBMDkwNwYIKwYBBQUHAgEWK2h0dHA6Ly9jZXJ0aWZpY2F0ZXMu\r\n"
                                 "Z29kYWRkeS5jb20vcmVwb3NpdG9yeS8wCAYGZ4EMAQIBMHYGCCsGAQUFBwEBBGow\r\n"
                                 "aDAkBggrBgEFBQcwAYYYaHR0cDovL29jc3AuZ29kYWRkeS5jb20vMEAGCCsGAQUF\r\n"
                                 "BzAChjRodHRwOi8vY2VydGlmaWNhdGVzLmdvZGFkZHkuY29tL3JlcG9zaXRvcnkv\r\n"
                                 "Z2RpZzIuY3J0MB8GA1UdIwQYMBaAFEDCvSeOzDSDMKIz1/tss/C0LIDOMDEGA1Ud\r\n"
                                 "EQQqMCiCEyoudHJhY2NjaXZpbC5jb20uYXWCEXRyYWNjY2l2aWwuY29tLmF1MB0G\r\n"
                                 "A1UdDgQWBBQ8fuVBcQ2vemUWcDfrKq4bQhgwUTCCAX0GCisGAQQB1nkCBAIEggFt\r\n"
                                 "BIIBaQFnAHYA6D7Q2j71BjUy51covIlryQPTy9ERa+zraeF3fW0GvW4AAAGEQX9v\r\n"
                                 "BgAABAMARzBFAiAPAm21xgpIWj1zu0w8AYy8QhcP7H/TbfJWuVtHXH1ctgIhAJxP\r\n"
                                 "R6Z+4s+U95SUReNtlvu1f8c07j9T3hrp+WifcHRxAHYAejKMVNi3LbYg6jjgUh7p\r\n"
                                 "hBZwMhOFTTvSK8E6V6NS61IAAAGEQX9vrAAABAMARzBFAiEAzzz7cXQ1qEgONBRE\r\n"
                                 "u8acs7NJAb/Et0m44pAVWaHL/+UCIGd5ScT54u3UL/yc/eK0tRD6rYHaeMmvtHbM\r\n"
                                 "W8qLvihBAHUAs3N3B+GEUPhjhtYFqdwRCUp5LbFnDAuH3PADDnk2pZoAAAGEQX9w\r\n"
                                 "qAAABAMARjBEAiBNkt31Xmv2AnZVegVSnBGu4C4Lfkah7UsMI3/Rl77AXwIgQEuE\r\n"
                                 "ENCNW7uuVaYm8S/4BNfZV40UMICgIJChnXT/Ev4wDQYJKoZIhvcNAQELBQADggEB\r\n"
                                 "ALCrcOH25pXZxlAWArXrreoVKti8gJjKUWKZ5nRdTbm+ODmoyu4CecEttGxrSCrH\r\n"
                                 "vRlYeZfUHIQ5KR9kk2QIbsH86H1GlKclD9rvK/8EfnNL29wJO8kSITsZN55vauTd\r\n"
                                 "4LakwT3bD/i4hc7CHraoHTAWeYEimbW6JrNNDjTYPzge5QnLLhA+xnE14J9rNIu\r\n"
                                 "+732Vttud+lwLQpUUWJBMi8l+T16uhOMncQSQiJ8gEyYG5/nJL9vJpVkR47lUz5a\r\n"
                                 "AA8RiwSsem+dQjMdssEDnvV2mqsFu3jKh/GUvE7bmNL+Kizs8SKfrYxvSdu6VMa0\r\n"
                                 "sQiMwL0OiY0esCweAfvdvxc=\r\n"
                                 "-----END CERTIFICATE-----\r\n"
                                };

/* Fill your private.pem.key wiht LINE ENDING */
const char pem_CA[] PROGMEM = {"-----BEGIN CERTIFICATE-----\r\n"
                               "MIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ\r\n"
                               "RTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD\r\n"
                               "VQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX\r\n"
                               "DTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y\r\n"
                               "ZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy\r\n"
                               "VHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr\r\n"
                               "mD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr\r\n"
                               "IZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK\r\n"
                               "mpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu\r\n"
                               "XmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy\r\n"
                               "dc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye\r\n"
                               "jl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1\r\n"
                               "BE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3\r\n"
                               "DQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92\r\n"
                               "9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx\r\n"
                               "jkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0\r\n"
                               "Epn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz\r\n"
                               "ksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS\r\n"
                               "R9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp\r\n"
                               "-----END CERTIFICATE-----\r\n"
                              };


#define MQTT_RESP_200 "res/200"
#define MQTT_RESP_202 "res/202"
#define MQTT_RESP_204 "res/204"
#define MQTT_CONNECTION_LOST "connectionlost"
#define ignitionPin 25
#define boostEN 13

bool isSignalQualityGood = false;
bool isSimNetworkWorking = false;
bool isMqttHubConnected = false;
bool isMqttProvConnected = false;
bool isDeviceProvisioned = false;
bool isDeviceUpdated = false;
bool isInputStarted = false;
bool isInputDone = false;
bool isGpsFixAcquired = false;
bool resetFlag = false;


char hub_server[50];
char hub_client_id[50];
char hub_username[130];

char provPollTopic[170];

String Lat = "";
String Long = "";
float prevLat;
float prevLong;
float disTravelled;
float battPercent;

unsigned int min_rssi = 2;
unsigned long signalTimeoutStart = 0;
unsigned long timeoutPeriod = 0;

unsigned long updateTimeoutStart = 0;
unsigned long updateTimeoutPeriod = 0;

unsigned long inputStartTime = 0;
unsigned long inputTimeout = 60000;

unsigned int comm_pdp_index = 1;  // The range is 1 ~ 16
unsigned int comm_ssl_index = 2;  // The range is 0 ~ 5
unsigned int comm_mqtt_index = 5; // The range is 0 ~ 5
Mqtt_Version_t version = MQTT_V4;
Mqtt_Qos_t mqtt_qos = AT_MOST_ONCE;

String operationalTime = "";

GNSS_Work_Mode_t mode = STAND_ALONE;

JsonObject respJSON;
DynamicJsonDocument respJSONDoc(512);

struct employee_input_info {
  int currentUpdateVer;
  int max_employee_number;
  int max_job_code;
  int max_cost_code;
  int max_run_hours;
  int employee_number_digits;
  int job_code_digits;
  int cost_code_digits;
  int run_hours_digits;
};

struct deviceData {
  float Lat;
  float Long;
  int employee_number;
  int job_code;
  int cost_code; // 3
  int run_hours;
  //  float disTravelled;
  float battPercent;
  char speedKmh[10] = "";
  char accelState[15] = "";
  char time[25];
};

unsigned int savedDataCount = 0;
unsigned int savedDataPostCount = 0;

int currentUpdateVer = 1;
int max_employee_number;
int max_job_code;
int max_cost_code;
int max_run_hours;
int employee_number_digits;
int job_code_digits;
int cost_code_digits;
int run_hours_digits;

String employee_number = "";
String job_code = "";
String cost_code = ""; // 3
String run_hours = "";
char input_key;

const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {13, 12, 14, 27}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {26, 25, 33, 32}; //connect to the column pinouts of the keypad


WisLTEBG96MQTT WisLTE(ATSerial, DSerial);
LiquidCrystal_I2C lcd(0x27, 20, 4);
ExternalEEPROM extEEPROM;
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();
sensors_event_t event;
float px, py, pz;
char accelState[15];
char speedKmh[10];
//WisLTEBG96SSL ssl(ATSerial, DSerial);
//WisLTEBG96Serial serial(ATSerial, DSerial);


void setup() {

  ATSerial.begin(115200);
  DSerial.begin(115200);
  while (ATSerial.read() >= 0);

  pinMode(ignitionPin, INPUT);
  pinMode(boostEN, OUTPUT);
  digitalWrite(boostEN, HIGH);
  delay(1000);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print(F("Intializing"));
  lcd.setCursor(1, 2);
  lcd.print(F("Device..."));
  Wire.begin();

  if (extEEPROM.begin() == false)
  {
    Serial.println("No memory detected. Freezing.");
  }
  Serial.println("Memory detected!");

  //  Serial.println("size; ");
  //  Serial.println(sizeof(deviceData));

  Serial.print("Mem size in bytes: ");
  Serial.println(extEEPROM.length());
  //  while(!WisLTE.InitModule()){
  //
  //  }
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print(F("Device"));
  lcd.setCursor(1, 2);
  lcd.print(F("intialized..."));
  delay(1000);
  TurnGpsOn();
  EEPROM_read_data();

  //  if(!accel.begin()){
  //      Serial.println(F("No valid sensor found"));
  //      lcd.clear();
  //      lcd.setCursor(1,1);
  //      lcd.print(F("acceleratormeter"));
  //      lcd.setCursor(1,2);
  //      lcd.print(F("not connected!"));
  //      delay(1000);
  //  }
  //  else{
  //    Serial.println(F("G sensor found...\n"));
  //  }
  //  accel.setRange(ADXL345_RANGE_8_G);
  //  accel.getEvent(&event);
  //  px = event.acceleration.x/9.8;
  //  py = event.acceleration.y/9.8;
  //  pz = event.acceleration.z/9.8;

  keypad.addEventListener(startRide);

}

void loop() {

  if ( millis() - signalTimeoutStart >= timeoutPeriod ) {
    CheckSignalQuality();
    if ( !isSignalQualityGood ) {
      //      lcd.clear();
      //      lcd.setCursor(1,0);
      //      lcd.print(F("Sginal Quality"));
      //      lcd.setCursor(1,1);
      //      lcd.print(F("Bad!"));
      //      lcd.setCursor(1,2);
      //      lcd.print(F("skipping posting"));
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print(F("please"));
      lcd.setCursor(1, 1);
      lcd.print(F("wait..."));
      delay(1000);
    }
  }
  if ( !isSimNetworkWorking & isSignalQualityGood ) {
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print(F("please"));
    lcd.setCursor(1, 1);
    lcd.print(F("wait..."));
    configureBG96(); // configure APN and PDP index
    if ( isSimNetworkWorking ) {
      if ( !configureMQTTS() ) { // Configure SSL and MQTT
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print(F("please"));
        lcd.setCursor(1, 1);
        lcd.print(F("wait..."));
        delay(1000);
      }
    }
    else {
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print(F("please"));
      lcd.setCursor(1, 1);
      lcd.print(F("wait..."));
    }

  }

  if ( !isDeviceProvisioned & !isMqttProvConnected & isSimNetworkWorking & isSignalQualityGood ) {
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print(F("please"));
    lcd.setCursor(1, 1);
    lcd.print(F("wait..."));
    connectToProvService(); // Connect to Azure provisioning service
    if (!isMqttProvConnected) {
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print(F("please"));
      lcd.setCursor(1, 1);
      lcd.print(F("wait..."));
      delay(1000);
    }
  }

  if ( !isDeviceProvisioned & isMqttProvConnected & isSimNetworkWorking & isSignalQualityGood ) {
    getProvisioned(); // register device with provision service and save Hub info
    if (!isDeviceProvisioned) {
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print(F("please"));
      lcd.setCursor(1, 1);
      lcd.print(F("wait..."));
      delay(1000);
    }
    else {
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print(F("please"));
      lcd.setCursor(1, 1);
      lcd.print(F("wait..."));
      delay(1000);
    }
  }

  if ( !isMqttHubConnected & isDeviceProvisioned & isSimNetworkWorking & isSignalQualityGood ) {
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print(F("please"));
    lcd.setCursor(1, 1);
    lcd.print(F("wait..."));
    connectToHub(); // Connect to IoT Hub
    if (!isMqttHubConnected) {
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print(F("please"));
      lcd.setCursor(1, 1);
      lcd.print(F("wait..."));
      delay(1000);
    }
    else {
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print(F("please"));
      lcd.setCursor(1, 1);
      lcd.print(F("wait..."));
      delay(1000);
    }
  }

  if ( millis() - updateTimeoutStart >= updateTimeoutPeriod ) {
    if ( !isDeviceUpdated & isMqttHubConnected & isSimNetworkWorking & isSignalQualityGood) {
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print(F("please"));
      lcd.setCursor(1, 1);
      lcd.print(F("wait..."));
      checkDeviceUpdate(); // Check for updates and apply them
      if (!isDeviceUpdated) {
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print(F("please"));
        lcd.setCursor(1, 1);
        lcd.print(F("wait..."));
        delay(1000);
      }
      else {
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print(F("please"));
        lcd.setCursor(1, 1);
        lcd.print(F("wait..."));
        delay(1000);
      }
    }
  }

  inputStartTime = millis();
  while ( (millis() - inputStartTime <= inputTimeout) & !isInputDone ) {
    if ( !isInputStarted ) {
      lcd.clear();
      lcd.setCursor(1, 1);
      lcd.print("Press any key to");
      lcd.setCursor(1, 2);
      lcd.print("start data input..");
      delay(150);
    }
    if ( !isInputDone ) {
      keypad.getKey();
    }
    //    Serial.println(millis() - inputStartTime);
  }
  if ( !isInputDone ) {
    employee_number = "";
    job_code = "";
    cost_code = ""; // 3
    run_hours = "";
    isInputStarted = false;
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print(F("please"));
    lcd.setCursor(1, 1);
    lcd.print(F("wait..."));
  }
  else {
    inputStartTime = millis();
    while (millis() - inputStartTime <= 10000) {
      if ( !isInputStarted ) {
        lcd.clear();
        lcd.setCursor(1, 1);
        lcd.print("Press any key to");
        lcd.setCursor(1, 2);
        lcd.print("end Ride...");
        delay(150);
      }
      if ( isInputDone ) {
        keypad.getKey();
      }
    }
    if (isInputDone) {
      isInputStarted = false;
    }
  }

  if ( isMqttHubConnected & isSignalQualityGood ) {
    getAndSendData();
  }
  else {
    saveDataLocally();
  }

}

void getBattPercent()
{
  int analogValue = analogRead(A0);
  float volt = analogValue * (5.0 / 1023.0);
  if (volt <= 3.5) {
    battPercent = 0;
    return;
  }
  battPercent = map(volt, 3.5, 4.2, 0, 100);
  Serial.print("analogValue : ");
  Serial.println(analogValue);
  Serial.print("Battery volt : ");
  Serial.println(volt);
  Serial.print("Battery percentage : ");
  Serial.println(battPercent);
}

void getTime()
{
  char time1[25];
  //  Cmd_Status_t status = 0;
  WisLTE.DevClock(time1, READ_MODE);
  char *ptr = strstr((const char *)time1, "+");
  *ptr = '\0';
  ptr = strstr((const char *)time1, ",");
  operationalTime = String(ptr + 1);
}

void setDefaultTime()
{
  char time1[25];
  strcpy(time1, "80/01/06,00:00:00+20");
  //  Cmd_Status_t status = 1;
  WisLTE.DevClock(time1, WRITE_MODE);
}

void displayInfo()
{
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("vehicle speed");
  lcd.setCursor(6, 1);
  lcd.print(String(speedKmh) + "kmh");
  lcd.setCursor(1, 2);
  lcd.print("Operational Time");
  lcd.setCursor(5, 3);
  lcd.print(operationalTime);
  delay(1000);
}

void saveDataLocally()
{
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(F("please"));
  lcd.setCursor(1, 1);
  lcd.print(F("wait..."));
  getGpsLoc();
  //  getAccelInfo();
  getDistance();
  getTime();
  getBattPercent();
  displayInfo();
  delay(1000);

  deviceData data1;
  data1.Lat = Lat.toFloat();
  data1.Long = Long.toFloat();
  data1.employee_number = employee_number.toInt();
  data1.job_code = job_code.toInt();
  data1.cost_code = cost_code.toInt();
  data1.run_hours = run_hours.toInt();
  //  data1.disTravelled = disTravelled;
  data1.battPercent = battPercent;
  strcpy(data1.speedKmh, speedKmh);
  strcpy(data1.accelState, accelState);
  strcpy(data1.time, operationalTime.c_str());

  if (savedDataCount * sizeof(data1) <= 64000) {
    Serial.print("\n\nsaving data...");
    Serial.println(operationalTime.c_str());
    extEEPROM.put(sizeof(data1)*savedDataCount, data1);
    savedDataCount++;
  }
}

void getDistance()
{
  float distance = 0;
  if (prevLat == 0 & prevLong == 0) {
    prevLat = Lat.toFloat();
    prevLong = Long.toFloat();
    //    Serial.print("Lat = ");
    //    Serial.println(prevLat);
    //    Serial.print("Long = ");
    //    Serial.println(prevLong);
  }
  else {
    if (Lat != "" & Long != "") {
      distance = getDistanceBetween( prevLat, prevLong, Lat.toFloat(), Long.toFloat());
      disTravelled += (distance / 1000);

      //      Serial.print("dis = ");
      //      Serial.println(distance);
      prevLat = Lat.toFloat();
      prevLong = Long.toFloat();
    }
  }

}

float getDistanceBetween(float lat1, float long1, float lat2, float long2)
{

  float delta = radians(long1 - long2);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  return delta * 6372795;
}

void getAndSendData()
{
  char dataPubTopic[70];
  char serializedData[256];
  if (savedDataCount == 0) {
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print(F("please"));
    lcd.setCursor(1, 1);
    lcd.print(F("wait..."));
    getGpsLoc();
    //    getAccelInfo();
    //    getDistance();
    getBattPercent();
    getTime();
    displayInfo();

    clearJSON();
    respJSONDoc["GPS Lat"] = Lat;
    respJSONDoc["GPS Long"] = Long;
    //    respJSONDoc["Distance traveled"] = disTravelled;
    respJSONDoc["Vehicle speed"] = speedKmh;
    respJSONDoc["Vehicle state"] = accelState;
    respJSONDoc["Employee Num"] = employee_number;
    respJSONDoc["Job Code"] = job_code;
    respJSONDoc["Cost Code"] = cost_code;
    respJSONDoc["Run Hours"] = run_hours;
    respJSONDoc["Batt%"] = battPercent;
    respJSONDoc["Operational Time"] = operationalTime;

    sprintf(dataPubTopic, "%s%s%s", "devices/", hub_client_id, "/messages/events/");
    serializeJson(respJSONDoc, serializedData);

    if ( WisLTE.MQTTPublishMessagesEx(comm_mqtt_index, 1, AT_LEAST_ONCE, dataPubTopic, false, serializedData, strlen(serializedData) ) != 0) {
      isMqttHubConnected = false;
      Serial.println("/nisMqttHubConnected1 = false/n");
      return;
    }

  }
  else if (savedDataCount > savedDataPostCount) {
    unsigned long startTime = millis();
    while (millis() - startTime <= 60000) {
      deviceData data2;
      extEEPROM.get(savedDataPostCount * sizeof(data2), data2);

      clearJSON();
      respJSONDoc["GPS Lat"] = String(data2.Lat, 4);
      respJSONDoc["GPS Long"] = String(data2.Long, 4);
      //      respJSONDoc["Distance traveled"] = String(data2.disTravelled,4);
      respJSONDoc["Vehicle speed"] = data2.speedKmh;
      respJSONDoc["Vehicle state"] = data2.accelState;
      respJSONDoc["Employee Num"] = String(data2.employee_number);
      respJSONDoc["Job Code"] = String(data2.job_code);
      respJSONDoc["Cost Code"] = String(data2.cost_code);
      respJSONDoc["Run Hours"] = String(data2.run_hours);
      respJSONDoc["Batt%"] = String(data2.battPercent);
      respJSONDoc["Operational Time"] = data2.time;


      sprintf(dataPubTopic, "%s%s%s", "devices/", hub_client_id, "/messages/events/");
      serializeJson(respJSONDoc, serializedData);
      if ( WisLTE.MQTTPublishMessagesEx(comm_mqtt_index, 1, AT_LEAST_ONCE, dataPubTopic, false, serializedData, strlen(serializedData) ) != 0) {
        isMqttHubConnected = false;
        Serial.println("/nisMqttHubConnected2 = false/n");
        return;
      }
      savedDataPostCount++;
      if (savedDataCount == savedDataPostCount) {
        savedDataCount = 0;
        savedDataPostCount = 0;
        extEEPROM.erase();
        break;
      }
    }
  }
  else {
    getGpsLoc();
    //    getDistance();
    getTime();
    getBattPercent();
    displayInfo();

    clearJSON();
    respJSONDoc["GPS Lat"] = Lat;
    respJSONDoc["GPS Long"] = Long;
    //    respJSONDoc["Distance Traveled"] = disTravelled;
    respJSONDoc["Vehicle speed"] = speedKmh;
    respJSONDoc["Vehicle state"] = accelState;
    respJSONDoc["Employee Num"] = employee_number;
    respJSONDoc["Job Code"] = job_code;
    respJSONDoc["Cost Code"] = cost_code;
    respJSONDoc["Run Hours"] = run_hours;
    respJSONDoc["Operational Time"] = operationalTime;

    sprintf(dataPubTopic, "%s%s%s", "devices/", hub_client_id, "/messages/events/");
    serializeJson(respJSONDoc, serializedData);
    if ( WisLTE.MQTTPublishMessagesEx(comm_mqtt_index, 1, AT_LEAST_ONCE, dataPubTopic, false, serializedData, strlen(serializedData) ) != 0) {
      Serial.println("/nisMqttHubConnected3 = false/n");
      isMqttHubConnected = false;
      return;
    }
  }
}

void getAccelInfo()
{
  float sumX = 0, sumY = 0;
  float avgX = 0, avgY = 0;
  float i;
  for (i = 0; i < 10; i++) {
    accel.getEvent(&event);
    sumX += abs(event.acceleration.x / 9.8 - px);
    sumY += abs(event.acceleration.y / 9.8 - py);
  }
  avgX = sumX / i;
  avgY = sumY / i;

  // Serial.print("avgX = ");
  // Serial.println(avgX);
  // Serial.print("avgY = ");
  // Serial.println(avgY);

  if ( (avgX > 0.3 | avgY > 0.3) & digitalRead(ignitionPin)) {
    sprintf(accelState, "Moving");
    //    Serial.println("moving... ");
  }
  else if ((avgX > 0.3 | avgY > 0.3) & !digitalRead(ignitionPin)) {
    sprintf(accelState, "Towed");
    //    Serial.println("idle... ");
  }
  else {
    sprintf(accelState, "Idle");
  }
  px = event.acceleration.x / 9.8;
  py = event.acceleration.y / 9.8;
  pz = event.acceleration.z / 9.8;
}

void TurnGpsOn()
{
  //  if(!WisLTE.TurnOnGNSS(mode, WRITE_MODE)){
  //    lcd.clear();
  //    lcd.setCursor(1,1);
  //    lcd.print("GPS inactive...");
  //    lcd.setCursor(1,2);
  //    lcd.print("Please check network");
  //    delay(1000);
  //  }
  //  else{
  //    Serial.println("\r\nOpen the GNSS Function Fali!");
  //    if(WisLTE.TurnOnGNSS(mode, READ_MODE)){
  //        Serial.println("\r\nThe GNSS Function is Opened!");
  //        lcd.clear();
  //        lcd.setCursor(1,2);
  //        lcd.print("GPS activated...");
  //        delay(1000);
  //        //WisLTE.TurnOffGNSS();
  //
  //    }
  //  }
  while (!WisLTE.TurnOnGNSS(mode, WRITE_MODE)) {
    Serial.println("\r\nOpen the GNSS Function Fali!");
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print(F("please"));
    lcd.setCursor(1, 1);
    lcd.print(F("wait..."));
    delay(100);
    if (WisLTE.TurnOnGNSS(mode, READ_MODE)) {
      Serial.println("\r\nThe GNSS Function is Opened!");
      WisLTE.TurnOffGNSS();
    }
  }
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(F("please"));
  lcd.setCursor(1, 1);
  lcd.print(F("wait..."));
  //  delay(1000);
  //  WisLTE.setGNSSPriority(0);
}

void getGpsLoc()
{
  Lat = "";
  Long = "";
  char GpsPosition[128];
  char *token;
  int i = 0;
  unsigned long startTime = 0;

  //  WisLTE.setGNSSPriority(1);
  //  delay(500);
  startTime = millis();
  while (millis() - startTime <= 180000) {
    if (!WisLTE.GetGNSSPositionInformation(GpsPosition)) {
      Lat = "";
      Long = "";
      strcpy(accelState, "");
      strcpy(speedKmh, "");
    }
    else {
      Serial.print("\n\nNEMA senstence = ");
      Serial.println(GpsPosition);
      token = strtok(GpsPosition, ",");
      while (i < 8)
      {
        token = strtok(NULL, ",");
        if ( i == 0 )
          Lat = String(token);
        else if ( i == 1 ) {
          Long = String(token);
        }
        else if ( i == 7 ) {
          //          Serial.print("Speed = ");
          //          Serial.println(token);
          //          Serial.print("\n\n");
          sprintf(speedKmh, String(token).c_str());
        }


        if (strcmp(speedKmh, "")) {
          sprintf(accelState, "");
        }
        else if (digitalRead(ignitionPin) == 0 & atof(speedKmh) > 1) {
          sprintf(accelState, "Towed");
        }
        else if (digitalRead(ignitionPin) == 1 & atof(speedKmh) > 1) {
          sprintf(accelState, "Moving");
        }
        else {
          sprintf(accelState, "idle");
        }

        i++;
      }
      //      delay(500);
      //      WisLTE.setGNSSPriority(0);
      return;
    }
    delay(1500);
  }
  //  delay(500);
  //  WisLTE.setGNSSPriority(0);

}

void CheckSignalQuality()
{
  char err_code[60];
  if ( WisLTE.RegisterAndCheckSignalQuality(err_code, min_rssi) ) {
    isSignalQualityGood = true;
    //    Serial.println(err_code);
    timeoutPeriod = 0;
    return;
  }
  else {
    isSignalQualityGood = false;
    isSimNetworkWorking = false;
    Serial.println(err_code);
    signalTimeoutStart = millis();
    timeoutPeriod = 60000;
    return;
  }
}

void saveHubInfo()
{
  const char *server = respJSON["registrationState"]["assignedHub"];
  const char *client_id = respJSON["registrationState"]["deviceId"];
  strcpy(hub_server, server);
  strcpy(hub_client_id, client_id);
  sprintf(hub_username, "%s%s%s%s", hub_server, "/", hub_client_id, "/?api-version=2018-06-30");

  //  Serial.println(hub_server);
  //  Serial.println(hub_client_id);
  //  Serial.println(hub_username);
}

bool connectToHub()
{
  unsigned int mqtt_port = 8883;
  WisLTE.CloseMQTTClient(comm_mqtt_index);
  WisLTE.CloseMQTTNetwork(comm_mqtt_index);
  int ret = WisLTE.OpenMQTTNetwork(comm_mqtt_index, hub_server, mqtt_port);
  if (ret == 2) {
    isMqttHubConnected = false;
    isSimNetworkWorking = false;
    resetFlag = true;
    return false;
  }
  else if ( ret == 0) {
    Serial.println(F("\r\nSet the MQTT Service Address Success!"));
  }
  else {
    Serial.println(F("\r\nSet the MQTT Service Address Fail!"));
    isMqttHubConnected = false;
    isSimNetworkWorking = false;
    return false;
  }

  if ( WisLTE.CreateMQTTClient(comm_mqtt_index, hub_client_id, hub_username, "") != 0) {
    Serial.println(F("\r\nCreate a MQTT Client Fail!"));
    //    if(!WisLTE.CloseMQTTNetwork(comm_mqtt_index)){
    //      isSimNetworkWorking = false;
    //      return false;
    //    }
    isMqttHubConnected = false;
    return false;
  }
  else {
    Serial.println(F("\r\nCreate a MQTT Client Success!"));
  }
  isMqttHubConnected = true;
  return true;
}

bool connectToProvService()
{
  char prov_server[] = "global.azure-devices-provisioning.net";
  unsigned int mqtt_port = 8883;
  char prov_client_id[] = "itechdpstest";
  char prov_username[] = "0ne001D3E80/registrations/itechdpstest/api-version=2019-03-31";

  WisLTE.CloseMQTTClient(comm_mqtt_index);
  WisLTE.CloseMQTTNetwork(comm_mqtt_index);

  if ( WisLTE.OpenMQTTNetwork(comm_mqtt_index, prov_server, mqtt_port) != 0) {
    Serial.println(F("\r\nSet the MQTT Service Address Fail!"));
    isSimNetworkWorking = false;
    isMqttProvConnected = false;
    return false;
  }
  else {
    Serial.println(F("\r\nSet the MQTT Service Address Success!"));
  }

  if ( WisLTE.CreateMQTTClient(comm_mqtt_index, prov_client_id, prov_username, "") != 0) {
    Serial.println(F("\r\nCreate a MQTT Client Fail!"));
    //    if(!WisLTE.CloseMQTTNetwork(comm_mqtt_index)){
    //      isSimNetworkWorking = false;
    //      return false;
    //    }
    isMqttProvConnected = false;
    return false;
  }
  else {
    Serial.println(F("\r\nCreate a MQTT Client Success!"));
  }
  isMqttProvConnected = true;
  return true;
}


bool configureMQTTS()
{
  char ssl_error[100];
  if (!init_ssl(comm_ssl_index, ssl_error)) {
    Serial.println(ssl_error);

  }
  else {
    Serial.println(ssl_error);
    Serial.println(F("SSL Configured..."));
  }
  if (!WisLTE.SetMQTTConfigureParameters(comm_mqtt_index, comm_pdp_index, version, 150, SERVER_DISCARD_INFORMATION)) {
    Serial.println(F("\r\nConfig the MQTT Parameter Fail!"));
    //        int e_code;
    //        if (WisLTE.returnErrorCode(e_code)){
    //            Serial.print("\r\nERROR CODE: ");
    //            Serial.println(e_code);
    //            Serial.println("Please check the documentation for error details.");
    //        }
    return false;
  }
  else {
    Serial.println(F("\r\nConfig the MQTT Parameter Success!"));
  }


  if (!WisLTE.SetMQTTEnableSSL(comm_mqtt_index, comm_ssl_index, true)) {
    Serial.println(F("\r\nEnable the SSL Fail!"));
    return false;
  }
  else {
    Serial.println(F("\r\nEnable the SSL Success!"));
  }
  return true;
}

bool configureBG96()
{
  char APN[] = "";
  char inf[50];
  char apn_error[50];

  if (resetFlag) {
    if (WisLTE.SetDevFunctionality(MINIMUM_FUNCTIONALITY) != 1) {
      resetFlag = true;
      return false;
    }
    delay(1000);
    if (WisLTE.SetDevFunctionality(FULL_FUNCTIONALITY) != 1) {
      resetFlag = true;
      return false;
    }
    resetFlag = false;
  }

  WisLTE.SetDevCommandEcho(false);

  WisLTE.DeactivateDevAPN(comm_pdp_index);

  if (WisLTE.GetDevInformation(inf)) {
    Serial.println(inf);
  }
  else {
    isSimNetworkWorking = false;
    return false;
  }
  cleanBuffer(inf);
  if (WisLTE.GetDevVersion(inf)) {
    Serial.println(inf);
  }
  else {
    isSimNetworkWorking = false;
    return false;
  }

  if (!WisLTE.InitAPN(comm_pdp_index, APN, "", "", apn_error)) {
    Serial.println(apn_error);
    isSimNetworkWorking = false;
    return false;
  }
  Serial.println(apn_error);

  TurnGpsOn();
  //  WisLTE.GetDevNetSignalQuality(rssi);
  //  Serial.print("rssi: ");
  //  Serial.println(rssi);
  //  if(rssi)
  isSimNetworkWorking = true;
  return true;
}

bool checkDeviceUpdate()
{
  updateTimeoutStart = millis();
  updateTimeoutPeriod = 3600000;
  String resp = "";
  char getPropSubTopic[] = "$iothub/twin/res/#";
  char getPropPubTopic[] = "$iothub/twin/GET/?$rid=1";
  char sendPropPubTopic[] = "$iothub/twin/PATCH/properties/reported/?$rid=1";

  if (WisLTE.MQTTSubscribeTopic(comm_mqtt_index, 1, getPropSubTopic, mqtt_qos) != 0) {
    isMqttHubConnected = false;
    return false;
  }

  if ( WisLTE.MQTTPublishMessagesEx(comm_mqtt_index, 1, AT_LEAST_ONCE, getPropPubTopic, false, "{}", strlen("{}") ) != 0) {
    isMqttHubConnected = false;
    return false;
  }

  resp = getResponse(5);
  if (!checkResp(resp, MQTT_RESP_200, isDeviceUpdated)) {
    return false;
  }

  if (respJSON["desired"]["$version"] == currentUpdateVer) {
    Serial.println();
    Serial.println(F("Device already updated..."));
    isDeviceUpdated = true;
    return true;
  }
  else {
    Serial.println(F("Updating device..."));
    currentUpdateVer = respJSON["desired"]["$version"].as<int>();
    max_employee_number = respJSON["desired"]["settings"]["maxEN"].as<int>();
    max_job_code = respJSON["desired"]["settings"]["maxJC"].as<int>();
    max_cost_code = respJSON["desired"]["settings"]["maxCC"].as<int>();
    max_run_hours = respJSON["desired"]["settings"]["maxRH"].as<int>();
    employee_number_digits = respJSON["desired"]["settings"]["ENdigits"].as<int>();
    job_code_digits = respJSON["desired"]["settings"]["JCdigits"].as<int>();
    cost_code_digits = respJSON["desired"]["settings"]["CCdigits"].as<int>();
    run_hours_digits = respJSON["desired"]["settings"]["RHdigits"].as<int>();
    if (!EEPROM_wirte_data()) {
      return false;
    }
    Serial.println(F("Upadate Done..."));
  }

  clearJSON();
  respJSONDoc["updatestatus"] = "Updated";
  char reportedProp[50];
  serializeJson(respJSONDoc, reportedProp);
  //  Serial.println(reportedProp);

  if ( WisLTE.MQTTPublishMessagesEx(comm_mqtt_index, 1, AT_LEAST_ONCE, sendPropPubTopic, false, reportedProp, strlen(reportedProp) ) != 0) {
    isMqttHubConnected = false;
    return false;
  }

  resp = getResponse(5);
  if (!checkResp(resp, MQTT_RESP_204, isDeviceUpdated)) {
    return false;
  }

  if ( WisLTE.MQTTUnsubscribeTopic(comm_mqtt_index, 1, getPropSubTopic) != 0) {
    isMqttHubConnected = false;
    return false;
  }
  Serial.println(F("Twin Property GET Topic unsubscribed"));
  isDeviceUpdated = true;
  return true;
}

bool getProvisioned()
{
  String resp = "";
  char provRespTopic[] = "$dps/registrations/res/#";
  char provPubTopic[] = "$dps/registrations/PUT/iotdps-register/?$rid=1";
  char provPollTopic[170];

  if (WisLTE.MQTTSubscribeTopic(comm_mqtt_index, 1, provRespTopic, mqtt_qos) != 0) {
    isMqttProvConnected = false;
    return false;
  }

  if ( WisLTE.MQTTPublishMessagesEx(comm_mqtt_index, 1, AT_LEAST_ONCE, provPubTopic, false, "{}", strlen("{}") ) != 0)
  {
    isMqttProvConnected = false;
    return false;
  }

  resp = getResponse(5);
  if (!checkResp(resp, MQTT_RESP_202, isDeviceProvisioned)) {
    return false;
  }

  if (respJSON["operationId"] != "null" & respJSON["operationId"] != "" ) {
    cleanBuffer(provPollTopic);
    sprintf(provPollTopic, "%s%s%s", "$dps/registrations/GET/iotdps-get-operationstatus/?$rid=1", "&operationId=", respJSON["operationId"].as<const char *>());
  }
  else {
    isDeviceProvisioned = false;
    return false;
  }

  resp = MQTT_RESP_202;
  while (resp == MQTT_RESP_202)
  {
    if ( WisLTE.MQTTPublishMessagesEx(comm_mqtt_index, 1, AT_LEAST_ONCE, provPollTopic, false, "{}", strlen("{}") ) != 0) {
      isMqttProvConnected = false;
      return false;
    }

    resp = getResponse(5);
    //    if(respJSON["status"] == "failed"){
    //
    //    }else
    if (resp == MQTT_RESP_200) {
      break;
    }
    delay(2000);
  }
  if (!checkResp(resp, MQTT_RESP_200, isDeviceProvisioned)) {
    return false;
  }

  if ( WisLTE.MQTTUnsubscribeTopic(comm_mqtt_index, 1, provRespTopic) != 0) {
    isMqttProvConnected = false;
    return false;
  }
  Serial.println(F("Porvision Topic unsubscribed"));

  saveHubInfo();

  isDeviceProvisioned = true;

  if (!WisLTE.CloseMQTTClient(comm_mqtt_index)) {
    isMqttProvConnected = false;
    return false;
  }
  isMqttProvConnected = false;
  return true;
}


String getResponse(int timeout)
{
  char mqtt_recv[512];
  String resp = "";
  char *resp_pointer;
  char *obj_pointer;
  clearJSON();
  Mqtt_URC_Event_t ret = WisLTE.WaitCheckMQTTURCEvent(mqtt_recv, timeout);
  switch (ret)
  {
    case MQTT_RECV_DATA_EVENT:
      {
        //      Serial.println(mqtt_recv);
        resp_pointer = strstr((const char *)mqtt_recv, "res/");
        for (int i = 0; i < 7; i++) {
          resp += *resp_pointer;
          resp_pointer++;
        }
        Serial.println();
        Serial.println(resp);
        int chr = '"';
        obj_pointer = strrchr((const char *)mqtt_recv, chr);
        *obj_pointer = '\0';
        obj_pointer = strstr((const char *)mqtt_recv, ",");
        //          Serial.println(obj_pointer+2);
        deserializeJson(respJSONDoc, (const char *)(obj_pointer + 2));
        respJSON = respJSONDoc.as<JsonObject>();
        //          Serial.println();
        //          Serial.print(F("memory usage: "));
        //          Serial.println(respJSONDoc.memoryUsage());
        return String(resp);
        break;
      }
    case MQTT_STATUS_EVENT:
      {
        resp = MQTT_CONNECTION_LOST;
        return resp;
        break;
      }
    case MQTT_TIMEOUT:
      {
        Serial.println(F("\nMQTT unkown error"));
        resp = MQTT_CONNECTION_LOST;
        return resp;
        break;
      }
    default:
      {
        Serial.println(F("\nMQTT unkown error"));
        resp = MQTT_CONNECTION_LOST;
        return resp;
        break;
      }
  };
  return MQTT_CONNECTION_LOST;
}

bool checkResp(String resp, String resp_test, bool &flag)
{
  if (resp == MQTT_CONNECTION_LOST) {
    isMqttProvConnected = false;
    isMqttHubConnected = false;
    return false;
  }
  else if (resp == resp_test) {
    return true;
  }
  else {
    flag = false;
    return false;
  }
}
void clearJSON()
{
  respJSONDoc.clear();
  respJSON.clear();
}

void cleanBuffer(char *buf)
{
  memset(buf, '\0', strlen(buf));
}


bool DeleteFiles(char *filename)
{
  char cmd[32], buf[32];
  strcpy(cmd, FILE_DELETE_FILES);
  sprintf(buf, "=\"%s\"", filename);
  strcat(cmd, buf);
  if (WisLTE.sendAndSearch(cmd, RESPONSE_OK, RESPONSE_ERROR, 2)) {
    return true;
  }
  return false;
}

bool UploadFiles(String filename, String u_file)
{
  char cmd[32], buf[32],buf1[32],buf2[100];
  strcpy(cmd, FILE_UPLOAD_FILES);
  filename.toCharArray(buf1, filename.length());
  u_file.toCharArray(buf2, u_file.length());
 // sprintf(buf, "=\"%s\",%d", filename, strlen(u_file));
  sprintf(buf, "=\"%s\",%d", buf1, u_file.length());
  strcat(cmd, buf);
  if (WisLTE.sendAndSearch(cmd, RESPONSE_CONNECT, RESPONSE_ERROR, 5)) {
    //if (WisLTE.sendDataAndCheck(u_file, RESPONSE_OK, RESPONSE_ERROR, 10)) 
   if (WisLTE.sendDataAndCheck(buf2, RESPONSE_OK, RESPONSE_ERROR, 10)) 
    {
      return true;
    }
  }
  return false;
}
bool init_ssl(unsigned int ssl_index, char *err_code) {

  unsigned long start_time = millis();
  char *e_str;
  int f_err_code;
  char ch;
  String cert = "";

  if (!WisLTE.SetSSLParameters(ssl_index, TLS_1_2, SUPPORT_ALL_ABOVE, 300)) {
    e_str = "\r\nSSL ERROR: An error occurred while setting the ssl parameter.\r\n";
    strcpy(err_code, e_str);
    return false;
  }

 
   for (unsigned int k = 0; k < strlen_P(pem_CA); k++) {
     ch = pgm_read_byte_near(pem_CA + k);
     cert.concat(ch);
   }
   //  passing CA_cert
   while (!UploadFiles((char *)ssl_ca_cert_name, cert.c_str())){
       if(WisLTE.returnErrorCode(f_err_code)){
           if (f_err_code == 407){
               start_time = millis();
               while (!DeleteFiles((char *)ssl_ca_cert_name)){
                   if(millis() - start_time >= 10*1000UL){
                       e_str = "\r\nSSL ERROR: The ssl ca cert file exists. An error occurred while deleting the original file during the re-upload process.\r\n";
                       strcpy(err_code, e_str);
                       return false;
                   }
               }
           }
       }else if(millis() - start_time >= 30*1000UL){
           sprintf(e_str, "\r\nSSL ERROR: Error uploading file, error code: %d ,Please check the corresponding documentation for details.\r\n", f_err_code);
           strcpy(err_code, e_str);
           return false;
       }
   }
   start_time = millis();
   cert="";
   for (unsigned int k = 0; k < strlen_P(pem_cert); k++) {
     ch = pgm_read_byte_near(pem_cert + k);
     cert.concat(ch);
   }
   while (!UploadFiles((const char *)ssl_client_cert_name, cert.c_str())){
       if(WisLTE.returnErrorCode(f_err_code)){
           if (f_err_code == 407){
               start_time = millis();
               while (!DeleteFiles((char *)ssl_client_cert_name)){
                   if(millis() - start_time >= 10*1000UL){
                       e_str = "\r\nSSL ERROR: The ssl ca cert file exists. An error occurred while deleting the original file during the re-upload process.\r\n";
                       strcpy(err_code, e_str);
                       return false;
                   }
               }
           }
       }else if(millis() - start_time >= 30*1000UL){
           sprintf(e_str, "\r\nSSL ERROR: Error uploading file, error code: %d ,Please check the corresponding documentation for details.\r\n", f_err_code);
           strcpy(err_code, e_str);
           return false;
       }
   }
   start_time = millis();
   cert="";
   for (unsigned int k = 0; k < strlen_P(pem_pkey); k++) {
     ch = pgm_read_byte_near(pem_pkey + k);
     cert.concat(ch);
   }
   while (!UploadFiles((const char *)ssl_client_key_name, cert.c_str())){
       if(WisLTE.returnErrorCode(f_err_code)){
           if (f_err_code == 407){
               start_time = millis();
               while (!DeleteFiles((char *)ssl_client_key_name)){
                   if(millis() - start_time >= 10*1000UL){
                       e_str = "\r\nSSL ERROR: The ssl ca cert file exists. An error occurred while deleting the original file during the re-upload process.\r\n";
                       strcpy(err_code, e_str);
                       return false;
                   }
               }
           }
       }else if(millis() - start_time >= 30*1000UL){
           sprintf(e_str, "\r\nSSL ERROR: Error uploading file, error code: %d ,Please check the corresponding documentation for details.\r\n", f_err_code);
           strcpy(err_code, e_str);
           return false;
       }
   }
  start_time = millis();
  while (!WisLTE.SetSSLCertificate(ssl_index, (char *) ssl_ca_cert_name, (char *) ssl_client_cert_name, (char *) ssl_client_key_name , true)) { //ssl_ca_cert_name, ssl_client_cert_name, ssl_client_key_name
    if (millis() - start_time >= 30 * 1000UL) {
      e_str = "\r\nSSL ERROR: An error occurred while setting the ssl certificate.\r\n";
      strcpy(err_code, e_str);
      return false;
    }
  }

  e_str = "\r\nSSL OK: The ssl were successfully initialized.\r\n";
  strcpy(err_code, e_str);
  return true;

}

bool EEPROM_wirte_data()
{
  if ( max_employee_number <= 0 | currentUpdateVer <= 1 | max_job_code <= 0 | max_cost_code <= 0 | max_run_hours <= 0) {
    currentUpdateVer = 1;
    max_employee_number = 500;
    max_job_code = 500;
    max_cost_code = 500;
    max_run_hours = 1000;
    return false;
  }
  else if (employee_number_digits < 4 | job_code_digits < 4 | cost_code_digits < 3 | run_hours_digits < 4) {
    employee_number_digits = 4;
    job_code_digits = 4;
    cost_code_digits = 3;
    run_hours_digits = 4;
    return false;
  }
  employee_input_info data_limits = {
    currentUpdateVer,
    max_employee_number,
    max_job_code,
    max_cost_code,
    max_run_hours,
    employee_number_digits,
    job_code_digits,
    cost_code_digits,
    run_hours_digits
  };
  EEPROM.put(0, data_limits);
  return true;
}

void EEPROM_read_data()
{
  employee_input_info data_limits;
  EEPROM.get(0, data_limits);
  currentUpdateVer = data_limits.currentUpdateVer;
  max_employee_number = data_limits.max_employee_number;
  max_job_code = data_limits.max_job_code;
  max_cost_code = data_limits.max_cost_code;
  max_run_hours = data_limits.max_run_hours;
  employee_number_digits = data_limits.employee_number_digits;
  job_code_digits = data_limits.job_code_digits;
  cost_code_digits = data_limits.cost_code_digits;
  run_hours_digits = data_limits.run_hours_digits;

  if ( max_employee_number <= 0 | currentUpdateVer <= 1 | max_job_code <= 0 | max_cost_code <= 0 | max_run_hours <= 0) {
    currentUpdateVer = 1;
    max_employee_number = 500;
    max_job_code = 500;
    max_cost_code = 500;
    max_run_hours = 1000;
  }
  if (employee_number_digits < 4 | job_code_digits < 4 | cost_code_digits < 3 | run_hours_digits < 4) {
    employee_number_digits = 4;
    job_code_digits = 4;
    cost_code_digits = 3;
    run_hours_digits = 4;
  }
  Serial.print("max_employee_number");
  Serial.println(max_employee_number);
  Serial.print("max_job_code");
  Serial.println(max_job_code);
  Serial.print("max_cost_code");
  Serial.println(max_cost_code);
  Serial.print("max_run_hours");
  Serial.println(max_run_hours);
  Serial.print("employee_number_digits");
  Serial.println(employee_number_digits);
  Serial.print("job_code_digits");
  Serial.println(job_code_digits);
  Serial.print("cost_code_digits");
  Serial.println(cost_code_digits);
  Serial.print("run_hours_digits");
  Serial.println(run_hours_digits);

}

void startRide(KeypadEvent input_key) {
  if (keypad.getState() == PRESSED)
  {
    if ( !isInputDone )
    {
      if ( !isInputStarted ) {
        isInputStarted = true;
        lcd.clear();
        lcd.setCursor(1, 1);
        lcd.print("Enter Employee Num");
        return;
      }
      if (employee_number.length() < employee_number_digits)
      {
        lcd.clear();
        lcd.setCursor(1, 1);
        lcd.print("Enter Employee Num");
        lcd.setCursor(2, 2);
        Serial.println();
        Serial.print("input key is: ");
        Serial.println(input_key);
        if (input_key == 'A' | input_key == 'B' | input_key == 'D' | input_key == '*' | input_key == '#')
        {
          Serial.println("Not Allowed!");
          lcd.clear();
          lcd.print("Input Not Allowed!");
          delay(500);
          lcd.clear();
          lcd.setCursor(1, 1);
          lcd.print("Enter Employee Num");
          lcd.setCursor(2, 2);
          lcd.print(employee_number);
          Serial.print("employee_number1 is: ");
          Serial.println(employee_number);
        }
        else if (input_key == 'C')
        {
          if (employee_number.length() > 0)
          {
            employee_number.remove(employee_number.length() - 1);
          }
          lcd.clear();
          lcd.setCursor(1, 1);
          lcd.print("Enter Employee Num");
          lcd.setCursor(2, 2);
          lcd.print(employee_number);
          Serial.print("employee_number2 is: ");
          Serial.println(employee_number);
        }
        else
        {
          employee_number.concat(input_key);
          lcd.print(employee_number);
          Serial.print("employee_number3 is: ");
          Serial.println(employee_number);
          inputStartTime = millis();
        }
        if (employee_number.toInt() < 0 | employee_number.toInt() > max_employee_number)
        {
          lcd.clear();
          lcd.print("Invalid Input!");
          Serial.print(" Invalid Input!");
          employee_number = "";
          lcd.clear();
          lcd.setCursor(1, 1);
          lcd.print("Enter Employee Num");
          return;
        }
        if (employee_number.length() == employee_number_digits) {
          lcd.clear();
          lcd.setCursor(1, 1);
          lcd.print("Enter Job Code");
          lcd.setCursor(2, 2);
          return;
        }
        return;
      }


      if (job_code.length() < job_code_digits)
      {
        lcd.clear();
        lcd.setCursor(1, 1);
        lcd.print("Enter Job Code");
        lcd.setCursor(2, 2);
        Serial.println(input_key);
        if (input_key == 'A' | input_key == 'B' | input_key == 'D' | input_key == '*' | input_key == '#')
        {
          Serial.println("Not Allowed!");
          lcd.clear();
          lcd.print("Input Not Allowed!");
          delay(500);
          lcd.clear();
          lcd.setCursor(1, 1);
          lcd.print("Enter Job Code");
          lcd.setCursor(2, 2);
          lcd.print(job_code);
          Serial.println(job_code);
        }
        else if (input_key == 'C')
        {
          if (job_code.length() > 0)
          {
            job_code.remove(job_code.length() - 1);
          }
          lcd.clear();
          lcd.setCursor(1, 1);
          lcd.print("Enter Job Code");
          lcd.setCursor(2, 2);
          lcd.print(job_code);
          Serial.println(job_code);
        }
        else
        {
          job_code.concat(input_key);
          lcd.print(job_code);
        }
        if (job_code.toInt() < 0 | job_code.toInt() > max_job_code)
        {
          lcd.clear();
          lcd.print("Invalid Input!");
          job_code = "";
          lcd.clear();
          lcd.setCursor(1, 1);
          lcd.print("Enter Job Code");
          return;
        }
        if (job_code.length() == job_code_digits) {
          lcd.clear();
          lcd.setCursor(1, 1);
          lcd.print("Enter Cost Code");
          lcd.setCursor(2, 2);
          return;
        }
        return;
      }



      if (cost_code.length() < cost_code_digits)
      {
        lcd.clear();
        lcd.setCursor(1, 1);
        lcd.print("Enter Cost Code");
        lcd.setCursor(2, 2);
        Serial.println(input_key);
        if (input_key == 'A' | input_key == 'B' | input_key == 'D' | input_key == '*' | input_key == '#')
        {
          Serial.println("Not Allowed!");
          lcd.clear();
          lcd.print("Input Not Allowed!");
          delay(500);
          lcd.clear();
          lcd.setCursor(1, 1);
          lcd.print("Enter Cost Code");
          lcd.setCursor(2, 2);
          lcd.print(cost_code);
          Serial.println(cost_code);
        }
        else if (input_key == 'C')
        {
          if (cost_code.length() > 0)
          {
            cost_code.remove(cost_code.length() - 1);
          }
          lcd.clear();
          lcd.setCursor(1, 1);
          lcd.print("Enter Cost Code");
          lcd.setCursor(2, 2);
          lcd.print(cost_code);
          Serial.println(cost_code);
        }
        else
        {
          cost_code.concat(input_key);
          lcd.print(cost_code);
        }
        if (cost_code.toInt() < 0 | cost_code.toInt() > max_cost_code)
        {
          lcd.clear();
          lcd.print("Invalid Input!");
          cost_code = "";
          lcd.clear();
          lcd.setCursor(1, 1);
          lcd.print("Enter Cost Code");
          return;
        }
        if (cost_code.length() == cost_code_digits) {
          lcd.clear();
          lcd.setCursor(1, 1);
          lcd.print("Enter Run Hours");
          lcd.setCursor(2, 2);
          return;
        }
        return;
      }



      if (run_hours.length() < run_hours_digits)
      {
        lcd.clear();
        lcd.setCursor(1, 1);
        lcd.print("Enter Run Hours");
        lcd.setCursor(2, 2);
        Serial.println(input_key);
        if (input_key == 'A' | input_key == 'B' | input_key == 'D' | input_key == '*' | input_key == '#')
        {
          Serial.println("Not Allowed!");
          lcd.clear();
          lcd.print("Input Not Allowed!");
          delay(500);
          lcd.clear();
          lcd.setCursor(1, 1);
          lcd.print("Enter Run Hours");
          lcd.setCursor(2, 2);
          lcd.print(run_hours);
          Serial.println(run_hours);
        }
        else if (input_key == 'C')
        {
          if (run_hours.length() > 0)
          {
            run_hours.remove(run_hours.length() - 1);
          }
          lcd.clear();
          lcd.setCursor(1, 1);
          lcd.print("Enter Run Hours");
          lcd.setCursor(2, 2);
          lcd.print(run_hours);
          Serial.println(run_hours);
        }
        else
        {
          run_hours.concat(input_key);
          lcd.print(run_hours);
        }
        if (run_hours.toInt() < 0 | run_hours.toInt() > max_run_hours)
        {
          lcd.clear();
          lcd.print("Invalid Input!");
          run_hours = "";
          lcd.clear();
          lcd.setCursor(1, 1);
          lcd.print("Enter Run Hours");
          return;
        }
        if (run_hours.length() == run_hours_digits) {
          lcd.clear();
          lcd.setCursor(1, 0);
          lcd.print("EN: ");
          lcd.print(employee_number);
          lcd.setCursor(11, 0);
          lcd.print("JC: ");
          lcd.print(job_code);
          lcd.setCursor(1, 1);
          lcd.print("CC: ");
          lcd.print(cost_code);
          lcd.setCursor(11, 1);
          lcd.print("RH: ");
          lcd.print(run_hours);
          lcd.setCursor(1, 2);
          lcd.print("A: Save");
          lcd.setCursor(1, 3);
          lcd.print("B: Re-Enter");
          Serial.println("at the end: ");
          Serial.println(employee_number);
          Serial.println(job_code);
          Serial.println(cost_code);
          Serial.println(run_hours);
          return;
        }
        return;
      }
      if (input_key == 'A') {
        Serial.println("pressed A");
        isInputDone = true;
        isInputStarted = false;
        setDefaultTime();
        return;
      }
      else if (input_key == 'B') {
        employee_number = "";
        job_code = "";
        cost_code = "";
        run_hours = "";
        isInputStarted = false;
        return;
      }
      else {
        return;
      }
    }
    else
    {
      if ( !isInputStarted ) {
        isInputStarted = true;
        lcd.clear();
        lcd.setCursor(1, 1);
        lcd.print("Are you Sure?");
        lcd.setCursor(1, 2);
        lcd.print("A: Yes  B: No");
        inputStartTime = millis();
        return;
      }

      if (input_key == 'B') {
        isInputDone = true;
        isInputStarted = false;
        inputStartTime = 0;
        return;
      }
      else if (input_key == 'A') {
        employee_number = "";
        job_code = "";
        cost_code = "";
        run_hours = "";
        prevLat = 0;
        prevLong = 0;
        disTravelled = 0;
        isInputStarted = false;
        isInputDone = false;
        inputStartTime = 0;
        return;
      }
      else {
        return;
      }
    }

  }
  return;
}

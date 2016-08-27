// Please pick either RFM69_ONLY for RFM69 module support, or comment
// it out for ESP8266-01 module support. In RFM69 mode, this device will sends
// data to gateway which is a RFM69-enabled server device. The gateway will handle
// all other operations such as send alert email/SMS etc. In ESP8266 mode, the alert
// email will be sent from this device directly.

//#define RFM69_ONLY
#define LCD_MONITOR

#ifdef LCD_MONITOR
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#endif

#ifdef RFM69_ONLY
#include <RFM69.h>         //get it here: http://github.com/lowpowerlab/rfm69
#include <WirelessHEX69.h> //get it here: https://github.com/LowPowerLab/WirelessProgramming
#include <SPI.h>           //comes with Arduino IDE (www.arduino.cc)
#include <LowPower.h>
#else
#include <SoftwareSerial.h>
#include "ESP8266.h"
#include <avr/pgmspace.h>
#endif

#define SERIAL_EN         //comment out if you don't want any serial output
#define BUZZER_ENABLE     //uncomment this line if you have the BUZZER soldered and want the sketch to 

#define SENDLOOPS 40 //default:80 //if no message was sent for this many sleep loops/cycles, then force a send
#define READ_SAMPLES 3
#define HISTERESIS 1.3  //(cm) only send a message when new reading is this many centimeters different
#define DIST_READ_LOOPS 2 //read distance every this many sleeping loops (ie if sleep time is 8s then 2 loops => a read occurs every 16s)
//*****************************************************************************************************************************

#ifdef RFM69_ONLY
//*****************************************************************************************************************************
// ADJUST THE SETTINGS BELOW DEPENDING ON YOUR HARDWARE/TRANSCEIVER SETTINGS/REQUIREMENTS
//*****************************************************************************************************************************
#define GATEWAYID   1    // this is the node ID of your gateway (which is probably tied to a Raspberry Pi)
#define NODEID      96   // must be unique to each sensor node
#define NETWORKID   100  // every node must match the same network ID to hear messages from each other and take actions directly
#define FREQUENCY   RF69_433MHZ
#define ENCRYPTKEY  "sampleEncryptKey" //has to be same 16 characters/bytes on all nodes, not more not less!
#define IS_RFM69HW      //uncomment only for RFM69HW! Leave out if you have RFM69W!

#define SLEEP_FASTEST SLEEP_15MS
#define SLEEP_FAST SLEEP_250MS
#define SLEEP_SEC SLEEP_1S
#define SLEEP_LONG SLEEP_2S
#define SLEEP_LONGER SLEEP_4S
#define SLEEP_LONGEST SLEEP_8S
period_t sleepTime = SLEEP_LONG; //period_t is an enum type defined in the LowPower library (LowPower.h)
#endif

#define BATT_MONITOR  A3   //through 1Meg+470Kohm and 0.1uF cap from battery VCC - this ratio divides the voltage to bring it below 3.3V where it is scaled to a readable range
#define BATT_READ_LOOPS  SENDLOOPS  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings

#ifdef SERIAL_EN
  #define SERIAL_BAUD     115200
  #define DEBUG(input)   {Serial.print(input); delay(1);}
  #define DEBUGln(input) {Serial.println(input); delay(1);}
  #define SERIALFLUSH() {Serial.flush();}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
  #define SERIALFLUSH();
#endif

#define LEDLEAK       5   // LED output pin for leak detection, this will go high if either water sensor 1 or 2 detects water
#define BUZZER        7		// used for audio notifications of water leaks or end of the dryer run cycle
#define LEAKSENSOR    A2
#define TRIG          4  // digital pin wired to TRIG pin of ultrasonic sensor
#define ECHO          3  // digital pin wired to ECHO pin of ultrasonic sensor
#define MAX_DISTANCE 150  // maximum valid distance
#define MIN_DISTANCE   2  // minimum valid distance
#define MAX_ADJUST_DISTANCE (MAX_DISTANCE-GRN_LIMIT_UPPER)   //this is the amount by which the RED_LIMIT_UPPER can by increased
#define LED           13  // Moteinos have LEDs on D9 and Arduino Uno on D13

#define FUNC_PIN  6 // RFM69/ESP8266 selection - high : RFM69, low : ESP8266

#ifndef RFM69_ONLY

const char  SSID[] PROGMEM = {"yourWifiSSID"};
const char  PASSWORD[] PROGMEM = {"WifiPassword"};
const char EMAIL_TO[] PROGMEM = "yourEmailAddress";
const char SUBJECT_LEAK[] PROGMEM = "Leaking...";
const char EMAIL_CONTENT_LEAK[] PROGMEM = "Water on the floor!";
const char SUBJECT_LEVEL[] PROGMEM = "Sump pump Alert...";
const char EMAIL_CONTENT_LEVEL[] PROGMEM = "Water is only 20cm below surface and rising!";
const char EMAIL_FROM[] PROGMEM = "emailFromAddress";
const char SMTPSERVER[] PROGMEM = "SMTP-Server-e.g.- smtp.comcast.net";
const char EMAIL_FROM_BASE64[] PROGMEM = "base64-encoded-emailFromAddress"; //https://www.base64encode.org/
const char EMAIL_PASSWORD_BASE64[] PROGMEM = "base64-encoded-password"; 
#define SMTPPORT  587

#define ESPRX_PIN  8  // UNO : 8 Mega : 10
#define ESPTX_PIN  9  // UNO : 9 Mega : 11
#endif

byte sendLen;
byte sendLoops=0;
byte distReadLoops=0;
byte battReadLoops=0;
float distance=0;
float prevDistance=0;
float batteryVolts = 5;
char* BATstr="BAT:5.00v"; //longest battery voltage reading message = 9chars
char* DISTstr="99999.99cm"; //longest distance reading message = 5chars
float readDistance(byte samples=1); //take 1 samples by default

int leakSensor1value;
boolean leakStatus1 = false;        // the ongoing state to compare to the current status to see if it changed
boolean currentLeakStatus1 = false; // value measured every time in the loop

#ifdef RFM69_ONLY
char payload[] = "123 ABCDEFGHIJKLMNOPQRSTUVWXYZ";
RFM69 radio;
#endif

int ModeSelectionState = 0;
int RFM69_MODE = 0;

#ifdef LCD_MONITOR
// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
#endif

#ifndef RFM69_ONLY
// Mega and Mega 2560 : only the following can be used for RX: 10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8 (62),
// A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68), A15 (69).
SoftwareSerial mySerial(ESPRX_PIN, ESPTX_PIN);
ESP8266 wifi(mySerial, 9600); // make sure the ESP8266-01 baud rate is 9600
#endif

void setup() {
  pinMode(FUNC_PIN, INPUT);
  pinMode(LEAKSENSOR, INPUT);
  
#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
#endif

#ifdef RFM69_ONLY
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  #ifdef IS_RFM69HW
    radio.setHighPower(); //uncomment only for RFM69HW!
  #endif
  radio.encrypt(ENCRYPTKEY);

  delay(2000); 
#endif

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

#ifdef BUZZER_ENABLE
  pinMode(BUZZER, OUTPUT);
  buzzer(50, 2, 100);
#endif

#ifdef LCD_MONITOR
  lcd.begin(20,4);         // initialize the lcd for 20 chars 4 lines, turn on backlight
  lcd.clear();
  writeStaticMsg("Distance: ", 0, 1);
  writeStaticMsg("Leak Status: ", 0, 2);
  writeStaticMsg("Voltage: ", 0, 3);
  writeStaticMsg("Mode:", 14, 0);
#endif

  ModeSelectionState = digitalRead(FUNC_PIN);
  if (ModeSelectionState == HIGH) {
    RFM69_MODE = 1;
#ifdef LCD_MONITOR
    putItOnScreen("R", "", 19, 0);
#endif
  }
  else {
#ifdef LCD_MONITOR
    putItOnScreen("E", "", 19, 0);
#endif   
#ifndef RFM69_ONLY
    // ESP8266 settings
    if (wifi.setOprToStation()) {
      DEBUGln("to station OK");
    }
    else {
      DEBUGln("to station Error");
    }

    if (wifi.disableMUX()) {
      DEBUGln("Disable Mux (multi connection) ... OK"); 
    }
    else {
      DEBUGln("Disable Mux ... Error");
    }

    char buf_ssid[6];
    char buf_pwd[16];
    strcpy_P(buf_ssid, SSID);
    strcpy_P(buf_pwd, PASSWORD);
  
    if (wifi.joinAP(buf_ssid, buf_pwd)) {
        DEBUG("Join AP success. ");
        DEBUGln("IP: ");       
        DEBUGln(wifi.getLocalIP().c_str());
    } else {
        DEBUGln("Join AP failure");
    }
#endif
  }

  readDistance(); //first reading seems to always be low
  readBattery();
}

#ifdef LCD_MONITOR
void writeStaticMsg(const char* msg, int col, int row) {
  lcd.setCursor(col, row);
  lcd.print(msg);
}

void writeStaticMsg(int asciiInt) {
  lcd.write(asciiInt);
}

void putItOnScreen(float st, int decimales, const char*unit)
{
  char sp[16];
  dtostrf( st, 4, decimales, sp );
  // send *s as bytes of date
  int len = strlen(sp);
  for (int i=0; i<len; i++){
    lcd.write(sp[i]);
  }
  
  lcd.write(unit);
}

void putItOnScreen(const char* value, const char*unit, int col, int row)
{
  lcd.setCursor(col,row);
  for (int i = 0; i < 20 - col; i++) {
    lcd.write(" ");
  }
  lcd.setCursor(col,row);
  lcd.write(value);
  lcd.write(unit);
}

#endif

bool send_flag = false;
long previousMillis = 0;
  int count = 0;

void loop() {
  char buff[25];
  const char* send_type="";
  
  unsigned long currentMillis = millis();
  
  DEBUGln("Start...");
  leakSensor1value = analogRead(LEAKSENSOR);
  Blink(LEDLEAK, 20);
  
  // Setup the current values of the leak sensors so they can be compared to the stored state
  if (leakSensor1value > 850) 
    currentLeakStatus1 = false;
  else currentLeakStatus1 = true;

  if (leakSensor1value < 850 && (currentLeakStatus1 != leakStatus1)){
    DEBUGln("LEDLEAK - High");
  	digitalWrite(LEDLEAK, HIGH);
   	leakStatus1 = true;
#ifdef RFM69_ONLY
    transmitStatus(9901, leakStatus1);  // we created '9901' to mean its water leak sensor 1
#else
    send_flag = true;
    send_type = "leak";
#endif
  }
  if (leakSensor1value > 850 && (currentLeakStatus1 != leakStatus1)){
    digitalWrite(LEDLEAK, LOW);
  	leakStatus1 = false;
#ifdef RFM69_ONLY
    transmitStatus(9901, leakStatus1);
#endif
  }
    
  if (leakStatus1 == true){  // flash LED and beeper in case of a water leak detected
#ifdef LCD_MONITOR
    putItOnScreen("Leaking", "", 13, 2);
#endif
    LEDONOFF(LEDLEAK, "on");
    buzzer(50, 2, 100);
  }
  else {
#ifdef LCD_MONITOR
    putItOnScreen("Dry", "", 13, 2);
#endif
    LEDONOFF(LEDLEAK, "off");
  }

  if (battReadLoops--<=0) {//only read battery every BATT_READ_LOOPS cycles
    readBattery();
    battReadLoops = BATT_READ_LOOPS-1;
  }
  
  if (distReadLoops--<=0) {
    distance = readDistance(READ_SAMPLES);
    distReadLoops = DIST_READ_LOOPS-1;
  }

  float diff = distance - prevDistance;
  DEBUG("distance="); DEBUG(distance); 
  DEBUG(" ");
  if ((diff > HISTERESIS || diff < -HISTERESIS) || (sendLoops--<=0)) { 
    //only send a new message if the distance has changed more than the HISTERESIS or if sendloops has expired
    if (distance > MAX_DISTANCE || distance < MIN_DISTANCE)
      DISTstr = "0"; // zero, out of range
    else {
      dtostrf(distance,3,2, DISTstr);
      if (distance < 20) {
        DEBUGln(currentMillis - previousMillis);
        if ((previousMillis == 0) || ((currentMillis - previousMillis) > 60000)) {
          previousMillis = currentMillis;

          send_flag = true;
          send_type = "level";
        }
      }
      else {
        previousMillis = 0;
      }
    }

    dtostrf(batteryVolts,3,2, BATstr);
    if (diff > HISTERESIS || diff < -HISTERESIS) {
      sprintf(buff, "%scm BAT:%s", DISTstr, BATstr);
    } else {
      //distance has not changed significantly so only send last battery reading
      sprintf(buff, "BAT:%s", BATstr);
    }
    sendLen = strlen(buff);

    digitalWrite(LED, HIGH);
    DEBUG(buff);
    
    if (RFM69_MODE) {
#ifdef RFM69_ONLY
      if (radio.sendWithRetry(GATEWAYID, buff, sendLen)) {
        prevDistance = distance;
        DEBUG(" - ACK:OK! RSSI:");
        DEBUGln(radio.RSSI);
      }
      else DEBUGln(" - ACK:NOK...");
#endif
    }
#ifdef LCD_MONITOR
    // send to LCD
    putItOnScreen(DISTstr, "cm", 10, 1);
    putItOnScreen(BATstr, "v", 9, 3);    
#endif
    digitalWrite(LED, LOW);
    //reset send loop counter
    sendLoops = SENDLOOPS-1;
  }

  if (RFM69_MODE) {
#ifdef RFM69_ONLY
    radio.sleep();
    SERIALFLUSH();

    LowPower.powerDown(sleepTime, ADC_OFF, BOD_OFF); //put microcontroller to sleep to save battery life
#endif
  }
  else {
    delay(2000);
  }

  // send email
  if(send_flag){
    DEBUGln("Sending email...");
    do {
      if(send_flag){
        if(do_next(send_type)){ 
          DEBUGln(count);
          count++;
        }
      }
    }
    while (send_flag == true);
    DEBUGln("Email sent.");
  }
}

#ifdef RFM69_ONLY
void transmitStatus(int item, int status){  
    //delay(50);
    sprintf(payload, "%d:%d", item, status);
    byte buffLen=strlen(payload);
    radio.sendWithRetry(GATEWAYID, payload, buffLen);
}
#endif

#ifdef BUZZER_ENABLE
void beep(unsigned char delayPeriod, unsigned char beepNumber){
  for (int b=0; b < beepNumber; b++) {
    for (int i=0; i < delayPeriod; i++) {
      digitalWrite(BUZZER, HIGH);                     // Almost any value can be used except 0 and 255
                                                      // experiment to get the best tone
      delayMicroseconds(150 + delayPeriod);           // wait for a delayms ms - 192 was orig value
      digitalWrite(BUZZER, LOW);                   // 0 turns it off
      delayMicroseconds(150 + delayPeriod);           // wait for a delayms ms   
    }
    
    delay(100);
  }
}  
#endif

void readBattery()
{
  unsigned int readings=0;
  for (byte i=0; i<5; i++) { //take several samples, and average
    readings += analogRead(BATT_MONITOR);
  }
  
  batteryVolts = ((float)(readings/5.0) * 3.3 * 5)/(1023*2.976);
}

float readDistance(byte samples)
{
  if (samples == 0) samples = 1;
  if (samples > 10) samples = 10;

#ifdef RFM69_ONLY
  //need about 60-75ms after power up before HC-SR04 will be usable, so just sleep in the meantime
  LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
#else
  delay(75);
#endif
  PING();
#ifdef RFM69_ONLY
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
#else
  delay(15);
#endif
  
  unsigned long duration = 0;
  for (byte i=0; i<samples; i++)
  {
    duration += PING();
    if (samples > 1) {
#ifdef RFM69_ONLY
      LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
#else
      delay(15);
#endif
    }
  }

  return microsecondsToCentimeters(duration / samples);
}

long PING()
{
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH);
}

float microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74.0 / 2.0f;
}

float microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return (float)microseconds / 29.0f / 2.0f;
}

void Blink(byte pin, byte delayMs)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(delayMs);
  digitalWrite(pin, LOW);
  delay(delayMs);
}

void LEDONOFF(byte pin, const char* value)
{
  pinMode(pin, OUTPUT);
  if (value == "on") {
    digitalWrite(pin, HIGH);
  }
  else {
    digitalWrite(pin, LOW);
  }
}

boolean do_next(const char* send_type)
{
    char *tempStr;
    char tempChars[50];
    char buff_1[50];

    switch(count){ 
    case 0:{
        strcpy_P(buff_1, SMTPSERVER);
        return wifi.createTCP(buff_1, SMTPPORT);
        break;
    }
    case 1:
        tempStr = "HELO computer.com\r\n";
        return wifi.send((const uint8_t*)tempStr, strlen(tempStr)); 
        break;
    case 2:
        tempStr = "AUTH LOGIN\r\n";
        return wifi.send((const uint8_t*)tempStr, strlen(tempStr)); 
        break;
    case 3:
        strcpy_P(buff_1, EMAIL_FROM_BASE64);
        return wifi.send((const uint8_t*)buff_1, strlen(buff_1));
        break;
    case 4:
        tempStr = "\r\n";
        return wifi.send((const uint8_t*)tempStr, strlen(tempStr));
        break;
    case 5:
        // send password base 64 encoded, upon successful login the server will reply with 235.
        strcpy_P(buff_1, EMAIL_PASSWORD_BASE64);
        return wifi.send((const uint8_t*)buff_1, strlen(buff_1)); 
        break;
    case 6:
        tempStr = "\r\n";
        return wifi.send((const uint8_t*)tempStr, strlen(tempStr));
        break;
    case 7: {
        // send "MAIL FROM:<emali_from@domain.com>" command
        strcpy_P(buff_1, EMAIL_FROM);
        strcpy(tempChars, "MAIL FROM:<"); // If 50 is not long enough change it, do the same for the array in the other cases
        strcat(tempChars,buff_1);
        strcat(tempChars,">\r\n");
        tempStr = tempChars;
        return wifi.send((const uint8_t*)tempStr, strlen(tempStr));         
        break;
    }
    case 8: {
        // send "RCPT TO:<email_to@domain.com>" command
        strcpy_P(buff_1, EMAIL_TO);
        strcpy(tempChars, "RCPT TO:<");
        strcat(tempChars,buff_1);
        strcat(tempChars,">\r\n");  
        tempStr = tempChars; 
        return wifi.send((const uint8_t*)tempStr, strlen(tempStr));
        break;
    }
    case 9:
        // Send "DATA"  command, the server will reply with something like "334 end message with \r\n.\r\n."
        tempStr = "DATA\r\n";
        return wifi.send((const uint8_t*)tempStr, strlen(tempStr));
        break;
    case 10: {
        // apply "FROM: from_name <from_email@domain.com>" header
        strcpy_P(buff_1, EMAIL_FROM);
        strcpy(tempChars, "FROM: ");
        strcat(tempChars,buff_1);
        strcat(tempChars," ");
        strcat(tempChars,"<");
        strcat(tempChars,buff_1);
        strcat(tempChars,">\r\n");
        tempStr = tempChars;
        return wifi.send((const uint8_t*)tempStr, strlen(tempStr));
        break;  
    }
    case 11: {
        strcpy_P(buff_1, EMAIL_TO);
        strcpy(tempChars, "TO: ");
        strcat(tempChars,buff_1);
        strcat(tempChars," ");
        strcat(tempChars,"<");
        strcat(tempChars,buff_1);
        strcat(tempChars,">\r\n");
        tempStr = tempChars;
        return wifi.send((const uint8_t*)tempStr, strlen(tempStr));
        break;
    }
    case 12: {
        if (send_type == "leak")
          strcpy_P(buff_1, SUBJECT_LEAK);
        else
          strcpy_P(buff_1, SUBJECT_LEVEL);
        strcpy(tempChars, "SUBJECT: ");
        strcat(tempChars, buff_1);
        tempStr = tempChars;
        return wifi.send((const uint8_t*)tempStr, strlen(tempStr));
        break;
    }
    case 13: 
        tempStr = "\r\n\r\n\r\n";
        return wifi.send((const uint8_t*)tempStr, strlen(tempStr));
        break;
    case 14:
        if (send_type == "leak")
          strcpy_P(buff_1, EMAIL_CONTENT_LEAK);
        else
          strcpy_P(buff_1, EMAIL_CONTENT_LEVEL);
        return wifi.send((const uint8_t*)buff_1, strlen(buff_1));
        break;
    case 15:
        tempStr = "\r\n.\r\n";
        return wifi.send((const uint8_t*)tempStr, strlen(tempStr));
        break;
    case 16:
        tempStr = "QUIT";
        return wifi.send((const uint8_t*)tempStr, strlen(tempStr));
        break;
    case 17:
        wifi.releaseTCP();
        return true;
        break;
    case 18:
        send_flag = false;
        count = 0;
        DEBUGln("Done!");
        return false; 
        break;
    default:
        break;
    }
}
#ifdef BUZZER_ENABLE
void buzzer(byte soundTime, byte repeats, byte repeatsDelay)
{
  for (byte i=0;i<=repeats;i++)
  {
    tone(BUZZER, 5500); //4500hz makes a nice audible sound from a 3.3v Moteino digital pin
    delay(soundTime);
    noTone(BUZZER);
    if (repeats>0) delay(repeatsDelay);
  }
}
#endif

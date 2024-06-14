#include <GSMSimSMS.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Define the serial ports for GSM and GPS
HardwareSerial SerialGSM(1); // UART1

#define GSM_RX_PIN 16
#define GSM_TX_PIN 17

// // Define pins for ultrasonic sensor
// const int trigPin = 9;
// const int echoPin = 10;
// Define the pins for the ultrasonic sensor
const int trigPin = 9;
const int echoPin = 11;
#define RESET_PIN 10

static volatile int num = 0;
GSMSimSMS sms(SerialGSM, RESET_PIN); // GSMSimSMS inherits from GSMSim.

unsigned long previousMillis = 0;
const long interval = 60000;

void setup() {
  Serial.begin(115200); // Serial for debug

  SerialGSM.begin(115200, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);

  while (!SerialGSM) {
    ; // wait for module to connect
  }
}

void loop() {
  Serial.println("Begin to listen to incoming messages...");
  checkmessage();
  delay(2000);

  while (SerialGSM.available()) {
    Serial.println(SerialGSM.readString());
  }

  while (Serial.available()) {
    SerialGSM.println(Serial.readString());
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    sendsms();
  }
}

int sendsms() {
//  boolean newData = false;

  if (true) {
    // newData = false;
    Serial.print("Start...");
    sms.init();

    Serial.print("Set Phone Function... ");
    Serial.println(sms.setPhoneFunc(1));
    delay(1000);

    Serial.print("is Module Registered to Network?... ");
    Serial.println(sms.isRegistered());
    delay(1000);

    Serial.print("Signal Quality... ");
    Serial.println(sms.signalQuality());
    delay(1000);

    Serial.print("Operator Name... ");
    Serial.println(sms.operatorNameFromSim());
    delay(1000);

    Serial.print("Init SMS... ");
    Serial.println(sms.initSMS());
    delay(1000);

    Serial.print("List Unread SMS... ");
    Serial.println(sms.list(true));
    delay(1000);


    String postData = 
    delay(2000);
    Serial.println(postData);

    // Convert String to char*
    char postDataChar[postData.length() + 1];
    postData.toCharArray(postDataChar, postData.length() + 1);

    Serial.print("SMS to any number... ");
    Serial.println(sms.send("", postDataChar));
  }
  return 1;
}

void checkmessage() {
  if (SerialGSM.available()) {
    String buffer = SerialGSM.readString();
    num++;
    Serial.print(num);
    Serial.print(". ");

    if (buffer.indexOf("+CMTI:") != -1) {
      Serial.print("SMS Index No... ");
      int indexno = sms.indexFromSerial(buffer);
      Serial.println(indexno);

      Serial.print("Who sent the message?...");
      Serial.println(sms.getSenderNo(indexno));

      Serial.print("Read the message... ");
      Serial.println(sms.readFromSerial(buffer));
    } else {
      Serial.println(buffer);
    }
  }
}

int8_t sendATcommand(char* ATcommand, char* expected_answer, unsigned int timeout) {
  uint8_t x = 0, answer = 0;
  char response[100];
  unsigned long previous;

  memset(response, '\0', 100);
  delay(100);

  while (SerialGSM.available() > 0) SerialGSM.read();

  if (ATcommand[0] != '\0') {
    SerialGSM.println(ATcommand);
  }

  x = 0;
  previous = millis();

  do {
    if (SerialGSM.available() != 0) {
      response[x] = SerialGSM.read();
      x++;
      if (strstr(response, expected_answer) != NULL) {
        answer = 1;
      }
    }
  } while ((answer == 0) && ((millis() - previous) < timeout));

  Serial.println(response);
  return answer;
}

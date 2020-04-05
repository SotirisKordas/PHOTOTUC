

// nRF24L01 radio transceiver external libraries
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <printf.h>
#include "TimerObject.h"

// Enumeration defined to represent the state of the state machine that we are in 
enum stage {
  SYS_INIT,
  USER_CONFIG,
  SYS_TRIG
};

enum stage systemState = SYS_INIT;


int dummyZero = -2; // variable helpful as dummy Threshold
int recInit = -1; // variable helpful as a recognition message durin System Init

int sensorPin = A0; //defines the Pin number where we get the sensor value
int sensorValue = 0; //variable that holds the sensor value

// define daud rate
#define BAUD_RATE 57600

// chip select and RF24 radio setup pins
#define PIN_CE 9
#define PIN_CSN 10

// change this number for inclusion of more sensors. Maximum 6
#define MAX_SENSORS 2

// this constant is used for 6 repetitions of the ACK messages to the base for the Threshold and Operator messages
#define ACK_TRIES 6

//instance of RF24 radio
RF24 sensorRadio(PIN_CE, PIN_CSN);

// set this to appropriate sensor unit number minus 1
#define NODE_ID 1

// setup radio pipe addresses for the sensor to read and write
const uint64_t rAddresses[] = {0xB00B1E50D2LL, 0xB00B1E50C3LL};  //Create pipe addresses for the 2 nodes to recieve data, the "LL" is for LongLong type
const uint64_t wAddresses[] = {0xB00B1E50B1LL, 0xB00B1E50A4LL};  //Create pipe addresses for the 2 nodes to transmit data, the "LL" is for LongLong type

// simple array for each sensor data, in the form [sensor_id, return_count]
int baseData[2][2] = {{1,1}, {2,1}};

int initRC = 0;
int thresRC = 0;
int opRC = 0;
int initialDelay;
int currThresh;
int threshRCCtr = 0;
int currOP;
int opRCCtr = 0;


 
// here we declare a few timers that we are going to use
TimerObject *initRepeat = new TimerObject(1050);
TimerObject *dataRepeat = new TimerObject(1050);
TimerObject *threshRCRepeat = new TimerObject(550);

void setup() {
  // put your setup code here, to run once:
  randomSeed(analogRead(0)); //create unique seed value for random number generation
  initialDelay = random(200,300);
  Serial.begin(BAUD_RATE);
  pinMode(sensorPin, INPUT);
  
  printf_begin();
  sensorRadio.begin();
  sensorRadio.setPALevel(RF24_PA_LOW);
  //sensorRadio.setDataRate(RF24_1MBPS);
  //sensorRadio.setChannel(0x76);
  sensorRadio.enableDynamicPayloads();
  sensorRadio.setAutoAck(false);
  sensorRadio.stopListening();
  sensorRadio.openReadingPipe(1, rAddresses[NODE_ID]);
  sensorRadio.openWritingPipe(wAddresses[NODE_ID]);
  sensorRadio.printDetails();
  //sensorRadio.setRetries(5,5);
  //sensorRadio.enableAckPayload();
  //sensorRadio.setPayloadSize(sizeof(baseData));
  initRepeat->setOnTimer(&waitForBaseInit);
  dataRepeat->setOnTimer(&sendCurrData);
  threshRCRepeat->setOnTimer(&sendTRC);
  delay(initialDelay); // this introduces a small delay before starting to send init messages to avoid conflicts with other sensors
  initRepeat->Start();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(systemState == SYS_INIT) {
    if(initRC==0){
      initRepeat->Update();
    } else {
        initRepeat->Stop();
        systemState = USER_CONFIG ;
        delay(20000);
        dataRepeat->Start();
    }
  } else if (systemState == USER_CONFIG){
      dataRepeat->Update();
      waitForThreshold();
      if(thresRC == 1) {
        if(threshRCCtr < ACK_TRIES) {
          sendTRC();
          threshRCCtr++;
        } else {
            waitForThreshold();
        }
      }
      if(opRC == 1) {
        if(opRCCtr < ACK_TRIES) {
          sendORC();
          opRCCtr++;
      } else {
          endConfig();
      }
    }
  } else if (systemState == SYS_TRIG) {
     Serial.println("Trigger stage");
  }
}

// This function is used during Initialization of the system. The sensor
// sends an Init message to the base. If the message was sent then the 
// sensor goes to RX mode and waits for 200ms to get a reply message. If
// the base responds then the InitRC variable is set, to let the sensor know
// that it gor recognized. Then the sensor goes to the stage of User Config
// If after 200ms the base hasn't responded, the sensor resndes the message
// in intervals of 1s.  
void waitForBaseInit (void) {
  byte pipeN=0; 
  int gotByte;
  if(sensorRadio.write(&recInit, sizeof(recInit))) {
    sensorRadio.startListening();
    unsigned long startTimer = millis();
    bool timeout = false;
    while(!sensorRadio.available() && !timeout) {
      if(millis()-startTimer>200) {
        timeout =true;
      }
    }
    if(timeout) {
      Serial.println("No base response for System Init");
    }
    else {
      sensorRadio.read(&gotByte, sizeof(gotByte));
      if(gotByte == -1) {
         Serial.println("The base got the System Init value");
         initRC=1;
      }
    }
    sensorRadio.stopListening();
  }else {
     Serial.println("The System Init message wasn't sent");
  }
}

// When in Use Config stage the sensors should send their current value in 
// intervals according to the Timers used for the call of this function. 
void sendCurrData (void) {
  sensorRadio.stopListening();
  baseData[NODE_ID][1] = analogRead(sensorPin);
  if(sensorRadio.write(&baseData[NODE_ID], sizeof(baseData[NODE_ID]))) {
    Serial.println("Current sensor value succesfully sent to base");
  }
  sensorRadio.startListening();
}

// This function is used in User Config stage to check if the base has sent
// a message that has a Threshold value. If so, the sensor goes in Trigger stage
void waitForThreshold (void) {
  int gotByte;
  if(sensorRadio.available()) {
    sensorRadio.read(&gotByte, sizeof(gotByte));
    if(gotByte == -2) {
      if(thresRC==0) {
        thresRC=1;
        currThresh = gotByte;
        dataRepeat->Stop();
        //sensorRadio.flush_tx();
      } 
    } else if((gotByte==-3) || (gotByte==-4)) {
        if(opRC==0) {
          opRC=1;
          currOP = gotByte;   
        }
    }
  } 
} 

// This function is called from the MCU of the sensor 
// to send an ACK to the base after receiving the threshold
void sendTRC (void) {
  sensorRadio.stopListening();
  if(sensorRadio.write(&dummyZero, sizeof(dummyZero))) {
    Serial.println("Base got Threshold ack");
  }
  sensorRadio.startListening();
}

// This function is called from the MCU of the sensor 
// to send an ACK to the base after receiving the operator
void sendORC (void) {
  sensorRadio.stopListening();
  if(sensorRadio.write(&currOP, sizeof(currOP))) {
    Serial.println("Base got the Operator ack");
  }
  sensorRadio.startListening();
}

// this function is used to end the user configuration stage and move to trigger stage
void endConfig (void) {
  systemState = SYS_TRIG;
}


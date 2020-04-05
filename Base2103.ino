/*************************************************************************
   Base unit - nRF24L01+ radio communications
 *                                                                       *
        This is the base unit of our system

        At initiliazitation stage the Base is in PRX mode waiting
        a recognition message for the sensors. At User Config stage 
        the Base stays in PRX mode, sending a negative value as ACK
        messages to the Sensors. When User Config is done the Base 
        remains in PRX mode, awaiting for the Sensors to send a 
        possible trigger value. This code is testing for this procedure.
 *                                                                       *
        Author: S. Kordas
 *                                                                       *
          Last modified: 21/03/2019
 *                                                                       *
 *************************************************************************/

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <printf.h>
#include <TimerObject.h>

// Enumeration defined to represent the state of the state machine that we are in 
enum stage {
  SYS_INIT,
  USER_CONFIG,
  SYS_TRIG
};

enum stage systemState = SYS_INIT;


int dummyZero = -2; // variable that helps us change to Trigger mode 

// Variable table that holds operator > or < for each sensor  
// During User Config stage depending on user choice the value might change
// form -3 to -4. -3 means > and -4 means <

int dummyOp [2] = {-3,-3};

int recInit= -1;
#define confirm 2 
boolean confirmPress=false;

// define daud rate
#define BAUD_RATE 57600

// chip select and RF24 radio setup pins
#define PIN_CE 9
#define PIN_CSN 10


#define MAX_SENSORS 2 // change that to add more sensors to your system

uint8_t sensorCtr = 0; // variable that holds the number of sensors recognized in Initialization stage

uint8_t tmp = 0; // a variable that helps us determine if the initialization stage completed succesfully
uint8_t tmpTrig = 0;

RF24 baseRadio(PIN_CE,PIN_CSN);

// setup radio pipe addresses for each node for the base to read and write
// To add extra sensors, add the addresses for each sensor accordingly
// Also change the size of the tables 
const uint64_t wAddresses[] = {0xB00B1E50D2LL, 0xB00B1E50C3LL};  //Create pipe addresses for the 2 nodes to recieve data, the "LL" is for LongLong type
const uint64_t rAddresses[] = {0xB00B1E50B1LL, 0xB00B1E50A4LL};  //Create pipe addresses for the 2 nodes to transmit data, the "LL" is for LongLong type

// simple array for each sensor data, in the form [sensor_id, return_count]
// for support of more sensors change the table size and starting values accordingly
int sensorData[MAX_SENSORS][MAX_SENSORS] = {{1, 1}, {2, 1}};

// arrays to suggest that the Init and Threshold options were received
int sensorRCInit[MAX_SENSORS] = {0,0};
int sensorRCThresh [MAX_SENSORS] = {0,0};
int sensorRCOP [MAX_SENSORS] = {0,0};

TimerObject *initStart = new TimerObject(1000); 
byte initCount=0;
boolean initTimeout = false;

void setup() {
  // put your setup code here, to run once:
  pinMode(confirm, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(BAUD_RATE);
  printf_begin();
  Serial.println("Init test ");
  baseRadio.begin();
  baseRadio.setAutoAck(false);
  baseRadio.setPALevel(RF24_PA_LOW);
 // baseRadio.setDataRate(RF24_1MBPS);
  //baseRadio.setChannel(0x76);
  baseRadio.enableDynamicPayloads();
  //baseRadio.setRetries(5, 5);
  //baseRadio.enableAckPayload();
  //baseRadio.setPayloadSize(sizeof(sensorData));
  baseRadio.openReadingPipe(1,rAddresses[0]);
  baseRadio.openReadingPipe(2,rAddresses[1]);
  baseRadio.startListening();
  baseRadio.printDetails();
  initStart -> setOnTimer(&checkInitTimeout);
  initStart -> Start();
}


void checkConfirm () {
  if(digitalRead(confirm)) {
    confirmPress =!confirmPress;
    baseRadio.flush_rx();
  }
  //confirmPress =!confirmPress;
}

void loop () {
  // put your main code here, to run repeatedly:
  if(systemState == SYS_INIT) {
    initStart->Update();
    if(!initTimeout) {
      checkRadioForInit();
    } else if(initTimeout) {
        endInit();   
    }
  } else if(systemState == USER_CONFIG) {
      if(confirmPress) {
        sendThresh();
        int checkRC = checkRCThresh();
        if(checkRC == sensorCtr) {
          endConfig();
        }
      } else {
          getCurrentData();
          checkConfirm();
      }
  } else if(systemState == SYS_TRIG) {
      Serial.println("Trigger stage");
  }
}

// This function is called right after we get a recognition message from a sensor
// The functions responds to the sensor with the same value to let it know the base
// received the initialization message
bool sendRecValid (byte pipeNum) {
  bool worked;
  baseRadio.stopListening();
  baseRadio.openWritingPipe(wAddresses[pipeNum - 1]);
  if(!baseRadio.write(&recInit, sizeof(recInit))) {
    worked = false;
  } else {
      worked = true;
  }
  baseRadio.startListening();
  return worked; 
}




// The base should wait for Initilization messages from sensors for 15s
// This function checks if this specific amount of time has passed

void checkInitTimeout (void) {
  initCount++;
  if(initCount >= 15) { 
    initTimeout = true; 
  }
}


// This function checks the Radio for Initialization messages from sensors
// While there are payloads available the Base checks which sensor sent the
// payload. If the sensor hasn't already sent an Initialization message, then
// the corresponding value of the array that is used to know which sensor
// was already recognized is set to 1. If the sensor has already been recognized
// the message gets ignored. If the sensor has not been recognized before
// the base responds with the same Initialization message so that the sensor can know
// it got recognized. If the response fails to be sent, an accordng message is printed
void checkRadioForInit () {
  byte pipeNum = 0; 
  int gotByte = 0;
  while(baseRadio.available(&pipeNum)) {
    baseRadio.read(&gotByte, sizeof(gotByte));
    if(gotByte == -1) {
      if(sensorRCInit[pipeNum - 1] == 0) {
        if(sendRecValid(pipeNum)) {
          Serial.print("Sensor "); 
          Serial.print(pipeNum - 1);
          Serial.println(" got the recognition response");
          sensorRCInit[pipeNum - 1] = 1;
          sensorCtr++;
        } else {
            Serial.println("Response to the sensor failed");
        }
      }
    }  
  }
}

// This functions is used when we are ready to end the Initialization stage
// and go into User Configuration stage. 
void endInit (void) {
  if(sensorCtr != 0) {
    for(byte node = 0; node < MAX_SENSORS; node++) {
      if(sensorRCInit[node] == 1) {
        tmp++;
      }
    }
    if(tmp == sensorCtr) {
      systemState = USER_CONFIG ;
      Serial.print("Starting operation with "); 
      Serial.print(sensorCtr);
      Serial.println(" sensor/s");
    } else {
        Serial.println("Something went wrong with one of recognized sensors");
    }
     
  } else {
    Serial.println("No sensors were recognized. Reset the system");
    delay(1000);
  }
  initStart->Stop();
}

// This function is called from the base in User Config stage to get the sensor data
// The base waits for messages from the sensors, if there is a payload available
// the base shows the senssor number and the according value.
void getCurrentData () {
  byte pipeNum = 0;
  while(baseRadio.available(&pipeNum)) {
    baseRadio.read(&sensorData[pipeNum - 1], sizeof(sensorData[pipeNum - 1]));
    Serial.print(F("Sensor ")); 
    Serial.print(pipeNum - 1); 
    Serial.print(" sent value :");
    Serial.println(sensorData[pipeNum - 1][1]);
    baseRadio.flush_rx();
  }
}

void sendThresh (void) {
  int gotByteT;
  byte pipeNumT;
  int gotByteO;
  byte pipeNumO;
  for(byte node = 0; node < MAX_SENSORS; node++) {
    //baseRadio.stopListening();
    if((sensorRCInit[node] == 1) && (sensorRCThresh[node]) != 1) {
      baseRadio.stopListening();
      baseRadio.openWritingPipe(wAddresses[node]);
      if(baseRadio.write(&dummyZero, sizeof(dummyZero))) {
        baseRadio.startListening(); 
        unsigned long startTimer = millis();
        bool timeout = false;
        //pipeNum = node + 1;
        while(!baseRadio.available(&pipeNumT)) {
          if(millis() - startTimer > 1000) {
            timeout = true;
          }
        }
        if(timeout) {
          Serial.println("Sensor did not respond for the Threshold");
        } else {
            baseRadio.read(&gotByteT, sizeof(gotByteT));
            if(gotByteT == -2) {
              Serial.println(gotByteT);
              if(sensorRCThresh[pipeNumT - 1] == 0) {
                sensorRCThresh[pipeNumT - 1] = 1 ;
                Serial.print("Sensor "); 
                Serial.print(node); 
                Serial.println(" got the Threshold");
                delay(100);
                baseRadio.flush_rx();
              }
            }
        }     
      } else {
          Serial.println("Theshold failed to be sent to the sensor");
      }
      baseRadio.stopListening();
    }
    
    if((sensorRCThresh[node] == 1) && (sensorRCOP[node] != 1)) {
       baseRadio.stopListening();
       baseRadio.openWritingPipe(wAddresses[node]);
       if(baseRadio.write(&dummyOp[node], sizeof(dummyOp[node]))) {
         Serial.println("Actually sent the operator");
         baseRadio.startListening(); 
         unsigned long startTimer = millis();
         bool timeout = false;
         while(!baseRadio.available(&pipeNumO)) {
           if(millis() - startTimer > 1000) {
             timeout = true;
           }
         }
         if(timeout) {
           Serial.println("Sensor did not respond for the operator");
         } else {
             baseRadio.read(&gotByteO, sizeof(gotByteO));
             Serial.println(gotByteO);
             if((gotByteO == -3) || (gotByteO == -4)) {
               if(sensorRCOP[pipeNumO - 1] == 0) {
                 sensorRCOP[pipeNumO - 1] = 1;
                 Serial.print("Sensor "); 
                 Serial.print(node); 
                 Serial.println(" got the Operator");
                 baseRadio.flush_rx(); 
                 delay(100);
               }
             }
         }    
       }
    }
  }
  baseRadio.startListening();
}


int checkRCThresh (void) {
  int tmp=0;
  for(byte node = 0; node < MAX_SENSORS; node++) {
    if((sensorRCThresh[node] == 1) && (sensorRCOP[node] == 1)) {
      tmp++;
      //Serial.print("RCThresh = "); Serial.println(sensorRCThresh[node]);
    }
  }
  return tmp; 
}

void endConfig (void) {
  systemState = SYS_TRIG;
}


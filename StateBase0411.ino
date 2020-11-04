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
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <printf.h>
#include <TimerObject.h>

#include <OneButton.h>

//#include "SSD1306Ascii.h"
//#include "SSD1306AsciiWire.h"


// Enumeration defined to represent the state of the state machine that we are in 
enum stage {
  SYS_INIT,
  USER_CONFIG,
  SYS_TRIG
};

// Enumeration used to determine whether the timeout has passed for multiple sensors triggers to be "simultaneously" sent

enum stage systemState = SYS_INIT;

boolean trigFail = false;

#define MAX_SENSORS 2 // change that to add more sensors to your system

int dummyZero[MAX_SENSORS] = {100, 100}; // variable that helps us change to Trigger mode 

// Variable table that holds operator > or < for each sensor  
// During User Config stage depending on user choice the value might change
// form -3 to -4. -3 means > and -4 means <

int dummyOp[MAX_SENSORS] = {-3,-3};

int recInit= -1;
#define CONFIRM 2 
boolean confirmPress=false;


// define daud rate
#define BAUD_RATE 57600

// chip select and RF24 radio setup pins
#define PIN_CE 9
#define PIN_CSN 10

// define pin for the flash when it's to be triggered 
#define PIN_FLASH 6

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

// These variables are used with help of timers in order for the Initilization process to be exactly 15 seconds
TimerObject *initStart = new TimerObject(1000); 
byte initCount= 0;
boolean initTimeout = false;

// These variables are used with the help of a timer to check if the timeout window for the Thresholds has passed
TimerObject *triggTimer = new TimerObject(2);
byte triggCount = 0;
boolean triggTimeout = false; 

boolean triggerTrue[MAX_SENSORS] = {false, false}; // array of boolean values for each sensor that turn into true when the according sensor sends a trigger
byte triggerLogic = 1; // byte that represents if we want an OR logic or an AND logic between the sensor values in the trigger condition. 0 is for AND and 1 is for OR
byte newTrigger = false; //boolean value that will be used to determine if a message the base receives during SYS_TRIG is a new trigger for a sensor that hasn't already sent a trigger
int triggerCtr = 0; //variable that represents how many sensors have already sent their trigger value
bool timeout = false;
//unsigned long startTimer;

// The following variables are used for the menu in system configuration mode 
int pic = 0;
int maxPics_L1 = 5000;
int maxPics_L2 = 3;
int currentPointLine;
int menuLayer = 1; 

OneButton button(CONFIRM, true);

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


//#if (SSD1306_LCDHEIGHT != 64)
//#error("Height incorrect, please fix Adafruit_SSD1306.h!");
//#endif

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
//#define RST_PIN -1

//SSD1306AsciiWire display;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000L);
//  #if RST_PIN >= 0
//    display.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
//  #else // RST_PIN >= 0
//    display.begin(&Adafruit128x64, I2C_ADDRESS);
//  #endif // RST_PIN >= 0
//
//  display.setFont(System5x7);
//  
//  #if INCLUDE_SCROLLING == 0
//  #error INCLUDE_SCROLLING must be non-zero.  Edit SSD1306Ascii.h
//  #endif //  INCLUDE_SCROLLING
//  // Set auto scrolling at end of window.
//  display.setScrollMode(SCROLL_MODE_AUTO);
  Serial.begin(BAUD_RATE);
  //SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
//  display.display();
  button.attachClick(click);
  button.attachDoubleClick(doubleClick);
  button.attachLongPressStop(longPressStop);
  button.attachLongPressStart(longPressStart);
  pinMode(CONFIRM, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_FLASH, OUTPUT);
  
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
  triggTimer -> setOnTimer(&checkTriggTimeout);
  initStart -> Start();
}


void checkConfirm (void) {
  if(digitalRead(CONFIRM)) {
    confirmPress =!confirmPress;
    //baseRadio.flush_rx();
  }
  //confirmPress =!confirmPress;
}

// This function checks if the button for sending the
// Thresholds and Operators hase been pressed

void loop () {
  // put your main code here, to run repeatedly:
  drawMenu();
  if(systemState == SYS_INIT) {
    initStart->Update();
    if(!initTimeout) {
      checkRadioForInit();
      //drawInitMenu();
    } else if(initTimeout) {
        drawInitMenu();
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
          //drawConfigMenu();
          checkConfirm();
      }
  } else if(systemState == SYS_TRIG) {
      //Serial.println("Trigger stage");
      //checkForTriggers();
      //digitalWrite(PIN_FLASH, HIGH);
      if(sensorCtr == 1) {
        newTrigger = checkForTriggers();
        if(newTrigger == true) {
          digitalWrite(PIN_FLASH, HIGH);
          //delay(1000);
        } else {
            while(newTrigger == false){
              newTrigger = checkForTriggers();
            }
            digitalWrite(PIN_FLASH, HIGH);
            //delay(1000);
        }      
      } else {
          if(triggerLogic == 1) { 
            //newTrigger = checkForTriggers();
            while(newTrigger == false) {
              newTrigger = checkForTriggers();
            }
            if(newTrigger == true) {
              triggerCtr = sensorCtr;
              digitalWrite(PIN_FLASH, HIGH);
              //delay(1000);
            }
          } else {
              
              if(triggerCtr == 0) {
                while(newTrigger == false) {
                  newTrigger = checkForTriggers();
                  //Serial.println("Kollaei edo");
                }
                triggTimer->Start();
                newTrigger = checkForTriggers();
              } else if ((triggerCtr !=0) && (triggerCtr < sensorCtr)) {
                  while((newTrigger == false) && (triggTimeout == false)) {
                    newTrigger = checkForTriggers();
                    triggTimer->Update();
                    //Serial.println(triggTimeout);
                    //if(newTrigger == true) {
                      //break;
                    //}
                    //unsigned long startTimer = millis();
                    //bool timeout = false;
                    //while((newTrigger == false)) {
                      //newTrigger = checkForTriggers();
                      //if(millis() - startTimer > 1000 ){
                        //timeout = true; 
                        //trigFail = true;
                        //break;
                      //}
                    }
                    if(triggTimeout == true) {
                      trigFail = true;
                      //triggerCtr--;
                      Serial.print("Failed attempt ");
                      Serial.println(trigFail);
                      triggTimer->Stop();
                    } else {
                        //Serial.println("WTF");
                    }
                  //} else {
                      //digitalWrite(LED_BUILTIN, HIGH);
                      //delay(200);
                  //}
                  
              } else if(triggerCtr == sensorCtr) {
                  if(trigFail == true) {
                    digitalWrite(LED_BUILTIN, HIGH);
                    //Serial.println("paizei kati edo");
                  } else {
                      digitalWrite(PIN_FLASH, HIGH);
                      Serial.println(trigFail);
                  }
                  triggTimer->Stop();
              }
              
              newTrigger = checkForTriggers();
              //drawTriggMenu();
//              if(newTrigger == true) {
//                unsigned long startTimer = micros();
//                bool timeout = false;
//                while((triggerCtr != sensorCtr) && (newTrigger == false)) {
//                  
//                }
//              }
//              unsigned long startTimer = millis();
//              bool timeout = false;
          }
      }
      //digitalWrite(PIN_FLASH, HIGH);
      //delay(500);
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

// When we have more than one sensors and the condition between sensors is AND
// The base waits until one of the sensors is triggered. When that happens this 
// Timer function helps us determine if the trigger window has elapsed

void checkTriggTimeout (void) {
  triggCount++;
  if(triggCount >= 1) {
    triggTimeout = true;
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

void checkRadioForInit (void) {
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
    //Serial.println("No sensors were recognized. Reset the system");
    //delay(1000);
  }
  initStart->Stop();
  //screenRefresh();
}

// This function is called from the base in User Config stage to get the sensor data
// The base waits for messages from the sensors, if there is a payload available
// the base shows the senssor number and the according value.

void getCurrentData (void) {
  byte pipeNum = 0;
  while(baseRadio.available(&pipeNum)) {
    baseRadio.read(&sensorData[pipeNum - 1], sizeof(sensorData[pipeNum - 1]));
    Serial.print(F("Sensor ")); 
    Serial.print(pipeNum - 1); 
    Serial.print(" sent value :");
    Serial.println(sensorData[pipeNum - 1][1]);
    //baseRadio.flush_rx();
  }
}

// This function is used to send to all the sensors, one by one, both the Threshold 
// And the Operator to which they should sned a trigger value to the base

void sendThresh (void) {
  int gotByte;
  byte pipeNum;
  //int gotByteO;
  //byte pipeNumO;
  for(byte node = 0; node < MAX_SENSORS; node++) {
    if((sensorRCInit[node] == 1) && (sensorRCThresh[node]) != 1) {
      baseRadio.stopListening();
      baseRadio.openWritingPipe(wAddresses[node]);
      if(baseRadio.write(&dummyZero[node], sizeof(dummyZero[node]))) {
        baseRadio.startListening(); 
        unsigned long startTimer = millis();
        bool timeout = false;
        //pipeNum = node + 1;
        while(!baseRadio.available(&pipeNum)) {
          if(millis() - startTimer > 1000) {
            timeout = true;
          }
        }
        if(timeout) {
          Serial.println("Sensor did not respond for the Threshold");
        } else {
            baseRadio.read(&gotByte, sizeof(gotByte));
            if(gotByte == dummyZero[pipeNum-1]) {
              //Serial.println(gotByte);
              if(sensorRCThresh[pipeNum - 1] == 0) {
                sensorRCThresh[pipeNum - 1] = 1 ;
                //Serial.print("Sensor "); 
                //Serial.print(node); 
                //Serial.println(" got the Threshold");
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
         //Serial.println("Actually sent the operator");
         baseRadio.startListening(); 
         unsigned long startTimer = millis();
         bool timeout = false;
         while(!baseRadio.available(&pipeNum)) {
           if(millis() - startTimer > 1000) {
             timeout = true;
           }
         }
         if(timeout) {
           Serial.println("Sensor did not respond for the operator");
         } else {
             baseRadio.read(&gotByte, sizeof(gotByte));
             //Serial.println(gotByte);
             if((gotByte == -3) || (gotByte == -4)) {
               if(sensorRCOP[pipeNum - 1] == 0) {
                 sensorRCOP[pipeNum - 1] = 1;
                 //Serial.print("Sensor "); 
                 //Serial.print(node); 
                // Serial.println(" got the Operator");
                 delay(100);
                 baseRadio.flush_rx(); 
               }
             }
         }    
       }
    }
  }
  baseRadio.startListening();
}

// This function is used to check if all recognized sensors 
// Have received both the Threshold and the Operator

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

// This function is used to get out of USer Configuration stage of 
// The System and go to the actual waiting for triggers

void endConfig (void) {
  baseRadio.startListening();
  systemState = SYS_TRIG;
  screenRefresh();
}

// This function is used to check the base radio for new messages
// During the time when the base wiats for trigger messages. If a 
// message is received from a valid sensor that exists and hasn't 
// already sent a trigger, then the function changes values to the
// according tables and returns a true value. In any other case it 
// returns a false value 

boolean checkForTriggers (void) {
  byte pipeNum;
  int sensorValue = 0;
  boolean newTrigger = false;
  if(baseRadio.available(&pipeNum)) {
    baseRadio.read(&sensorValue, sizeof(sensorValue));
    if((sensorRCInit[pipeNum - 1] == 1) && (triggerTrue[pipeNum - 1] == 0)) {
      triggerTrue[pipeNum - 1] = 1;
      sensorData[pipeNum - 1][1] = sensorValue;
      triggerCtr++;
      //Serial.print(F("Sensor ")); 
      //Serial.print(pipeNum - 1); 
      //Serial.print(" sent trigger value :");
      //Serial.println(sensorData[pipeNum - 1][1]);
      newTrigger = true;
    }

  } else {
     //Serial.println("Searching, but nothing was sent");
  }
  return newTrigger;
}

/*********************************************************************************
      This fucntion is used to draw the main menu for all stages of the system
 ********************************************************************************/
void drawMenu (void) {
  display.setTextSize(0.5);
  display.setTextColor(WHITE);
  //display.set1X();
  if(systemState == SYS_INIT) {
    drawInitMenu();  
  } else if(systemState == USER_CONFIG) {
      drawConfigMenu();         
  } else if(systemState == SYS_TRIG) {
      drawTriggMenu();
  }
  display.println();  
}

/******************************************************************************
      This function is called to draw the menu of the Initilization stage
 *****************************************************************************/
void drawInitMenu (void) {
   screenHeader();
   int startingPointRow = 11; 
   display.setTextSize(1);
   display.setTextColor(WHITE);
   //display.set1X();
   for(byte node = 0 ; node < MAX_SENSORS ; node++) {
     if((sensorRCInit[node] == 1) && (startingPointRow < 56)) {
       display.setCursor(0, startingPointRow);
       display.print("Sensor ");
       display.setCursor(40, startingPointRow);
       display.print(node);
       display.setCursor(45, startingPointRow);
       display.println(" got rec");
       startingPointRow = startingPointRow + 7;
//       display.print("Sensor ");
//       display.print(node);
//       display.println("got rec");
     }
   }
   //screenRefresh();
   if(initTimeout && (sensorCtr != 0)) {
     display.println();
     display.setCursor(0, 25);
     display.print("Sensors : ");
     display.setCursor(60, 25);
     display.print(sensorCtr);
     //screenRefresh();
   } else if(initTimeout && (sensorCtr == 0)) {
       display.println();
       display.setCursor(0,startingPointRow);
       //display.println("No sensors available");
       startingPointRow = startingPointRow + 10;
       display.setCursor(0, startingPointRow);
       display.print("Reset system");
       //screenRefresh();
   }
   screenRefresh();
}

// This function is called to draw the menu of the User Configuration stage

void drawConfigMenu (void) {
  screenHeader();
  screenRefresh();
}

/*************************************************************************
       This function is called to draw the menu of the Trigger stage
 ************************************************************************/
void drawTriggMenu (void) {
  screenHeader();
  int startingPointRow = 11; 
  display.setTextSize(0.5);
  display.setTextColor(WHITE);
  //display.set1X();
  if(triggerCtr != sensorCtr) {
    for(byte node = 0 ; node < MAX_SENSORS ; node++) {
      if((triggerTrue[node] == 1) && (startingPointRow < 56)) {
        display.setCursor(0, startingPointRow);
        display.print("Sensor ");
        display.setCursor(40, startingPointRow);
        display.print(node);
        display.setCursor(45, startingPointRow);
        display.println(" triggered");
        startingPointRow = startingPointRow + 5;
//       display.print("Sensor ");
//       display.print(node);
//       display.println("got rec");
      }
    }
  } else {
      display.print("System Triggered");
  }
}

//This fuction is called when a single press is detected on the button
// Depending on the menu choice the function acts differently

void click (void) {
  
}

// This function is called when a double click is detected on the button
// Depending on the menu choice the user is on, the functions acts differently

void doubleClick (void) {
  
}

// This function is called when a long press of the button is detected
// Depending on the menu choice that the user is on the function acts
// in a different way

void longPressStart (void) {
  
}

// This function is called to detect if a long press has been stopped
// Depending on which menu choice the user is on the function acts accordingly

void longPressStop (void) {
  
}

// This function is called to print to the screen a header. 
// The header is different for each system Stage

/*************************************************************************
       This function is called to print to the screen a header. 
       The header is different for each system Stage
*************************************************************************/
void screenHeader (void) {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  //display.set1X();
  if(systemState == SYS_INIT) {
    display.setCursor(25,0);
    display.println("Initialization");
    display.drawLine(0,9,128,9,WHITE);
  } else if(systemState == USER_CONFIG) {
      //display.clearField(22,0,4);
      display.setCursor(25,0);
      display.println("Configuration");
      //display.drawLine(0,9,128,9,WHITE);
      int startingPointRow = 0;
      int startingPointLine = 7;
      for(byte node = 0 ; node < MAX_SENSORS ; node++) {
        //if((sensorRCInit[node] == 1) && (startingPointRow < 56)) {
          display.setCursor(startingPointRow, startingPointLine);
          display.print("S");
          startingPointRow = startingPointRow + 5;
          display.setCursor(startingPointRow, startingPointLine);
          display.print(node);
          startingPointRow = startingPointRow + 5;
          display.setCursor(startingPointRow, startingPointLine);
          display.print(":");
          startingPointRow = startingPointRow + 5;
          display.setCursor(startingPointRow, startingPointLine);
          if (sensorRCInit[node] == 1) {
            display.print(sensorData[node][1]);
          } else {
              display.print("-");
          }

          startingPointRow = startingPointRow + 30;
          if(startingPointRow >= 127) {
            startingPointLine = startingPointLine + 7;
            startingPointRow = 0;
          }
          //startingPointLine = startingPointLine + 7;
        //}

      }
      startingPointLine = startingPointLine + 7 ;
      display.drawLine(0,startingPointLine,128,startingPointLine,WHITE);
      currentPointLine = startingPointLine + 2;
  } else if(systemState == SYS_TRIG) {
      display.setCursor(25,0);
      display.println("Trigger Stage");
      display.drawLine(0,9,128,9,WHITE);
  }
  display.println();
//  for(byte i = 0; i < 22; i++) {
//    display.print("");
//  }
//  display.println("Initilization");
//  for(byte i =0; i < 128; i++) {
//    display.print(".");
//  }
//  display.println();
}

// This function is called to refresh what the display prints

void screenRefresh (void) {
  display.display();
  display.clearDisplay();
  //display.clear();
}

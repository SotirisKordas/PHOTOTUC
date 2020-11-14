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
          First Created: 21/03/2019
          Last Modified: 11/11/2020
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

// Change that to add more sensors to your system. Maximum 6 sensors
#define MAX_SENSORS 2 

// Variable that helps us change to Trigger mode. For more sensors add value 100 inside the brackets according to the number os sensors 
// The 100 value is default for the utilization of the system. User changes values in Configuration Mode
int dummyZero[MAX_SENSORS] = {100, 100};  
int tmpSel[MAX_SENSORS][4] = {
    {0,0,0,0},
    {0,0,0,0}
  };

// Variable table that holds operator > or < for each sensor  
// During User Config stage depending on user choice the value might change
// From -3 to -4. -3 means > and -4 means <
// For more sensors add corresponding values accordingly
int dummyOp[MAX_SENSORS] = {-3,-3};

int recInit= -1;

boolean confirmPress=false;


// define daud rate
#define BAUD_RATE 57600

// chip select and RF24 radio setup pins
#define PIN_CE 9
#define PIN_CSN 10

// Define pin for the flash when it's to be triggered 
#define PIN_FLASH 6

// Variable that holds the number of sensors recognized in Initialization stage
uint8_t sensorCtr = 0; 

// Variable that helps us determine if the initialization stage completed succesfully
uint8_t tmp = 0; 
uint8_t tmpTrig = 0;

RF24 baseRadio(PIN_CE,PIN_CSN);

// Setup radio pipe addresses for each node for the base to read and write
// To add extra sensors, add the addresses for each sensor accordingly
const uint64_t wAddresses[] = {0xB00B1E50D2LL, 0xB00B1E50C3LL};  //Create pipe addresses for the 2 nodes to recieve data, the "LL" is for LongLong type
const uint64_t rAddresses[] = {0xB00B1E50B1LL, 0xB00B1E50A4LL};  //Create pipe addresses for the 2 nodes to transmit data, the "LL" is for LongLong type

// Simple array for each sensor data, in the form [sensor_id, return_count]
// For support of more sensors change the table size and starting values accordingly
int sensorData[MAX_SENSORS][MAX_SENSORS] = {{1, 1}, {2, 1}};

// Arrays to suggest that the Init,Threshold and Operator options were received
// To support more sensors add 0 values inside the brackets according to the number of sensors
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

// Array of boolean values for each sensor that turn into true when the according sensor sends a trigger
// To support more sensors add false values inside the brackets according to the number of sensors
boolean triggerTrue[MAX_SENSORS] = {false, false}; 

// Byte that represents if we want an OR logic or an AND logic between the sensor values in the trigger condition. 
// 0 is for AND and 1 is for OR
byte triggerLogic = 0; 

// Boolean value that will be used to determine if a message the base receives during SYS_TRIG 
// is a new trigger for a sensor that hasn't already sent a trigger
byte newTrigger = false; 

// Variable that represents how many sensors have already sent their trigger value
int triggerCtr = 0;

// Variable that stores the amount of delay the user chooses between true trigger function and flash/camera trigger
int triggerDelay=0;

bool timeout = false;
//unsigned long startTimer;

// The following variables are used for the menu in system configuration mode 
int pic = 1000;
int maxPics_L1 = 5000;
int maxPics_L2 = 3;
int currentPointLine;
int menuLayer = 1; 
byte selection = 0;

// The following variables are used for the buttons and navigation in the menu
#define CONFIRM 2 
#define BUTTON_BRD 12
OneButton buttonConfirm(CONFIRM, true);
OneButton button(BUTTON_BRD, true);
long lastMillis = 0;
long maxTime = 30000;

// OLED display width, in pixels
#define SCREEN_WIDTH 128 

// OLED display height, in pixels
#define SCREEN_HEIGHT 64 

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Define proper RST_PIN if required.
//#define RST_PIN -1

//SSD1306AsciiWire display;

void setup() {
  
  // Put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000L);
  
  // Setup for buttons 
  button.attachClick(click);
  button.attachDoubleClick(doubleClick);
  button.attachLongPressStop(longPressStop);
  button.attachLongPressStart(longPressStart);
  //button.setDebounceTicks(100);
  //button.setClickTicks(600);

  // The following lines are used for the setup of the screen 
  
  //#if INCLUDE_SCROLLING == 0
  //#error INCLUDE_SCROLLING must be non-zero.  Edit SSD1306Ascii.h
  //#endif //  INCLUDE_SCROLLING
  // Set auto scrolling at end of window.
  //display.setScrollMode(SCROLL_MODE_AUTO);
  Serial.begin(BAUD_RATE);
  //SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();

  // The following lines are used to set input and output pins
  pinMode(CONFIRM, INPUT);
  pinMode(BUTTON_BRD, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_FLASH, OUTPUT);
  
  printf_begin();
  Serial.println("Init test ");

  // The following lines setup the RF settings for the base system
  // To support mosre sensors repeat lines 223 and 224 accordingly
  // Some lines are commented and can be used for different settings is needed
  baseRadio.begin();
  baseRadio.setAutoAck(false);
  baseRadio.setPALevel(RF24_PA_LOW);
  //baseRadio.setDataRate(RF24_1MBPS); // Uncomment for Data Rate Settings
  //baseRadio.setChannel(0x76);  //Uncomment for Channel Settings
  baseRadio.enableDynamicPayloads();
  //baseRadio.setRetries(5, 5);  //Uncomment for Retries 
  //baseRadio.enableAckPayload();  //Uncomment for Acks
  //baseRadio.setPayloadSize(sizeof(sensorData));  //Uncomment for Payload Size
  baseRadio.openReadingPipe(1,rAddresses[0]);
  baseRadio.openReadingPipe(2,rAddresses[1]);
  baseRadio.startListening();
  baseRadio.printDetails();

  //The following functions is called to calculate the tmpSel initial values
//  for(int i = 0 ; i ++ ; i < MAX_SENSORS) {
//    calculateTmpSel(i);  
//  }
  

  // The following lines are used for settings of timers that are needed
  initStart -> setOnTimer(&checkInitTimeout);
  triggTimer -> setOnTimer(&checkTriggTimeout);
  initStart -> Start();

  
}

/*************************************************************************
       This function checks if the button for sending the
       Thresholds and Operators has been pressed
 *************************************************************************/
void checkConfirm (void) {
  if(digitalRead(CONFIRM)) {
    confirmPress =!confirmPress;
    //baseRadio.flush_rx();
  }
  //confirmPress =!confirmPress;
}

/****************************************************************************************
      This is the loop function. It is executed on every cycle and is the main code 
 ***************************************************************************************/
void loop () {
  // put your main code here, to run repeatedly
  button.tick();
  if (millis() >= (lastMillis+maxTime)) {
    pic = 1000;
    menuLayer=1;
  }
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

/*******************************************************************************************
       This function is called right after we get a recognition message from a sensor
       The functions responds to the sensor with the same value to let it know the base
       received the initialization message
 ******************************************************************************************/
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

/*******************************************************************************
       The base should wait for Initilization messages from sensors for 15s
       This function checks if this specific amount of time has passed
 ******************************************************************************/
void checkInitTimeout (void) {
  initCount++;
  if(initCount >= 15) { 
    initTimeout = true; 
  }
}

/***************************************************************************************
       When we have more than one sensors and the condition between sensors is AND
       The base waits until one of the sensors is triggered. When that happens this 
       Timer function helps us determine if the trigger window has elapsed
 **************************************************************************************/
void checkTriggTimeout (void) {
  triggCount++;
  if(triggCount >= 1) {
    triggTimeout = true;
  }
}

/**********************************************************************************************
       This function checks the Radio for Initialization messages from sensors
       While there are payloads available the Base checks which sensor sent the
       payload. If the sensor hasn't already sent an Initialization message, then
       the corresponding value of the array that is used to know which sensor
       was already recognized is set to 1. If the sensor has already been recognized
       the message gets ignored. If the sensor has not been recognized before
       the base responds with the same Initialization message so that the sensor can know
       it got recognized. If the response fails to be sent, an accordng message is printed
 *********************************************************************************************/
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

/**********************************************************************************
       This functions is used when we are ready to end the Initialization stage
       and go into User Configuration stage. 
 *********************************************************************************/
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

/******************************************************************************************
       This function is called from the base in User Config stage to get the sensor data
       The base waits for messages from the sensors, if there is a payload available
       the base shows the senssor number and the according value.
 *****************************************************************************************/
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

/******************************************************************************************
       This function is used to send to all the sensors, one by one, both the Threshold 
       And the Operator to which they should sned a trigger value to the base
 *****************************************************************************************/
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

/*************************************************************************
       This function is used to check if all recognized sensors 
       Have received both the Threshold and the Operator
 ************************************************************************/
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

/*************************************************************************
       This function is used to get out of USer Configuration stage of 
       The System and go to the actual waiting for triggers
 *************************************************************************/
void endConfig (void) {
  baseRadio.startListening();
  systemState = SYS_TRIG;
  screenRefresh();
}

/*************************************************************************
       This function is used to check the base radio for new messages
       During the time when the base wiats for trigger messages. If a 
       message is received from a valid sensor that exists and hasn't 
       already sent a trigger, then the function changes values to the
       according tables and returns a true value. In any other case it 
       returns a false value 
 *************************************************************************/
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
     //screenRefressh();
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

/*****************************************************************************************
       This function is called to draw the menu of the User Configuration stage
 ****************************************************************************************/
void drawConfigMenu (void) {
  screenHeader();
  int startingPointRow = 20;
  display.setTextSize(1);
  display.setTextColor(WHITE);
  /*************************************************************************
           The next lines of code are used for the fisrt layer
           of the menu. 
   *************************************************************************/

    //    LAYER 1   //
  //    CHOICE 1    //
  if (pic == 1000) {
    //button.tick();
    //screenHeader();
    if(currentPointLine <= 20) {
      display.setCursor(0, currentPointLine);  
      display.print (">SET THRESHOLD");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("TRIGGER DELAY");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("TRIGGER LOGIC");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("HELP");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("CONFIRM");
    //screenRefresh();
    }

    else if(currentPointLine > 20) {
      display.setCursor(0, currentPointLine);  
      display.print (">SET THRESHOLD");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("TRIGGER DELAY");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("TRIGGER LOGIC");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("HELP");
    }
  }

  //    LAYER 1   //
  //    CHOICE 2    //
  if (pic == 2000) {
    //button.tick();
    if(currentPointLine <= 20) {
      display.setCursor(0, currentPointLine);  
      display.print ("SET THRESHOLD");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print (">TRIGGER DELAY");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("TRIGGER LOGIC");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("HELP");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("CONFIRM");
    //screenRefresh();
    }

    else if(currentPointLine > 20) {
      display.setCursor(0, currentPointLine);  
      display.print ("SET THRESHOLD");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print (">TRIGGER DELAY");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("TRIGGER LOGIC");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("HELP");
    }
  }

  //    LAYER 1   //
  //    CHOICE 3    //
  if (pic == 3000) {
    //button.tick();
    if(currentPointLine <= 20) {
      display.setCursor(0, currentPointLine);  
      display.print ("SET THRESHOLD");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("TRIGGER DELAY");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print (">TRIGGER LOGIC");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("HELP");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("CONFIRM");
    //screenRefresh();
    }

    else if(currentPointLine > 20) {
      display.setCursor(0, currentPointLine);  
      display.print ("SET THRESHOLD");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("TRIGGER DELAY");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine); 
      display.print (">TRIGGER LOGIC");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("HELP");
    }
  }

  //    LAYER 1   //
  //    CHOICE 4    //
  if (pic == 4000) {
    //button.tick();
    if(currentPointLine <= 20) {
      display.setCursor(0, currentPointLine);  
      display.print ("SET THRESHOLD");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("TRIGGER DELAY");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("TRIGGER LOGIC");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print (">HELP");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("CONFIRM");
    //screenRefresh();
    }

    else if(currentPointLine > 20) {
      display.setCursor(0, currentPointLine);  
      display.print ("SET THRESHOLD");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("TRIGGER DELAY");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("TRIGGER LOGIC");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print (">HELP");
    }
  }  

  //    LAYER 1   //
  //    CHOICE 5    //
  if (pic == 5000) {
    //button.tick();
    if(currentPointLine <= 20) {
      display.setCursor(0, currentPointLine);  
      display.print ("SET THRESHOLD");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("TRIGGER DELAY");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("TRIGGER LOGIC");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("HELP");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print (">CONFIRM");
    //screenRefresh();
    }

    else if(currentPointLine > 20) {
      display.setCursor(0, currentPointLine);  
      display.print (">CONFRIM");
//      currentPointLine = currentPointLine + 8;
//      display.setCursor(0, currentPointLine);  display.print (" TRIGGER DELAY");
//      currentPointLine = currentPointLine + 8;
//      display.setCursor(0, currentPointLine);  display.print (" TRIGGER LOGIC");
//      currentPointLine = currentPointLine + 8;
//      display.setCursor(0, currentPointLine);  display.print (" HELP");
    }
  }

/*************************************************************************
         The next lines of code are used for the second layer
         of the menu. To support more sensors uncommment the
         according choices. By default only two sensors are
         supported, so only choices 1100 and 1200 are
         uncommented. Make sure to change accordingly the 
         conditions in button handling for correct navigation  
 *************************************************************************/


  //    LAYER 2  //
  //    CHOICE 1.1 //
  if (pic == 1100) {
    button.tick();
    //screenHeader();
    //display.setCursor(0, currentPointLine);
    for (byte node = 0 ; node < MAX_SENSORS ; node++) {
      display.setCursor(0, currentPointLine);
      if (node == 0){
        if (sensorRCInit[node] == 1) {
          display.print (">SENSOR 0");
        } else {
          display.print ("N/A");
        }
        currentPointLine = currentPointLine + 8;
      } else {
          if (sensorRCInit[node] == 1) {
            display.print ("SENSOR ");
            display.setCursor(50 , currentPointLine);
            display.print (node);
          } else {
              //display.print ("SENSOR ");
              display.setCursor(0 , currentPointLine);
              //display.print (node);
              display.print ("N/A");           
          }
          currentPointLine = currentPointLine + 8;
      }
    }
    //screenRefresh();
  }

  //    LAYER 2  //
  //    CHOICE 1.2 //
  if (pic == 1200) {
    button.tick();
    //screenHeader();
    //display.setCursor(0, currentPointLine);
    for (byte node = 0 ; node < MAX_SENSORS ; node++) {
      display.setCursor(0, currentPointLine);
      if (node == 1){
        if (sensorRCInit[node] == 1) {
          display.print (">SENSOR 1");
        } else {
          display.print ("N/A");
        }
        currentPointLine = currentPointLine + 8;
      } else {
          if (sensorRCInit[node] == 1) {
            display.print ("SENSOR ");
            display.setCursor(50 , currentPointLine);
            display.print (node);
          } else {
              //display.print ("SENSOR ");
              display.setCursor(0 , currentPointLine);
              //display.print (node);
              display.print ("N/A");           
          }
          currentPointLine = currentPointLine + 8;
      }
    }
    //screenRefresh();
  }

//  //    LAYER 2  //
//  //    CHOICE 1.3 //
//  if (pic == 1300) {
//    //display.setCursor(0, currentPointLine);
//    for (byte node = 0 ; node < MAX_SENSORS ; node++) {
//      display.setCursor(0, currentPointLine);
//      if (node == 2){
//        if (sensorRCInit[node] == 1) {
//          display.print (">SENSOR 2");
//        } else {
//          display.print ("N/A");
//        }
//        currentPointLine = currentPointLine + 8;
//      } else {
//          if (sensorRCInit[node] == 1) {
//            display.print ("SENSOR ");
//            display.setCursor(25 , currentPointLine);
//            display.print (node);
//          } else {
//              //display.print ("SENSOR ");
//              display.setCursor(0 , currentPointLine);
//              //display.print (node);
//              display.print ("N/A");           
//          }
//          currentPointLine = currentPointLine + 8;
//      }
//    }
//  }
//
//  //    LAYER 2  //
//  //    CHOICE 1.4 //
//  if (pic == 1400) {
//    display.setCursor(0, currentPointLine);
//    for (byte node = 0 ; node < MAX_SENSORS ; node++) {
//      if (node == 3){
//        if (sensorRCInit[node] == 1) {
//          display.print (">SENSOR 3");
//        } else {
//          display.print ("N/A");
//        }
//        currentPointLine = currentPointLine + 8;
//      } else {
//          if (sensorRCInit[node] == 1) {
//            display.print ("SENSOR ");
//            display.setCursor(25 , currentPointLine);
//            display.print (node);
//          } else {
//              //display.print ("SENSOR ");
//              display.setCursor(0 , currentPointLine);
//              //display.print (node);
//              display.print ("N/A");           
//          }
//          currentPointLine = currentPointLine + 8;
//      }
//    }
//  }
//
//
//  //    LAYER 2  //
//  //    CHOICE 1.5 //
//  if (pic == 1500) {
//    display.setCursor(0, currentPointLine);
//    if (sensorCtr >= 4) {
//      if (sensorRCInit[4] == 1) {
//        display.print (">SENSOR 4");
//      } else {
//        display.print ("N/A");
//      }
//      currentPointLine = currentPointLine + 8;
//      display.setCursor(0, currentPointLine);
//      if (sensorRCInit[5] == 1) {
//        display.print ("SENSOR 5");
//      } else {
//          display.print ("N/A");
//      }
////      currentPointLine = currentPointLine + 8;
////      display.setCursor(0, currentPointLine);
////      display.print ("BACK");
//    } else {
//       for (byte node = 0 ; node < MAX_SENSORS ; node++) {
//         display.setCursor(0, currentPointLine);
//         if (node == 4) {
//           if (sensorRCInit[node] == 1) {
//             display.print (">SENSOR 4");
//           } else {
//               display.print ("N/A");
//           }
//         } else {
//             if (sensorRCInit[node] == 1) {
//               display.print ("SENSOR ");
//               display.setCursor(25 , currentPointLine);
//               display.print (node);
//             } else {
//                 //display.print ("SENSOR ");
//                 display.setCursor(0 , currentPointLine);
//                 //display.print (node);
//                 display.print ("N/A");           
//             }             
//         }
//       }
//    }
//  }
//
//  //    LAYER 2  //
//  //    CHOICE 1.6 //
//  if (pic == 1600) {
//    display.setCursor(0, currentPointLine);
//    if (sensorCtr >= 4) {
//      if (sensorRCInit[4] == 1) {
//        display.print ("SENSOR 4");
//      } else {
//        display.print ("N/A");
//      }
//      currentPointLine = currentPointLine + 8;
//      display.setCursor(0, currentPointLine);
//      if (sensorRCInit[5] == 1) {
//        display.print (">SENSOR 5");
//      } else {
//          display.print ("N/A");
//      }
////      currentPointLine = currentPointLine + 8;
////      display.setCursor(0, currentPointLine);
////      display.print ("BACK");      
//    } else {
//        if (sensorRCInit[5] == 1) {
//          display.print (">SENSOR 5");
//        } else {
//            display.print ("N/A");
//        }
////        currentPointLine = currentPointLine + 8;
////        display.setCursor(0, currentPointLine);
////        display.print ("BACK");            
//    }
//  }

  //    LAYER 2  //
  //    CHOICE 2.1 //  
  if (pic == 2100) {
    if(currentPointLine <= 20) {
      display.setCursor(0, currentPointLine);  
      display.print (">50 ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("100ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("150ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("200ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("300ms");
    } else if(currentPointLine > 20) {
        display.setCursor(0, currentPointLine);  
        display.print (">50 ms");
        currentPointLine = currentPointLine + 8;
        display.setCursor(0, currentPointLine);  
        display.print ("100ms");
        currentPointLine = currentPointLine + 8;
        display.setCursor(0, currentPointLine);  
        display.print ("150ms");
        currentPointLine = currentPointLine + 8;
        display.setCursor(0, currentPointLine);  
        display.print ("200ms");
        //currentPointLine = currentPointLine + 8;
        //display.setCursor(0, currentPointLine);  display.print (" 300ms");
    }

  } 

  //    LAYER 2  //
  //    CHOICE 2.2 //  
  if (pic == 2200) {
    if(currentPointLine <= 20) {
      display.setCursor(0, currentPointLine);  
      display.print ("50 ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print (">100ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("150ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("200ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("300ms");
    } else if(currentPointLine > 20) {
        display.setCursor(0, currentPointLine);  
        display.print ("50 ms");
        currentPointLine = currentPointLine + 8;
        display.setCursor(0, currentPointLine);  
        display.print (">100ms");
        currentPointLine = currentPointLine + 8;
        display.setCursor(0, currentPointLine);  
        display.print ("150ms");
        currentPointLine = currentPointLine + 8;
        display.setCursor(0, currentPointLine);  
        display.print ("200ms");
        //currentPointLine = currentPointLine + 8;
        //display.setCursor(0, currentPointLine);  display.print (" 300ms");
    }

  }
  //    LAYER 2  //
  //    CHOICE 2.3 //  
  if (pic == 2300) {
    if(currentPointLine <= 20) {
      display.setCursor(0, currentPointLine);  
      display.print ("50 ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("100ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print (">150ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("200ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("300ms");
    } else if(currentPointLine > 20) {
        display.setCursor(0, currentPointLine);  
        display.print ("50 ms");
        currentPointLine = currentPointLine + 8;
        display.setCursor(0, currentPointLine); 
        display.print ("100ms");
        currentPointLine = currentPointLine + 8;
        display.setCursor(0, currentPointLine);  
        display.print (">150ms");
        currentPointLine = currentPointLine + 8;
        display.setCursor(0, currentPointLine);  
        display.print ("200ms");
        //currentPointLine = currentPointLine + 8;
        //display.setCursor(0, currentPointLine);  display.print (" 300ms");
    }

  }

  //    LAYER 2  //
  //    CHOICE 2.4 //  
  if (pic == 2400) {
    if(currentPointLine <= 20) {
      display.setCursor(0, currentPointLine);  
      display.print ("50 ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("100ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("150ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print (">200ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("300ms");
    } else if(currentPointLine > 20) {
        display.setCursor(0, currentPointLine);  
        display.print ("50 ms");
        currentPointLine = currentPointLine + 8;
        display.setCursor(0, currentPointLine);  
        display.print ("100ms");
        currentPointLine = currentPointLine + 8;
        display.setCursor(0, currentPointLine);  
        display.print ("150ms");
        currentPointLine = currentPointLine + 8;
        display.setCursor(0, currentPointLine);  
        display.print (">200ms");
        //currentPointLine = currentPointLine + 8;
        //display.setCursor(0, currentPointLine);  display.print (" 300ms");
    }

  }

  //    LAYER 2  //
  //    CHOICE 2.5 //  
  if (pic == 2500) {
    if(currentPointLine <= 20) {
      display.setCursor(0, currentPointLine);  
      display.print ("50 ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("100ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("150ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print ("200ms");
      currentPointLine = currentPointLine + 8;
      display.setCursor(0, currentPointLine);  
      display.print (">300ms");
    } else if(currentPointLine > 20) {
        //display.setCursor(0, currentPointLine);  display.print ("50 ms");
        //currentPointLine = currentPointLine + 8;
        //display.setCursor(0, currentPointLine);  display.print ("100ms");
        //currentPointLine = currentPointLine + 8;
        //display.setCursor(0, currentPointLine);  display.print ("150ms");
        //currentPointLine = currentPointLine + 8;
        //display.setCursor(0, currentPointLine);  display.print ("200ms");
        //currentPointLine = currentPointLine + 8;
        display.setCursor(0, currentPointLine);  
        display.print (">300ms");
    }

  }

  //    LAYER 2  //
  //    CHOICE 3.1 //  
  if (pic == 3100) {
    display.setTextSize(3);
    display.setCursor(0, currentPointLine);  
    display.println (">OR");
    //currentPointLine = currentPointLine + 8;
    //display.setCursor(0, currentPointLine);  
    display.println ("AND");
  }

  //    LAYER 2  //
  //    CHOICE 3.2 //  
  if (pic == 3200) {
    display.setTextSize(3);
    display.setCursor(0, currentPointLine);  
    display.println ("OR");
    //currentPointLine = currentPointLine + 8;
    //display.setCursor(0, currentPointLine);  
    display.println (">AND");
  }

  //    LAYER 2  //
  //    CHOICE 4.1 //  
  if (pic == 4100) {
    display.setCursor(0, currentPointLine);
    display.print("1 PRESS: NEXT CHOICE");
    currentPointLine = currentPointLine + 8;
    display.setCursor(0, currentPointLine); 
    display.print("DOUBLE PRESS: GO BACK");
    currentPointLine = currentPointLine + 8;
    display.setCursor(0, currentPointLine); 
    display.print("LONG PRESS: CONFIRM");
  } 

  //    LAYER 2  //
  //    CHOICE 5.1 //  
  if (pic == 5100) {
    display.setCursor(0, currentPointLine); 
    display.print("PROJECT NAME: PHOTOTUC");
    currentPointLine = currentPointLine + 8;
    display.setCursor(0, currentPointLine); 
    display.print("CREATOR: SOTIRIS KORDAS");
    currentPointLine = currentPointLine + 8;
    display.setCursor(0, currentPointLine); 
    display.print("YEAR: 2020");
  }   


 /*************************************************************************
         The next lines of code are used for the third layer
         of the menu. To support more sensors uncommment the
         according choices. By default only two sensors are
         supported, so only the sets of 1100 and 1200 are 
         uncommented. Make sure to change accordingly the 
         conditions in button handling for correct navigation  
 *************************************************************************/

  //    LAYER 3  //
  //    CHOICE 1.1.1 //
  if (pic == 1110) {
    if (sensorRCInit[0] == 1) {
      display.setTextSize(2);
      display.setCursor(0, currentPointLine);
      display.print (">THRESH:"); 
      display.setTextSize(1);
      currentPointLine = currentPointLine + 3;
      display.setCursor(100,currentPointLine);
      display.print(dummyZero[0]);
      display.setTextSize(2);
      display.println();
      //currentPointLine = currentPointLine + 8;
      //display.setCursor(0, currentPointLine);
      display.println("OPERATOR");
    } else {
        display.setCursor(0, currentPointLine);
        display.print("INVALID.GO BACk");
    }
  }


  //    LAYER 3  //
  //    CHOICE 1.1.2 //
  if (pic == 1120) {
    if (sensorRCInit[0] == 1) {
      display.setTextSize(2);
      display.setCursor(0, currentPointLine);
      display.print ("THRESH:"); 
      display.setTextSize(1);
      currentPointLine = currentPointLine + 3;
      display.setCursor(100,currentPointLine);
      display.print(dummyZero[0]);
      display.setTextSize(2);
      display.println();
      //currentPointLine = currentPointLine + 8;
      //display.setCursor(0, currentPointLine);
      display.println(">OPERATOR");
    } else {
        display.setCursor(0, currentPointLine);
        display.print("INVALID.GO BACk");
    }
  }

  //    LAYER 3  //
  //    CHOICE 1.2.1 //
  if (pic == 1210) {
    if (sensorRCInit[1] == 1) {
      display.setTextSize(2);
      display.setCursor(0, currentPointLine);
      display.print (">THRESH:"); 
      display.setTextSize(1);
      currentPointLine = currentPointLine + 3;
      display.setCursor(100,currentPointLine);
      display.print(dummyZero[1]);
      display.setTextSize(2);
      display.println();
      //currentPointLine = currentPointLine + 8;
      //display.setCursor(0, currentPointLine);
      display.println("OPERATOR");
    } else {
        display.setCursor(0, currentPointLine);
        display.print("INVALID.GO BACk");
    }
  }


  //    LAYER 3  //
  //    CHOICE 1.2.2 //
  if (pic == 1220) {
    if (sensorRCInit[1] == 1) {
      display.setTextSize(2);
      display.setCursor(0, currentPointLine);
      display.print ("THRESH:"); 
      display.setTextSize(1);
      currentPointLine = currentPointLine + 3;
      display.setCursor(100,currentPointLine);
      display.print(dummyZero[1]);
      display.setTextSize(2);
      display.println();
      //currentPointLine = currentPointLine + 8;
      //display.setCursor(0, currentPointLine);
      display.println(">OPERATOR");
    } else {
        display.setCursor(0, currentPointLine);
        display.print("INVALID.GO BACk");
    }
  }

 /*****************************************************************************************
      Below are the choices for another 4 sensors when choosing between
      threshold and operator choice. 
      Uncomment the according section depending on how many more sensors are needed
 *****************************************************************************************/

 /*
      //    LAYER 3  //
  //    CHOICE 1.3.1 //
  if (pic == 1310) {
    if (sensorRCInit[2] == 1) {
      display.setTextSize(2);
      display.setCursor(0, currentPointLine);
      display.print (">THRESH:"); 
      display.setTextSize(1);
      currentPointLine = currentPointLine + 3;
      display.setCursor(100,currentPointLine);
      display.print(dummyZero[2]);
      display.setTextSize(2);
      display.println();
      //currentPointLine = currentPointLine + 8;
      //display.setCursor(0, currentPointLine);
      display.println("OPERATOR");
    } else {
        display.setCursor(0, currentPointLine);
        display.print("INVALID.GO BACk");
    }
  }


  //    LAYER 3  //
  //    CHOICE 1.3.2 //
  if (pic == 1320) {
    if (sensorRCInit[2] == 1) {
      display.setTextSize(2);
      display.setCursor(0, currentPointLine);
      display.print ("THRESH:"); 
      display.setTextSize(1);
      currentPointLine = currentPointLine + 3;
      display.setCursor(100,currentPointLine);
      display.print(dummyZero[2]);
      display.setTextSize(2);
      display.println();
      //currentPointLine = currentPointLine + 8;
      //display.setCursor(0, currentPointLine);
      display.println(">OPERATOR");
    } else {
        display.setCursor(0, currentPointLine);
        display.print("INVALID.GO BACk");
    }
  }

  //    LAYER 3  //
  //    CHOICE 1.4.1 //
  if (pic == 1410) {
    if (sensorRCInit[3] == 1) {
      display.setTextSize(2);
      display.setCursor(0, currentPointLine);
      display.print (">THRESH:"); 
      display.setTextSize(1);
      currentPointLine = currentPointLine + 3;
      display.setCursor(100,currentPointLine);
      display.print(dummyZero[3]);
      display.setTextSize(2);
      display.println();
      //currentPointLine = currentPointLine + 8;
      //display.setCursor(0, currentPointLine);
      display.println("OPERATOR");
    } else {
        display.setCursor(0, currentPointLine);
        display.print("INVALID.GO BACk");
    }
  }


  //    LAYER 3  //
  //    CHOICE 1.4.2 //
  if (pic == 1420) {
    if (sensorRCInit[3] == 1) {
      display.setTextSize(2);
      display.setCursor(0, currentPointLine);
      display.print ("THRESH:"); 
      display.setTextSize(1);
      currentPointLine = currentPointLine + 3;
      display.setCursor(100,currentPointLine);
      display.print(dummyZero[3]);
      display.setTextSize(2);
      display.println();
      //currentPointLine = currentPointLine + 8;
      //display.setCursor(0, currentPointLine);
      display.println(">OPERATOR");
    } else {
        display.setCursor(0, currentPointLine);
        display.print("INVALID.GO BACk");
    }
  }

    //    LAYER 3  //
  //    CHOICE 1.5.1 //
  if (pic == 1510) {
    if (sensorRCInit[4] == 1) {
      display.setTextSize(2);
      display.setCursor(0, currentPointLine);
      display.print (">THRESH:"); 
      display.setTextSize(1);
      currentPointLine = currentPointLine + 3;
      display.setCursor(100,currentPointLine);
      display.print(dummyZero[4]);
      display.setTextSize(2);
      display.println();
      //currentPointLine = currentPointLine + 8;
      //display.setCursor(0, currentPointLine);
      display.println("OPERATOR");
    } else {
        display.setCursor(0, currentPointLine);
        display.print("INVALID.GO BACk");
    }
  }


  //    LAYER 3  //
  //    CHOICE 1.5.2 //
  if (pic == 1520) {
    if (sensorRCInit[4] == 1) {
      display.setTextSize(2);
      display.setCursor(0, currentPointLine);
      display.print ("THRESH:"); 
      display.setTextSize(1);
      currentPointLine = currentPointLine + 3;
      display.setCursor(100,currentPointLine);
      display.print(dummyZero[4]);
      display.setTextSize(2);
      display.println();
      //currentPointLine = currentPointLine + 8;
      //display.setCursor(0, currentPointLine);
      display.println(">OPERATOR");
    } else {
        display.setCursor(0, currentPointLine);
        display.print("INVALID.GO BACk");
    }
  }

    //    LAYER 3  //
  //    CHOICE 1.6.1 //
  if (pic == 1610) {
    if (sensorRCInit[5] == 1) {
      display.setTextSize(2);
      display.setCursor(0, currentPointLine);
      display.print (">THRESH:"); 
      display.setTextSize(1);
      currentPointLine = currentPointLine + 3;
      display.setCursor(100,currentPointLine);
      display.print(dummyZero[5]);
      display.setTextSize(2);
      display.println();
      //currentPointLine = currentPointLine + 8;
      //display.setCursor(0, currentPointLine);
      display.println("OPERATOR");
    } else {
        display.setCursor(0, currentPointLine);
        display.print("INVALID.GO BACk");
    }
  }


  //    LAYER 3  //
  //    CHOICE 1.6.2 //
  if (pic == 1620) {
    if (sensorRCInit[5] == 1) {
      display.setTextSize(2);
      display.setCursor(0, currentPointLine);
      display.print ("THRESH:"); 
      display.setTextSize(1);
      currentPointLine = currentPointLine + 3;
      display.setCursor(100,currentPointLine);
      display.print(dummyZero[5]);
      display.setTextSize(2);
      display.println();
      //currentPointLine = currentPointLine + 8;
      //display.setCursor(0, currentPointLine);
      display.println(">OPERATOR");
    } else {
        display.setCursor(0, currentPointLine);
        display.print("INVALID.GO BACk");
    }
  }
 */

  /*************************************************************************
         The next lines of code are used for the fourth layer
         of the menu. To support more sensors uncommment the
         according choices. By default only two sensors are
         supported, so only the sets of 1100 and 1200 are 
         uncommented. Make sure to change accordingly the 
         conditions in button handling for correct navigation  
  *************************************************************************/

  /*******************************************************************************
         This part contains what the menu will show when choosing
         for the corresponding operator of each sensor.
         Further below there are commented parts that can be unommented
         for support of more sensors. Uncomment accoring to the sensor number
  *******************************************************************************/
  
  //    LAYER 4  //
  //    CHOICE 1.1.2.1 //
  if (pic == 1121) {
    display.setTextSize(2);
    display.setCursor(0, currentPointLine);
    display.print (">GREATER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 4;
    display.setCursor(110, currentPointLine);
    display.print(">");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
    //display.setCursor(0, currentPointLine);
    display.print("LESSER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 19;
    display.setCursor(110, currentPointLine);
    display.print("<");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
  }

   //    LAYER 4  //
  //    CHOICE 1.1.2.2 //
  if (pic == 1122) {
    display.setTextSize(2);
    display.setCursor(0, currentPointLine);
    display.print ("GREATER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 4;
    display.setCursor(110, currentPointLine);
    display.print(">");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
    //display.setCursor(0, currentPointLine);
    display.print(">LESSER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 19;
    display.setCursor(110, currentPointLine);
    display.print("<");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
  }

  //    LAYER 4  //
  //    CHOICE 1.2.2.1 //
  if (pic == 1221) {
    display.setTextSize(2);
    display.setCursor(0, currentPointLine);
    display.print (">GREATER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 4;
    display.setCursor(110, currentPointLine);
    display.print(">");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
    //display.setCursor(0, currentPointLine);
    display.print("LESSER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 19;
    display.setCursor(110, currentPointLine);
    display.print("<");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
  }

  //    LAYER 4  //
  //    CHOICE 1.2.2.2 //
  if (pic == 1222) {
    display.setTextSize(2);
    display.setCursor(0, currentPointLine);
    display.print ("GREATER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 4;
    display.setCursor(110, currentPointLine);
    display.print(">");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
    //display.setCursor(0, currentPointLine);
    display.print(">LESSER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 19;
    display.setCursor(110, currentPointLine);
    display.print("<");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
  }

 /*****************************************************************************************
      Below are the choices for another 4 sensors when choosing between
      greater or lesser comparison for the operator value.
      Uncomment the according section depending on how many more sensors are needed
 *****************************************************************************************/ 
  /*
  //    LAYER 4  //
  //    CHOICE 1.3.2.1 //
  if (pic == 1321) {
    display.setTextSize(2);
    display.setCursor(0, currentPointLine);
    display.print (">GREATER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 4;
    display.setCursor(110, currentPointLine);
    display.print(">");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
    //display.setCursor(0, currentPointLine);
    display.print("LESSER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 19;
    display.setCursor(110, currentPointLine);
    display.print("<");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
  }

  //    LAYER 4  //
  //    CHOICE 1.3.2.2 //
  if (pic == 1322) {
    display.setTextSize(2);
    display.setCursor(0, currentPointLine);
    display.print ("GREATER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 4;
    display.setCursor(110, currentPointLine);
    display.print(">");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
    //display.setCursor(0, currentPointLine);
    display.print(">LESSER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 19;
    display.setCursor(110, currentPointLine);
    display.print("<");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
  }

  //    LAYER 4  //
  //    CHOICE 1.4.2.1 //
  if (pic == 1421) {
    display.setTextSize(2);
    display.setCursor(0, currentPointLine);
    display.print (">GREATER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 4;
    display.setCursor(110, currentPointLine);
    display.print(">");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
    //display.setCursor(0, currentPointLine);
    display.print("LESSER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 19;
    display.setCursor(110, currentPointLine);
    display.print("<");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
  }

  //    LAYER 4  //
  //    CHOICE 1.4.2.2 //
  if (pic == 1422) {
    display.setTextSize(2);
    display.setCursor(0, currentPointLine);
    display.print ("GREATER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 4;
    display.setCursor(110, currentPointLine);
    display.print(">");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
    //display.setCursor(0, currentPointLine);
    display.print(">LESSER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 19;
    display.setCursor(110, currentPointLine);
    display.print("<");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
  }

    //    LAYER 4  //
  //    CHOICE 1.5.2.1 //
  if (pic == 1521) {
    display.setTextSize(2);
    display.setCursor(0, currentPointLine);
    display.print (">GREATER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 4;
    display.setCursor(110, currentPointLine);
    display.print(">");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
    //display.setCursor(0, currentPointLine);
    display.print("LESSER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 19;
    display.setCursor(110, currentPointLine);
    display.print("<");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
  }

  //    LAYER 4  //
  //    CHOICE 1.5.2.2 //
  if (pic == 1522) {
    display.setTextSize(2);
    display.setCursor(0, currentPointLine);
    display.print ("GREATER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 4;
    display.setCursor(110, currentPointLine);
    display.print(">");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
    //display.setCursor(0, currentPointLine);
    display.print(">LESSER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 19;
    display.setCursor(110, currentPointLine);
    display.print("<");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
  }

    //    LAYER 4  //
  //    CHOICE 1.6.2.1 //
  if (pic == 1621) {
    display.setTextSize(2);
    display.setCursor(0, currentPointLine);
    display.print (">GREATER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 4;
    display.setCursor(110, currentPointLine);
    display.print(">");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
    //display.setCursor(0, currentPointLine);
    display.print("LESSER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 19;
    display.setCursor(110, currentPointLine);
    display.print("<");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
  }

  //    LAYER 4  //
  //    CHOICE 1.6.2.2 //
  if (pic == 1622) {
    display.setTextSize(2);
    display.setCursor(0, currentPointLine);
    display.print ("GREATER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 4;
    display.setCursor(110, currentPointLine);
    display.print(">");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
    //display.setCursor(0, currentPointLine);
    display.print(">LESSER:");
    display.setTextSize(1);
    currentPointLine = currentPointLine + 19;
    display.setCursor(110, currentPointLine);
    display.print("<");
    //currentPointLine = currentPointLine + 8;
    display.setTextSize(2);
    display.println();
  }
  */

  /*******************************************************************************
         This part contains what the menu will show when choosing
         for the corresponding threshold of each sensor.
         Further below there are commented parts that can be unommented
         for support of more sensors. Uncomment accoring to the sensor number
  *******************************************************************************/


  //    LAYER 4  //
  //    CHOICE 1.1.1.1 //
  if (pic == 1111) {
    display.setTextSize(2);
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[0][0]);
    display.print(tmpSel[0][1]);
    display.print(tmpSel[0][2]);
    display.print(tmpSel[0][3]);
    startingPointRow = 0;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.1.1.2 //
  if (pic == 1112) {
    display.setTextSize(2);
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[0][0]);
    display.print(tmpSel[0][1]);
    display.print(tmpSel[0][2]);
    display.print(tmpSel[0][3]);
    startingPointRow = 12;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.1.1.3 //
  if (pic == 1113) {
    display.setTextSize(2);
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[0][0]);
    display.print(tmpSel[0][1]);
    display.print(tmpSel[0][2]);
    display.print(tmpSel[0][3]);
    startingPointRow = 24;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.1.1.4 //
  if (pic == 1114) {
    display.setTextSize(2);
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[0][0]);
    display.print(tmpSel[0][1]);
    display.print(tmpSel[0][2]);
    display.print(tmpSel[0][3]);
    startingPointRow = 36;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.1.1.5 //
  if (pic == 1115) {
    display.setTextSize(2);
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[0][0]);
    display.print(tmpSel[0][1]);
    display.print(tmpSel[0][2]);
    display.print(tmpSel[0][3]);
    startingPointRow = 0;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(">CONFIRM");
  }

 //    LAYER 4  //
  //    CHOICE 1.2.1.1 //
  if (pic == 1211) {
    display.setTextSize(2);
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[1][0]);
    display.print(tmpSel[1][1]);
    display.print(tmpSel[1][2]);
    display.println(tmpSel[1][3]);
    startingPointRow = 0;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.2.1.2 //
  if (pic == 1212) {
    display.setTextSize(2);
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[1][0]);
    display.print(tmpSel[1][1]);
    display.print(tmpSel[1][2]);
    display.println(tmpSel[1][3]);
    startingPointRow = 12;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.2.1.3 //
  if (pic == 1213) {
    display.setTextSize(2);
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[1][0]);
    display.print(tmpSel[1][1]);
    display.print(tmpSel[1][2]);
    display.println(tmpSel[1][3]);
    startingPointRow = 24;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.2.1.4 //
  if (pic == 1214) {
    display.setTextSize(2);
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[1][0]);
    display.print(tmpSel[1][1]);
    display.print(tmpSel[1][2]);
    display.println(tmpSel[1][3]);
    startingPointRow = 36;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.2.1.5 //
  if (pic == 1215) {
    display.setTextSize(2);
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[1][0]);
    display.print(tmpSel[1][1]);
    display.print(tmpSel[1][2]);
    display.print(tmpSel[1][3]);
    startingPointRow = 0;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(">CONFIRM");
  }

/*****************************************************************************************
      Below are the choices for another 4 sensors when choosing the threshold value.
      Uncomment the according section depending on how many more sensors are needed
 *****************************************************************************************/   
  /*
  //    LAYER 4  //
  //    CHOICE 1.3.1.1 //
  if (pic == 1311) {
    display.setTextSize(2);
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[2][0]);
    display.print(tmpSel[2][1]);
    display.print(tmpSel[2][2]);
    display.print(tmpSel[2][3]);
    startingPointRow = 0;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.3.1.2 //
  if (pic == 1312) {
    int startingPointRow = 0;
    display.setTextSize(2);
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[2][0]);
    display.print(tmpSel[2][1]);
    display.print(tmpSel[2][2]);
    display.print(tmpSel[2][3]);
    startingPointRow = 12;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.3.1.3 //
  if (pic == 1313) {
    int startingPointRow = 0;
    display.setTextSize(2);
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[2][0]);
    display.print(tmpSel[2][1]);
    display.print(tmpSel[2][2]);
    display.print(tmpSel[2][3]);
    startingPointRow = 24;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.3.1.4 //
  if (pic == 1314) {
    int startingPointRow = 0;
    display.setTextSize(2);
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[2][0]);
    display.print(tmpSel[2][1]);
    display.print(tmpSel[2][2]);
    display.print(tmpSel[2][3]);
    startingPointRow = 36;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.3.1.5 //
  if (pic == 1315) {
    int startingPointRow = 0;
    display.setTextSize(2);
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[2][0]);
    display.print(tmpSel[2][1]);
    display.print(tmpSel[2][2]);
    display.print(tmpSel[2][3]);
    //startingPointRow = 0;
    //currentPointLine = currentPointLine + 16;
    //display.setCursor(startingPointRow, currentPointLine);
    //display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(">CONFIRM");
  }

 //    LAYER 4  //
  //    CHOICE 1.4.1.1 //
  if (pic == 1411) {
    int startingPointRow = 0;
    display.setTextSize(2);
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[3][0]);
    display.print(tmpSel[3][1]);
    display.print(tmpSel[3][2]);
    display.print(tmpSel[3][3]);
    startingPointRow = 0;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.4.1.2 //
  if (pic == 1412) {
    int startingPointRow = 0;
    display.setTextSize(2);
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[3][0]);
    display.print(tmpSel[3][1]);
    display.print(tmpSel[3][2]);
    display.print(tmpSel[3][3]);
    startingPointRow = 12;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.4.1.3 //
  if (pic == 1413) {
    int startingPointRow = 0;
    display.setTextSize(2);
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[3][0]);
    display.print(tmpSel[3][1]);
    display.print(tmpSel[3][2]);
    display.print(tmpSel[3][3]);
    startingPointRow = 24;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.4.1.4 //
  if (pic == 1414) {
    int startingPointRow = 0;
    display.setTextSize(2);
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[3][0]);
    display.print(tmpSel[3][1]);
    display.print(tmpSel[3][2]);
    display.print(tmpSel[3][3]);
    startingPointRow = 36;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.4.1.5 //
  if (pic == 1415) {
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[3][0]);
    display.print(tmpSel[3][1]);
    display.print(tmpSel[3][2]);
    display.print(tmpSel[3][3]);
    //startingPointRow = 36;
    //currentPointLine = currentPointLine + 16;
    //display.setCursor(startingPointRow, currentPointLine);
    //display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(">CONFIRM");
  }

 //    LAYER 4  //
  //    CHOICE 1.5.1.1 //
  if (pic == 1511) {
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[4][0]);
    display.print(tmpSel[4][1]);
    display.print(tmpSel[4][2]);
    display.print(tmpSel[4][3]);
    startingPointRow = 0;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.5.1.2 //
  if (pic == 1512) {
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[4][0]);
    display.print(tmpSel[4][1]);
    display.print(tmpSel[4][2]);
    display.print(tmpSel[4][3]);
    startingPointRow = 12;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.5.1.3 //
  if (pic == 1513) {
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[4][0]);
    display.print(tmpSel[4][1]);
    display.print(tmpSel[4][2]);
    display.print(tmpSel[4][3]);
    startingPointRow = 24;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.5.1.4 //
  if (pic == 1514) {
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[4][0]);
    display.print(tmpSel[4][1]);
    display.print(tmpSel[4][2]);
    display.print(tmpSel[4][3]);
    startingPointRow = 36;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.5.1.5 //
  if (pic == 1515) {
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[4][0]);
    display.print(tmpSel[4][1]);
    display.print(tmpSel[4][2]);
    display.print(tmpSel[4][3]);
    //startingPointRow = 36;
    //currentPointLine = currentPointLine + 16;
    //display.setCursor(startingPointRow, currentPointLine);
    //display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(">CONFIRM");
  }

 //    LAYER 4  //
  //    CHOICE 1.6.1.1 //
  if (pic == 1611) {
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[5][0]);
    display.print(tmpSel[5][1]);
    display.print(tmpSel[5][2]);
    display.print(tmpSel[5][3]);
    startingPointRow = 0;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.6.1.2 //
  if (pic == 1612) {
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[5][0]);
    display.print(tmpSel[5][1]);
    display.print(tmpSel[5][2]);
    display.print(tmpSel[5][3]);
    startingPointRow = 12;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.6.1.3 //
  if (pic == 1613) {
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[5][0]);
    display.print(tmpSel[5][1]);
    display.print(tmpSel[5][2]);
    display.print(tmpSel[5][3]);
    startingPointRow = 24;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.6.1.4 //
  if (pic == 1614) {
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[5][0]);
    display.print(tmpSel[5][1]);
    display.print(tmpSel[5][2]);
    display.print(tmpSel[5][3]);
    startingPointRow = 36;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 8;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }

  //    LAYER 4  //
  //    CHOICE 1.6.1.5 //
  if (pic == 1615) {
    int startingPointRow = 0;
    display.setCursor(startingPointRow, currentPointLine);
    display.print(tmpSel[5][0]);
    display.print(tmpSel[5][1]);
    display.print(tmpSel[5][2]);
    display.print(tmpSel[5][3]);
    //startingPointRow = 36;
    //currentPointLine = currentPointLine + 16;
    //display.setCursor(startingPointRow, currentPointLine);
    //display.print("^");
    startingPointRow = 0;
    currentPointLine = currentPointLine + 16;
    display.setCursor(startingPointRow, currentPointLine);
    display.print("CONFIRM");
  }
  */
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

/******************************************************************************
       This fuction is called when a single press is detected on the button
       Depending on the menu choice the function acts differently
 *****************************************************************************/
void click (void) {
  lastMillis = millis();
  //Serial.println(pic);
  
 /***********************************************************************************
        The following lines of code are used to determine the action of a single
        button press on the first layer of the menu
 ***********************************************************************************/
  if (menuLayer == 1) {
    if (pic == 0) {
      pic = 1000;
    } else if (pic == 1000) {
        pic = 2000;
    } else if (pic == 2000) {
        pic = 3000;
    } else if (pic == 3000) {
        pic = 4000;
    } else if (pic == 4000) {
        pic = 5000;
    } else if (pic == 5000) {
        pic = 1000;
    }
  }

 /***********************************************************************************
        The following lines of code are used to determine the action of a single
        button press on the second layer of the menu. To support more sensors 
        change the value of the condition in line 1318 and 1320 accordingly.
        Maximum of 6 sensors is suppoerted, maximum value of condition is 1700
 ***********************************************************************************/
  if (menuLayer == 2) {
    if (pic > 1000 && pic < 1200) {
      pic = pic + 100;
    } else if (pic >= 1200 && pic <= 2000) {
        pic = 1100;
    } else if (pic > 2000 && pic < 2500) {
        pic = pic + 100;
    } else if (pic >= 2500 && pic <= 3000){
        pic = 2100;
    } else if (pic > 3000 && pic < 3200){
        pic = pic + 100;
    } else if (pic >= 3200 && pic <= 4000) {
        pic = 3100;
    }
  }

  if (menuLayer == 3) {
    if ((pic >= 1100) && (pic < 1120)) {
      pic = pic + 10;
    } else if ((pic >= 1120) && (pic < 1200)) {
        pic = 1110;
    } else if ((pic >= 1200) && (pic < 1220)) {
        pic = pic + 10;
    } else if ((pic >= 1220) && (pic < 1300)) {
        pic = 1210;
    }

    /************************************************************************************
           The following lines should be uncommented accorind to how many
           sensors the user want to have activated. These are the conditions
           for the button actions on layer 3 of the menu, when the user will
           chooce for each sensor a threshold or an operator. For just 2 sensors
           leave everything as is.
     ************************************************************************************/
     /*   else if ((pic >= 1300) && (pic < 1320)) {
          pic = pic +10;
      } else if ((pic >= 1320) && (pic < 1400)) {
          pic = 1310;
      } else if ((pic >= 1400) && (pic < 1420)) {
          pic = pic +10;
      } else if ((pic >= 1420) && (pic < 1500)) {
          pic = 1410;
      } else if ((pic >= 1500) && (pic < 1520)) {
          pic = pic +10;
      } else if ((pic >= 1520) && (pic < 1600)) {
          pic = 1510;
      } else if ((pic >= 1600) && (pic < 1620)) {
          pic = pic + 10;
      } else if ((pic >= 1620) && (pic < 1700)) {
          pic = 1610;
      } */
  }  

  if (menuLayer == 4) {

    /********************************************************************************
          The following lines are used to detemine the single click action when
          the user has chosen to select an operator. To support more sensors
          uncomment the according if statements 
     *******************************************************************************/
    if ((pic == 1121) || (pic == 1221)) { /*|| (pic == 1321) || (pic == 1421) || (pic == 1521) || (pic == 1621) to add more sensors add accordingly to the if statement */
      pic++;
    } else if (pic == 1122) {
        pic = 1121;
    //} else if (pic == 1221) {
        //pic = 1222;
    } else if (pic == 1222) {
        pic = 1221;
    }/*else if (pic == 1321) {
        pic = 1322;
    } else if (pic == 1322) {
        pic = 1321;
    } else if (pic == 1421) {
        pic = 1422;
    } else if (pic == 1422) {
        pic = 1421;
    } else if (pic == 1521) {
        pic = 1522;
    } else if (pic == 1522) {
        pic = 1521;
    } else if (pic == 1621) {
        pic = 1622;
    } else if (pic == 1622) {
        pic = 1621;
    } */

    /********************************************************************************
          The following lines are used to detemine the single click action when
          the user has chosen to select a threshold. To support more sensors
          uncomment the according if statements 
     *******************************************************************************/
      else if ((pic >= 1110) && (pic < 1115)) {
        pic = pic + 1;
    } else if ((pic >= 1115) && (pic < 1120)) {
        pic = 1111;
    } else if ((pic >= 1210) && (pic < 1215)) {
        pic = pic + 1;
    } else if ((pic >= 1215) && (pic < 1220)) {
        pic = 1211;
    }/* else if ((pic >= 1310) && (pic < 1315)) {
        pic = pic + 1;
    } else if ((pic >= 1315) && (pic < 1320)) {
        pic = 1311;
    } else if ((pic >= 1410) && (pic < 1415)) {
        pic = pic + 1;
    } else if ((pic >= 1415) && (pic < 1420)) {
        pic = 1411;
    } else if ((pic >= 1510) && (pic < 1515)) {
        pic = pic + 1;
    } else if ((pic >= 1515) && (pic < 1520)) {
        pic = 1511;
    } else if ((pic >= 1610) && (pic < 1615)) {
        pic = pic + 1;
    } else if ((pic >= 1615) && (pic < 1620)) {
        pic = 1611;
    }*/
  }
  
}

/**************************************************************************************
       This function is called when a double click is detected on the button
       Depending on the menu choice the user is on, the functions acts differently
 *************************************************************************************/
void doubleClick (void) {
  lastMillis = millis();
  if (menuLayer == 2) {

    /************************************************************************************************
          Following lines are used for double button press action in second layer
    ************************************************************************************************/
    if (pic > 1000 && pic <= 1700) {
          pic = 1000;
    } else if (pic > 2000 && pic <= 3000) {
        pic = 2000;
    } else if (pic > 3000 && pic <= 4000) {
        pic = 3000; 
    } else if (pic > 4000 && pic <= 5000) {
        pic = 4000;
    } else if (pic > maxPics_L1) {
        pic = maxPics_L1;
    }
    menuLayer = menuLayer - 1;
  } else if (menuLayer == 3) {

    /************************************************************************************************
          Following lines are used for double button press action in the thrird layer.
          Uncomment the else if cases according to the number of sensors
    ************************************************************************************************/
      if ((pic >= 1110) || (pic < 1200)) {
        pic = 1100;
      } else if ((pic >= 1210) || (pic < 1300)) {
          pic = 1200;
      } /*else if ((pic >= 1310) || (pic < 1400)) {
          pic = 1300;
      } else if ((pic >= 1410) || (pic < 1500)) {
          pic = 1400;
      } else if ((pic >= 1510) || (pic < 1600)) {
          pic = 1500;
      } else if ((pic >= 1610) || (pic < 1700)) {
          pic = 1600; 
      } */
      menuLayer = menuLayer - 1;
  } else if (menuLayer == 1) {
    
  } else if (menuLayer == 4) {

    /************************************************************************************************
          Following lines are used for double button press action when in the fourth layer.
          Uncomment the else if cases according to the number of sensors
    ************************************************************************************************/
      if ((pic == 1121) || (pic == 1122)) {
        pic = 1120;
      } else if ((pic == 1221) || (pic == 1222)) {
          pic = 1220;
      } /*else if ((pic == 1321) || (pic == 1322)) {
          pic = 1320;
      } else if ((pic == 1421) || (pic == 1422)) {
          pic = 1420;
      } else if ((pic == 1521) || (pic == 1522)) {
          pic = 1520;
      } else if ((pic == 1621) || (pic == 1622)) {
          pic = 1620;
      } */

        else if ((pic >= 1110) && (pic < 1120)) {
          pic = 1110;
      } else if ((pic >= 1210) && (pic < 1220)) {
          pic = 1210;
      }/* else if ((pic >= 1310) && (pic < 1320)) {
          pic = 1310;
      } else if ((pic >= 1410) && (pic < 1420)) {
          pic = 1410;
      } else if ((pic >= 1510) && (pic < 1520)) {
          pic = 1510;
      } else if ((pic >= 1610) && (pic < 1620)) {
          pic = 1610;
      }*/

      
      menuLayer = menuLayer - 1;
  }
}
/******************************************************************************
       This function is called when a long press of the button is detected
       Depending on the menu choice that the user is on the function acts
       in a different way
 *****************************************************************************/
void longPressStart (void) {
  lastMillis = millis();

  if (menuLayer == 1) {
    pic = pic + 100;
    menuLayer = menuLayer + 1;
  } else if (menuLayer == 2) {

    /************************************************************************************************
          Following lines are used for long button press action when choosing for trigger delay
    ************************************************************************************************/
    if ((pic >= 2000) && (pic <= 2500)) {
      if (pic == 2100) {
        triggerDelay = 50;
      } else if (pic == 2200) {
          triggerDelay = 100;
      } else if (pic == 2300) {
          triggerDelay == 150;
      } else if (pic == 2400) {
          triggerDelay == 200;
      } else if (pic == 2500) {
          triggerDelay == 300;
      }
      pic = 2000;
      menuLayer = menuLayer - 1;
    }

    /************************************************************************************************
          Following lines are used for long button press action when choosing for trigger logic
    ************************************************************************************************/
    if ((pic >= 3000) && (pic <= 3200)) {
      if (pic == 3100) {
        triggerLogic = 1;
      } else if (pic = 3200) {
          triggerLogic = 0;
      }
      pic = 3000;
      menuLayer = menuLayer - 1;
    }
    /************************************************************************************************
          Following lines are used for long button press action when choosing between sensors.
          Uncomment the else if cases according to the number of sensors
    ************************************************************************************************/
    if((pic >= 1000) && (pic < 1700)) {
      if (pic == 1100) {
        pic = 1110; 
      } else if (pic == 1200) {
          pic = 1210;
      }/* else if (pic == 1300) {
          pic = 1310;
      } else if (pic == 1400) {
          pic = 1410;
      } else if (pic == 1500) {
          pic = 1510;
      } else if (pic == 1600) {
          pic = 1610;
      }*/
      menuLayer = menuLayer + 1;
    }
      //pic = pic + 10;
      //menuLayer = menuLayer + 1;
  } else if (menuLayer == 3) {

    /************************************************************************************************
          Following lines are used for long button press action when choosing to configure
          threshold or operator for a specific sensor.
          Uncomment the else if cases according to the number of sensors
    ************************************************************************************************/
      /*if (pic == 1120) {
        pic = 1121;
      } else if (pic == 1220) {
          pic = 1221;
      } */ /*else if (pic == 1320) {
          pic = 1321;
      } else if (pic == 1420) {
          pic = 1421;
      } else if (pic == 1520) {
          pic = 1521;
      } else if (pic == 1621) {
          pic = 1621;
      } */
      if ((pic == 1120) || (pic == 1220) || (pic == 1320) || (pic == 1420) || (pic == 1520) || (pic == 1620)) {
          pic = pic + 1;
      } else if ((pic == 1110) || (pic == 1210) || (pic == 1310) || (pic == 1410) || (pic == 1510) || (pic == 1610)) {
           pic = pic + 1;
      }
      //pic = pic + 1;
      menuLayer = menuLayer + 1;
  } else if (menuLayer == 4) {

    /************************************************************************************************
          Following lines are used for long button press action when choosing
          between the two comparison choices to the threshold.
          Uncomment the else if cases according to the number of sensors
    ************************************************************************************************/

    /************************************************************************************************
          Following section determines the behaviour of long press in the operator choice
          sub menu. Uncomment the commented else if statements below to support more sensors
    ************************************************************************************************/    
      if (pic == 1121) {
        dummyOp[0] = -3;
        pic = 1120;
        menuLayer = menuLayer - 1;
      } else if (pic == 1122) {
          dummyOp[0] = -4;
          pic = 1120;
          menuLayer = menuLayer - 1;
      } else if (pic == 1221) {
          dummyOp[1] = -3;
          pic = 1220;
          menuLayer = menuLayer - 1;
      } else if (pic == 1222) {
          dummyOp[1] = -4;
          pic = 1220;
          menuLayer = menuLayer - 1;
      } /*else if (pic == 1321) {
          dummyOp[2] = -3;
          pic = 1320;
      } else if (pic == 1322) {
          dummyOp[2] = -4;
          pic = 1320;
      } else if (pic == 1421) {
          dummyOp[3] = -3;
          pic = 1420;
      } else if (pic == 1422) {
          dummyOp[3] = -4;
          pic = 1420;
      } else if (pic == 1521) {
          dummyOp[4] = -3;
          pic = 1520;
      } else if (pic == 1522) {
          dummyOp[4] = -4;
          pic = 1520;
      } else if (pic == 1621) {
          dummyOp[5] = -3;
          pic = 1620;
      } else if (pic == 1622) {
          dummyOp[5] = -4;
          pic = 1620;
      }*/

    /************************************************************************************************
          Following section determines the behaviour of long press in the threshold choice
          sub menu. Uncomment the commented else if statements below to support more sensors
    ************************************************************************************************/  
    //          SENSOR 0           //
      else if (pic == 1111) {
        if (tmpSel[0][0] < 1) { //The MSB can't be higher than 1 cause only threshold values between 0 and 1024 are accepted
          tmpSel[0][0]++;
          tmpSel[0][1] = 0; //If the MSB is 1 then the 2n MSB can only be 0 since 1024 is the largest threshold value
          if (tmpSel[0][2] >= 2) {
            tmpSel[0][2] = 0;
          }
          if (tmpSel[0][3] >= 4) {
            tmpSel[0][3] = 0;
          }
        } else {
            tmpSel[0][0] = 0;
        }
        //menuLayer = menuLayer - 1;
      } else if (pic == 1112) {
          if (tmpSel[0][0] == 1) {
            tmpSel[0][1] = 0;
          } else {
              if (tmpSel[0][1] < 9) {
                tmpSel[0][1]++;
              } else {
                  tmpSel[0][1] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1113) {
          if (tmpSel[0][0] == 1) {
            if (tmpSel[0][2] < 2) {
              tmpSel[0][2]++;
            } else {
                tmpSel[0][2] = 0;
            }
          } else {
              if (tmpSel[0][2] < 9) {
                tmpSel[0][2]++;
              } else {
                  tmpSel[0][2] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1114) {
          if (tmpSel[0][0] == 1) {
            if (tmpSel[0][3] < 4) {
              tmpSel[0][3]++;
            } else {
                tmpSel[0][3] = 0;
            }
          } else {
              if (tmpSel[0][3] < 9) {
                tmpSel[0][3]++;
              } else {
                  tmpSel[0][3] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1115) {
          dummyZero[0] = (tmpSel[0][0]*1000) + (tmpSel[0][1]*100) + (tmpSel[0][2]*10) + tmpSel[0][3] ;
          menuLayer = menuLayer - 1;
          pic = 1110;
      }
      //menuLayer = menuLayer - 1;

    //          SENSOR 1           //
      else if (pic == 1211) {
        if (tmpSel[1][0] < 1) { //The MSB can't be higher than 1 cause only threshold values between 0 and 1024 are accepted
          tmpSel[1][0]++;
          tmpSel[1][1] = 0; //If the MSB is 1 then the 2n MSB can only be 0 since 1024 is the largest threshold value
          if (tmpSel[1][2] >= 2) {
            tmpSel[1][2] = 0;
          }
          if (tmpSel[1][3] >= 4) {
            tmpSel[1][3] = 0;
          }
        } else {
            tmpSel[1][0] = 0;
        }
        //menuLayer = menuLayer - 1;
      } else if (pic == 1212) {
          if (tmpSel[1][0] == 1) {
            tmpSel[1][1] = 0;
          } else {
              if (tmpSel[1][1] < 9) {
                tmpSel[1][1]++;
              } else {
                  tmpSel[1][1] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1213) {
          if (tmpSel[1][0] == 1) {
            if (tmpSel[1][2] < 2) {
              tmpSel[1][2]++;
            } else {
                tmpSel[1][2] = 0;
            }
          } else {
              if (tmpSel[1][2] < 9) {
                tmpSel[1][2]++;
              } else {
                  tmpSel[1][2] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1214) {
          if (tmpSel[1][0] == 1) {
            if (tmpSel[1][3] < 4) {
              tmpSel[1][3]++;
            } else {
                tmpSel[1][3] = 0;
            }
          } else {
              if (tmpSel[1][3] < 9) {
                tmpSel[1][3]++;
              } else {
                  tmpSel[1][3] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1215) {
          dummyZero[1] = (tmpSel[1][0]*1000) + (tmpSel[1][1]*100) + (tmpSel[1][2]*10) + tmpSel[1][3] ;
          menuLayer = menuLayer - 1;
          pic = 1210;
      }
    /*
    //          SENSOR 2           //
      else if (pic == 1311) {
        if (tmpSel[2][0] < 1) { //The MSB can't be higher than 1 cause only threshold values between 0 and 1024 are accepted
          tmpSel[2][0]++;
          tmpSel[2][1] = 0; //If the MSB is 1 then the 2n MSB can only be 0 since 1024 is the largest threshold value
          if (tmpSel[2][2] >= 2) {
            tmpSel[2][2] = 0;
          }
          if (tmpSel[2][3] >= 4) {
            tmpSel[2][3] = 0;
          }
        } else {
            tmpSel[2][0] = 0;
        }
        //menuLayer = menuLayer - 1;
      } else if (pic == 1312) {
          if (tmpSel[2][0] == 1) {
            tmpSel[2][1] = 0;
          } else {
              if (tmpSel[2][1] < 9) {
                tmpSel[2][1]++;
              } else {
                  tmpSel[2][1] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1313) {
          if (tmpSel[2][0] == 1) {
            if (tmpSel[2][2] < 2) {
              tmpSel[2][2]++;
            } else {
                tmpSel[2][2] = 0;
            }
          } else {
              if (tmpSel[2][2] < 9) {
                tmpSel[2][2]++;
              } else {
                  tmpSel[2][2] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1314) {
          if (tmpSel[2][0] == 1) {
            if (tmpSel[2][3] < 4) {
              tmpSel[2][3]++;
            } else {
                tmpSel[2][3] = 0;
            }
          } else {
              if (tmpSel[2][3] < 9) {
                tmpSel[2][3]++;
              } else {
                  tmpSel[2][3] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1315) {
          dummyZero[2] = (tmpSel[2][0]*1000) + (tmpSel[2][1]*100) + (tmpSel[1][2]*10) + tmpSel[2][3] ;
          menuLayer = menuLayer - 1;
          pic = 1310;
      }

    //          SENSOR 3           //
      else if (pic == 1411) {
        if (tmpSel[3][0] < 1) { //The MSB can't be higher than 1 cause only threshold values between 0 and 1024 are accepted
          tmpSel[3][0]++;
          tmpSel[3][1] = 0; //If the MSB is 1 then the 2n MSB can only be 0 since 1024 is the largest threshold value
          if (tmpSel[3][2] >= 2) {
            tmpSel[3][2] = 0;
          }
          if (tmpSel[3][3] >= 4) {
            tmpSel[3][3] = 0;
          }
        } else {
            tmpSel[3][0] = 0;
        }
        //menuLayer = menuLayer - 1;
      } else if (pic == 1412) {
          if (tmpSel[3][0] == 1) {
            tmpSel[3][1] = 0;
          } else {
              if (tmpSel[3][1] < 9) {
                tmpSel[3][1]++;
              } else {
                  tmpSel[3][1] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1413) {
          if (tmpSel[3][0] == 1) {
            if (tmpSel[3][2] < 2) {
              tmpSel[3][2]++;
            } else {
                tmpSel[3][2] = 0;
            }
          } else {
              if (tmpSel[3][2] < 9) {
                tmpSel[3][2]++;
              } else {
                  tmpSel[3][2] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1414) {
          if (tmpSel[3][0] == 1) {
            if (tmpSel[3][3] < 4) {
              tmpSel[3][3]++;
            } else {
                tmpSel[3][3] = 0;
            }
          } else {
              if (tmpSel[3][3] < 9) {
                tmpSel[3][3]++;
              } else {
                  tmpSel[3][3] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1415) {
          dummyZero[3] = (tmpSel[3][0]*1000) + (tmpSel[3][1]*100) + (tmpSel[3][2]*10) + tmpSel[3][3] ;
          menuLayer = menuLayer - 1;
          pic = 1410;
      }    

    //          SENSOR 4           //
      else if (pic == 1511) {
        if (tmpSel[4][0] < 1) { //The MSB can't be higher than 1 cause only threshold values between 0 and 1024 are accepted
          tmpSel[4][0]++;
          tmpSel[4][1] = 0; //If the MSB is 1 then the 2n MSB can only be 0 since 1024 is the largest threshold value
          if (tmpSel[4][2] >= 2) {
            tmpSel[4][2] = 0;
          }
          if (tmpSel[4][3] >= 4) {
            tmpSel[4][3] = 0;
          }
        } else {
            tmpSel[4][0] = 0;
        }
        //menuLayer = menuLayer - 1;
      } else if (pic == 1512) {
          if (tmpSel[4][0] == 1) {
            tmpSel[4][1] = 0;
          } else {
              if (tmpSel[4][1] < 9) {
                tmpSel[4][1]++;
              } else {
                  tmpSel[4][1] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1513) {
          if (tmpSel[4][0] == 1) {
            if (tmpSel[4][2] < 2) {
              tmpSel[4][2]++;
            } else {
                tmpSel[4][2] = 0;
            }
          } else {
              if (tmpSel[4][2] < 9) {
                tmpSel[4][2]++;
              } else {
                  tmpSel[4][2] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1514) {
          if (tmpSel[4][0] == 1) {
            if (tmpSel[4][3] < 4) {
              tmpSel[4][3]++;
            } else {
                tmpSel[4][3] = 0;
            }
          } else {
              if (tmpSel[4][3] < 9) {
                tmpSel[4][3]++;
              } else {
                  tmpSel[4][3] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1515) {
          dummyZero[4] = (tmpSel[4][0]*1000) + (tmpSel[4][1]*100) + (tmpSel[4][2]*10) + tmpSel[4][3] ;
          menuLayer = menuLayer - 1;
          pic = 1510;
      }          

    //          SENSOR 4           //
      else if (pic == 1511) {
        if (tmpSel[4][0] < 1) { //The MSB can't be higher than 1 cause only threshold values between 0 and 1024 are accepted
          tmpSel[4][0]++;
          tmpSel[4][1] = 0; //If the MSB is 1 then the 2n MSB can only be 0 since 1024 is the largest threshold value
          if (tmpSel[4][2] >= 2) {
            tmpSel[4][2] = 0;
          }
          if (tmpSel[4][3] >= 4) {
            tmpSel[4][3] = 0;
          }
        } else {
            tmpSel[4][0] = 0;
        }
        //menuLayer = menuLayer - 1;
      } else if (pic == 1512) {
          if (tmpSel[4][0] == 1) {
            tmpSel[4][1] = 0;
          } else {
              if (tmpSel[4][1] < 9) {
                tmpSel[4][1]++;
              } else {
                  tmpSel[4][1] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1513) {
          if (tmpSel[4][0] == 1) {
            if (tmpSel[4][2] < 2) {
              tmpSel[4][2]++;
            } else {
                tmpSel[4][2] = 0;
            }
          } else {
              if (tmpSel[4][2] < 9) {
                tmpSel[4][2]++;
              } else {
                  tmpSel[4][2] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1514) {
          if (tmpSel[4][0] == 1) {
            if (tmpSel[4][3] < 4) {
              tmpSel[4][3]++;
            } else {
                tmpSel[4][3] = 0;
            }
          } else {
              if (tmpSel[4][3] < 9) {
                tmpSel[4][3]++;
              } else {
                  tmpSel[4][3] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1515) {
          dummyZero[4] = (tmpSel[4][0]*1000) + (tmpSel[4][1]*100) + (tmpSel[4][2]*10) + tmpSel[4][3] ;
          menuLayer = menuLayer - 1;
          pic = 1510;
      }   

    //          SENSOR 5           //
      else if (pic == 1611) {
        if (tmpSel[5][0] < 1) { //The MSB can't be higher than 1 cause only threshold values between 0 and 1024 are accepted
          tmpSel[5][0]++;
          tmpSel[5][1] = 0; //If the MSB is 1 then the 2n MSB can only be 0 since 1024 is the largest threshold value
          if (tmpSel[5][2] >= 2) {
            tmpSel[5][2] = 0;
          }
          if (tmpSel[5][3] >= 4) {
            tmpSel[5][3] = 0;
          }
        } else {
            tmpSel[5][0] = 0;
        }
        //menuLayer = menuLayer - 1;
      } else if (pic == 1612) {
          if (tmpSel[5][0] == 1) {
            tmpSel[5][1] = 0;
          } else {
              if (tmpSel[5][1] < 9) {
                tmpSel[5][1]++;
              } else {
                  tmpSel[5][1] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1613) {
          if (tmpSel[5][0] == 1) {
            if (tmpSel[5][2] < 2) {
              tmpSel[5][2]++;
            } else {
                tmpSel[5][2] = 0;
            }
          } else {
              if (tmpSel[5][2] < 9) {
                tmpSel[5][2]++;
              } else {
                  tmpSel[5][2] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1614) {
          if (tmpSel[5][0] == 1) {
            if (tmpSel[5][3] < 4) {
              tmpSel[5][3]++;
            } else {
                tmpSel[5][3] = 0;
            }
          } else {
              if (tmpSel[5][3] < 9) {
                tmpSel[5][3]++;
              } else {
                  tmpSel[5][3] = 0;
              }
          }
          //menuLayer = menuLayer - 1;
      } else if (pic == 1615) {
          dummyZero[5] = (tmpSel[5][0]*1000) + (tmpSel[5][1]*100) + (tmpSel[5][2]*10) + tmpSel[5][3] ;
          menuLayer = menuLayer - 1;
          pic = 1610;
      }                       
      */
  }
}

/***************************************************************************************
       This function is called to detect if a long press has been stopped
       Depending on which menu choice the user is on the function acts accordingly
 **************************************************************************************/
void longPressStop (void) {
  
}

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

/*************************************************************************
       This function is called to refresh what the display prints
 *************************************************************************/
void screenRefresh (void) {
  display.display();
  display.clearDisplay();
  //display.clear();
}

/*************************************************************************
       This function is called of by the drawConfig function
       when the first layer of the menu is to be shown
*************************************************************************/
void drawConfigMenuLayer1 () {
  
}

/*************************************************************************
       This function is called of by the drawConfig function
       when the second layer of the menu is to be shown
*************************************************************************/
void drawConfigMenuLayer2 () {
  
}

/*************************************************************************
       This function is called of by the drawConfig function
       when the third layer of the menu is to be shown
*************************************************************************/
void drawConfigMenuLayer3 () {
  
}

/*************************************************************************
       This function is called of by the drawConfig function
       when the fourth layer of the menu is to be shown
*************************************************************************/
void drawConfigMenuLayer4() {
  
}

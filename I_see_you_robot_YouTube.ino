
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <Servo.h>
#include <NewPing.h>

#define TRIGGER_PIN  7        // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     6        // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200      // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

Servo myservo;                // create servo object to control a servo
int pos = 0;                  // variable to store the servo position

int ledPin = 13;              // pin for the LED

int pirPin = 8;               // choose the input pin (for PIR sensor)
int pirState = LOW;           // we start, assuming no motion detected
int pirVal = 0;               // variable for reading the pin status

static unsigned long timer = millis();

SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

void setup()
{
  mySoftwareSerial.begin(9600);
  Serial.begin(115200);

  pinMode(pirPin, INPUT);       // declare pir sensor as input
  pinMode(ledPin, OUTPUT);      // declare LED as output
  
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  Serial.println(F("DFPlayer Mini online."));
  
  myDFPlayer.setTimeOut(500); //Set serial communictaion time out 500ms
  
  //----Set volume----
  myDFPlayer.volume(20);  //Set volume value (0~30).
  
  //----Set different EQ----
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  
  //----Set device we use SD as default----
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  
  //----Mp3 play----

  myDFPlayer.play(3);  //Play the first mp3
  delay(3000);

  //----Read imformation----
  Serial.println(myDFPlayer.readState()); //read mp3 state
  Serial.println(myDFPlayer.readVolume()); //read current volume
  Serial.println(myDFPlayer.readEQ()); //read EQ setting
  Serial.println(myDFPlayer.readFileCounts()); //read all file counts in SD card
  Serial.println(myDFPlayer.readCurrentFileNumber()); //read current play file number
  Serial.println(myDFPlayer.readFileCountsInFolder(3)); //read fill counts in folder SD:/03

myservo.attach(9);  // attaches the servo on pin 9 to the servo object
myservo.writeMicroseconds(1500); // midway point
delay(2000);   
}

void loop()
{

 pirVal = digitalRead(pirPin);  // read input value
  if (pirVal == HIGH) {            // check if the input is HIGH    
      digitalWrite(ledPin, HIGH); // turn LED OFF
    
    if (pirState == LOW) {
      Serial.println("Motion detected!");
      pirState = HIGH;      

       if (millis() - timer > 5000) 
       {
        timer = millis();
        myDFPlayer.play(1);  // is anyone there
        }
    }
    sweepServo();
    
  } else {
      
          
      if (pirState == HIGH){      
      Serial.println("Motion ended!");
      pirState = LOW;
      digitalWrite(ledPin, LOW); // turn LED OFF
      myDFPlayer.play(4);  // sleep mode activated
      delay(2000);
    }
  }
  
 //sweepServo();

 
/*
 myDFPlayer.play(2);  // no hard feelings
 delay(3000);

 myDFPlayer.play(3);  // sentry mode activated
 delay(3000);

 myDFPlayer.play(4);  // sleep mode activated
 delay(3000);

 myDFPlayer.play(5);  // target acquired
 delay(3000);

 myDFPlayer.play(6);  // target lost
 delay(3000);

 myDFPlayer.play(7);  // there you are 2
 delay(3000);

 myDFPlayer.play(8);  // there you are
 delay(3000);

 myDFPlayer.play(9);  // who's there
 delay(3000);

 myDFPlayer.play(10);  // your business is appreciated
 delay(3000);

 myDFPlayer.play(11);  // activated
 delay(3000);

 myDFPlayer.play(12);  // are you still there
 delay(3000);

 myDFPlayer.play(13);  // could you come over here
 delay(3000);

 myDFPlayer.play(14);  // deploying
 delay(3000);

 myDFPlayer.play(15);  // dispensing product
 delay(3000);

 myDFPlayer.play(16);  // gotcha
 delay(3000);

 myDFPlayer.play(17);  // hello
 delay(3000);

 myDFPlayer.play(18);  // hey its me
 delay(3000);

 myDFPlayer.play(19);  // i don't blame you
 delay(3000);

 myDFPlayer.play(20);  // i don't hate you
 delay(3000);

 myDFPlayer.play(21);  // i see you
 delay(3000);
*/
}

void pingSensor() 
{
  delay(10);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  int pingcm = sonar.ping_cm();
  Serial.print(pingcm); 
  Serial.println("cm");
  if ((pingcm <= 20) && (pingcm != 0)) 
    { 
     myservo.detach();  
     rndmTarget();
     //myDFPlayer.play(7);  // there you are
     //delay(2000);
     myDFPlayer.play(5);  // target aquired
     delay(2000);
     while ((sonar.ping_cm() <= 20) && (sonar.ping_cm() != 0)) 
      {
        Serial.println("Target Aquired");
        delay(50);        
        }
     if (sonar.ping_cm() >= 20 && sonar.ping_cm() != 0)
      {
        myDFPlayer.play(6);  // target lost
        delay(3000);
        myDFPlayer.play(12);  // are you still there
        delay(2000);
        }
    }  
}

void sweepServo() 
{
  myservo.attach(9);
  for (pos = 0; pos <= 180; pos += 4) { // goes from 0 degrees to 180 degrees      
    //Serial.print("Servo Position ");
    //Serial.println(pos);
    pingSensor();
    myservo.write(pos); 
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 4) { // goes from 180 degrees to 0 degrees
    //Serial.print("Servo Position ");
    //Serial.println(pos);
    pingSensor();
    myservo.write(pos); 
    delay(10);                       // waits 15ms for the servo to reach the position
  }

  myservo.detach();  // attaches the servo on pin 9 to the servo object
}


void rndmTarget()
{
  int randNum = random(1, 5);
  if (randNum == 1) { myDFPlayer.play(7); }  // there you are 2
  if (randNum == 2) { myDFPlayer.play(8); }  // there you are
  if (randNum == 3) { myDFPlayer.play(16);}  // gotcha
  if (randNum == 4) { myDFPlayer.play(21);}  // i see you
    
  delay(2000);
}

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

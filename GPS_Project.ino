#include <IRremote.h>  
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial2(A2, A1); // RX, TX
static const uint32_t GPSBaud = 9600;

// Define Motor pin
#define speedPinR 5    //  RIGHT PWM pin connect MODEL-X ENA
#define RightDirectPin1  7    //Right Motor direction pin 1 to MODEL-X IN1 
#define RightDirectPin2  8    //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6    // Left PWM pin connect MODEL-X ENB
#define LeftDirectPin1  9    //Left Motor direction pin 1 to MODEL-X IN3 
#define LeftDirectPin2  10   //Left Motor direction pin 1 to MODEL-X IN4 

// Motor Control
void go_Advance(void)  //Forward
{
  digitalWrite(RightDirectPin1,HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  analogWrite(speedPinL,80);
  analogWrite(speedPinR,80);
}
void go_Left(void) //Turn left
{
  digitalWrite(RightDirectPin1,LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  analogWrite(speedPinL,0);
  analogWrite(speedPinR,80);
}
void go_Right(void) //Turn right
{
  digitalWrite(RightDirectPin1,HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  analogWrite(speedPinL,80);
  analogWrite(speedPinR,0);
}
void go_Back(void)  //Reverse
{
  digitalWrite(RightDirectPin1,LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  analogWrite(speedPinL,80);
  analogWrite(speedPinR,80);
}
void stop_Stop(void)    //Stop
{
  digitalWrite(RightDirectPin1,LOW);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,LOW);
}


//******************************************************************************************************
// Compass Variables & Setup

#include <Wire.h>
#include <QMC5883LCompass.h>
QMC5883LCompass compass;
uint8_t x, y, z;
uint8_t desired_heading;                                               // initialize variable - stores value for the new desired heading
uint8_t compass_heading;                                               // initialize variable - stores value calculated from compass readings                                              // the amount of deviation that is allowed in the compass heading - Adjust as Needed
uint8_t Heading_A;                                                     // variable to store compass heading
uint8_t Heading_B;                                                     // variable to store compass heading in Opposite direction
uint8_t pass = 0;                                                      // variable to store which pass the robot is on

//******************************************************************************************************                   
// get latest compass value
void getaCompass()
 {  
  compass.read();
  int compass_heading=compass.getAzimuth();
 }
//Set Heading
void setHeading()
{
   for (int i=0; i <= 5; i++)                                    // Take several readings from the compass to insure accuracy
      { 
         getaCompass();                                            // get the current compass heading
      }                                               
    desired_heading = compass_heading;                           // set the desired heading to equal the current compass heading
    Heading_A = compass_heading;                                 // Set Heading A to current compass 
    Heading_B = compass_heading + 180;                           // Set Heading B to current compass heading + 180  

      if (Heading_B >= 360)                                      // if the heading is greater than 360 then subtract 360 from it, heading must be between 0 and 360
         {
          Heading_B = Heading_B - 360;
         }
     
    Serial.print(F("Compass Heading Set: ")); 
    Serial.print(compass_heading);   
    Serial.print(F(" Degrees"));

    Serial.print(F("desired heading"));
    Serial.println(desired_heading);
    Serial.print(F("compass heading"));
    Serial.println(compass_heading);

}
//******************************************************************************************************
//Set GPS
unsigned long Distance_To_Home;                                    // variable for storing the distance to destination
uint8_t GPS_Course;                                                    // variable to hold the gps's determined course to destination
uint8_t Number_of_SATS;                                                // variable to hold the number of satellites acquired
TinyGPSPlus gps;   
uint8_t ac =0;                                                         // GPS array counter
uint8_t wpCount = 0;                                                   // GPS waypoint counter
double Home_LATarray[50];                                          // variable for storing the destination Latitude - Only Programmed for 5 waypoint
double Home_LONarray[50];                                          // variable for storing the destination Longitude - up to 50 waypoints
void getGPS()                                                 // Get Latest GPS coordinates
{
    while (mySerial2.available() > 0)
    gps.encode(mySerial2.read());
} 
//******************************************************************************************************
//Set Waypoint
void setWaypoint()                                            // Set up to 5 GPS waypoints
{
if (wpCount >= 0)
  {
    Serial.print(F("GPS Waypoint "));
    Serial.print(wpCount + 1);
    Serial.print(F(" Set "));
    getGPS();                                                 // get the latest GPS coordinates
    compass.read();                                             // update latest compass heading     
                                               
    Home_LATarray[ac] = gps.location.lat();                   // store waypoint in an array   
    Home_LONarray[ac] = gps.location.lng();                   // store waypoint in an array   
                                                              
    Serial.print(F("Waypoint #1: "));
    Serial.print(Home_LATarray[0],6);
    Serial.print(F(" "));
    Serial.println(Home_LONarray[0],6);
    Serial.print(F("Waypoint #2: "));
    Serial.print(Home_LATarray[1],6);
    Serial.print(F(" "));
    Serial.println(Home_LONarray[1],6);
    Serial.print(F("Waypoint #3: "));
    Serial.print(Home_LATarray[2],6);
    Serial.print(F(" "));
    wpCount++;                                                  // increment waypoint counter
    ac++;                                                       // increment array counter
        
  }         
  else {Serial.print(F("Waypoints Full"));}
}
// *************************************************************************************************************************************************
// GPS INFO
void gpsInfo()                                                  // displays Satellite data to user
  {
        Number_of_SATS = (int)(gps.satellites.value());         //Query Tiny GPS for the number of Satellites Acquired 
        Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),Home_LATarray[ac], Home_LONarray[ac]);  //Query Tiny GPS for Distance to Destination    
        Serial.print(F("Lat:"));
        Serial.print(gps.location.lat(),6);
        Serial.print(F(" Lon:"));
        Serial.print(gps.location.lng(),6);
        Serial.print(F(" "));
        Serial.print(Number_of_SATS); 
        Serial.print(F(" SATs "));
        Serial.print(Distance_To_Home);
        Serial.print(F("m")); 
        Serial.print(F("Distance to Home "));
        Serial.println(Distance_To_Home);
  
  }
  //******************************************************************************************************
  //gowaypoint
   void goWaypoint()
{   
Serial.println(F("Go to Waypoint"));
Serial.print(F("Home_Latarray "));
Serial.print(Home_LATarray[ac],6);
Serial.print(F(" "));
Serial.println(Home_LONarray[ac],6);   

Serial.print(F("Distance to Home"));   
Serial.print(Distance_To_Home);

Serial.print(F("ac= "));
Serial.print(ac);

 while (true)  
  {                                                                // Start of Go_Home procedure 
  getaCompass();                                                    // Update Compass heading                                          
  getGPS();                                                        // Tiny GPS function that retrieves GPS data - update GPS location// delay time changed from 100 to 10
  
  if (millis() > 5000 && gps.charsProcessed() < 10)                // If no Data from GPS within 5 seconds then send error
    Serial.println(F("No GPS data: check wiring"));     
 
  Distance_To_Home = TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),Home_LATarray[ac], Home_LONarray[ac]);  //Query Tiny GPS for Distance to Destination
  GPS_Course = TinyGPSPlus::courseTo(gps.location.lat(),gps.location.lng(),Home_LATarray[ac],Home_LONarray[ac]);                               //Query Tiny GPS for Course to Destination   
   
    if (Home_LATarray[ac] == 0) {
      Serial.print(F("End of Waypoints"));
      stop_Stop();      
      break;
      }      
    if (Distance_To_Home == 0)                                   // If the Vehicle has reached it's Destination, then Stop
        {
        stop_Stop();                                               // Stop the robot after each waypoint is reached
        Serial.println(F("You have arrived!"));                    // Print to Bluetooth device - "You have arrived"          
        ac++;                                                    // increment counter for next waypoint
        break;                                                   // Break from Go_Home procedure and send control back to the Void Loop 
                                                                 // go to next waypoint
        
        }   
   
   
   if ( abs(GPS_Course - compass_heading) <= 15)                  // If GPS Course and the Compass Heading are within x degrees of each other then go Forward                                                                  
                                                                  // otherwise find the shortest turn radius and turn left or right  
       {
         go_Advance();                                               // Go Forward
       } else 
         {                                                       
            uint8_t x = (GPS_Course - 360);                           // x = the GPS desired heading - 360
            uint8_t y = (compass_heading - (x));                      // y = the Compass heading - x
            uint8_t z = (y - 360);                                    // z = y - 360
            
            if ((z <= 180) && (z >= 0))                           // if z is less than 180 and not a negative value then turn left otherwise turn right
                  { go_Left();  }
             else { go_Right(); }               
        } 
    

  }                                                              // End of While Loop

  
}
//******************************************************************************************************
//Clear Wayopoint
void clearWaypoints()
{
   memset(Home_LATarray, 0, sizeof(Home_LATarray));             // clear the array
   memset(Home_LONarray, 0, sizeof(Home_LONarray));             // clear the array
   wpCount = 0;                                                 // reset increment counter to 0
   ac = 0;
   
   Serial.print(F("GPS Waypoints Cleared"));                      // display waypoints cleared
}

//******************************************************************************************************
// Bluetooth Variables & Setup

String str;                                                        // raw string received from android to arduino
uint8_t blueToothVal;                                                  // stores the last value sent over via bluetooth
//Bluetooth
void bluetooth()
{
 while (Serial.available())                                    // Read bluetooth commands over Serial1 // Warning: If an error with Serial1 occurs, make sure Arduino Mega 2560 is Selected
 {  
  {  
      str = Serial.readStringUntil('\n');                      // str is the temporary variable for storing the last sring sent over bluetooth from Android device
      //Serial.print(str);                                      // for testing purposes
  } 
    
    blueToothVal = (str.toInt());                               //  convert the string 'str' into an integer and assign it to blueToothVal
    Serial.print(F("BlueTooth Value "));
    Serial.println(blueToothVal);    


 switch (blueToothVal) 
 {
      case 1:                                
        Serial.println(F("Forward"));
        go_Advance();
        break;

      case 2:                 
        Serial.println(F("Reverse"));
        go_Back();
        break;

      case 3:         
        Serial.println(F("Left"));
        go_Left();
        break;
        
      case 4:                     
        Serial.println(F("Right"));
        go_Right();
        break;
        
      case 5:                                            
        Serial.println(F("Stop Car"));
        stop_Stop();
        break; 

      case 6:                 
        setWaypoint();
        break;
      
      case 7:        
        goWaypoint();
        break;  

      case 10:
        setHeading();
        break; 

      case 11:
         gpsInfo();
        break;
        
      case 16:
        clearWaypoints();
        break;  

      case 17:                    // finish with waypoints
        ac = 0;
        Serial.print(F("Waypoints Complete"));
        break;
      

 } // end of switch case
}
}

//setup
void setup()
{
  pinMode(RightDirectPin1, OUTPUT); 
  pinMode(RightDirectPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
  pinMode(LeftDirectPin1, OUTPUT);
  pinMode(LeftDirectPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop_Stop();    
  Serial.begin(9600);
  mySerial2.begin(9600);
  compass.init();
}
void loop()
{ 
  getaCompass();                                                   // Update the Compass Heading
  bluetooth();                                                     // Run the Bluetooth procedure to see if there is any data being sent via BT                                                    
  getGPS();                                                        // Update the GPS location
}

// Accident Location Notification system 
#include "TinyGPS++.h"
#include "SoftwareSerial.h"

#include "TinyGPS.h"
#include "math.h"

#include <String.h>
float test;

#include <SoftwareSerial.h>
#include <Wire.h>
#include <ADXL345.h>

//gyro code :
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int minVal=265;
int maxVal=402;

double x;
double y;
double z;
//gyro code.

String inString = "";    

String LAT1, secondVal, LON1, forthVal;                                       //Decleration for fetching details from GPS 
double lt,ln;  

//SoftwareSerial serial_connection(10,11); //RX=pin 10, TX=pin 11
//TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data       
double x, y, x1, e, y1, dist[2], a, b, c, d, dlat, dlon, lati[2],longi[2],mylat,mylon;                        //Decleratio for calculating distance
double leastdist;
int i,l=3;

SoftwareSerial mySerial(9, 10);
int k=0,XT=0.75,YT=1.85,ZT=3;                                                 //Decleration for setting threshold values for acceleration
ADXL345 adxl;


void setup()
{
  Serial.begin(9600);

  //gyro code 
    Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  //gyro code .
  
  Serial.println("Start listening for GPS Shield...");
  
  mySerial.begin(9600);   // Setting the baud rate of GSM Module  
  Serial.begin(9600);    // Setting the baud rate of Serial Monitor (Arduino)
  delay(100);
  k=0;
  adxl.powerOn();

  //set activity/ inactivity thresholds (0-255)
  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
 
  //look of activity movement on this axes - 1 == on; 0 == off 
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
 
  //look of inactivity movement on this axes - 1 == on; 0 == off
  adxl.setInactivityX(1);
  adxl.setInactivityY(1);
  adxl.setInactivityZ(1);
 
  //look of tap movement on this axes - 1 == on; 0 == off
  adxl.setTapDetectionOnX(0);
  adxl.setTapDetectionOnY(0);
  adxl.setTapDetectionOnZ(1);
 
  //set values for what is a tap, and what is a double tap (0-255)
  adxl.setTapThreshold(50); //62.5mg per increment
  adxl.setTapDuration(15); //625us per increment
  adxl.setDoubleTapLatency(80); //1.25ms per increment
  adxl.setDoubleTapWindow(200); //1.25ms per increment
 
  //set values for what is considered freefall (0-255)
  adxl.setFreeFallThreshold(7); //(5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(45); //(20 - 70) recommended - 5ms per increment
 
  //setting all interrupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  //register interrupt actions - 1 == on; 0 == off  
  adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);

  lati[0]=15.8497;
  longi[0]=74.4977;
  lati[1]=13.3409;
  longi[1]=74.7421;

}



void loop()
{
 //  while (Serial.available() > 0) 
  //{
   inString = Serial.readString();
      
    
    for (int i = 0; i < inString.length(); i++)                                                                //Fetches my present coordinates
    {
      if (inString.substring(i, i+1) == "*")
      {
        LAT1= inString.substring(1, i);
        secondVal = inString.substring(i+1);
        //Serial.println("Latitude: " + (firstVal) + ", second: " + (secondVal)); 
      }
    }

    for (int i = 0; i < secondVal.length(); i++) 
    {
      if (secondVal.substring(i, i+1) == "%")
      {
        LON1= secondVal.substring(0, i);
        forthVal = secondVal.substring(i+1);
        //Serial.println("Longitude: " + (thirdVal) + ", forth: " + (forthVal));    
      }
    }
   mylat= LAT1.toFloat();                                                                                     //Converts string value of coordinates fetched to double type
   mylon= LON1.toFloat();
   Serial.println(mylat);
   Serial.println(mylon);
    delay(1000);
   //Serial.println(lt,5);
   //Serial.println(ln,5);
   //Serial.println("Latitude: " + (LAT1) + ",    Longitude: " + (LON1) + ""); 
    //Serial.println(test, 5);
 //Boring accelerometer stuff   
  int x,y,z;  
  adxl.readXYZ(&x, &y, &z); //read the accelerometer values and store them in variables  x,y,z
  // Output x,y,z values 
/*  Serial.print("values of X , Y , Z: ");
  Serial.print(x);
  Serial.print(" , ");
  Serial.print(y);
  Serial.print(" , ");
  Serial.println(z);
  */
  double xyz[3];
  double ax,ay,az;                                                                            //Decleration for getting acceleration from ADXL345
  adxl.getAcceleration(xyz);
  ax = xyz[0];                                                                                    
  ay = xyz[1];
  az = xyz[2];
/* Serial.print("X=");
  Serial.print(ax);
    Serial.println(" g");
  Serial.print("Y=");
  Serial.print(ay);
    Serial.println(" g");
  Serial.print("Z=");
  Serial.print(az);
    Serial.println(" g");
  Serial.println("**********************");
 */
//  mylat =  12.9951;
 //mylon = 74.8094;
 //if (Serial.available()>0)

 //gyro code 

   Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
    int xAng = map(AcX,minVal,maxVal,-90,90);
    int yAng = map(AcY,minVal,maxVal,-90,90);
    int zAng = map(AcZ,minVal,maxVal,-90,90);

       x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
       y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
       z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

       //gyro code.

       /*   Serial.print("AngleX= ");
     Serial.println(x);

     Serial.print("AngleY= ");
     Serial.println(y);

     Serial.print("AngleZ= ");
     Serial.println(z);
     Serial.println("-----------------------------------------");
     delay(1000);
      */
  
   if(ax>=(XT+sin(3.142*x/180)||ay>=YT+sin(3.142*y/180)||az>=ZT)                                                                          //Condition to check if acceleration crosses the Threshold
   {
    //x1=
    //y1=
    for(i=0;i<2;i=i+1)
    {
     dlon = mylon- longi[i] ;
     dlat =  mylat-lati[i];
     //Serial.println(mylat);
     //Serial.println(mylon);
     dlon = dlon* (3.142 / 180);
     dlat = dlat * (3.142 / 180);
     a = (sin(dlat / 2) * sin(dlat / 2)) + cos(lati[i] * (3.142 / 180)) * cos(mylat * (3.142 / 180)) * (sin(dlon / 2) * sin(dlon / 2));

     c = 2 * asin( sqrt(a));


     dist[i] = 6400 * c ; //(where R is the radius of the Earth);
    }

    
    leastdist=dist[0];                                                                                 //Assigning leastdist to the dis1[0]
   
    for (i=1;i<2;i=i+1)                                           
    { if(leastdist>dist[i])
      leastdist=dist[i];                                                                              //Loop for calculation of leastdist
    }

    
    if(leastdist==dist[0] && k==0)
   
     {
      mySerial.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
      delay(1000);  // Delay of 1 second
      mySerial.println("AT+CMGS=\"+917975993756\"\r"); // Replace x with mobile number
      delay(1000);
      mySerial.print("Emergency @:");
      mySerial.print(dist[0]);// "+String());// The SMS text you want to send
      mySerial.println("Km");
      mySerial.print("Latitude @:");
      mySerial.println(mylat);
      mySerial.print("Longitude @:");
      mySerial.println(mylon);
        //delay(100);
     
      Serial.println(dist[0]);// "+String());// The SMS text you want to send
      delay(100);
      mySerial.println((char)26);// ASCII code of CTRL+Z for saying the end of sms to  the module 
      delay(1000);
      k=1;
      }
  
     

    else if(leastdist==dist[1])
    {
      if ( k==0)
      {
      mySerial.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
      delay(1000);  // Delay of 1 second
      mySerial.println("AT+CMGS=\"+917975993756\"\r"); // Replace x with mobile number
      delay(1000);
      mySerial.print("Emergency @:");
      mySerial.print(dist[1]);// "+String());// The SMS text you want to send
      mySerial.println("Km");
      mySerial.print("Latitude @:");
      mySerial.println(mylat);
      mySerial.print("Longitude @:");
      mySerial.println(mylon);
      delay(100);
      mySerial.println((char)26);// ASCII code of CTRL+Z for saying the end of sms to  the module 
      delay(1000);
      k=1;
      }
  
    }

  //}
  //}    
  

 if (mySerial.available()>0)
   Serial.write(mySerial.read());
    delay(500);

}
}

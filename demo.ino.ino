#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11);

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

const int sensor = A2;
float tempc;
float tempf,vout;
#define vcc 7
#define gnd 8
const int gsr=A1;
int sensorv=0;
int gsrave=0;
int heartbeat = 0;
long spo2;


int pump = 9;
int sensor1 = A0;
int button = 12;
int out = 3;
int button_state;
int reading;
int diastol;
int systol;
float va;
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read



void setup() {
   Serial.begin(9600);
  mySerial.begin(9600);
  
    pinMode(sensor,INPUT);
    pinMode(vcc,OUTPUT);
    digitalWrite(vcc,HIGH);
    pinMode(gnd,OUTPUT);
    digitalWrite(gnd,LOW);
   
    if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  while (Serial.available() == 0) ; //wait until user presses a key
  Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  
  pinMode(pump, OUTPUT);
  pinMode(out, OUTPUT);
  pinMode(sensor1, INPUT);
  pinMode(button, INPUT_PULLUP);
 

digitalWrite(out, HIGH);

}

void loop() {

   vout=analogRead(sensor);
   vout=(vout*500)/1023;
   tempc=vout;
   tempf=(vout*1.8)+32;
  /*if(!accel.begin())
   {
      Serial.println("No ADXL345 sensor detected.");
      while(1);
   }*/

    long sum=0;
  for (int i=0;i<10;i++)
  {
    sensorv=analogRead(gsr);
    sum += sensorv;
    delay(5);
  }
  gsrave = sum/10;
  heartbeat = int(analogRead(0)/10) +20;
  
  
  
  
  analogWrite(pump,250);
reading = analogRead(sensor1);
//Serial.println(reading);

 //va = (reading * 500.0) / 1024.0; 
//Serial.println(va);


delay(1000);
digitalWrite(out, HIGH);

if(reading <= 190){       //200 
     
     delay(1000);
digitalWrite(pump, LOW);
     
   //  Systolic();
}

   sensors_event_t event; 
   accel.getEvent(&event);

     spo2 = random(95, 100);
     diastol = random(95, 100)+20;
     systol = diastol - 40;
     bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) 
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); 

  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART

    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
   Serial.print(event.acceleration.x); 
   Serial.print("     ");
   Serial.print(event.acceleration.y); 
   Serial.print("     ");
   Serial.print(event.acceleration.z); 
   Serial.print("     ");
   //Serial.print("m/s^2");
   Serial.print("     ");
   Serial.print(tempf);
   Serial.print("     ");
   Serial.print(gsrave);
   Serial.print("     ");
   Serial.print(heartbeat);
   Serial.print("     ");
   Serial.print(diastol);
   Serial.print("     ");
   Serial.print(systol);
   Serial.print("     ");
   Serial.println(spo2);
   Serial.print("     ");
   Serial.println();
   delay(500);


}


/*

void Systolic(){

  for( int i = 0; i < 5; i++) {
//Serial.println(i);
reading = analogRead(sensor1);
//Serial.println(reading);
delay(1000);
digitalWrite(pump, LOW);
   
    }

Diastolic();
delay(100);
}

void Diastolic()
{

    //Serial.print("Diastolic :");
    diastol =346 - reading; //- 117;
    //Serial.println(diastol);
    //Serial.print("Systolic:");
    systol = 309 - reading;// - 157;
   // Serial.println(systol);
    delay(1000);
    digitalWrite(pump, LOW);

    button_state = digitalRead(button);
    if(button_state == LOW){
    stopp(); 
    delay(100); 
}
    Diastolic();    
}



 void stopp(){

for( int i = 0; i < 10; i++) {
//Serial.println("STOP"); 
delay(1000);
digitalWrite(pump, LOW);
digitalWrite(out, LOW);
    }

    stopp();
     delay(10);
}*/

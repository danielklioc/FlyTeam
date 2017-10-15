// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector   
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//button and bocina to rescalibrate
#define button 7
#define bocina 6

#define led_resp 13 
bool blinkResp;

#define OnOffPin 5
#define flexPin A2
#define commutPin 2
#define minPin 3
#define maxPin 4
int minF=350;
int maxF=130;
int minA=-60;
int maxA=60;
bool OnOffVal;
bool commutVal;
bool minVal;
bool maxVal;

/*  ----------------------------------------------------------------
    http://www.prometec.net/duplex-nrf2401
    Prog_79B_Emisor
    
    Usando un NRF2401 para comunicar dos Arduinos en modo Duplex
    Programa Emisor:
--------------------------------------------------------------------  
*/ 

#include <SPI.h>
// Transmitter labraries
#include "nRF24L01.h"
#include "RF24.h"

int hear[1];    // Acel right hand
int msg[3] ;    // Array a transmitir // 2x Servo and motor speed
int resp[2];    // Array a recivir // Aiplane response
RF24 radio(9,10); //radio channels
const uint64_t pipes[2] = { 0xF0F0F0F0E2LL, 0xF0F0F0F0D3LL };
bool ok=false;
unsigned long int started_waiting_at;
unsigned long int msec;

void setup(){
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(250000);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));

    // configure LED for output
    pinMode(led_resp, OUTPUT);
     
      //RADIO         
    pinMode(10, OUTPUT);
    radio.begin();
    radio.setRetries(15,15);    // Maximos reintentos 
    radio.setPayloadSize(32);   // Reduce el payload de 32 si tienes problemas
    radio.setChannel(115); 
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
    radio.startListening();
    
     calibrate();
}

void calibrate(){
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(977);
    mpu.setYAccelOffset(-2957);
    mpu.setZAccelOffset(1273);
    mpu.setXGyroOffset(-1305);
    mpu.setYGyroOffset(-8);
    mpu.setZGyroOffset(-38);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {

        //bocina
        tone(bocina,1000,100);

        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else {
        tone(bocina,1000,100);
        delay(150);
        tone(bocina,500,500);
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
} 

void loop(){
    msec=millis();
    //restart values
    int x=digitalRead(button);
    minVal=digitalRead(minPin);
    maxVal=digitalRead(maxPin);
    if(minVal==0){
      commutVal=digitalRead(commutPin);
              Serial.print(minVal);
              Serial.print("min\t");
      if(commutVal==0){
        minF=analogRead(flexPin);              
              Serial.println(minF);
      }
      else{
        minA=ypr[1]*180/M_PI;
              Serial.println(minA);
      }
    }
    if(maxVal==0){ 
      commutVal=digitalRead(commutPin);
              Serial.print(maxVal);
              Serial.print("max\t");
      if(commutVal==0){
        maxF=analogRead(flexPin);
              Serial.println(maxF);
      }
      else{
        maxA=ypr[1]*180/M_PI;
              Serial.println(maxA);
      }
    }
    if(x==0){
      while (x==0) x=digitalRead(button);
     calibrate();
    }



    
    radio.setChannel(115); 
    if ( radio.available() ){ // Si hay datos disponibles
        bool done = false;
        while (!done){        // Espera aqui hasta recibir algo
           done=radio.read(hear,sizeof(hear));
              
              Serial.print("Dato Recibido = ");      
              Serial.print(hear[0]);
              Serial.print("\t");
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

     if (mpuIntStatus & 0x02) {  

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        // flush buffer to prevent overflow
        mpu.resetFIFO();
        
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print(ypr[0]*180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1]*180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2]*180/M_PI);  
            Serial.print("\t");  
        #endif


        msg[0]=map(hear[0],minA,maxA,30,160);
        msg[0]=constrain(msg[0],30,160);
        msg[1]=map(ypr[1]*180/M_PI,minA,maxA,150,20);
        msg[1]=constrain(msg[1],20,150);

        msg[2]=analogRead(flexPin);
        msg[2]=map(msg[2],minF,maxF,0,60);
        msg[2]=constrain(msg[2],0,60);
              
      Serial.print(msg[0]);
      Serial.print("\t");
      Serial.print(msg[1]);
      Serial.print("\t");
      Serial.print(msg[2]);
      Serial.print("\t");


  OnOffVal=digitalRead(OnOffPin);
  if(OnOffVal==0){
    
    radio.stopListening(); 
    radio.setChannel(95); 
    bool ok = radio.write(msg, sizeof(msg) );

    if (ok){
        Serial.print("ok...");
    }
    else Serial.print("failed");
    Serial.print("\t");
      
    radio.startListening();    //Volvemos a la escucha
    started_waiting_at = millis();
    bool timeout = false;
    while ( ! radio.available() && ! timeout )  // Esperasmos repsuesta hasta 200ms
      if (millis() - started_waiting_at > 15 )timeout = true;
    
    if ( timeout ){
         Serial.print("Failed, response timed out");
         Serial.print("\t");
    }
    else{
        // Leemos el mensaje recibido 
        radio.read(resp, sizeof(resp) );
        int temp=resp[0];
        int hum=resp[1];
        Serial.print("Temperature: ");
        Serial.print(resp[0]);
        Serial.print("\t Humidity");
        Serial.print(resp[1]);
        Serial.print("\t");
        blinkResp = !blinkResp;
        digitalWrite(led_resp, blinkResp);

  if ((millis() - msec)>>5000){      
  //Serial.print("temperature: ");
  Serial.print("...");
  Serial.print(temp);
  Serial.print(" .");
  //Serial.print("humidity: ");
  Serial.println(hum);
  }
      }
  }
     }
  Serial.print(millis() - msec);
  Serial.println();
  
        }
    }
}

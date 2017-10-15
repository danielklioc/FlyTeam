/*  ----------------------------------------------------------------
    http://www.prometec.net/duplex-nrf2401
    Prog_79B_Emisor
    
    Usando un NRF2401 para comunicar dos Arduinos en modo Duplex
    Programa Receptor:
--------------------------------------------------------------------  
*/ 
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#include <DHT.h>
#define DHTPIN 2
#define DHTTYPE DHT11 

// Instantiates and initializes the dht object
DHT dht(DHTPIN, DHTTYPE);

RF24 radio(9,10);
const uint64_t pipes[2] = { 0xF0F0F0F0E2LL, 0xF0F0F0F0D3LL };

int hear[3];
int resp[2];

#define motor 6 
#define led_resp 8 
bool blinkResp;

//servos
#include <Servo.h>
Servo servoR;
Servo servoL;

//timer para servos
unsigned long timer;

long int msec;
long int prevmsec;


void setup()
{
    pinMode(10,OUTPUT);    
    pinMode(motor,OUTPUT);    
    pinMode(led_resp,OUTPUT); 
    Serial.begin(250000);
    
    // Start the communication with the DHT sensor by callibg the begin method of the dht object
    dht.begin();
  
    radio.begin();
    radio.setRetries(15,15);
    radio.setPayloadSize(32);
    radio.setChannel(95); 
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);
    
    servoR.attach(3);
    servoL.attach(4);
    radio.startListening();
              Serial.print("Start OK");
}

void loop(){

    if ( radio.available() ){ // Si hay datos disponibles
        bool done = false;
        while (!done){        // Espera aqui hasta recibir algo
           done=radio.read(hear,sizeof(hear));
           
           if(millis()-timer>=0){
            if(hear[0]!=95) servoR.write(hear[0]);
            if(hear[1]!=85) servoL.write(hear[1]);  
            analogWrite(motor,hear[2]);
           }
           timer=millis();
           
              Serial.print("Dato Recibido =");      
              Serial.print("\t");
              Serial.print(hear[0]);
              Serial.print("\t");
              Serial.print(hear[1]);
              Serial.print("\t");
              Serial.print(hear[2]);
              Serial.print("\t");  
              
              // blink LED to indicate activity
              blinkResp = !blinkResp;
              digitalWrite(led_resp, blinkResp);

            radio.stopListening();  // Dejamos d escuchar para poder hablar    int temp = dht.readTemperature();
            int temp = dht.readTemperature();
            int hum = dht.readHumidity();
            resp[0]=temp;
            resp[1]=hum;
            
            radio.write(resp, sizeof(resp) );
            Serial.print("Enviando Respuesta :");
            Serial.print(resp[0]);
            Serial.print("\t");
            Serial.print(resp[1]);
            Serial.print("\t");
            
            radio.startListening();    // Volvemos a la escucha para recibir mas paquetes

            if(blinkResp==true){ 
              prevmsec=millis();
              Serial.print(millis() - msec);
            }
            if(blinkResp==false){ 
              msec=millis();
              Serial.print(millis() - prevmsec);
            }
            Serial.println();
       }
    }        
}

// #include <SPI.h>
// #include <RF24.h>
// #include <printf.h>

// RF24 radio(7,8); //CE,CSN ports

// byte addresses [][6]={"1Node","2Node"};
// void setup() {
//   // put your setup code here, to run once:
//   radio.begin();
//   radio.setDataRate(RF24_250KBPS);
//   radio.setPALevel(RF24_PA_LOW);

//   radio.openWritingPipe(addresses[0]);
//   radio.openReadingPipe(1,addresses[1]);
//   radio.startListening();

//   Serial.begin(9600);
//   printf_begin();

//   radio.printDetails();
// }

// void loop() {
//   // put your main code here, to run repeatedly:

// }
// 18 Mar 2018 - simple program to verify connection between Arduino
//      and nRF24L01+
//  This program does NOT attempt any communication with another nRF24

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <printf.h>

#define CE_PIN   10
#define CSN_PIN 9

const byte thisSlaveAddress[5] = {'R','x','A','A','A'};
const byte thisSlaveADDR[5] = {'R','R','R','R','R'};

RF24 radio(CE_PIN, CSN_PIN);

char dataReceived[10]; // this must match dataToSend in the TX
bool newData = false;


void setup() {
    Serial.begin(9600);
    printf_begin();

    Serial.println("CheckConnection Starting");
    Serial.println();
    Serial.println("FIRST WITH THE DEFAULT ADDRESSES after power on");
    Serial.println("  Note that RF24 does NOT reset when Arduino resets - only when power is removed");
    Serial.println("  If the numbers are mostly 0x00 or 0xff it means that the Arduino is not");
    Serial.println("     communicating with the nRF24");
    Serial.println();
    radio.begin();
    radio.openReadingPipe(1, thisSlaveADDR);
    radio.printDetails();
    Serial.println();
    Serial.println();
    Serial.println("AND NOW WITH ADDRESS AAAxR  0x41 41 41 78 52   ON P1");
    Serial.println(" and 250KBPS data rate");
    Serial.println();
    radio.openReadingPipe(1, thisSlaveAddress);
    radio.setDataRate( RF24_250KBPS );
    radio.printDetails();
    Serial.println();
    Serial.println();
}


void loop() {

}
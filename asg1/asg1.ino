// DISCLAIMER/CITATION: most baremetal code was found by referring to application notes C examples given in the atmega328p datasheet
//                      as well some simplification and manual conversion to baremetal throughout the referenced library files wherever appropriate

// NOTE: .h files were not submitted due to
// - limit of 5 files per submission
// - .zip were not accepted

// Compiler directives to compute UBBR
// clock speed set to 16 MHz
#define F_CPU 16000000 // F_CPU needs to be declared BEFORE including delay.h
// standard baudrate setting 4800 bits/s
#define BAUD 4800
// formula from datasheet page 146 --> UBRRn = ((f_OSC/16 * BAUD) - 1)
#define MYUBRR F_CPU/16/BAUD-1 // asynchronous normal mode

#include <avr/io.h>
#include <string.h>
#include <avr/delay.h>

// Include the library:
#include "SharpIR.h"
#include "HCSR04.h"

// Define model and input pin:
#define IRPin A0
#define model 1080

// Create variable to store the distance:
int distance_cm;

// global status vars
// MODE 0 --> IR sensor
// MODE 1 --> US sensor
int MODE = 1;

// US SENSOR SPECIFIC
int trigPin = 12; // yellow jumper --> pin 12: PB4
int echoPin = 8;// blue jumper --> pin 8: PB0

// IR SENSOR SPECIFIC
long duration, cm, inches;

// We are using model GP2Y0A21YK0F i.e. 1080
// the code for this class object was taken from here: https://github.com/guillaume-rico/SharpIR/blob/master/SharpIR.cpp
// and formatted/simplified/ported to baremetal as needed
SharpIR mySensor = SharpIR(IRPin, model);
// this code was originally taken from here: https://github.com/Martinsos/arduino-lib-hc-sr04/blob/master/src/HCSR04.cpp
// and formatted/simplified/ported to baremetal as needed
UltraSonicDistanceSensor distanceSensor(trigPin, echoPin);

int index = 0;
char buffer[256];

// this method was taken from: https://www.geeksforgeeks.org/program-count-digits-integer-3-different-methods
// used to get the number of characters in an integer value to be passed to the serial write method
int count_digit(long long n)
{
    if (n == 0)
        return 1;
    int count = 0;
    while (n != 0) {
        n = n / 10;
        ++count;
    }
    return count;
}

void USART_init(unsigned int ubrr) {
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr >> 8); // Shift needed to clear out first 8 bits
    UBRR0L = (unsigned char) (ubrr);

    // Enable receiver and transmitter
    UCSR0B = ((1 << RXEN0) | (1 << TXEN0));

    // Set two stop bits with USBS0, set frame to 8 bits of data with UCSz00
    // based on datasheet example code page 149
    UCSR0C = ((1 << USBS0) | (3 << UCSZ00));
}

// based on C code example page 150
void USART_transmit(char data) {
    // wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));

    // place data into buffer and send
    UDR0 = data;
}

// send multiple chars given string pointer and length
void msg_transmit(char* data, int len) {
    for (int i = 0; i < len; i++) {
        USART_transmit(data[i]);
    }
}

void set_brightness(int brightness) {
    // "the result of the compare can be used by the waveform generator to generate a PWM or variable frequency output on the output compare pins (OC2A and OC2B)"
    // page 117
    OCR2A = brightness;
    OCR2B = brightness;
}

int show_mode() {
  msg_transmit("0 --> IR sensor\n", 16);
  msg_transmit("1 --> US sensor\n", 16);

  if (MODE == 0) { 
    msg_transmit("You chose: option 0 --> IR sensor\n", 35);
    return MODE;
  } else if (MODE == 1) { 
    msg_transmit("You chose: option 1 - US sensor\n", 32);
    return MODE;
  } else {
    msg_transmit("Invalid option, try again\n", 26);
    _delay_ms(1000);
  }
}

// the calm before the storm...
void pause_champ() {
  msg_transmit("Starting in 3 seconds.", 22);
  _delay_ms(1000);
  msg_transmit(".", 1);
  _delay_ms(1000);
  msg_transmit(".", 1);
  _delay_ms(1000);
}

// to reset just close/reopen serial monitor via: ctrl + shift + m
void setup() {
  // Begin serial communication at a baudrate of 4800:
  USART_init(MYUBRR);

  show_mode(); // display to user current mode: 0 --> IR or 1 --> US sensor
  pause_champ(); // wait for 3 seconds before starting the readings...

  // pinMode(11, OUTPUT); // D11 = PB3 = D3
  DDRB |= (1 << PB3); // set external LED D3 to output mode

  // setup for analogwrite equivalent via PWM
  // OCR = Output Compare Register, 2 for Clock 2
  OCR2A = 255; // set duty cycles to maximum value
  OCR2B = 255;
  
  // timer/counter2 control register A
  // set fast PWM mode, non inverting
  // page 128
  // COM2A1 --> clear OC2A on compare match
  // COM2B1 --> clear OC2B on compare match
  TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // WGM --> WaveForm Generation Mode
  // set prescaler to 64 (i.e. divide clock speed of timer 1)
  TCCR2B = (1 << CS22);
}

char distance_str[3];
int numOfDigits = 0;

void loop() {
  if (MODE == 0) { // IR SENSOR
    distance_cm = mySensor.distance();   
  }
  else if (MODE == 1) { // US SENSOR
    distance_cm = distanceSensor.measure_distance_cm();
  }
    
  // Print the measured distance to the serial monitor:
  msg_transmit("Mean distance: ", 15);
  itoa(distance_cm, distance_str, 10);
  numOfDigits = count_digit(distance_cm);

  msg_transmit(distance_str, numOfDigits);
  msg_transmit(" cm\n", 4);

  // spec: "12cm or less - 100%"
  if (distance_cm <= 12) {
    //analogWrite(11, 0);
    // LED D3 is active low
    set_brightness(1);
    PORTB &= ~(1 << PB3);
    
    //digitalWrite(13, HIGH); // pin 13 = PB5
    PORTB |= (1 << PB5);
  }
  // spec: "42cm or more - 0%"
  else if (distance_cm >= 42) {
    //analogWrite(11, 255);
    //digitalWrite(11, HIGH); // pin 11 = PB3
    set_brightness(255);
    PORTB |= (1 << 3);
    //digitalWrite(13,HIGH); // pin 13 = PB5
    PORTB |= (1 << 5);
  }
  else {
    // spec: "linearly increases from 42 cm to 12 cm"
    // formula explanation: (measured distance - min distance)/(distance range)) * 255
    // e.g. when distance measured at 12 cm then 0/30 * 255 = 0 --> ext. LED set to max brightness
    // e.g. when distance measured at 42 cm then 30/30 * 255 = 255 --> ext. LED set to min brightness (i.e. simply OFF)
    set_brightness((255*((float) (distance_cm - 12)/30)));
    //digitalWrite(13, LOW); // pin 13 = PB5
    PORTB &= ~(1 << PB5);
  }
}

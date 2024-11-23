/*
  SharpIR
  Arduino library for retrieving distance (in cm) from the analog GP2Y0A21Y and GP2Y0A02YK
  From an original version of Dr. Marcal Casas-Cartagena (marcal.casas@gmail.com)     
  
    Version : 1.0 : Guillaume Rico
    + Remove average and use median
    + Definition of number of sample in .h
    + Define IR pin as input
    Version : 1.1 : Thibaut Mauon
    + Add SHARP GP2Y0A710K0F for 100cm to 500cm by Thibaut Mauron
    Version : 1.2 : Archery2000
    + Add Median of Medians algorithm to speed up sensor reading computation
  https://github.com/guillaume-rico/SharpIR
    
    Original comment from Dr. Marcal Casas-Cartagena :
   The Sahrp IR sensors are cheap but somehow unreliable. I've found that when doing continous readings to a
   fix object, the distance given oscilates quite a bit from time to time. For example I had an object at
   31 cm. The readings from the sensor were mainly steady at the correct distance but eventually the distance
   given dropped down to 25 cm or even 16 cm. That's quite a bit and for some applications it is quite
   unacceptable. I checked the library http://code.google.com/p/gp2y0a21yk-library/ by Jeroen Doggen
   (jeroendoggen@gmail.com) and what the author was doing is to take a bunch of readings and give an average of them
   The present library works similary. It reads a bunch of readings (avg), it checks if the current reading
   differs a lot from the previous one (tolerance) and if it doesn't differ a lot, it takes it into account
   for the mean distance.
   The distance is calculated from a formula extracted from the graphs on the sensors datasheets
   After some tests, I think that a set of 20 to 25 readings is more than enough to get an accurate distance
   Reading 25 times and return a mean distance takes 53 ms. For my application of the sensor is fast enough.
   This library has the formulas to work with the GP2Y0A21Y and the GP2Y0A02YK sensors but exanding it for
   other sensors is easy enough.
*/

#ifdef Arduino
  #include "Arduino.h"
#elif defined(SPARK)
  #include "Particle.h"
  #include "math.h"
#endif
#include "SharpIR.h"

// Initialisation function
//  + irPin : is obviously the pin where the IR sensor is attached
//  + sensorModel is a int to differentiate the two sensor models this library currently supports:
//    > 1080 is the int for the GP2Y0A21Y and 
//    > 20150 is the int for GP2Y0A02YK and 
//    > 100500 is the long for GP2Y0A710K0F
//    The numbers reflect the distance range they are designed for (in cm)
SharpIR::SharpIR(int irPin, long sensorModel) {
  
    int distanceSensorPin = 0; // A0
    _irPin=irPin;
    _model=sensorModel;
    
    // Define pin as Input
    //pinMode (_irPin, INPUT);

    // the higher the prescaler, the lower the frequency
    // ADEN is for ADC (Analog to Digital Conversion) mode
    ADCSRA |= (1 << ADEN) | (1 << ADPS2);
    // this register holds the converted value from the distance sensor
    ADMUX |= distanceSensorPin | (1 << REFS0);
  
    // attempt to refer to external AREF set by potentiometer
    analogReference(EXTERNAL);
}

// Sort an array
void SharpIR::sort(int a[], int size) {
    for(int i=0; i<(size-1); i++) {
        bool flag = true;
        for(int j=0; j<(size-(i+1)); j++) {
            if(a[j] > a[j+1]) {
                int t = a[j];
                a[j] = a[j+1];
                a[j+1] = t;
                flag = false;
            }
        }
        if (flag) break;
    }
}

// Read distance and compute it
int SharpIR::distance() {
  // model: 1080
  // sample size: 25
    int ir_val[NB_SAMPLE] = {};
    int adc_val[NB_SAMPLE] = {};
    int distanceCM;
    float current;
    int median;


    for (int i = 0; i < NB_SAMPLE; i++) {
        // start conversion
        ADCSRA |= (1 << ADSC);

        // wait until it's clear
        while (!(ADCSRA & (1 << ADIF)));

        // Reset to 1 for the next conversion
        ADCSRA |= (1 << ADIF);

        ir_val[i] = ADC;
    }
    
    // Get the approx median
    // Sorting all ADC values
      sort(ir_val, NB_SAMPLE);      
      median = ir_val[NB_SAMPLE/2+1];
      
      // for experimentation sake
      // Serial.print("ADC: "); 
      // Serial.print(median);
      // Serial.println();

    distanceCM = 29.988 * pow(map(median, 0, 1023, 0, 5000)/1000.0, -1.173);

    return distanceCM;
}

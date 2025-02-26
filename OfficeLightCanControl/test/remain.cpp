#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

const int LED_PIN = 1;
const int DAC_RANGE = 4096;
int counter = 0;

void setup() 
{// the setup function runs once
 Serial.begin(115200);
 analogReadResolution(12); //default is 10
 analogWriteFreq(60000); //60KHz, about max
 analogWriteRange(DAC_RANGE); //100% duty cycle
}

void loop() {// the loop function runs cyclically
 int read_adc;
 analogWrite(LED_PIN, counter); // set led PWM
 delay(1); //delay 1ms
 read_adc = analogRead(A0); // read analog voltage
 counter = counter + 1;
 if (counter > DAC_RANGE) // if counter saturates
 counter = 0; // reset counter
 //format that Serial Plotter likes
 Serial.printf("Const:0,DAC_RANGE:%d,ADC_read:%d,Counter:%d\n", DAC_RANGE, read_adc, counter);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}







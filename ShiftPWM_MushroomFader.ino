// You can choose the latch /pin yourself.
// 74HC595N pins:
// IC PIN 14 (DS) -> 11 (Data)
// IC PIN 12 (ST_CP) -> 8 (Latch)
// IC PIN 11 (SH_CP) -> 13 (Clock)
const int ShiftPWM_latchPin = 8;
// If your LED's turn on if the pin is low, set this to true, otherwise set it to false.
const bool ShiftPWM_invertOutputs = false;
// You can enable the option below to shift the PWM phase of each shift register by 8 compared to the previous.
// This will slightly increase the interrupt load, but will prevent all PWM signals from becoming high at the same time.
// This will be a bit easier on your power supply, because the current peaks are distributed.
const bool ShiftPWM_balanceLoad = false;

#include <ShiftPWM.h>   // inclu  de ShiftPWM.h after setting the pins!
#include <math.h>

// Here you set the number of brightness levels, the update frequency and the number of shift registers.
// These values affect the load of ShiftPWM.
// Choose them wisely and use the PrintInterruptLoad() function to verify your load.
unsigned char maxBrightness = 255;
unsigned char minBrightness = 25;
unsigned char pwmFrequency = 75;
unsigned int numRegisters = 1; // we have one shift register
unsigned int numOutputs = numRegisters * 8;
unsigned int fadingMode = 0; // LED fading mode
unsigned int activeFadingMode = 0; // Current LED fading mode
unsigned long startTime = 0; // start time for the chosen fading mode
/*
 * Change these to change the way the lights wave and pulse
 */
float factor = 1450.0; // factor for the pulsing LED formula
float multiplier = 5; // multiplier for the pulsing LED formula
// Analog input pins
const int ldrPin = A1; // light detection
const int DARKNESS = 50; // the value for the LDR to report that it's dark
// ldr value
int ldrValue = 0;
unsigned long fadeTime = 1000; // initial fade time, can be altered by turning the pot

// Function prototypes
unsigned char getPulseBrightness(unsigned long tm, float offset = 0.0, float multiplier = 2.0, float threshold = 108.0);

/*
   Set up the system here
*/
void setup() {
  while (!Serial) {
    delay(100);
  }**********************/00
  Serial.begin(9600);

  // initialize the pushbutton pin as an input:
  pinMode(ldrPin, INPUT);

  // Sets the number of 8-bit registers that are used.
  ShiftPWM.SetAmountOfRegisters(numRegisters);

  // SetPinGrouping allows flexibility in LED setup.
  // If your LED's are connected like this: RRRRGGGGBBBBRRRRGGGGBBBB, use SetPinGrouping(4).
  ShiftPWM.SetPinGrouping(1); //This is the default, but I added here to demonstrate how to use the funtion

  ShiftPWM.Start(pwmFrequency, maxBrightness);
  Serial.println("Lights READY!");
}

/*
   Loop that makes things work
*/
void loop() {

  // check controls
  // read pot value for the factor/multiplier
  if (isDark()) {
    activeFadingMode = 3;
  } else {
    activeFadingMode = fadingMode;
  }
  //  Serial.print("Active Fading Mode: ");
  //  Serial.println(activeFadingMode);
  // put your main code here, to run repeatedly:
  switch (activeFadingMode) {
    case 0:
      // Turn all LED's off.
      ShiftPWM.SetAll(0);
      break;
    case 1:
      oneByOne(1500);
      break;
    case 2:
      inOutTwo(1500);
      break;
    case 3:
      mushroomFlicker(factor, multiplier);
      break;
    case 4:
      inOutAll(1500);
      break;
    case 5:
      // Turn all LED's on.
      ShiftPWM.SetAll(255);
      break;
    default:
      Serial.println("Unknown Mode!");
      delay(1000);
      break;
  }
}
/*
   Fade LEDs in and out one by one using a liner fade
*/
void oneByOne(unsigned long fadeTime) { // Fade in and fade out all outputs one at a time
  unsigned char brightness;
  unsigned long loopTime = numOutputs * fadeTime * 2;
  unsigned long time = millis() - startTime;
  unsigned long timer = time % loopTime;
  unsigned long currentStep = timer % (fadeTime * 2);

  int activeLED = timer / (fadeTime * 2);
  if (currentStep <= fadeTime ) {
    brightness = currentStep * maxBrightness / fadeTime; ///fading in
  }
  else {
    brightness = maxBrightness - (currentStep - fadeTime) * maxBrightness / fadeTime; ///fading out;
  }
  ShiftPWM.SetAll(0);
  ShiftPWM.SetOne(activeLED, brightness);
}
/*
   Fade LEDS in and out by twos using a linear fade
*/
void inOutTwo(unsigned long fadeTime) { // Fade in and out 2 outputs at a time
  unsigned long loopTime = numOutputs * fadeTime;
  unsigned long time = millis() - startTime;
  unsigned long timer = time % loopTime;
  unsigned long currentStep = timer % fadeTime;
  int activeLED = timer / fadeTime;
  unsigned char brightness = currentStep * maxBrightness / fadeTime;

  ShiftPWM.SetAll(0);
  ShiftPWM.SetOne((activeLED + 1) % numOutputs, brightness);
  ShiftPWM.SetOne(activeLED, maxBrightness - brightness);
}
/*
   Fade LEDs in and out in a 'chain' using the pulsing waveform.
   LEDs appear to 'chase' each other making sure that all LEDs in the
   chain are part of the fading action at the same time.
*/
void mushroomFlicker(float factor, float multiplier) { // Fade in and out 2 outputs at a time
  unsigned char brightness;
  unsigned long time = millis();//-startTime;

  for (int i = 0; i < numOutputs; i++) {
    // 104.237470709 = 245/(e - 1/e) is calculated for a max of 245 so we can have a min brightness threshold of 10.
    brightness = getPulseBrightness(time, i * factor, multiplier, 104.237470709) + 10;
    //brightness = brightness<minBrightness?minBrightness:brightness;
    ShiftPWM.SetOne(i, brightness);
  }
}
/*
   Fade all LEDs in and out at the same time using a linear fade.
*/
void inOutAll(unsigned long fadeTime) { // Fade in all outputs
  unsigned char brightness;
  unsigned long time = millis() - startTime;
  unsigned long currentStep = time % (fadeTime * 2);

  if (currentStep <= fadeTime ) {
    brightness = currentStep * maxBrightness / fadeTime; ///fading in
  }
  else {
    brightness = maxBrightness - (currentStep - fadeTime) * maxBrightness / fadeTime; ///fading out;
  }
  ShiftPWM.SetAll(brightness);
}
/*
   Test the light dependent resistor value
*/
bool isDark() {
  float ldrValueAvg = 0;
  float sensorValue = 0;
  int numReadings = 10;
  int filterWeight = 16;
  float ldrValue = analogRead(ldrPin);
  ldrValue = (1023 - ldrValue) * 10 / ldrValue;
  //  Serial.print("LDR: ");
  //  Serial.println(ldrValue);
  
  if (ldrValue > DARKNESS) {
    return true;
  } else {
    ldrValueAvg = ldrValue;
    // employ some sensor smoothing via a 'low pass' filter so we don't get annoying flicker
    for (int i=0; i<numReadings; i++) {
      sensorValue = analogRead(ldrPin);
      sensorValue = (1023 - sensorValue) * 10 / sensorValue;
      ldrValueAvg = ldrValueAvg + ( sensorValue - ldrValueAvg) / filterWeight;
      delay(1);
    }
    if (ldrValueAvg > DARKNESS) {
      return true;
    } else {
      return false;
    }
  }
}
/*
   Calculate the brightness value based on a pulsing waveform take from:
   http://sean.voisen.org/blog/2011/10/breathing-led-with-arduino/
*/
unsigned char getPulseBrightness(unsigned long tm, float offset, float multiplier, float threshold) {
  return (exp(sin((tm + offset) * PI / (1000.0 * multiplier))) - 0.36787944) * threshold;
}



/*
      This code was quick and dirty, based on a PCM audio example in the
      arduino playground: http://playground.arduino.cc/Code/PCMAudio

      It's been heavely modified for use with RC to generate something that's
      a bit like an engine sound. I've started work on making the program
      readable, still some to do though.
      https://github.com/BeigeMatchbox/mojoEngineSim/blob/master/README.md

      Enhancements, done by TheDIYGUY999 in january 2017: https://github.com/TheDIYGuy999/Rc_Engine_Sound
        - more sounds added,
        - also works on a 8MHz MCU, but not in servo throttle mode
*/

// All the required settings are done in settings.h!
#include "settings.h" // <<------- SETTINGS

const float codeVersion = 1.1; // Software revision

// Stuff not to play with! ----------------------------------------------------------------------------
#define SPEAKER 3                               // This is kept as 3, original code had 11 as option, but this conflicts with SPI
volatile uint16_t currentSmpleRate = BASE_RATE; // Current playback rate, this is adjusted depending on engine RPM
boolean audioRunning = false;                   // Audio state, used so we can toggle the sound system
boolean engineOn = true;                        // Signal for engine on / off
uint16_t curVolume = 0;                         // Current digi pot volume, used for fade in/out
volatile uint16_t curEngineSample;              // Index of current loaded sample
uint8_t  lastSample;                            // Last loaded sample
int16_t  currentThrottle = 0;                   // 0 - 1000, a top value of 1023 is acceptable
//uint8_t  throttleByte = 0;                      // Raw throttle position in SPI mode, gets mapped to currentThrottle
//uint8_t  spiReturnByte = 0;                     // The current RPM mapped to a byte for SPI return
volatile int16_t pulseWidth = 0;                // Current pulse width when in PWM mode
volatile boolean pulseAvailable;                  // RC signal pulses are coming in
#define FREQ 16000000L                          // Always 16MHz, even if running on a 8MHz MCU!

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup()
{
  // setup complete, so start making sounds
  setupPcm();
}

//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//

void loop() {
  doSerialLineCommandThrottle();
}

//
// =======================================================================================================
// THROTTLES
// =======================================================================================================
//

// Serial line command ------------------------------------------------------------------------------
void doSerialLineCommandThrottle() {
  if (throttleByte > 0) {
      if (!audioRunning) setupPcm();
      currentThrottle = throttleByte << 2;
    }
    else if (audioRunning) stopPlayback();
  }
  else {
    if (throttleByte > 0) {
      if (!audioRunning) setupPcm();
      currentSmpleRate = FREQ / (BASE_RATE + long((throttleByte << 2) * TOP_SPEED_MULTIPLIER));
    }
    else if (audioRunning) stopPlayback();
  }
}

//
// =======================================================================================================
// MASS SIMULATION
// =======================================================================================================
//

void manageSpeed() {

  static int16_t prevThrottle = 0xFFFF;
  static int16_t currentRpm = 0;
  const  int16_t maxRpm = 8184; //8184
  const  int16_t minRpm = 0;

  static unsigned long throtMillis;
  static unsigned long startStopMillis;
  static unsigned long volMillis;

  // Engine RPM -------------------------------------------------------------------------------------
  if (millis() - throtMillis > 5) { // Every 5ms
    throtMillis = millis();

    if (currentThrottle + 18 > currentRpm) {
      currentRpm += 18;
      if (currentRpm > maxRpm) currentRpm = maxRpm;
      prevThrottle = currentThrottle;

    }
    else if (currentThrottle - 12 < currentRpm) {
      currentRpm -= 12;
      if (currentRpm < minRpm) currentRpm = minRpm;
      prevThrottle = currentThrottle;
    }

    //if (currentRpm >> 2 < 255) spiReturnByte = currentRpm >> 2;
    //else spiReturnByte = 255;
    //if (currentRpm >> 2 < 0) spiReturnByte = 0;

    currentSmpleRate = FREQ / (BASE_RATE + long(currentRpm * TOP_SPEED_MULTIPLIER) );
  }
}

// Write pot subfunction -----------------------------------------------------------------------------
//void writePot(uint8_t data) {
  // This function should get a value from 0 - 127
  // It would be trivial to convert this to work with
  // an I2C device.

  //if (data > VOL_MAX) data = VOL_MAX; // cap it just in case

  //digitalWrite(POT_CS, LOW);
  //shiftOut(POT_SDO, POT_SCK, MSBFIRST, 0x00);
  //shiftOut(POT_SDO, POT_SCK, MSBFIRST, data);
  //digitalWrite(POT_CS, HIGH);
//}

//
// =======================================================================================================
// PCM setup
// =======================================================================================================
//

void setupPcm()
{
  pinMode(SPEAKER, OUTPUT);
  audioRunning = true;

  // Set up Timer 2 to do pulse width modulation on the speaker pin.
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));                         // Use internal clock (datasheet p.160)

  TCCR2A |= _BV(WGM21) | _BV(WGM20);                        // Set fast PWM mode  (p.157)
  TCCR2B &= ~_BV(WGM22);

  TCCR2A = (TCCR2A | _BV(COM2B1)) & ~_BV(COM2B0);           // Do non-inverting PWM on pin OC2B (p.155)
  TCCR2A &= ~(_BV(COM2A1) | _BV(COM2A0));                   // On the Arduino this is pin 3.
  TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10); // No prescaler (p.158)

  OCR2B = pgm_read_byte(&idle_data[0]);                     // Set initial pulse width to the first sample.

  // Set up Timer 1 to send a sample every interrupt.
  cli();

  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);             // Set CTC mode (Clear Timer on Compare Match) (p.133)
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));             // Have to set OCR1A *after*, otherwise it gets reset to 0!

  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10); // No prescaler (p.134)

  OCR1A = FREQ / BASE_RATE;                                // Set the compare register (OCR1A).
  // OCR1A is a 16-bit register, so we have to do this with
  // interrupts disabled to be safe.

  TIMSK1 |= _BV(OCIE1A);                                   // Enable interrupt when TCNT1 == OCR1A (p.136)

  lastSample = pgm_read_byte(&idle_data[idle_length - 1]);
  curEngineSample = 0;
  sei();


  uint8_t target = map(currentThrottle, 0, 1023, VOL_MIN, VOL_MAX); // Fadein the volume pot
  for (uint8_t i = 0; i < target; i ++) {
    curVolume = i;
    //writePot(curVolume);
    delay(1);
  }
}

// ----------------------------------------------------------------------------------------------
void stopPlayback()
{
  // Fadeout the volume pot
  for (uint8_t i = curVolume; i > 0; i--) {
    curVolume = i;
    //writePot(i);
    delay(1);
  }

  audioRunning = false;

  TIMSK1 &= ~_BV(OCIE1A); // Disable playback per-sample interrupt.
  TCCR1B &= ~_BV(CS10);   // Disable the per-sample timer completely.
  TCCR2B &= ~_BV(CS10);   // Disable the PWM timer.

  digitalWrite(SPEAKER, LOW);
}

//
// =======================================================================================================
// NO RC SIGNAL PULSE?
// =======================================================================================================
//

void noPulse() {
  static unsigned long pulseDelayMillis;

  if (pulseAvailable) pulseDelayMillis = millis(); // reset delay timer, if pulses are available

  if (millis() - pulseDelayMillis > 100) {
    engineOn = false; // after 100ms delay, switch engine off
    curEngineSample = 0; // go to first sample
  }
  else engineOn = true;
}

//
// =======================================================================================================
// INTERRUPTS
// =======================================================================================================
//

// Uses a pin change interrupt and micros() to get the pulsewidth at pin 2 ---------------------------
void getPulsewidth() {
  unsigned long currentMicros = micros();
  boolean currentState = PIND & B00000100; // Pin 2 is PIND Bit 2 (=digitalRead(2) )
  static unsigned long prevMicros = 0;
  static boolean lastState = LOW;

  if (lastState == LOW && currentState == HIGH) {    // Rising edge
    prevMicros = currentMicros;
    pulseAvailable = true;
    lastState = currentState;
  }
  else if (lastState == HIGH && currentState == LOW) { // Falling edge
    pulseWidth = currentMicros - prevMicros;
    pulseAvailable = false;
    lastState = currentState;
  }
}

// This is the main playback interrupt, keep this nice and tight!! -----------------------------------
ISR(TIMER1_COMPA_vect) {
  OCR1A = currentSmpleRate;

  if (curEngineSample >= idle_length) { // Loop the sample
    curEngineSample = 0;
  }

  if (engineOn) {
    OCR2B = pgm_read_byte(&idle_data[curEngineSample]); // Volume
    curEngineSample++;
  }
  else OCR2B = 255; // Stop engine (volume = 0)
}

// Shawn Cruise 2016
// Please see wiring diagram in this repository

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#define redPin A0
#define yellowPin A1
#define greenPin A2
#define power2pin 3
#define power1pin 6
#define power0pin 7
#define b1pin 5
#define photoCellPin A5
#define gpsRxPin 9
#define gpsTxPin 8
#define GPSECHO  false
#define chComma ','

boolean sdOK = true;

SoftwareSerial mySerial(gpsTxPin, gpsRxPin); // tx,rx
Adafruit_GPS GPS(&mySerial);
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
uint32_t timer = millis();


void setup() {

  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(b1pin, INPUT);
  pinMode(power2pin, INPUT);
  pinMode(power1pin, INPUT);
  pinMode(power0pin, INPUT);

  digitalWrite(b1pin, HIGH);

  setRYG(true, true, true);
  delay(1000);
  setRYG(false, false, false);

  if (!SD.begin(4))
  {
    sdOK = false;
  }

  // drop into an infinite loop to signal the SD is not working.
  // user would power off, fix, and restart
  while (!sdOK)
  {
    setRYG(true, true, true);
    delay(1000);
    setRYG(false, false, false);
    delay(1000);
  }

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  delay(1000);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  }
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

// return a string with the digit 0 to 7 to represent the binary switch settings
String getFeatureType()
{
  short featureType = 0;

  if (digitalRead(power2pin) == HIGH)
  {
    featureType += 4;
  }

  if (digitalRead(power1pin) == HIGH)
  {
    featureType += 2;
  }

  if (digitalRead(power0pin) == HIGH)
  {
    featureType += 1;
  }

  return String(featureType);
}

// sets the red, yellow or green LED 
void setRYG(boolean R, boolean Y, boolean G)
{

  digitalWrite(redPin, LOW);
  digitalWrite(yellowPin, LOW);
  digitalWrite(greenPin, LOW);

  if (R)
  {
    digitalWrite(redPin, HIGH);
  }
  if (Y)
  {
    digitalWrite(yellowPin, HIGH);
  }
  if (G)
  {
    digitalWrite(greenPin, HIGH);
  }
}

// sets the LEDs based on the GPS state.  
// no fix is red
// fix with HDOP > 1.5 is yellow
// fix with HDOP <= 1.5 is green
void updateLEDs()
{

  if (!GPS.fix)
  {
    setRYG(true, false, false);
  }
  else
  {
    if (GPS.HDOP > 1.5)
    {
      setRYG( false, true, false);
    }
    else
    {
      setRYG(false, false, true);
    }
  }
}

// records the location, analog reading from the light seonsor, and the 0-7 type to a text file on the microSD card
// data is in CSV format
// green LED blinks 3x if successful write
// red LED blinks 3x if unsuccessful write
void recordLoc()
{

  setRYG(false, false, false);
  delay(500);
  File dataFile = SD.open("datalog.csv", FILE_WRITE);

  if (dataFile)
  {
    String dataString = "";

    dataString += String(GPS.latitudeDegrees, 5) + chComma;
    dataString += String(GPS.longitudeDegrees, 5) + chComma;
    dataString += String(analogRead(photoCellPin), DEC) + chComma;
    dataString += getFeatureType();

    dataFile.println(dataString);
    dataFile.close();

    for (int i = 0;  i < 3; i++)
    {
      setRYG(false, false, true);
      delay(500);
      setRYG(false, false, false);
      delay(500);
    }
  }
  else
  {
    for (int j = 0;  j < 3; j++)
    {
      setRYG(true, false, false);
      delay(500);
      setRYG(false, false, false);
      delay(500);
    }
  }
}

// is the button pressed
boolean b1Pressed()
{
  if (digitalRead(b1pin) == LOW)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// main loop
void loop() {

  if (GPS.fix)
  {
    if (b1Pressed())
    {
      recordLoc();
    }
  }

  if (GPS.newNMEAreceived()) {
    //if the GPS object can't parse it, return and wait for the next one
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  // if timer or mills wrap around, reset the timer
  if (timer > millis())  timer = millis();

  // update the display every second
  if (millis() - timer > 1000) {
    timer = millis(); // reset the timer
    updateLEDs();
  }
}


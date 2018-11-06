//-------------------------------------------------------------------------------
//  TinyCircuits GPS Tracker Tutorial Program
//  Last updated 27 February 2017 (1.02)
//  
//  Using the GPS TinyShield, the Flash Memory TinyShield, and the TinyDuino,
//  this program turns the stack into a miniature GPS tracker and data logger.
//  The code detects which sentence is being read and formats the string accordingly.
//  In order to reduce the number of writes, we write one NMEA sentence per 10 seconds, 
//  which can be modified.
//
//  With the Telit SE868 V2 module with Glonass support, some messages come through
//  as GN** sentences instead of GP**. These are changed back to GP** before logging
//  so that they don't cause problems with programs like Google Earth.
//  Some GPS modules have been shipped with 4800 baud instead of 9600- try this if
//  you see bad data.
//
//  The Software Serial library should be modified for a larger buffer- 256 is enough
//  for GGA and RMC sentences at 1Hz. In SoftwareSerial.cpp, the change looks like:
//  #define _SS_MAX_RX_BUFF 256
//
//  Written by Ben Rose & Lilith Freed for TinyCircuits, http://TinyCircuits.com
//
//-------------------------------------------------------------------------------

//This may need to be set to 4800 baud
const int GPSBaud = 9600;

#include "SoftwareSerial256.h"
#include <SPIFlash.h>
#include "okami.h"
#include <Wire.h>
#define Addr 0x1E               // 7-bit address of HMC5883 compass

// The chip/slave select pin is pin 5 for the Flash Memory TinyShield
const uint8_t flashCS = 5; 
unsigned long address = 0;



// The SPIFlash object for the chip. Passed the chip select pin in the constructor.
SPIFlash flash(flashCS); 

// The Arduino pins used by the GPS module
const uint8_t GPS_ONOFFPin = A3;
const uint8_t GPS_SYSONPin = A2;
const uint8_t GPS_RXPin = A1;
const uint8_t GPS_TXPin = A0;
const uint8_t chipSelect = 10;

// The GPS connection is attached with a software serial port
SoftwareSerial Gps_Serial(GPS_RXPin, GPS_TXPin);

// Set which sentences should be enabled on the GPS module
// GPGGA - 
char nmea[] = {'1'/*GPGGA*/, '0'/*GNGLL*/, '0'/*GNGSA*/, '0'/*GPGSV/GLGSV*/, '1'/*GNRMC*/, '0'/*GNVTG*/, '0'/*not supported*/, '0'/*GNGNS*/};

void setup()
{
  Gps_Serial.begin(GPSBaud);
  Serial.begin(115200);
  //Serial.begin(9600);
  delay(100);                   // Power up delay
  Wire.begin();
  
  // Set operating mode to continuous
  Wire.beginTransmission(Addr); 
  Wire.write(byte(0x02));
  Wire.write(byte(0x00));
  Wire.endTransmission();
  Serial.println("Initializing Flash Memory...");
  Serial.println();
  pinMode(flashCS, OUTPUT); // Ensure chip select pin is an output.
  flash.begin(); // Boots the flash memory

  Serial.println("Determining write/read start point...");
  uint8_t flashBuffer[256];
  uint8_t foundStart = false;
  while (!foundStart) {
    flash.readByteArray(address, flashBuffer, 256);
    int i;
    for (i = 0; !foundStart && i < 256; i++) {
      if (flashBuffer[i] == 0xFF) // checks for the first logical 1
        foundStart = true;
    }
    address += i;
  }
  address--;
  Serial.println("Done.");
  Serial.println();

  unsigned long timer = millis();
  Serial.println("Send 'y' to start read mode. Write mode will begin in 10 seconds...");
  Serial.println();
  while(millis() < timer + 10000) {
    if(Serial.available()) {
      if(Serial.read() == 'y') {
        readFlash(address);
      }
    }
  }
  
  Serial.println("Now initiating write mode.");
  Serial.println();
  
  // Init the GPS Module to wake mode
  pinMode(GPS_SYSONPin, INPUT);
  digitalWrite(GPS_ONOFFPin, LOW);
  pinMode(GPS_ONOFFPin, OUTPUT);
  delay(100);
  Serial.print("Attempting to wake GPS module.. ");
  while (digitalRead( GPS_SYSONPin ) == LOW )
  {
    // Need to wake the module
    digitalWrite( GPS_ONOFFPin, HIGH );
    delay(5);
    digitalWrite( GPS_ONOFFPin, LOW );
    delay(100);
  }
  Serial.println("done.");
  delay(100);
  
  char command[] = "$PSRF103,00,00,00,01*xx\r\n";
  for (int i = 0; i < 8; i++) {
    command[10] = i + '0';
    command[16] = nmea[i];
    int c = 1;
    byte checksum = command[c++];
    while (command[c] != '*')
      checksum ^= command[c++];
    command[c + 1] = (checksum >> 4) + (((checksum >> 4) < 10) ? '0' : ('A' - 10));
    command[c + 2] = (checksum & 0xF) + (((checksum & 0xF) < 10) ? '0' : ('A' - 10));
    Gps_Serial.print(command);
    delay(20);
  }
  
  Serial.println();

}

int x_max=-10000;  // Starting values for hard iron calibration
int y_max=-10000;  // We want these values to be extreme in the 
int x_min=10000;   // opposite direction so it calibrates nicely
int y_min=10000;


void loop() {
  unsigned long startTime = millis();
  while (Gps_Serial.read() != '$') {
    //do other stuff here
  }
  while (Gps_Serial.available() < 5);
  Gps_Serial.read(); 
  Gps_Serial.read(); //skip two characters
  char c = Gps_Serial.read();
  //determine senetence type
  if (c == 'R' || c == 'G') {
    c = Gps_Serial.read();
    if (c == 'M') {
      logNMEA(1);
    } else if (c == 'G') {
      logNMEA(2);
    }
  }
  
  // Waits 10 seconds before reading next NMEA string
  while (millis() - startTime < 10000) {
    Gps_Serial.read(); // clears GPS serial buffer
  }
}

void logNMEA(uint8_t type) {
  uint8_t buffer[82];
  // Initializes buffer to null terminators to ensure proper writing to flash memory
  for(int i = 0; i < 82; ++i) {
    buffer[i] = '\0';
  }

    // Writes NMEA string to buffer
  buffer[0] = '$';
  int counter = 1;
  char c = 0;
  while (!Gps_Serial.available());
  c = Gps_Serial.read();
  while (c != '*') {
    buffer[counter++] = c;
    while (!Gps_Serial.available());
    c = Gps_Serial.read();
  }
  buffer[counter++] = c;
  while (!Gps_Serial.available());
  c = Gps_Serial.read();
  buffer[counter++] = c;
  while (!Gps_Serial.available());
  c = Gps_Serial.read();
  buffer[counter++] = c;
  buffer[counter++] = '\r';
  buffer[counter++] = '\n';

  buffer[2] = 'P'; // Changes GNRMC to GPRMC

 
  c = 1;
  byte checksum = buffer[c++];
  while (buffer[c] != '*')
    checksum ^= buffer[c++];
  buffer[c + 1] = (checksum >> 4) + (((checksum >> 4) < 10) ? '0' : ('A' - 10));
  buffer[c + 2] = (checksum & 0xF) + (((checksum & 0xF) < 10) ? '0' : ('A' - 10));

  double    lati;
  double    lon;
  double    angle;
  lati = conv_coords(latitude((char*)buffer));
  lon = conv_coords(longitude((char*)buffer));
  angle = get_angle(to_radians(48.8464021), to_radians(2.332417), to_radians(lati), to_radians(lon));

  // Writes buffer array to flash memory. Write length is variable to maximize memory.
  flash.writeCharArray(address, (char *)buffer, strlen((char *)buffer));
  address += strlen((char *)buffer); // Sets address ahead for length of the nmea string

  int x, y, z;

  // Initiate communications with compass
  Wire.beginTransmission(Addr);
  Wire.write(byte(0x03));       // Send request to X MSB register
  Wire.endTransmission();

  Wire.requestFrom(Addr, 6);    // Request 6 bytes; 2 bytes per axis
  if(Wire.available() <=6) {    // If 6 bytes available
    x = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();
  }
  
  // Print raw values
  Serial.print("X=");
  Serial.print(x);
  Serial.print(", Y=");
  Serial.print(y);
  Serial.print(", Z=");
  Serial.print(z);
  
  if(x > x_max) //Find values of hard iron distortion
    x_max = x;  //This will store the max and min values
  if(y >y_max)  //of the magnetic field around you
    y_max = y;
  if(y<y_min)
    y_min = y;
  if(x<x_min)
    x_min = x;
  
  //Print max and min values
  Serial.print(", Xmax=");
  Serial.print(x_max);
  Serial.print(", Ymax=");
  Serial.print(y_max); 
  Serial.print(", Xmin=");
  Serial.print(x_min);
  Serial.print(", Ymin=");
  Serial.print(y_min);
  
  int xoffset= (x_max+x_min)/2;
  int yoffset= (y_max+y_min)/2;
  
  int x_scale = x-xoffset; // Math to compensate for hard 
  int y_scale = y-yoffset; // iron distortions
  
  // Heading in radians
  float heading = atan2(x_scale,y_scale); 
  
  //Heading between 0 and 6.3 radians
  if(heading < 0)
    heading += 2*PI;
    
  if(heading>2*PI)
    heading -= 2*PI;
  
  //Conversion to degrees  
  int Degrees = fmod(heading * 180/M_PI + angle, 360); 
  
  
  Serial.print("Heading (degrees): "); Serial.print(Degrees);
  
  int LED = Degrees/17; //Led shield has 21 Leds. Dividing 360 by
              //17 will give us values from 0 to 21
              
  if (LED==0)     //since there is no Led 0, we will turn 
    LED=21;       //Led 21 on instead
   
  LedOn(LED);
  Serial.print("LED: "); Serial.println(LED);
  
  delay(40);
}

void readFlash(unsigned long address) {
  char command;
  do {
    // Clear Serial write buffer to prepare for read
    while(Serial.available()) {
      Serial.read();
    }
    // Menu
    Serial.println("Read Mode | Please select a command (1, 2, 3):");
    Serial.println("1. Read to serial monitor.");
    Serial.println("2. Erase all data.");
    Serial.println("3. Exit read mode.");
    while(!Serial.available());
    command = Serial.read();
    // Command select
    switch(command) {
      case '1':
        // Handles empty flash
        if(address == 0) {
          Serial.println("No available data.");
          Serial.println();
        } else {
          // Reads all available data to the Serial monitor
          for(unsigned long i = 0; i < address; ++i) {
            Serial.print((char)flash.readChar(i));
          }
          Serial.println();
        }
        break;
      case '2':
        // Erases data up until the first available byte.
        eraseData(address);
        address = 0; // resets the first available byte address to 0
        Serial.println("All data erased.");
        Serial.println();
        break;
      case '3':
        // Exits read mode.
        return;
      default:
        // Passes by invalid input.
        Serial.println("That is not a recognized command.");
        Serial.println();
        break;  
    }
  } while(command != 3);
}

// Erases the flash memory in 4KB sectors.
// Minimizes excess erase "writes".
void eraseData(unsigned long address) {
  unsigned long index = 0;
  while(index < address) {
    flash.eraseSector(index);
    index += 4096;
  }
}

void LedOn(int ledNum)
{
  for(int i=4;i<10;i++)
  {
    pinMode(i, INPUT);
    digitalWrite(i, LOW);
  };
  if(ledNum<1 || ledNum>21) return;
  char highpin[21]={6,7,6,8,5,8,8,7,9,7,9,8,5,9,6,9,9,5,6,5,7};
  char lowpin[21]= {7,6,8,6,8,5,7,8,7,9,8,9,9,5,9,6,4,6,5,7,5};

  ledNum--;
  digitalWrite(highpin[ledNum],HIGH);
  digitalWrite(lowpin[ledNum],LOW);
  pinMode(highpin[ledNum],OUTPUT);
  pinMode(lowpin[ledNum],OUTPUT);
}
#include "okami.h"

double latitude(char *string)
{
  int i;
  int j;
  char  *lat;

  i = 0;
  while (string[i] && string[i] != ',')
    i++;
  i++;
  if (!(lat = (char*)malloc(sizeof(char) * 10)))
    return (0);
  j = 0;
  while (j < 9)
    lat[j++] = string[i++];
  lat[j] = '\0';
  return (atof(lat));
}

double  conv_coords(float in_coords)
{
  //Initialize the location.
  double f = in_coords;
  // Get the first two digits by turning f into an integer, then doing an integer divide by 100;
  // firsttowdigits should be 77 at this point.
  int firsttwodigits = ((int)f)/100; //This assumes that f < 10000.
  double nexttwodigits = f - (double)(firsttwodigits*100);
  double theFinalAnswer = (double)(firsttwodigits + nexttwodigits/60.0);
  return theFinalAnswer;
}

double  longitude(char *string)
{
  int i;
  int j;
  int vir;
  char  *lon;

  i = 0;
  vir = 0;
  while (string[i] && vir < 3)
  {
    if (string[i++] == ',')
      vir++;
  }
  if (!(lon = (char*)malloc(sizeof(char) * 11)))
    return (0);
  j = 0;
  while (j < 10)
    lon[j++] = string[i++];
  lon[j] = '\0';
  return (atof(lon));
}

double to_degrees(double radians) {
    return (radians * (180.0 / M_PI));
    }

double  to_radians(double degrees) 
{
  return (degrees * M_PI / 180.0);
}

double  get_angle(float lat1, float lon1, float lat2, float lon2)
{
  double  angle;
  double  dLon;
  double  y;
  double  x;

  dLon = (lon2 - lon1);
  y = sin(dLon) * cos(lat2);
  x = (cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2)) * cos(dLon);
  angle = atan2(y, x);
  angle = to_degrees(angle);
  angle = fmod((angle + 360), 360);
  angle = fabs(180 - angle);
  return (angle);
}


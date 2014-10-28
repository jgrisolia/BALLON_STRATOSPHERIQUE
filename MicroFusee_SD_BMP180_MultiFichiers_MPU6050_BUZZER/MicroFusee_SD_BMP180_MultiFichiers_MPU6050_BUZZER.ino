/* BMP085 Extended Example Code by: Jim Lindblom
  SparkFun Electronics date: 1/18/11
  updated: 2/26/13
  license: CC BY-SA v3.0 - http://creativecommons.org/licenses/by-sa/3.0/
  
  Get pressure and temperature from the BMP085 and calculate 
  altitude. Serial.print it out at 9600 baud to serial monitor.

  Update (7/19/11): I've heard folks may be encountering issues
  with this code, who're running an Arduino at 8MHz. If you're 
  using an Arduino Pro 3.3V/8MHz, or the like, you may need to 
  increase some of the delays in the bmp085ReadUP and bmp085ReadUT functions.

  Update J. Grisolia (2014)
  BMP180
  MPU-6050
  BUZZER
  
 * SD card attached to SPI bus as follows:
 ** CS - pin 10
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13

Pins SDA & SCL
BMP085 or BMP180 wiring 
** SDA - A4
** SCL - A5
** VCC - 5V
** GND - GND

Pins SDA & SCL
MPU-6050
** SDA - A4
** SCL - A5
** VCC - 5V
** GND - GND 
*/

#include <Wire.h>
#include <SD.h>

#define DEBUG
//#define DEBUG_BMP085
//#define DEBUG_MPU6050

/////////////////////////////////////////////////////////////////////////////////////////////////
//GENERAL
int tempomesure = 25; //temporisation de la boucle
//long SOS = 300000; //temp à partir duquel tu actives le buzzer pour lancer le SOS de récupération au cas où la fusée soit perdue dans le champ de maïs (durée du vol max 2mn) => 5mn suffise à attendre et lancer le SOS
long SOS = 300000; //temp à partir duquel tu actives le buzzer pour lancer le SOS de récupération au cas où la fusée soit perdue dans le champ de maïs (durée du vol max 2mn) => 5mn suffise à attendre et lancer le SOS
unsigned long t0=0;
unsigned long prevMillis=0;

/////////////////////////////////////////////////////////////////////////////////////////////////
//BUZZER
int speakerPin = 8;
int numTones = 10;
int tones[] = {261, 277, 294, 311, 330, 349, 370, 392, 415, 440};
          // mid C   C#    D   D#   E     F   F#   G    G#   A

/////////////////////////////////////////////////////////////////////////////////////////////////
//BMP085
#define BMP085_ADDRESS 0x77  // I2C address of BMP085, idem BMP1080
const unsigned char OSS = 0;  // Oversampling Setting
// Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 

short temperature;
long pressure;

// Use these for altitude conversions
const float p0 = 101325;     // Pressure at sea level (Pa)
float altitude;

/////////////////////////////////////////////////////////////////////////////////////////////////
//SD CARD
const int chipSelect = 10;

long fileNum = 0;  // maximum 99999
String fileName;
char name[13];
File dataFile;

void incFileNum() { // generate next file name:
  String s = "dat" + String(++fileNum) + ".txt";
  s.toCharArray(name,13);
}

void save(String s) {
  dataFile = SD.open(name, FILE_WRITE);
  if (dataFile) {
    dataFile.println(s);
    dataFile.close();
    Serial.println(s);
  }  
  else Serial.println("error opening " + String(name));
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//ACCELEROMETRE MPU-6050
const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;


/////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  pinMode(chipSelect, OUTPUT);
 
  // Open serial communications and wait for port to open:
#if defined(DEBUG) || defined(DEBUG_BMP085)
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
#endif
 //Efface le fichier initial 
 // SD.remove("DATA.txt");

//GESTION DES NOMS DES FICHIERS
  //--------------------------------------------------
  incFileNum(); // set it to datxxxx.txt
  while (SD.exists(name)) incFileNum();

#ifdef DEBUG
  Serial.println("new file name: " + String(name));
#endif 
  //--------------------------------------------------
  save("DATA MICRO FUSEES");
  save("t(ms)|Altitude(m)*100|Pression(Pa)|Temperature(C)*0.1|AcX|AcY|AcZ|GyX|GyY|GyZ");

/////////////////////////////////////////////////////////////////////////////////////////////////
//CAPTEUR bmp085
  Wire.begin();
  bmp085Calibration();
//initialisation du temps: 
unsigned long prevMillis = millis();

/////////////////////////////////////////////////////////////////////////////////////////////////
//ACCELEROMETRE MPU-6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void loop()
{
  
  
if ((millis() - prevMillis) > tempomesure) {

///////////////////////////////////////////////////////////////  
//  ACCELEROMETRE MPU-6050
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
//  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
#ifdef DEBUG_MPU6050
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(333);
#endif 

  Wire.endTransmission(true); 
  
  ///////////////////////////////////////////////////////////////  
//  Capteur pression et temperature
  temperature = bmp085GetTemperature(bmp085ReadUT());
  pressure = bmp085GetPressure(bmp085ReadUP());
  altitude = (float)44330 * (1 - pow(((float) pressure/p0), 0.190295));

#ifdef DEBUG_BMP085
  Serial.print("Temperature: ");
  Serial.print(temperature, DEC);
  Serial.println(" *0.1 deg C");
  Serial.print("Pressure: ");
  Serial.print(pressure, DEC);
  Serial.println(" Pa");
  Serial.print("Altitude: ");
  Serial.print(altitude, 2);
  Serial.println(" m");
  Serial.println();
#endif 
  //delay(tempomesure);
///////////////////////////////////////////////////////////////
  // make a string for assembling the data to log:
  
  String dataString = "";
  //OLD
  //t0 = t0 + (millis() - prevMillis);
  //NEW
  t0 = millis();
/* 
#ifdef DEBUG
  Serial.print("millis():");
  Serial.print(millis());
  Serial.print(",");
  Serial.print("t0:");
  Serial.print(t0);
  Serial.print(",");
#endif 
 */ 
  unsigned int alti = altitude*100;
  unsigned long pressu = pressure;
  unsigned int tempe = temperature;

  dataString = String(t0) + "|" + String(alti) + "|" + String(pressu) + "|" + String(tempe)+ "|" + String(AcX)+ "|" + String(AcY)+ "|" + String(AcZ)+ "|" + String(GyX)+ "|" + String(GyY)+ "|" + String(GyZ);   
//  dataString = String(t0) + "," + String(alti) + "," + String(pressu) + "," + String(tempe); 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  
  //File dataFile = SD.open("DATA.txt", FILE_WRITE);
  File dataFile = SD.open(name, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    #ifdef DEBUG
        // print to the serial port too:
      Serial.println(dataString);
    #endif    
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }//fin du if dataFile 
  prevMillis = millis();
}//fin du if

while(millis()>SOS) {

   for (int i = 0; i < numTones; i++)
    {
    tone(speakerPin, tones[i]);
    delay(2000);
    noTone(speakerPin);
    delay(4000);
    }
    Serial.println("Emission d'un SOS");
    noTone(speakerPin);
}//fin du while

}// fin de loop

//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;
  
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  
  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();
  
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}


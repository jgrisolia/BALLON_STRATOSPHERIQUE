/* ARDUINO IDE GEIGER SD LOGGER ver 1.01 produced by RH Electronics http://rhelectronics.net
You can purchase the hardware for this project on RH Electronics website. The DIY kit include high quality pcb
and electronics components to solder this project.

software version history:
* ver. 1.00 first written

* ver. 1.01 Adafruit GPS Module support. GPS data is not parsed! Use http://rl.se/gprmc or http://gpsvisualizer.com  to decode for the map
output format example for LOG.TXT is
12/20/2013,14:55:04,22 CPM,3.35 uSV,4940 mV
$GPRMC,125647.000,A,3147.1112,N,03512.8736,E,0.40,293.86,201213,,,A*61

* ver. 1.02 sram memory optimization done. 
c:\Users\...\geiger>avr-size geiger.cpp.elf
   text    data     bss     dec     hex filename
  29794     238    1161   31193    79d9 geiger.cpp.elf
  

This code was sucsessfuly compiled on Arduino IDE 1.05 30032 bytes 02 Feb. 2014 with GPS enabled
------------------------------------------------------------------------------------------------------------------------------
This is open source code written by Alex Boguslavsky. I have used some parts of BroHogan's source code for reference, but 
I have made my own electrical circuit, software and pcb re-design because I think using 555 timer and 74hc14 is excessive when it possible 
to produce high voltage with PWM or led flashing with another IO. Partly this code is implementation of my own ideas
and algorithms and partly just copy-paste. The project IS NOT PIN TO PIN COMPATIBLE with BroHogan!

The main idea was to create simple portable radiation logger based on open arduino IDE. You are free to modify and 
improve this code under GNU licence. Take note, you are responsible for any modifications. Please send me your suggestions
or bug reports at support@radiohobbystore.com
------------------------------------------------------------------------------------------------------------------------------
Main Board Features:

1. High voltage is programmed to 400V. Voltage stabilization IS NOT strict, but when battery is too low it will correct the
output. With the default preset high voltage can be trimmed up to 480V, I have cheked with 1G resistor divider. If you need 500V
or more I recommed to use external voltage multiplier x2 or x3. Additional multiplier pcb included.

2. My improved counting algorithm: moving average with cheking of rapid changes.

3. Smart backlight. If enabled, backlight pin can be used as alarm pin for external load. Counter support 16x2 lcd only.

4. Sieverts or Rouentgen units for displaing current dose. User selection saved into eeprom for next load, but can be changed
with key "down" any time.

5. UART logging with RH Electronics "Radiation Logger" http://www.rhelectronics.net/store/radiation-logger.html
Other similar logging software is compatible. Please also visit: http://radmon.org

6. Short LED flash for each nuclear event from tube. Active 5V Buzzer for sound and alarm. Clicker sound similar to classic
geiger counters.

7. Fast bargraph on lcd display. No sense for background radiation, but can be very useful for elevated values.

8. Two tact swith buttons. One to swith units, second to show the time.

9. DIP components.


------------------------------------------------------------------------------------------------------------------------------
SD Logging Shield Features:

1. MicroSD card compatible 2Gb-8Gb recommended.

2. RTC DS1307 used to hold local date and time. Currently in 24 hours format.

3. Writing to RADLOG.CSV every 10 seconds. The file is compatible with "Radiation Logger" and can be viewed on your PC later
through "Radiation Logger" graphs viewer. The file contains date, time and cpm.

4. Logging to LOG.TXT file every minute. The file contains date, time, cpm, total absorbed radiation uSv and battery voltage.

5. Saving total absorbed value to DOSE.TXT The value can be zeroed manually on your computer.

6. CR2032 3V backup battery compatibility.

7. Easy pin-in-header shield installation under the main board.

8. SMD components.

--------------------------------------------------------------------------------------------------------------------------------
Additional hardware required:

1. GM Tube

2. MicroSD cart

3. CR2032 battery

4. USB TTL Arduino Module

-------------------------------------------------------------------------------------------------------------------------------
Warranty Disclaimer:
This Geiger Counter kit is for EDUCATIONAL PURPOSES ONLY. 
You are responsible for your own safety during soldering or using this device!
Any high voltage injury caused by this counter is responsibility or the buyer only. 
DO NOT BUY IF YOU DON'T KNOW HOW TO CARRY HIGH VOLTAGE DEVICES! 
The designer and distributor of this kit is not responsible for radiation dose readings 
you can see, or not see, on the display. You are fully responsible for you safety and health
in high radiation area. The counter dose readings cannot be used for making any decisions. 
Software and hardware of the kit provided AS IS, without any warranty for precision radiation measurements. 
When you buy the kit it mean you are agree with the disclaimer!


Copyright (C) <2013>  <Alex Boguslavsky RH Electronics>

http://rhelectronics.net
support@radiohobbystore.com

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

//----------------- load parameters--------------------//

#define RAD_LOGGER    true          // enable serial CPM logging to computer for "Radiation Logger"
#define GPS           false          // enable GPS on Serial Port 
#define SMART_BL      true          // if true LCD backlight controlled through ALARM level and key, else backlight is always on 
#define ACTIVE_BUZZER true          // true if active 5V piezo buzzer used. set false if passive soldered. 

#if (GPS)                          // correct possible mistake in definition
#define RAD_LOGGER    false
#endif


// install supplied libraries from lib.zip!
#include <Arduino.h>              
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <SPI.h>

#include "Configurator.h"
#include <Wire.h>
#include <DS1307.h>
#include <SD.h>
File myFile;



//-------------------------------------------------------//

//-----------------Define global project variables-------------------------//

unsigned long CurrentBat;                        // voltage of the battery
unsigned long counts;                            // counts counter
unsigned int  countsHSecond;                     // half second counter
static unsigned long value[] = {0,0,0,0,0,0};           // array variable for cpm moving algorithm
static unsigned long cpm;                               // cpm variable
static unsigned long rapidCpm;                          // rapidly changed cpm
static unsigned long minuteCpm;                         // minute cpm value for absorbed radiation counter
static unsigned long previousValue;                     // previous cpm value

unsigned long previousMillis = 0;                // millis counters
unsigned long previousMillis_bg  = 0;
unsigned long previousMillis_pereferal = 0;

int n = 0;                                       // counter for moving average array
long result;                                     // voltage reading result
int buttonStateDo = 1;                           // buttons status
int buttonStateUp = 1;
unsigned int barCount;                           // bargraph variable
static float absorbedDose;                              // absorbed dose
//float dose;                                      // radiation dose
//float minuteDose;                                // minute absorbed dose
//boolean savedEeprom = false;                     // eeprom flag

char absLog[]     = "LOG00.TXT";                 // name for log file with absorbed data
char fastLog[]    = "LOG00.CSV";                 // name for csv log file for "Radiation Logger"
char doseFile[]   = "DOSE.TXT";                  // name for dose file, keep only total absorbed dose value 
                                                 // using config file is preferred over eeprom logging
                                                 // if the file is not exist the software will create one

boolean shield_status = true;                    // flag for SD cart error
boolean fileFlag      = true;                    // flag for SD file
boolean event         = false;                   // GM tube event received, lets make flag 
//boolean limit         = false;                   // absorbed dose limit flag
int HR, MIN, SEC, DATE, MTH, YR;                 // time variables from RTC


// some texts variables
char absorbedDoseSD[6];                         // absorbed dose written to SD

// Buffer to store incoming commands from serial port
#if (GPS)
String inData;
String nmea;
#endif

//-------------Roentgen/ Sieverts conversion factor----------------//
float factor_Rn = (FACTOR_USV * 100000) / 877;   // convert to roengten
float factor_Sv = FACTOR_USV;
float factor_Now = factor_Sv;                    // sieverts by defautl
int units;                                       // 0 - sieverts; 1 - roentgen
//-----------------------------------------------------------------//


//-------------------Initilize LCD---------------------//
LiquidCrystal lcd(9, 4, 8, 7, 6, 5);
//----------------------------------------------------//



///////////////////////////////////////////////////////////////////////////////////////////
//------------------------------------SETUP AREA-----------------------------------------//
///////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  // zero some important variables first
  counts = 0;
  countsHSecond = 0;
  minuteCpm = 0;
  n = 0;
  cpm = 0;
  
  #if (GPS)
  // long string variable need static buffer
  inData.reserve(100);
  nmea.reserve(100);
  #endif
  
  // configure atmega IO
  pinMode(LIGHT, OUTPUT);         // turn on backlight
  digitalWrite(LIGHT, HIGH);  
  pinMode(LED, OUTPUT);           // configure led pin as output
  digitalWrite(LED, LOW);
  pinMode(2, INPUT);              // set pin INT0 input for capturing GM Tube events
  digitalWrite(2, HIGH);          // turn on pullup resistors 
  pinMode(BUTTON_UP, INPUT);      // set buttons pins as input with internal pullup
  digitalWrite(BUTTON_UP, HIGH);
  pinMode(BUTTON_DO, INPUT);
  digitalWrite(BUTTON_DO, HIGH);  
   
 // start uart on 9600 baud rate 
  Serial.begin(9600);            
  delay(1000);
  
  // if GPS used send initilize commands
  #if (GPS)
  nmeaToGPS(); 
  #endif  


  // start I2C for RTC on A4 A5
  Wire.begin();
  delay(100);

  
  
 // start LCD display  
  lcd.begin(16, 2);
 // create and load custom characters to lcd memory  
  lcd.createChar(6, batmid);   // create battery indicator for mid state
  lcd.createChar(7, batlow);   // create battery indicator for low state
  lcd.createChar(0, bar_0);    // load 7 custom characters in the LCD
  lcd.createChar(1, bar_1);
  lcd.createChar(2, bar_2);
  lcd.createChar(3, bar_3);
  lcd.createChar(4, bar_4);
  lcd.createChar(5, bar_5);

 // extract starting messages from memory and print it on lcd
  lcd.setCursor(0, 0);
  lcdprint_P((const char *)pgm_read_word(&(string_table[0]))); // "  Arduino IDE   "
  lcd.setCursor(0, 1);
  lcdprint_P((const char *)pgm_read_word(&(string_table[1]))); // " Geiger Counter "
  blinkLed(3,50);                                              // say hello!
  delay(2000);

  
    // starting SD cart
    
    pinMode(10, OUTPUT);          // making this pin output as SD library say
    pinMode(A2, OUTPUT);          // SD cart chip select pin
    lcd.setCursor(0, 0);
    lcdprint_P((const char *)pgm_read_word(&(string_table[4])));   //"STARTING SD CARD"   
    
    if (!SD.begin(A2)) {
      #if (ACTIVE_BUZZER)
      blinkLed(5, 100);          // blink 5 times fast if SD cart is missing or defective
      #endif
    shield_status = false;       // do not log to SD until new cart replaced
    }
    delay(100);
  

    
    if (shield_status = true) {
      
      for (uint8_t i = 0; i < 100; i++) {
        absLog[3] = i/10 + '0';              
        absLog[4] = i%10 + '0';
        fastLog[3] = i/10 + '0';              
        fastLog[4] = i%10 + '0';  
        
        if (! SD.exists(absLog)) {
            // only open a new file if it doesn't exist
            myFile = SD.open(absLog, FILE_WRITE);
            fileFlag = true;   // new file was created
            // if the file opened okay, write to it:
            if (myFile) {
            myFile.close();
            
            readDoseSD();                  // read absorbed dose file from SD
                                           // if dose.txt file is missing the software will create a new one later
            
            shield_status = true;
            lcd.setCursor(0, 1);
            lcdprint_P((const char *)pgm_read_word(&(string_table[5]))); // "LOG TO"
            lcd.setCursor(7, 1);
            lcd.print(absLog);
            delay(2000);
            }
            else{
            shield_status = false;     // set error status for SD shield, will not perform SD logging if false
            lcd.setCursor(0, 1);
            lcdprint_P((const char *)pgm_read_word(&(string_table[3]))); // "SD FILE ERROR"
          }           
          break; // leave the loop!
        }        

        
      }       
          if (fileFlag = false){     // file error
            #if (ACTIVE_BUZZER)
            blinkLed(5,100);                 
            #endif                           
            
                     

          }
          




     }
      
  
     delay(1000);

    // Print current time on lcd
    lcd.clear();
    printTimeRTC();
    delay(2000);
    
    
  //---------------starting tube high voltage----------------------//
 
  setPwmFrequency(10, PWN_FREQ);    //set PWM frequency
  CurrentBat = readVcc();  
  if (CurrentBat >= FUL_BAT){
    analogWrite(10, PWM_FUL);      //correct tube voltage
  }
  
  if (CurrentBat <= LOW_BAT){
    analogWrite(10, PWM_LOW);      //correct tube voltage

}
  
  if (CurrentBat < FUL_BAT && CurrentBat > LOW_BAT){
    analogWrite(10, PWM_MID);      //correct tube voltage
  } 
  delay(2000);  
//------------------------------------------------------------------//

//---------------------read EEPROM---------------------------------//

  units = EEPROM.read(UNITS_ADDR);
  
 
  lcd.clear();
  lcd.setCursor(0,0);
  lcdprint_P((const char *)pgm_read_word(&(string_table[2])));   //"TOTAL DOSE   uSv"
  lcd.setCursor(0,1);
  lcd.print(absorbedDose);
  delay(3000);
//------------------------------------------------------------------//

// prepare lcd
  lcd.clear();
  lcd.setCursor(10,0);
  lcd.print(absorbedDose);
  
//---------------------------------------------------------------//    


//---------------------Allow external interrupts on INT0---------//
  attachInterrupt(0, tube_impulse, FALLING);
  interrupts();
//---------------------------------------------------------------//

}

///////////////////////////////////////////////////////////////////////////////////////
//-----------------------------MAIN PROGRAM CYCLE IS HERE----------------------------//
///////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  unsigned long currentMillis = millis();
  #if (GPS) 
  ReadSerial();
  #endif

//--------------------------------makes beep and led---------------------------------//
   if (event == true){                    // make led and beep for every tube event
   tone(LED, 4000, LED_TIME);             // http://letsmakerobots.com/node/28278 
   event = false;
   }
//-----------------------------------------------------------------------------------//
//--------------------------What to do if buttons pressed----------------------------//
   // swith dose units
      if (readButtonDo()== LOW){
      units = 1 - units;
      digitalWrite(LIGHT, HIGH);
      if (units == 0){
      lcd.setCursor(0, 0); 
      lcdprint_P((const char *)pgm_read_word(&(string_table[6]))); // "Sieverts"
      //lcd.print(F("Sieverts  "));

       }
       else
       {
        lcd.setCursor(0, 0);
        lcdprint_P((const char *)pgm_read_word(&(string_table[7]))); // "Roentgen"
        //lcd.print(F("Roentgen  "));

    } 
    EEPROM.write (UNITS_ADDR, units); //save units to eeprom
    }

    
    if (readButtonUp()== LOW){
    digitalWrite(LIGHT, HIGH);
    printHours();
    }

//------------------------------------------------------------------------------------//



//---------------------------What to do every 500ms is--------------------------------//
  if(currentMillis - previousMillis_bg > 500){      
   previousMillis_bg = currentMillis;
   
     #if (SMART_BL)                           // check backlight alarm status
     if (countsHSecond > (ALARM / 60)){       // actually we need ALARM / 120 because it half second measure
       digitalWrite(LIGHT, HIGH);             // but I like how it works with ALARM / 60, make your choice 
      
      // another way  directly control of the bl led, keep commented!
      // PORTD |= _BV(3);        // turn on bl led, arduino IDE is very slow, we need direct approach to the port
     }
     #endif
   
   barCount = countsHSecond * 120;            // calculate and draw fast bargraph here   
   lcd.setCursor(10,1);
   lcdBar(barCount);
   countsHSecond = 0;
   
 
 }
//------------------------------------------------------------------------------------//


//-------------------------What to do every 10 seconds-------------------------------------//
 if(currentMillis - previousMillis > 10000) {      // calculating cpm value every 10 seconds
    previousMillis = currentMillis;
    
      lcd.setCursor(0, 0);  
      lcd.write("\xe4");                          // print Mu , use "u" if have problem with lcd symbols table
      lcd.setCursor(0, 1);
     // lcd.print(F("CPM:"));
      lcdprint_P((const char *)pgm_read_word(&(string_table[8])));  // "CPM:"
     value[n] = counts;
     previousValue = cpm;
     
     cpm = value[0] + value[1] + value[2] + value[3] +value[4] + value[5];

     if (n == 5)
     { 
       n = 0;
     }
     else
     {
       n++;
     }
     

     
//-------check if cpm level changes rapidly and make new recalculation-------//     
     if (previousValue > cpm){        
        rapidCpm = previousValue - cpm;
         if (rapidCpm >= 50){
           cpm = counts * 6;
         }
     }
     
         if (previousValue < cpm){
        rapidCpm = cpm - previousValue;
         if (rapidCpm >= 50){
           cpm = counts * 6;
         }
     }


// calculate and print radiation dose on lcd    
       clearLcd(4, 0, 6);
   //  lcd.setCursor(4, 0);
   //  lcd.print("      ");
     
     if (units == 0){
       factor_Now = factor_Sv;
       lcd.setCursor(1, 0);
       lcd.print(F("Sv:")); 
     }
     else{
       factor_Now = factor_Rn;
       lcd.setCursor(1, 0);
       lcd.print(F("Rn:")); 
     }

     float dose = cpm * factor_Now;
    
     if (dose >= 99.98){           // check if dose value are bigger than 99.99 to make the value printable on 5 lcd characters
      int roundDose;
      roundDose = (int) dose;
      lcd.setCursor(4, 0);
      lcd.print(roundDose);
     }
     else
     {

       pFloat(dose);
     }

// print cpm on lcd  
     clearLcd(4, 1, 6);
    // lcd.setCursor(4, 1);            
     //lcd.print("      ");
     lcd.setCursor(4, 1);
     lcd.print(cpm);
     
// turn on/off backlight alarm if cpm > < ALARM
     #if (SMART_BL)
     if (cpm> (ALARM / 3)){
       digitalWrite(LIGHT, HIGH);
       #if (ACTIVE_BUZZER)
       blinkLed(3,50);              // make additional alarm beeps
       #endif
     }
     else{
       digitalWrite(LIGHT, LOW);  
     }
     #endif

// make fast SD logging 
// this file can be opened by Radiation Logger later on your computer to see the graph
// format is "MM/DD/YYYY,HH:MM:SS,CPM"

     
     if (shield_status = true) {
        myFile = SD.open(fastLog, FILE_WRITE);
      // if the file opened okay, write to it:
      if (myFile) {
      printRadLog();
      shield_status = true;
      }
      else{
      shield_status = false;     // set error status for SD shield, will not perform SD logging if false
      
      } 
     }
     

// send cpm value to Radiation Logger http://radiohobbystore.com/radiation-logger/

     #if (RAD_LOGGER)
        Serial.print(cpm);  // send cpm data to Radiation Logger 
        Serial.write(' ');  // send null character to separate next data
     #endif  
     counts = 0;            // clear variable for next turn
}
//--------------------------------------------------------------------------------------------//

//------------------------What to do every minute---------------------------------------------//
 if(currentMillis - previousMillis_pereferal > 60000){      // check battery level every minute
   previousMillis_pereferal = currentMillis;
  
  float minuteDose = minuteCpm * factor_Sv;       // count radiation dose during last minute
  minuteDose = minuteDose / 60;                   // convert uSv/h to uSv units 
  absorbedDose += minuteDose;
//  if (absorbedDose > DOSE_LIMIT){
//     limit = true;
//  }
      
     if (shield_status = true) {
        myFile = SD.open(absLog, FILE_WRITE);
      // if the file opened okay, write to it:
      if (myFile) {
      printLog();
      shield_status = true;
      }
      else{
      shield_status = false;     // set error status for SD shield, will not perform SD logging if false
      } 
      
      SD.remove(doseFile);                     // delete old dose file with total absorbed dose
      myFile = SD.open(doseFile, FILE_WRITE);  // create new dose file with total absorbed dose
      myFile.print(absorbedDose);      
      myFile.close();      
     }
    
       clearLcd(10, 0, 6);
       lcd.setCursor(10,0);
       pFloat(absorbedDose);

 
      CurrentBat = readVcc();

      if (CurrentBat >= FUL_BAT){
        analogWrite(10, PWM_FUL);    //correct tube voltage if battery ok
        lcd.setCursor(15,0);
        lcd.write(' ');
      }
      
      if (CurrentBat <= LOW_BAT){
        analogWrite(10, PWM_LOW);    //correct tube voltage if battery low
        lcd.setCursor(15,0);
        lcd.write(7);    
      }
      
      if (CurrentBat < FUL_BAT && CurrentBat > LOW_BAT){
        analogWrite(10, PWM_MID);    //correct tube voltage if battery on the middle
        lcd.setCursor(15,0);
        lcd.write(6);    
      }        
             
       minuteCpm = 0;                         // reset minute cpm counter
 }  
//------------------------------------------------------------------------------------------//

//--------------------------------What to do every hour-------------------------------------//

 
//-----------------------------------------------------------------------------------------//

}

///////////////////////////////////////////////////////////////////////////////////////
//-----------------------------MAIN PROGRAM ENDS HERE--------------------------------//
///////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////
//-----------------------------SUB PROCEDURES ARE HERE-------------------------------//
///////////////////////////////////////////////////////////////////////////////////////  


//----------------------------------GM TUBE EVENTS ----------------------------------//
void tube_impulse ()                // interrupt care should short and fast as possible
{ 
  counts++;                         // increase 10 seconds cpm counter
  countsHSecond++;                  // increase 1/2 second cpm counter for bargraph
  minuteCpm++;                      // incerase 60 seconds cpm counter
  event = true;                     // make event flag
  
 // another way to make event flag by directly control of the led, keep commented!
 // PORTC |= _BV(3);                // turn on led, arduino IDE is very slow, we need direct approach to the port

}
//----------------------------------------------------------------------------------//



//------------SECRET ARDUINO PWM----------------//
//  http://playground.arduino.cc/Code/PwmFrequency
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
//------------SECRET ARDUINO PWM----------------//




//------------SECRET ARDUINO VOLTMETER----------------//
//  http://www.instructables.com/id/Secret-Arduino-Voltmeter/
unsigned long readVcc() { 
  unsigned long voltage;
  // READ 1.1V INTERNAL REFERENCE VOLTAGE
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); 
  ADCSRA |= _BV(ADSC); 
  while (bit_is_set(ADCSRA,ADSC));
  voltage = ADCL;
  voltage |= ADCH<<8;
  voltage = 1126400L / voltage; 
  return voltage;
}
//------------SECRET ARDUINO VOLTMETER----------------// 


//------------------read upper button with debounce-----------------//
int readButtonUp() {                         // reads upper button
  buttonStateUp = digitalRead(BUTTON_UP);
  if (buttonStateUp == 1){
    return HIGH;
  }
  else
  {
   delay(100);
    buttonStateUp = digitalRead(BUTTON_UP);
    if (buttonStateUp == 1){
      return HIGH;
    }
    else
    {
      return LOW;
    }
}
 
}
//------------------------------------------------------------------//

//------------------read bottom button with debounce-----------------//
int readButtonDo() {                         // reads bottom button
  buttonStateDo = digitalRead(BUTTON_DO);
  if (buttonStateDo == 1){
    return HIGH;
  }
  else
  {
   delay(100);
    buttonStateDo = digitalRead(BUTTON_DO);
    if (buttonStateDo == 1){
      return HIGH;
    }
    else
    {
      return LOW;
    }
}
}

//------------------------------------------------------------------//



//----------------------------------CPM bargraph-------------------------------------//
// made by DeFex     http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1264215873/0
// copied from BroHogan   https://sites.google.com/site/diygeigercounter/home
// adapted to show my custom bargraph scale symbols

void lcdBar(int c){                         // displays CPM as bargraph

  unsigned int scaler = BARGRAPH / 35;        
  unsigned int cntPerBar = (c / scaler);      // amount of bars needed to display the count
  unsigned int fullBlock = (cntPerBar / 6);   // divide for full "blocks" of 6 bars 
  unsigned int prtlBlock = (cntPerBar % 6 );  // calc the remainder of bars
  if (fullBlock >6){                          // safety to prevent writing to 7 blocks
    fullBlock = 6;
    prtlBlock = 0;
  }
  for (unsigned int i=0; i<fullBlock; i++){
    lcd.write(5);                          // print full blocks
  }
  lcd.write(prtlBlock>6 ? 5 : prtlBlock); // print remaining bars with custom char
  for (int i=(fullBlock + 1); i<8; i++){
    lcd.write(byte(0));                     // blank spaces to clean up leftover
  }  
}


//-----------------------------------------------------------------------------------//
////--------------------------EEPROM read write float----------------------------------//
////http://www.elcojacobs.com/storing-settings-between-restarts-in-eeprom-on-arduino/  //
//
//float eepromReadFloat(int address){
//   union u_tag {
//     byte b[4];
//     float fval;
//   } u;   
//   u.b[0] = EEPROM.read(address);
//   u.b[1] = EEPROM.read(address+1);
//   u.b[2] = EEPROM.read(address+2);
//   u.b[3] = EEPROM.read(address+3);
//   return u.fval;
//}
// 
//void eepromWriteFloat(int address, float value){
//   union u_tag {
//     byte b[4];
//     float fval;
//   } u;
//   u.fval=value;
// 
//   EEPROM.write(address  , u.b[0]);
//   EEPROM.write(address+1, u.b[1]);
//   EEPROM.write(address+2, u.b[2]);
//   EEPROM.write(address+3, u.b[3]);
//}
////-----------------------------------------------------------------------------------//

//-----------------------------print from progmem-----------------------------------//
// copied from BroHogan   https://sites.google.com/site/diygeigercounter/home

void lcdprint_P(const char *text) {  // print a string from progmem to the LCD
  /* Usage: lcdprint_P(pstring) or lcdprint_P(pstring_table[5].  If the string 
   table is stored in progmem and the index is a variable, the syntax is
   lcdprint_P((const char *)pgm_read_word(&(pstring_table[index])))*/
  while (pgm_read_byte(text) != 0x00)
    lcd.write(pgm_read_byte(text++));
}


//----------------------------------------------------------------------------------//


//--------------------------------RTC procedures------------------------------------//
// http://arduino.ru/forum/programmirovanie/ds1307-pokazyvayut-strannoe-vremya-kotoroe-stoit#comment-22002

void printTimeRTC(){                       // print time and date on lcd

      getFromRTC();
      lcd.setCursor(0, 0);
      if (MTH < 10){
        drawZerotoLcd();
      }
      lcd.print(MTH);
      lcd.write('/');
      if (DATE < 10){
        drawZerotoLcd();
      }      
      lcd.print(DATE);
      lcd.write('/'); 
      lcd.print(YR);
      lcd.setCursor(0, 1);
      if (HR < 10){
        drawZerotoLcd();
      }      
      lcd.print(HR);
      lcd.write(':'); 
      if (MIN < 10){
        drawZerotoLcd();
      }      
      lcd.print(MIN);
      lcd.write(':'); 
      if (SEC < 10){
        drawZerotoLcd();
      }      
      lcd.print(SEC);      

}

static void getFromRTC(){                         // get time and date from RTC
      HR  =  RTC.get(DS1307_HR,true);
      MIN =  RTC.get(DS1307_MIN,false);
      SEC =  RTC.get(DS1307_SEC,false);
      DATE=  RTC.get(DS1307_DATE,false);
      MTH =  RTC.get(DS1307_MTH,false);
      YR  =  RTC.get(DS1307_YR,false);
      }

void printHours(){                               // show hours when button pressed
     getFromRTC();
     lcd.setCursor(10,0);
       if (HR < 10){
        drawZerotoLcd();
      }      
      lcd.print(HR);
      lcd.write(':'); 
      if (MIN < 10){
        drawZerotoLcd();
      }      
      lcd.print(MIN);
      lcd.write(' ');
}

static void printTime(){
      if (MTH < 10){
        myFile.print("0");
      }
      myFile.print(MTH);
      myFile.print("/");
      if (DATE < 10){
        myFile.print("0");
      }      
      myFile.print(DATE);
      myFile.print("/"); 
      myFile.print(YR);
      myFile.print(",");
      if (HR < 10){
        myFile.print("0");
      }      
      myFile.print(HR);
      myFile.print(":"); 
      if (MIN < 10){
        myFile.print("0");
      }      
      myFile.print(MIN);
      myFile.print(":"); 
      if (SEC < 10){
        myFile.print("0");
      }      
      myFile.print(SEC);
      myFile.print(",");     
}  
//-----------------------------------------------------------------------------------//

//-----------------------------RADLOG.CSV PRINT--------------------------------------//
// print to csv file on SD cart, fast cpm logging
// this file can be opened by Radiation Logger later on your computer to see the graph
// format is "MM/DD/YYYY,HH:MM:SS,CPM"

 void printRadLog(){

      getFromRTC();
      printTime();
      myFile.println(cpm);
      myFile.close();
      } 
      
//------------------------------LOG00.TXT PRINT----------------------------------------//
// print to log.txt file on SD cart 
// format is "MM/DD/YYYY,HH:MM:SS,minuteCpm CPM,absorbedDose uSV,CurrentBat mV"
// the file contain date, time, cpm during last minute, total absorbed uSv, battery voltage mV

static void printLog(){

      getFromRTC();
      printTime();
      myFile.print(minuteCpm);
      myFile.print(" CPM,");
      myFile.print(absorbedDose);
      myFile.print(" uSV,");
      myFile.print(CurrentBat);
      myFile.println(" mV");
      #if (GPS)
      myFile.println(nmea); 
      nmea = "";
      #endif
      myFile.close();
      } 


//-------------------------------Read DOSE.TXT---------------------------------------//
// read absorbed dose from SD card

static void readDoseSD(){

  if (SD.exists(doseFile)) {            // check and read absorbed dose from dose file 
        myFile = SD.open(doseFile);
        int sentanceLenght = 0;
        for (uint32_t sentanceLenght = 0; sentanceLenght < myFile.size(); sentanceLenght++){
          myFile.seek(sentanceLenght);
          absorbedDoseSD[sentanceLenght] = myFile.read();
        } 
        myFile.close();
        absorbedDose = atof(absorbedDoseSD); 
  }
        else {                          // if file is not exist then create a new one  
        myFile = SD.open(doseFile, FILE_WRITE); 
        myFile.print("0.00");
        myFile.close(); 
        absorbedDose = 0.00;
        }  
        
}

//-----------------------------------------------------------------------------------//

//------------------------------------LED BLINK--------------------------------------//

void blinkLed(int i, int time){                        // make beeps and blink signals
    int ii;                                            // blink counter
    for (ii = 0; ii < i; ii++){
      
    digitalWrite(LED, HIGH);   // 
    delay(time);
    digitalWrite(LED, LOW);
    delay(time);
    }
}


//--------------------------------Print zero on lcd----------------------------------//

void drawZerotoLcd(){                               // print zero symbol when need it
  lcd.write('0');
}


//--------------------------------Print float on lcd---------------------------------//
// 

static void pFloat(float dd){                       // avoid printing float on lcd
     lcd.print(int(dd));                            // convert it to text before 
     lcd.write('.');                                // sending to lcd
      if ((dd-int(dd))<0.10) {
       drawZerotoLcd(); 
       lcd.print(int(100*(dd-int(dd))));
      }
      else{
       lcd.print(int(100*(dd-int(dd))));      
     }
}
//--------------------------------clear lcd zone-------------------------------------//
void clearLcd(byte x, byte y, byte zone){
  int ii;
  lcd.setCursor (x,y);
  for (ii = 0; ii < zone; ii++){
    lcd.write(' ');
  }
}
  
//--------------------------------GPS------------------------------------------------//
// read serial http://stackoverflow.com/questions/5697047/convert-serial-read-into-a-useable-string-using-arduino
// function valid only for Adafruit GPS module!!
// will read bytes and store it to inData string until new line symbol received
// GPS data is not parsed! Use http://rl.se/gprmc or http://www.gpsvisualizer.com/ to decode LOG00.TXT for the map

#if (GPS)
void ReadSerial(){
    while (Serial.available() > 0)
    {
        char recieved = Serial.read();
        inData += recieved; 

        // Process message when new line character is recieved
        if (recieved == '\n')
        {
            nmea = inData;
            inData = ""; // Clear recieved buffer
        }
    }
}


// starting Adafruit GPS for recommended minimum output
static void nmeaToGPS() {
  Serial.println(F("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"));   //recommended minimum only
  delay(250);
  Serial.println(F("$PMTK220,1000*1F"));                                    // 1Hz update time
}
#endif
///////////////////////////////////////////////////////////////////////////////////////
//-----------------------------SUB PROCEDURES ENDS HERE------------------------------//
///////////////////////////////////////////////////////////////////////////////////////     
  



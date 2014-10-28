#include <DS1307.h>

// this code will setup SD Shield RTC!
// upload it before you'll upload the main geiger.ino

//#include <LiquidCrystal.h>
#include <Wire.h>

//#include <DS1307.h>
#include <EEPROM.h>
#define UNITS_ADDR    0     // int 2 bytes

void setup(){
  //-------------------Initilize LCD---------------------//
 // LiquidCrystal lcd(9, 4, 8, 7, 6, 5);
  //----------------------------------------------------//

  int units = 0;                    // set default Sieverts units
  EEPROM.write (UNITS_ADDR, units); // save units to eeprom
  
  // start I2C for RTC on A4 A5
  Wire.begin();
  delay(100);


  
//--------------------Adjusting RTC time and date----------------------------------------//
  //  NEED TO UPLOAD THIS CODE JUST ONCE!
  // Be sure 3V battery is connected.
  // Edit values and then upload the scetch. 
  // Do not remove the battery! Otherwise you'll need to adjust again.
//--------------------------------------------------------------------------------------//    

   RTC.stop();
   RTC.set(DS1307_SEC,0);         //set the seconds
   RTC.set(DS1307_MIN,26);        //set the minutes
   RTC.set(DS1307_HR,22);         //set the hours
   RTC.set(DS1307_DOW,4);         //set the day of the week
   RTC.set(DS1307_DATE,11);       //set the date
   RTC.set(DS1307_MTH,9);        //set the month
   RTC.set(DS1307_YR,14);         //set the year
   RTC.start();

}  

void loop(){
  
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * ** * * * * * * * * 
*                                                                                                         *  
*                 P o o l   T e c h   C l i m a t e  C o n t r o l for Arduino                            *
*                                 a dew point vantilation system                                          *
*                                                                                                         *
*                                             A         K         K                                       *
*                                           A A    I    K       K                                         *
*                                         A   A         K     K                                           *
*                                       A     A    I    K   K K                                           *
*                                     A A A A A    I    K K     K                                         *
*                                   A         A    I    K         K                                       *
*                                                                                                         * 
*   This code bases on https://github.com/MakeMagazinDE/Taupunktluefter                                   *
*                                                                                                         *     
*   I mainly adapted it for my purpose (climate control for my pool tech shaft) Jun 2022.                 *
*   This repository can be found here https://github.com/AiKatHome/PoolTechClimateControl                 *
*                                                                                                         *            
*   + added a pair of sensors (now 2 inside and 2 outside) to detect defect sensors and false measures    *
*   + store 3 measures to average the results (again to compensate false measures)                        *
*   + there is a push button to switch the display backlight on/off                                       *
*                                                                                                         *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define SW_version "Version: 2.99 dev"

// This code needs the following libraries
#include <DHT.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>
#include <DS1307RTC.h>
#include <SD.h>
#include <SPI.h>

#define PINFAN      6  // Pin for the fan relay
#define PINDHT_SI1  5  // Data pin for DHT sensor 1 inside
#define PINDHT_SI2  4  // Data pin for DHT sensor 2 inside
#define PINDHT_SO1  7  // Data pin for DHT sensor 1 outside
#define PINDHT_SO2  8  // Data pin for DHT sensor 2 outside
#define PINERROR    9  // Pin for error LED
#define PINDISPLAY  3  // Pin to active the LCD display

#define FAN_ON LOW     // ouput to switch fan on
#define FAN_OFF HIGH   // ouput to switch fan off
bool fan;              // current fan state
bool error = true;     // error state
bool display = true;   // display on/off 

#define DHTTYPE_SI1 DHT22 // DHT 22 sensor 1 inside 
#define DHTTYPE_SI2 DHT22 // DHT 22 sensor 2 inside
#define DHTTYPE_SO1 DHT22 // DHT 22 sensor 1 outside
#define DHTTYPE_SO2 DHT22 // DHT 22 sensor 2 outside


// *******  correction values for DHT sensors  *******
float sensor_corr[4][2] =
{
  {-0.1,1.3},         // SI1 correction (needs to be added to the data): temp, humidity
  {-0.4,2.1},         // SI2 correction (needs to be added to the data): temp, humidity
  {-0.4,1.7},         // SO1 correction (needs to be added to the data): temp, humidity
  {-0.3,0.1}          // SO2 correction (needs to be added to the data): temp, humidity
};

//******* variables to handle temp and humidity measures *******
float t[4][3];              // Array to store for all 4 sensors the last 3 temperature measures
float h[4][3];              // Array to store for all 4 sensors the last 3 humidity measures
int measure_index = 0;      // measure array index: 0-2 used to access the array
int sensor_index = 0;       // sensor array index: 0-3 used to access the array
int pre_measure_index = 0;  // previous measure index: 0-2 used to access the array

#define DEW_POINT_MIN     8.0   // minimum dew point delta to switch on the fan
#define HYSTERESIS        2.0   // dew point delta between an switch on/off of the fan
#define TEMP_MIN_INSIDE   5.0   // minimum temperature inside, where the fan can be activated
#define TEMP_MIN_OUTSIDE  -5.0  // minimum temperature outside, where the fan can be activated

DHT dhtsi1(PINDHT_SI1, DHTTYPE_SI1);  //inside sendor 1 is now addressed via "dhtsi1"
DHT dhtsi2(PINDHT_SI2, DHTTYPE_SI2);  //inside sendor 2 is now addressed via "dhtsi2"
DHT dhtso1(PINDHT_SO1, DHTTYPE_SO1);  //outside sendor 1 is now addressed via "dhtso1"
DHT dhtso2(PINDHT_SO2, DHTTYPE_SO2);  //outside sendor 2 is now addressed via "dhtso2"

LiquidCrystal_I2C lcd(0x27,20,4); // LCD: setting the I2C address and display size
tmElements_t tm;                  // tm is the insance of the real time clock DS1307RTC

//******* variables for data logging on SD card *******
#define logHeaderLine F("Date|Time;Temp_SI1;Humidity_SI1;DrewPoint_SI1;Temp_SI2;Humidity_SI2;DrewPoint_SI2;Temp_SO1;Humidity_SO1;DrewPoint_SO;Temp_SO2;Humidity_SO2;DrewPoint_SO2;Fan_On/Off;Fan_Runtime;")
#define logFileName F("PTCC_Log.csv")   // File name to store the log data (file format: 8.3)!!!!
#define LogInterval 15                  // Interval in min to store the measures ( 5 = every 5 min)

bool logging = true;                    // Should data be logged on SD card?
String LogData = "" ;                   // Variable build the logging string
char stamp[17];                         // Variable fot the time stamp
unsigned int FanStart = 0;              // When was the fan started?
unsigned int FanRuntime = 0;            // How long was the fan running in min?
char StrFanRuntime[6];                  // Fan runtime as a string
uint8_t Today = 0;                      // Today's date; needed to detect the next day
bool DaySwitched=false;


void setup() 
{
  wdt_enable(WDTO_8S); // set watchdog timer to 8 seconds
  Serial.begin(9600);  // serial print, which can be used for debugging or if no LCD used
  lcd.init();
  lcd.backlight();   
  
  //******* data logging on SD card *******
  if (logging == true)
  { 
    lcd.setCursor(0,1);
    lcd.print(SW_version);        // display software version
    Serial.println(SW_version);
    RTC_start();                  // RTC modul test. If there is an error => no logging
    
    delay (4000);                 // Delay to read the display
    lcd.clear(); 
    wdt_reset();                  // Watchdog reset
    test_SD();                    // SD card test. If there is an error => no logging
    Today = tm.Day ;
    //******* record restart *******
    if (logging == true)          // could have changed, due to errors on RTC or SD
    {   
      make_time_stamp();   
      File logFile = SD.open(logFileName, FILE_WRITE);
      logFile.print(stamp);
      logFile.println(F(": Restart"));   // Needed to log how often it was restarted (manual, power on/off/ or by the watchdog)
      logFile.close();  
    }
  } 
    
  pinMode(PINFAN, OUTPUT);              // Define fan pin as output
  digitalWrite(PINFAN, FAN_OFF);        // turn fan off
  pinMode(PINERROR, OUTPUT);            // Define error pin as output
  digitalWrite(PINERROR, HIGH);         // turn on error led
  pinMode(PINDISPLAY, INPUT_PULLUP);    // Define display pin as input
     
  Serial.println(F("Testing sensors.."));
  lcd.clear();                  
  lcd.setCursor(0,0);
  lcd.print(F("Test "));
 
  
  byte Degree[8] = {B00111,B00101,B00111,B0000,B00000,B00000,B00000,B00000};      // define special char degree => °
  lcd.createChar(0, Degree);

  // start sensors  
  dhtsi1.begin();          // sensor inside one
  dhtsi2.begin();          // sensor inside two 
  dhtso1.begin();          // sensor outside one 
  dhtso2.begin();          // sensor outside two 

  measure_index=0;
  while (measure_index<3)         // read sensors at start-up and fill measure array with inital measuers
    {
      read_sensor(measure_index);
      measure_index++;
    }
  measure_index=0;
  pre_measure_index=2;
  display=false;
}

void loop() {
    
  if (digitalRead(PINDISPLAY)==LOW) toggle_backlight();
  
  if (error == true)              // check if we have faulty measures from sensors
    {
    lcd.backlight();
    error = false;
    digitalWrite(PINERROR, LOW);  // Error LED off    
    check_sensor(measure_index);  // recheck sensors 
    delay(2000);
    if (display==false) lcd.noBacklight();
    }
     
   if (error == true) 
   {
    digitalWrite(PINFAN, FAN_OFF); // If there still is an error => fan off 
    lcd.setCursor(0,3);
    lcd.print(F("Restart ....."));
    while (1);                     // This infinity loop will trigger a restart via the watchdog
   }
   wdt_reset();                    // Watchdog reset, if we had no error

  if (measure_index>2) measure_index=0; // measure index reset (2 is max)

  read_sensor (measure_index);

  average_measures ();

   //******* Drew point calculation *******
   float DrewPoint_SI1 = drewpoint(t[0][measure_index], h[0][measure_index]);
   float DrewPoint_SI2 = drewpoint(t[1][measure_index], h[1][measure_index]);
   float DrewPoint_SO1 = drewpoint(t[2][measure_index], h[2][measure_index]);
   float DrewPoint_SO2 = drewpoint(t[3][measure_index], h[3][measure_index]);

 
   //******* print measures of all 4 sensors on I2C display *******
   lcd.clear();
   measuresoutput(t[0][measure_index], h[0][measure_index],DrewPoint_SI1,0);
   measuresoutput(t[1][measure_index], h[1][measure_index],DrewPoint_SI2,1);
   measuresoutput(t[2][measure_index], h[2][measure_index],DrewPoint_SO1,2);
   measuresoutput(t[3][measure_index], h[3][measure_index],DrewPoint_SO2,3);
   Serial.println();

  if (digitalRead(PINDISPLAY)==LOW) toggle_backlight();
  delay(6000); 
  wdt_reset(); // rest watchdog

  lcd.clear();
  lcd.setCursor(0,0);
   
float DrewPointDiffI = abs(DrewPoint_SI1 - DrewPoint_SI2);
if (DrewPointDiffI>1) {
    lcd.println(F("ERROR diff DP inside > 1!"));
    Serial.println(F("ERROR diff dew point inside abs(SI1 - SI2) > 1!"));
    error=true;
  }
  else{
    error=false;
  }
float DrewPointI = (DrewPoint_SI1 + DrewPoint_SI2)/2;    // calcutlate average dew point inside

float DrewPointDiffO = abs(DrewPoint_SO1 - DrewPoint_SO2);
if (DrewPointDiffO>1) {
    lcd.println(F("ERROR diff DP outside > 1!"));
    Serial.println(F("ERROR diff dew point inside abs(SO1 - SO2) > 1!"));
    error=true;
  }
  else{
    error=false;
  }
float DrewPointO = (DrewPoint_SO1 + DrewPoint_SO2)/2;    // calcutlate average dew point outside  
  float DeltaDP = DrewPointI - DrewPointO;

  if (DeltaDP > (DEW_POINT_MIN + HYSTERESIS))fan = true;
  if (DeltaDP < (DEW_POINT_MIN))fan = false;
  if (t[0][measure_index] < TEMP_MIN_INSIDE )fan = false;
  if (t[1][measure_index] < TEMP_MIN_OUTSIDE )fan = false;

  if (fan == true)
  {
  digitalWrite(PINFAN, FAN_ON); // switch fan on
  lcd.print(F("Fan is ON"));  
  } 
  else 
  {                             
  digitalWrite(PINFAN, FAN_OFF); // switch fan off
  lcd.print(F("Fan is OFF"));
  }

  lcd.setCursor(0,1);
  lcd.print(F("Delta DP:    "));
  lcd.print(DeltaDP);
  lcd.write((uint8_t)0); // special char degree °
  lcd.setCursor(0,2);
  lcd.print(F("DP ins. dif:  "));
  lcd.print(DrewPointDiffI);
  lcd.setCursor(0,3);
  lcd.print(F("DP outs. dif: "));
  lcd.print(DrewPointDiffO);
   

  //******* log data *******
   if (logging == true)
   { 
      if  ( Today  != tm.Day)                                     // DaySwitched ==> save fan runtime
      {  
        DaySwitched = true;                                        
         if (FanStart > 0 ) FanRuntime += (1440 - FanStart);       // fan running calulate new runtime
         snprintf(StrFanRuntime,sizeof(StrFanRuntime),"%d;",FanRuntime); 
        Today = tm.Day;
        FanRuntime = 0;
      } 
      else 
      {
        strcpy( StrFanRuntime , "0;");    // No day switch
      }

      char buff[4];
      LogData="";
      dtostrf(t[0][measure_index], 2, 1, buff); LogData += buff ; LogData += ';';
      dtostrf(h[0][measure_index], 2, 1, buff); LogData += buff ; LogData += ';';
      dtostrf(DrewPoint_SI1, 2, 1, buff); LogData += buff ; LogData += ';';
      dtostrf(t[1][measure_index], 2, 1, buff); LogData += buff ; LogData += ';';
      dtostrf(h[1][measure_index], 2, 1, buff); LogData += buff;LogData += ';';
      dtostrf(DrewPoint_SI2, 2, 1, buff); LogData += buff;LogData += ';';
      dtostrf(t[2][measure_index], 2, 1, buff); LogData += buff ; LogData += ';';
      dtostrf(h[2][measure_index], 2, 1, buff); LogData += buff;LogData += ';';
      dtostrf(DrewPoint_SO1, 2, 1, buff); LogData += buff;LogData += ';';
      dtostrf(t[3][measure_index], 2, 1, buff); LogData += buff ; LogData += ';';
      dtostrf(h[3][measure_index], 2, 1, buff); LogData += buff;LogData += ';';
      dtostrf(DrewPoint_SI2, 2, 1, buff); LogData += buff;LogData += ';';
      if (fan == true) LogData +="1;"; else LogData += "0;";
      LogData += StrFanRuntime;
      
      save_to_SD(); // write data to SD card
   }
  
   if (digitalRead(PINDISPLAY)==LOW) {
     toggle_backlight();
     Serial.println("PINDISPLAY is low!");
    }
    else {
      Serial.println("PINDISPLAY is high!");
    }

      
  delay(4000);   // wait time between 2 measures
  wdt_reset();   // reset watchdog
  pre_measure_index=measure_index;
  measure_index++;
   
}

float drewpoint(float t, float r) {
  
  float a, b;
    
  if (t >= 0) 
  {
    a = 7.5;
    b = 237.3;
  } else if (t < 0) {
    a = 7.6;
    b = 240.7;
  }

  // satiation vapor pressure in hPa
  float svp = 6.1078 * pow(10, (a*t)/(b+t));

  // vapor pressure in hPa
  float vp = svp * (r/100);

  // v parameter
  float v = log10(vp/6.1078);

  // dew point in °C
  float dp = (b*v) / (a-v);
  return { dp };  
}


void software_Reset() // Startet das Programm neu, nicht aber die Sensoren oder das LCD 
  {
    asm volatile ("  jmp 0");  
  }

//******* SD card *******
File logFile;                    // variable for the CSV log file
#define PIN_SD_CS 10             // pin for the CS wire of the SD card reader

unsigned long TimeData = 0;      // needed to calculate next time to save data

void test_SD(){
  if (logging == true)
  {
    //******* SD card search *******
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Search for SD card.."));
  Serial.println(F("Search for SD card.."));
  lcd.setCursor(2,1);

  if (!SD.begin(PIN_SD_CS)){
   lcd.print(F("SD card NOT found!"));
   Serial.println(F("SD card NOT found!"));
   logging = false;
   } else {
    lcd.print(F("SD card OK!"));
    Serial.println(F("SD card OK!"));
    if (not SD.exists(logFileName) )  
    {
      logFile = SD.open(logFileName, FILE_WRITE); 
      logFile.println(logHeaderLine);               // write header
      logFile.close();  
    }    
   }
   
   delay(3000);   // time to read the display
   wdt_reset();   // reset watchdog
  }
}


//******* save measures to SD card *******
void save_to_SD()
{ 
  unsigned long t; 

 t = millis() / 60000;               
  if (  TimeData == 0 ) TimeData =  t;  
  
  if (((TimeData + LogInterval) <= t)  or ( DaySwitched ))
  {
    TimeData = t;
    DaySwitched=false;
    test_SD();
    wdt_reset();      // reset watchdog
      
      lcd.clear();
      lcd.print(F("Saving data set "));
      Serial.print(F("Saving data set "));   
      make_time_stamp();
      File logFile = SD.open(logFileName, FILE_WRITE);  // Open file
      Serial.println(LogData);  
      logFile.print (stamp);
      logFile.println( ';' + LogData );
    logFile.close();

    delay(4000);
  } 
}

//******* creates time stamp and writes it on the serial monitor *******
void make_time_stamp()
{
  RTC.read(tm);
  snprintf(stamp,sizeof(stamp),"%02d.%02d.%d %02d:%02d",tm.Day,tm.Month,tmYearToCalendar(tm.Year),tm.Hour,tm.Minute);
  Serial.println(stamp);
}

//******* function to stat and check the real time clock *******
bool RTC_start()
{ 
 if (logging == true)
 { 
  if (RTC.read(tm)) 
  {     
    make_time_stamp ();
    lcd.setCursor(2,0); 
    lcd.print(stamp);
    
  } else {
    if (RTC.chipPresent()) {
      Serial.println(F("No time on RTC"));
      lcd.clear();
      lcd.print(F("No time on RTC"));
      
      delay(2000);
    } else {
      Serial.println(F("No time on RTC"));
      lcd.clear();
      lcd.print(F("No time on RTC"));
      
      delay(2000);
    }
    logging = false;
     return(false);
  }
 }
}

//******* displays measures of all 4 sensors on the LCD and serial monitor *******
void measuresoutput(float t, float h, float tp, int row)
{
   char buffer_float[6];
   String buffer_row="";

   dtostrf(t,4,1,buffer_float);
   buffer_row=String(buffer_float)+"C ";  
   dtostrf(h,5,1,buffer_float);
   buffer_row=" "+buffer_row+String(buffer_float)+"% "; 
   dtostrf(tp,5,1,buffer_float);
   buffer_row=buffer_row+String(buffer_float)+"C";  
   lcd.setCursor(0,row);
   lcd.print(buffer_row);
   Serial.println(buffer_row);
   lcd.setCursor(5,row);
   lcd.write((uint8_t)0); // Special char ° for temperatur
   lcd.setCursor(19,row);
   lcd.write((uint8_t)0); // Special char ° for dew point
}

//******* checks all 4 DHT sensors *******
void check_sensor (int current_measure)
{
    int i = 0;
    char buffer[55];
   
    while (i<4)
    {
      if (isnan(h[i][current_measure]) || isnan(t[i][current_measure]) || h[i][current_measure] > 100 || h[i][current_measure] < 1 || t[i][current_measure] < -40 || t[i][current_measure] > 80 )  
      {
        sprintf(buffer,"ERROR sensor %i! t=%d; h=%d", i+1, t[i][current_measure], h[i][current_measure]);
        Serial.println(buffer);
        sprintf(buffer,"ERROR sensor %i!", i+1);
        lcd.setCursor(5,i);
        lcd.print(buffer);
        error = true;
        digitalWrite(PINERROR, HIGH); // Error LED on
      }
      else 
      {
       lcd.setCursor(5,i);
       sprintf(buffer,"Sensor %i is OK!", i+1);
       Serial.println(buffer);
       lcd.print(buffer);
      }
      i++;
       
      delay(1000);
     
    }
    Serial.println();
    wdt_reset();      // reset watchdog
}

//******* read all 4 sensors *******
void read_sensor (int i) {

  h[0][i] = dhtsi1.readHumidity()+sensor_corr[0][1];     // read humidity SI1, correct it and store it at h[i] 
  t[0][i] = dhtsi1.readTemperature()+sensor_corr[0][0];  // read temperature SI1, correct it and store it at t[i]
  h[1][i] = dhtsi2.readHumidity()+sensor_corr[1][1];     // read humidity SI2, correct it and store it at h[i] 
  t[1][i] = dhtsi2.readTemperature()+sensor_corr[1][0];  // read temperature SI2, correct it and store it at t[i]
  h[2][i] = dhtso1.readHumidity()+sensor_corr[2][1];     // read humidity SO1, correct it and store it at h[i] 
  t[2][i] = dhtso1.readTemperature()+sensor_corr[2][0];  // read temperature SO1, correct it and store it at t[i]
  h[3][i] = dhtso2.readHumidity()+sensor_corr[3][1];     // read humidity SO2, correct it and store it at h[i] 
  t[3][i] = dhtso2.readTemperature()+sensor_corr[3][0];  // read temperature SO2, correct it and store it at t[i]

  delay(1000);
}

//******* calculate average for latest 3 measures *******
void average_measures () {

  int i = 0;

  while (i < 4)
  {
    h[i][measure_index] = (h[i][0]+h[i][1]+h[i][2])/3;   // calculate average for humidity
    t[i][measure_index] = (t[i][0]+t[i][1]+t[i][2])/3;   // calculate average for temperature
    i++;
  }
  
}

//******* toggles the LCD display backlight should be on or off ********
void toggle_backlight () {
      
    display = !display;
    if (display == true) {
      lcd.backlight();
      Serial.println("Display backlight on!");
    }
    else {
      lcd.noBacklight();
      Serial.println("Display backlight off!");
    }
    delay(1000);
}
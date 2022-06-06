#define Software_version "Version: 2.99 dev"

// This code needs the following libraries
#include <DHT.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>
#include <DS1307RTC.h>
#include <SD.h>
#include <SPI.h>
//#include <printf.h>


tmElements_t tm;

#define RELAIPIN 6  // Anschluss des Luefter-Relais
#define DHTPIN_1 5  // Datenleitung fuer den DHT-Sensor 1 (innen)
#define DHTPIN_2 4  // Datenleitung fuer den DHT-Sensor 2 (aussen)
// Hinzugefuegt durch AiK start
#define DHTPIN_3 7  // Datenleitung fuer den DHT-Sensor 3 (2. innen)
#define DHTPIN_4 8  // Datenleitung fuer den DHT-Sensor 4 (2. aussen)
#define ERRORPIN 9  // Anschluss der roten Error-LED
#define DIPLAYPIN 1 // Taster der das Display/Ausgae aktiviert
// Hinzugefuegt durch AiK end

#define RELAIS_EIN LOW
#define RELAIS_AUS HIGH
bool rel;
bool fehler = true;
bool anzeige = true;

#define DHTTYPE_1 DHT22 // DHT 22 
#define DHTTYPE_2 DHT22 // DHT 22
// Hinzugefuegt durch AiK start
#define DHTTYPE_3 DHT22 // DHT 22 
#define DHTTYPE_4 DHT22 // DHT 22
// Hinzugefuegt durch AiK end   

// *******  Korrekturwerte der einzelnen Sensorwerte  *******
float korrektur[4][2] =
{
  {-0.1,1.3},         // S1 innen: Temperaturkorrketur, Luftfeutigkeitskorrektur
  {-0.4,2.1},         // S2 aussen: Temperaturkorrketur, Luftfeutigkeitskorrektur
  {-0.4,1.7},         // S3 innen: Temperaturkorrketur, Luftfeutigkeitskorrektur
  {-0.3,0.1}          // S4 aussen: Temperaturkorrketur, Luftfeutigkeitskorrektur
};      // Array fuer Sensorkorrketurwerte

//#define Korrektur_t_1 -0.1  // Korrekturwert Innensensor Temperatur
//#define Korrektur_t_2 -0.4  // Korrekturwert Aussensensor Temperatur
//#define Korrektur_h_1  1.3  // Korrekturwert Innensensor Luftfeuchtigkeit
//#define Korrektur_h_2  2.1  // Korrekturwert Aussensensor Luftfeuchtigkeit
//#define Korrektur_t_3 -0.4  // Korrekturwert 2. Innensensor Temperatur
//#define Korrektur_t_4 -0.3  // Korrekturwert 2. Aussensensor Temperatur
//#define Korrektur_h_3  1.7  // Korrekturwert 2. Innensensor Luftfeuchtigkeit
//#define Korrektur_h_4  0.1  // Korrekturwert 2. Aussensensor Luftfeuchtigkeit
//***********************************************************
int mw_index = 0;           // Messwerteindes fuer Messwerte-Arrays
int sensor_index = 0;       // Sensor Index fure Messwerte-Arrays 
float t[4][3];              // Array zum Speicher aller 4 Sensor der letzen 3 Temperaturmesswerte
float h[4][3];              // Array zum Speicher aller 4 Sensor der letzen 3 Luftfuechtigkeitsmesswerte
//***********************************************************

#define SCHALTmin   10.0// minimaler Taupunktunterschied, bei dem das Relais schaltet
#define HYSTERESE   1.0 // Abstand von Ein- und Ausschaltpunkt
#define TEMP1_min   8.0 // Minimale Innentemperatur, bei der die Lueftung aktiviert wird
#define TEMP2_min  -5.0 // Minimale Aussentemperatur, bei der die Lueftung aktiviert wird

DHT dht1(DHTPIN_1, DHTTYPE_1); //Der 1. Innensensor wird ab jetzt mit dht1 angesprochen
DHT dht2(DHTPIN_2, DHTTYPE_2); //Der 1. Aussensensor wird ab jetzt mit dht2 angesprochen
DHT dht3(DHTPIN_3, DHTTYPE_3); //Der 2. Innensensor wird ab jetzt mit dht3 angesprochen
DHT dht4(DHTPIN_4, DHTTYPE_4); //Der 2. Aussensensor wird ab jetzt mit dht4 angesprochen

LiquidCrystal_I2C lcd(0x27,20,4); // LCD: I2C-Addresse und Displaygroesse setzen

//*************************************** Variablen fuer das Datenlogging ***************************************
#define Headerzeile F("Datum|Zeit;Temperatur_S1;Feuchte_H1;Taupunkt_1;Temperatur_S2;Feuchte_H2;Taupunkt_2;Luefter_Ein/Aus;Laufzeit_Luefter;")

#define logFileName F("Luefter1.csv")  // Name der Datei zum Abspeichern der Daten (Dateinamen-Format: 8.3)!!!!
#define LogInterval 15                 // Wie oft werden die Messwerte aufgezeichnet ( 5 = alle 5 Minuten)

bool logging = true;                    // Sollen die Daten ueberhaupt protokolliert werden?
String LogData = "" ;                   // Variable zum Zusammensetzen des Logging-Strings.
char stamp[17];                         // Variable fuer den Zeitstempel.
unsigned int LuefterStart = 0;          // Wann wurde der Luefter eingeschaltet?
unsigned int LuefterLaufzeit = 0;       // Wie lange lief der Luefter?
char StrLuefterzeit[6];                 // Luefterlaufzeit als String zur weiteren Verwendung.
uint8_t Today = 0;                      // Das heutige Datum (nur Tag), zur Speicherung bei Tageswechsel.
bool Tageswechsel=false;
//********************************************************************************************************

void setup() 
{
  wdt_enable(WDTO_8S); // Watchdog timer auf 8 Sekunden stellen
  Serial.begin(9600);  // Serielle Ausgabe, falls noch kein LCD angeschlossen ist
  lcd.init();
  lcd.backlight();  
  
  //--------------------- Logging ------------------------------------------------------------------------------------  
  if (logging == true)
  { 
    lcd.setCursor(0,1);
    lcd.print(Software_version);  // Welche Softwareversion laeuft gerade
    Serial.println(Software_version);
    RTC_start();     // RTC-Modul testen. Wenn Fehler, dann kein Logging
    delay (4000);    // Zeit um das Display zu lesen
    lcd.clear(); 
     wdt_reset();  // Watchdog zuruecksetzen
    test_SD();       // SD-Karte suchen. Wenn nicht gefunden, dann kein Logging ausfuehren
    Today = tm.Day ;
    //------------------------------------------------ Neustart aufzeichnen -------------------------------------
  if (logging == true) // kann sich ja geaendert haben wenn Fehler bei RTC oder SD
  {   
    make_time_stamp();   
    File logFile = SD.open(logFileName, FILE_WRITE);
    logFile.print(stamp);
    logFile.println(F(": Neustart"));                    // Damit festgehalten wird, wie oft die Steuerung neu gestartet ist
    logFile.close();  
  }
  } //---------------------------------------------------------------------------------------------------------------------
    
  pinMode(RELAIPIN, OUTPUT);          // Relaispin als Output definieren
  digitalWrite(RELAIPIN, RELAIS_AUS); // Relais ausschalten
  // Hinzugefuegt durch AiK start
  pinMode(ERRORPIN, OUTPUT);          // Errorpin als Output definieren
  digitalWrite(ERRORPIN, HIGH); // Error-LED einschalten
  // Hinzugefuegt durch AiK end
  
  Serial.println(F("Teste Sensoren.."));

  lcd.clear();                  
  lcd.setCursor(0,0);
  lcd.print(F("Test"));
  lcd.setCursor(0,1);
  lcd.print(F("..."));
  
  byte Grad[8] = {B00111,B00101,B00111,B0000,B00000,B00000,B00000,B00000};      // Sonderzeichen ° definieren
  lcd.createChar(0, Grad);
  byte Strich[8] = {B00100,B00100,B00100,B00100,B00100,B00100,B00100,B00100};   // Sonderzeichen senkrechter Strich definieren
  lcd.createChar(1, Strich);
    
  dht1.begin(); // Sensoren starten
  dht2.begin();
  dht3.begin();
  dht4.begin();

 while (mw_index<2)
  {
    read_sensor(mw_index);
    mw_index++;
  }
  mw_index=0;

 
}

void loop() {
    
    if (fehler == true)  // Pruefen, ob gueltige Werte von den Sensoren kommen
    {
    fehler = false;
    digitalWrite(ERRORPIN, LOW); // Error-LED ausschalten    
     
    check_sensor(mw_index);
     // Hinzugefuegt durch AiK end

    delay(2000);  // Eigentlich sollte hie gewartet werden um das Display zu lesen, aber es fuehrt zum Neustart/Absturz WTF!!!!

    }
     
   if (fehler == true) 
   {
    digitalWrite(RELAIPIN, RELAIS_AUS); // Relais ausschalten 
    lcd.setCursor(0,3);
    lcd.print(F("CPU Neustart....."));
    while (1);  // Endlosschleife um das Display zu lesen und die CPU durch den Watchdog neu zu starten
   }
   wdt_reset();  // Watchdog zuruecksetzen

  if (mw_index>2) mw_index=0; // Messwerteindex zuruecksetzen

  read_sensor (mw_index);

   //**** Taupunkte errechnen********
   float Taupunkt_1 = taupunkt(t[0][mw_index], h[0][mw_index]);
   float Taupunkt_2 = taupunkt(t[1][mw_index], h[0][mw_index]);
   float Taupunkt_3 = taupunkt(t[2][mw_index], h[0][mw_index]);
   float Taupunkt_4 = taupunkt(t[3][mw_index], h[0][mw_index]);

 
   // Werteausgabe auf dem I2C-Display
   lcd.clear();

   werteausgabe(t[0][mw_index], h[0][mw_index],Taupunkt_1,0);
   werteausgabe(t[1][mw_index], h[1][mw_index],Taupunkt_2,1);
   werteausgabe(t[2][mw_index], h[2][mw_index],Taupunkt_3,2);
   werteausgabe(t[3][mw_index], h[3][mw_index],Taupunkt_4,3);
   Serial.println();

   delay(6000); // Zeit um das Display zu lesen
   wdt_reset(); // Watchdog zuruecksetzen

   lcd.clear();
   lcd.setCursor(0,0);
   
   float DeltaTP = Taupunkt_1 - Taupunkt_2;

   if (DeltaTP > (SCHALTmin + HYSTERESE))rel = true;
   if (DeltaTP < (SCHALTmin))rel = false;
   if (t[0][mw_index] < TEMP1_min )rel = false;
   if (t[1][mw_index] < TEMP2_min )rel = false;

   if (rel == true)
   {
    digitalWrite(RELAIPIN, RELAIS_EIN); // Relais einschalten
    lcd.print(F("Lueftung AN"));  
   } 
   else 
   {                             
    digitalWrite(RELAIPIN, RELAIS_AUS); // Relais ausschalten
    lcd.print(F("Lueftung AUS"));
   }

   lcd.setCursor(0,1);
   lcd.print("Delta TP: ");
   lcd.print(DeltaTP);
   lcd.write((uint8_t)0); // Sonderzeichen °C
   lcd.write('C');

   delay(4000);   // Wartezeit zwischen zwei Messungen
   wdt_reset();   // Watchdog zuruecksetzen
   mw_index++;
   
  //--------------------------------------------logging-----------------------------------------------------
   if (logging == true)
   { 
      if  ( Today  != tm.Day)                                                     // Tageswechsel ==> Luefterzeit abspeichern
      {  
        Tageswechsel = true;                                                    // ==> Sofort speichern (siehe SD.ino) ==> Nicht erst wenn LogIntervall abgelaufen ist
         if (LuefterStart > 0 ) LuefterLaufzeit += (1440 - LuefterStart);       // ==>Luefter laeuft gerade
         snprintf(StrLuefterzeit,sizeof(StrLuefterzeit),"%d;",LuefterLaufzeit); 
        Today = tm.Day;
        LuefterLaufzeit = 0;
      } 
      else 
      {
        strcpy( StrLuefterzeit , "0;");    // Kein Tageswechsel, nur Platzhalter abspeichern
      }

      char buff[4];
      LogData="";
      dtostrf(t[0][mw_index], 2, 1, buff); LogData += buff ; LogData += ';';
      dtostrf(h[0][mw_index], 2, 1, buff); LogData += buff ; LogData += ';';
      dtostrf(Taupunkt_1, 2, 1, buff); LogData += buff ; LogData += ';';
      dtostrf(t[1][mw_index], 2, 1, buff); LogData += buff ; LogData += ';';
      dtostrf(h[1][mw_index], 2, 1, buff); LogData += buff;LogData += ';';
      dtostrf(Taupunkt_2, 2, 1, buff); LogData += buff;LogData += ';';
      dtostrf(t[2][mw_index], 2, 1, buff); LogData += buff ; LogData += ';';
      dtostrf(h[2][mw_index], 2, 1, buff); LogData += buff;LogData += ';';
      dtostrf(Taupunkt_3, 2, 1, buff); LogData += buff;LogData += ';';
      dtostrf(t[3][mw_index], 2, 1, buff); LogData += buff ; LogData += ';';
      dtostrf(h[3][mw_index], 2, 1, buff); LogData += buff;LogData += ';';
      dtostrf(Taupunkt_2, 2, 1, buff); LogData += buff;LogData += ';';
      if (rel == true) LogData +="1;"; else LogData += "0;";
      LogData += StrLuefterzeit;
      
      save_to_SD(); // Daten auf die SD Karte speichern
   }
}
//--------------------------------------------------------------------------------------------------------

float taupunkt(float t, float r) {
  
  float a, b;
    
  if (t >= 0) 
  {
    a = 7.5;
    b = 237.3;
  } else if (t < 0) {
    a = 7.6;
    b = 240.7;
  }

  // Saettigungsdampfdruck in hPa
  float sdd = 6.1078 * pow(10, (a*t)/(b+t));

  // Dampfdruck in hPa
  float dd = sdd * (r/100);

  // v-Parameter
  float v = log10(dd/6.1078);

  // Taupunkttemperatur (°C)
  float tt = (b*v) / (a-v);
  return { tt };  
}


void software_Reset() // Startet das Programm neu, nicht aber die Sensoren oder das LCD 
  {
    asm volatile ("  jmp 0");  
  }

//--------------------------------------------SD.ino-start----------------------------------------------------
File logFile;                    // Variable fuer die csv-Datei
#define CS_PIN 10                // An diesem Pin ist die CS-Leitung angeschlossen

unsigned long TimeDaten = 0;    // zur Berechnung, wann wieder gespeichert wird


void test_SD(){
  if (logging == true)
  {
    // --------- SD-Karte suchen  -------------------
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Suche SD Karte.."));
  Serial.println(F("Suche SD Karte.."));
  lcd.setCursor(2,1);

  if (!SD.begin(CS_PIN)){
   lcd.print(F("SD nicht gefunden!"));
   Serial.println(F("SD nicht gefunden!"));
   logging = false;
   } else {
    lcd.print(F("SD  gefunden"));
    Serial.println(F("SD gefunden"));
    if (not SD.exists(logFileName) )  
    {
      logFile = SD.open(logFileName, FILE_WRITE); 
      logFile.println(Headerzeile); // Header schreiben
      logFile.close();  
    }    
   }
   delay(3000);   // Zeit um das Display zu lesen
   wdt_reset();   // Watchdog zuruecksetzen
  }
}


void save_to_SD()
{ 
  unsigned long t; 

 t = millis() / 60000;               
  if (  TimeDaten == 0 ) TimeDaten =  t;  

  // -------------------------  Sensorenwerte abspeichern ----------------------
  if (((TimeDaten + LogInterval) <= t)  or ( Tageswechsel ))
  {
    TimeDaten = t;
    Tageswechsel=false;
    test_SD();
    wdt_reset(); // Watchdog zuruecksetzen
      lcd.clear();
      lcd.print(F("Speichere Datensatz"));
      Serial.print(F("speicher Datensatz "));   
      make_time_stamp();
      File logFile = SD.open(logFileName, FILE_WRITE);  // Oeffne Datei
      Serial.println(LogData);  
      logFile.print (stamp);
      logFile.println( ';' + LogData );
    logFile.close(); 
    delay(4000);
  } 
}
//--------------------------------------------SD.ino-end----------------------------------------------------

//--------------------------------------------Zeit.ino-start----------------------------------------------------

void make_time_stamp()
{
  RTC.read(tm);
  snprintf(stamp,sizeof(stamp),"%02d.%02d.%d %02d:%02d",tm.Day,tm.Month,tmYearToCalendar(tm.Year),tm.Hour,tm.Minute);
  Serial.println(stamp);
}


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
      Serial.println(F("RTC hat keine Zeit"));
      lcd.clear();
      lcd.print(F("RTC hat keine Zeit"));
      delay(2000);
    } else {
      Serial.println(F("RTC hat keine Zeit"));
      lcd.clear();
      lcd.print(F("Kein Signal vom RTC"));
      delay(2000);
    }
    logging = false;
     return(false);
  }
 }
}
//--------------------------------------------Zeit.ino-end----------------------------------------------------


void werteausgabe(float t, float h, float tp, int row)
{
   char buffer_float[6];
   String buffer_row="";

   dtostrf(t,4,1,buffer_float);
   buffer_row=String(buffer_float)+"C ";  // Sonderzeichen °C
   dtostrf(h,5,1,buffer_float);
   buffer_row=" "+buffer_row+String(buffer_float)+"% "; 
   dtostrf(tp,5,1,buffer_float);
   buffer_row=buffer_row+String(buffer_float)+"C";  // Sonderzeichen °C   
   lcd.setCursor(0,row);
   lcd.print(buffer_row);
   Serial.println(buffer_row);
   lcd.setCursor(5,row);
   lcd.write((uint8_t)0); // Sonderzeichen °C Sensor Temperatur
   lcd.setCursor(19,row);
   lcd.write((uint8_t)0); // Sonderzeichen °C Taupunkt

}

void check_sensor (int mw_index)
{
    int i = 0;
    char buffer[55];
    while (i<4)
    {
      if (isnan(h[i][mw_index]) || isnan(t[i][mw_index]) || h[i][mw_index] > 100 || h[i][mw_index] < 1 || t[i][mw_index] < -40 || t[i][mw_index] > 80 )  
      {
        sprintf(buffer,"Fehler beim Auslesen von Sensor %i! t=%d; h=%d", i+1, t[i][mw_index], h[i][mw_index]);
        Serial.println(buffer);
        sprintf(buffer,"Fehler S%i!", i+1);
        lcd.setCursor(5,i);
        lcd.print(buffer);
        fehler = true;
        digitalWrite(ERRORPIN, HIGH); // Error-LED einschalten
      }
      else 
      {
       lcd.setCursor(5,i);
       sprintf(buffer,"S%i ist OK!", i+1);
       Serial.println(buffer);
       lcd.print(buffer);
      }
      i++;
      delay(1000);
    }
    Serial.println();
}

void read_sensor (int mw_index) {


      h[0][mw_index] = dht1.readHumidity()+korrektur[0][1];     // auslesen S1 Luftfeuchtigkeit und speichern unter h[mw_index] 
      t[0][mw_index] = dht1.readTemperature()+korrektur[0][0];  // auslesen S1 Temperatur und speichern unter t[mw_index]
      h[1][mw_index] = dht2.readHumidity()+korrektur[1][1];     // auslesen S2 Luftfeuchtigkeit und speichern unter h[mw_index] 
      t[1][mw_index] = dht2.readTemperature()+korrektur[1][0];  // auslesen S2 Temperatur und speichern unter t[mw_index]
      h[2][mw_index] = dht3.readHumidity()+korrektur[2][1];     // auslesen S3 Luftfeuchtigkeit und speichern unter h[mw_index] 
      t[2][mw_index] = dht3.readTemperature()+korrektur[2][0];  // auslesen S3 Temperatur und speichern unter t[mw_index]
      h[3][mw_index] = dht4.readHumidity()+korrektur[3][1];     // auslesen S4 Luftfeuchtigkeit und speichern unter h[mw_index] 
      t[3][mw_index] = dht4.readTemperature()+korrektur[3][0];  // auslesen S4 Temperatur und speichern unter t[mw_index]

      delay(1000);

}
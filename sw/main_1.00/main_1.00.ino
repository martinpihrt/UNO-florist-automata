/*
Martin Pihrt www.pihrt.com - zalevac kvetin s frekvencni sondou
Arduino 1.8.5 

todo hw  a plneni vody 1257 radek cas plneni udelat nastavovatko


                           Rs, E,DB4,DB5,DB6,DB7
LCD:                    pin 8, 9,  2,  3,  6,  7
LCD podsviceni:         pin 10
Tlacitka IN:            pin 14 (A0 delic 5 tlacitek)
nepouzito               pin 15 (A1) fotorezistor
Cidlo vlhka IN:         pin 5 (meri se frekvence ze snimace vlhkosti)
Serial:                 pin 0, 1
I2C:                    pin 18 (A4 SDA), 19 (A5 SCL) pouziva DS1307 
Teplota:                pin 16  DS18B20 ruda +5v, cerna GND, modrá(bila) Data
Cerpadlo:               pin 17 (pro motor cerpadla)
nepouzito               pin 4 
El. ventil privod vody: pin 11 (pro doplneni)
nepouzito:              pin 12 

adresa I2C:
0x68  DS1307 RTC
*/

char verze[]="FW05.10.18 V1.00"; // Verze programu
#define DEBUG                  // 1/0 Povoleni, nebo zakazani ladeni na serialu
#define baud_rate 115200       // Rychlost serialu nastavit Both NL a CR

// mereni frekvence z cidla vlhkosti 
#define cidlo_fMIN  5000       // Hz z cidla pri mokru ve vode 
#define cidlo_fMAX  60         // Hz z cidla pri suchu na vzduchu

#define cerpadlo_pin   17      // Cislo pinu pro cerpadlo
#define ventil_pin     11      // Cislo pinu pro ventil
#define temp_pin       16      // Cislo pinu cidla teploty
#define lcd_rs         8       // Cisla pinu pro lcd
#define lcd_en         9
#define lcd_db4        2
#define lcd_db5        3
#define lcd_db6        6
#define lcd_db7        7
#define lcd_led        10      // Cislo pinu podsviceni LCD
#define frek_pin       5       // Cislo pinu snimace vlhkosti (frekvence na vlhkost) na casovaci T1 cpu
#define button_pin     14      // Cislo pinu tlacitek (A0)

// import knihoven
//http://www.pjrc.com/teensy/td_libs_DS1307RTC.html
//http://www.pjrc.com/teensy/td_libs_Time.html
//https://www.pjrc.com/teensy/td_libs_TimeAlarms.html vychozi #define dtNBR_ALARMS 6   // max is 255 v souboru TimeAlarms.h pro casovace

#include "Time.h"              // Cas systemu
#include "TimeAlarms.h"        // Casovace
#include "DS1307RTC.h"         // Knihovna I2C DS1307
#include "LiquidCrystal.h"     // Knihovna LCD displeje
#include "EEPROM.h"            // Knihovna Eeprom
#include "Wire.h"              // Knihovna I2C
#include "OneWire.h"           // Knihovna pro Dallas Temperature OneWire DS18S20, DS18B20, DS1822
#include "DallasTemperature.h" // Knihovna pro rychle mereni teploty bez delay 1000
#include <avr/interrupt.h>     // Knihovna preruseni pro citace frekvence
#include <avr/wdt.h>           // Watchdog
/*watchdog
  wdt_disable();  
  wdt_enable(WDTO_15MS); wdt_enable(WDTO_8S); etc
  wdt_reset();  
*/

float frq;

OneWire oneWire(temp_pin);
DallasTemperature sensors(&oneWire);
float celsius;              // Hodnota teploty v C
byte vlhkost = 0;           // Hodnota vlhkosti  0-10 na stupnici  

tmElements_t tm;            // Objekt cas RTC

LiquidCrystal lcd(lcd_rs, lcd_en, lcd_db4, lcd_db5, lcd_db6, lcd_db7); // LCD displej (Rs,E,DB4,DB5,DB6,DB7)

// spinani cerpadla
boolean cerpadlo_makej = false; // Pokud je true bude cerpadlo zapinat a vypinat v intervalech
unsigned long aktualniCasCerpadlo = millis(); // Pomocna pro casovac cerpadla
unsigned long predchoziCasCerpadlo = 0;    // Pomocna pro casovac
unsigned long interval = 0;                // Pomocna pro casovac 
boolean cerpadlo_vystup_stav = true;       // Pomocna pro vystup cerpadla

// regulace zalevani dle casu, vlhkosti a teploty 
byte perioda_posun = 0;                    // Promena posun periody kazdy den +1 (1-7)
boolean perioda_je_tu = true;              // Pomocna urcuje ze je jiz x-ty den (menu "interval spinani" 1-7) po zapnuti napajeni je true
boolean CAS1 = false;                      // Hodi se true v nastaveny cas 1
boolean CAS2 = false;                      // Hodi se true v nastaveny cas 2
boolean CAS3 = false;                      // Hodi se true v nastaveny cas 3
unsigned long predchoziCasDoba = 0;        // Pomocna trvani casu 1-3 pro cerpadlo
unsigned long aktualniCasDoba = millis();  // Pomocna trvani casu 1-3 pro cerpadlo
boolean blokuj_mereni_vlhka = false;       // Povolime na lcd mereni vlhkosti a v casovaci mereni vlhka (kdyz cas1-3 je true a blokuje se mereni + lcd info
boolean blokuj_mereni_teploty = false;     // Povolime na lcd mereni teploty a v casovaci mereni teploty (kdyz cas1-3 je true a blokuje se mereni + lcd info
unsigned long trvani_doby = 0;             // Nastavuje dobu zalevani - pomocna

// definice hodnot pro pouziti s panelem a tlacitky
static int btnPUSHED = 0;  // pocitadlo stisknuti vsech tlacitek
boolean btnPUSH = false;   // signalizace prave/stale stisknuteho tlacitka
int lcd_key     = 5;       // zadne tlacitko neni stiskle  
int adc_key_in  = 0;
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

// definice pro menu
boolean save=false;                // Promena povoleni ulozeni do pameti eeprom
boolean menu=false;                // Promena je aktivni tlacitko menu
int menu_state = -1;               // -1 neni nic aktivni jinak (0-13) dle polozky v menu
byte LR_menu = 0;                  // Pomocna posun v menu vlevo /vpravo 0=den, 1=mesic, 2=rok, 3=hod, 4=min
int posun_lcd = 0;                 // Pomocna posun info na lcd 

// definice pro nastaveni datum a cas v serialu/lcd
int vterina = 0;
int minuta = 0;
int hodina = 0;
int den = 0;
int mesic = 0;
int rok = 0;

// symboly na lcd
// http://mikeyancey.com/hamcalc/lcd_characters.php
byte stupen[8] = {             // krouzek pro stupen Celsia
    0b00000,
    0b00100,
    0b01010,
    0b00100,
    0b00000,
    0b00000,
    0b00000,
    0b00000
};
byte teplomer[8] =             // Ikona teplomeru
  {
    0b00100,
    0b01010,
    0b01010,
    0b01110,
    0b01110,
    0b11111,
    0b11111,
    0b01110
};
byte kapka[8] =                // Ikona vodni kapky
  {
    0b00100,
    0b00100,
    0b01010,
    0b01010,
    0b10001,
    0b10001,
    0b10001,
    0b01110,
};
byte sipka_nahoru[8] = {        // ikona sipky smer nahoru
    0b00100,
    0b01110,
    0b11111,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00000
};
byte sipka_dolu[8] = {          // ikona sipky smer dolu
    0b00000,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b11111,
    0b01110,
    0b00100
};

// definice adresy eeprom a promenych z eeprom
#define adr_perioda_den  1         // perioda 1-7 den
byte perioda_den;
#define adr_cas1_hod     2         // cas1 0-23 hod
byte cas1_hod;
#define adr_cas1_min     3         // cas1 0-59 min (max 23:30) krok 30 min
byte cas1_min;
#define adr_doba1        4         // doba1 5-60 min krok 5 min
byte doba1;
#define adr_cas2_hod     5         // cas2 0-23 hod
byte cas2_hod;
#define adr_cas2_min     6         // cas2 0-59 min (max 23:30) krok 30 min
byte cas2_min;
#define adr_doba2        7         // doba2 5-60 min krok 5 min
byte doba2;
#define adr_cas3_hod     8         // cas3 0-23 hod
byte cas3_hod;
#define adr_cas3_min     9         // cas3 0-59 min (max 23:30) krok 30 min
byte cas3_min;
#define adr_doba3        10        // doba3 5-60 min krok 5 min
byte doba3;
#define adr_cerpadlo_on  11        // doba sepnuti cerpadla na 0-10 min krok 1 min
byte cerpadlo_on;
#define adr_pauza_cerpadlo_off  12 // doba cekani cerpadla na 0-5 min krok 1 min
byte pauza_cerpadlo_off;
#define adr_use_teplota  13        // pouzivat mereni teploty 1=off, 2=on
byte use_teplota;
#define adr_last_teplota 14        // posledni zmerena teplota (hodnota je float x1000)
long last_teplota;
#define adr_use_vlhkost  16        // pouzivat mereni vlhkosti 1=off, 2=on
byte use_vlhkost;
#define adr_last_vlhkost 17        // posledni zmerena vlhkost (namerena hodnota z AD 0-1023)
long last_vlhkost;

//**********************************************************************************************
// citac frekvence na D5 ********************************************************************************
volatile unsigned long timerCounts;
volatile boolean counterReady;
unsigned long overflowCount;
unsigned int timerTicks;
unsigned int timerPeriod;

void startCounting (unsigned int ms){
  counterReady = false;         // time not up yet
  timerPeriod = ms;             // how many 1 ms counts to do
  timerTicks = 0;               // reset interrupt counter
  overflowCount = 0;            // no overflows yet
  // reset Timer 1 and Timer 2
  TCCR1A = 0;             
  TCCR1B = 0;              
  TCCR2A = 0;
  TCCR2B = 0;
  // Timer 1 - counts events on pin D5
  TIMSK1 = bit (TOIE1);   // interrupt on Timer 1 overflow
  // Timer 2 - gives us our 1 ms counting interval
  // 16 MHz clock (62.5 ns per tick) - prescaled by 128
  //  counter increments every 8 µs. 
  // So we count 125 of them, giving exactly 1000 µs (1 ms)
  TCCR2A = bit (WGM21) ;       // CTC mode
  OCR2A  = 124;                // count up to 125  (zero relative!!!!)
  // Timer 2 - interrupt on match (ie. every 1 ms)
  TIMSK2 = bit (OCIE2A);       // enable Timer2 Interrupt
  TCNT1 = 0;                   // Both counters to zero
  TCNT2 = 0;     
  // Reset prescalers
  GTCCR = bit (PSRASY);        // reset prescaler now
  // start Timer 2
  TCCR2B =  bit (CS20) | bit (CS22) ;  // prescaler of 128
  // start Timer 1
  // External clock source on T1 pin (D5). Clock on rising edge.
  TCCR1B =  bit (CS10) | bit (CS11) | bit (CS12);
  }  // end of startCounting

ISR (TIMER1_OVF_vect){
  ++overflowCount;               // count number of Counter1 overflows  
}  // end of TIMER1_OVF_vect

//******************************************************************
//  Timer2 Interrupt Service is invoked by hardware Timer 2 every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz

ISR (TIMER2_COMPA_vect){// grab counter value before it changes any more
  unsigned int timer1CounterValue;
  timer1CounterValue = TCNT1;  // see datasheet, page 117 (accessing 16-bit registers)
  unsigned long overflowCopy = overflowCount;
  // see if we have reached timing period
  if (++timerTicks < timerPeriod) 
    return;  // not yet
  // if just missed an overflow
  if ((TIFR1 & bit (TOV1)) && timer1CounterValue < 256)
    overflowCopy++;
  // end of gate time, measurement ready
  TCCR1A = 0;    // stop timer 1
  TCCR1B = 0;    
  TCCR2A = 0;    // stop timer 2
  TCCR2B = 0;    
  TIMSK1 = 0;    // disable Timer1 Interrupt
  TIMSK2 = 0;    // disable Timer2 Interrupt
  // calculate total count
  timerCounts = (overflowCopy << 16) + timer1CounterValue;  // each overflow is 65536 more
  counterReady = true;              // set global flag for end count period
} // end of TIMER2_COMPA_vect

//**********************************************************************************************
// setup - loop ********************************************************************************
void setup() {
    inicializace();     // pomocne funkce (inicializace serial, lcd, cidlo svetla)
    init_piny();        // pomocne funkce (nastavy vystupy a vypne je)
    init_eeprom();      // eeprom (nacte ulozene hodnoty, pripadne ulozi default)
    firmware();         // pomocne funkce (ini serial a lcd, fw na serial a lcd)
    cti_serial();       // serial_rx_tx (komunikace se serialem)   
    nacti_RTC();        // pomocne funkce (nacteni RTC a tisk na serial a lcd)
    serial_clock_display(); // tisk casu na serial
    init_teplota();     // pomocne funkce cidlo DS18B20
    nastav_casovace();  // casovac (nastavi casovace) 
    wdt_enable(WDTO_4S);
    wdt_reset();    
} // end setup

void loop() {
    cti_serial();                // serial_rx_tx (komunikace se serialem)   
    cti_klavesnici();            // klavesnice (vyhodnocujeme klavesy)
    if (save) {save_eeprom();}   // ulozi do pameti z lcd menu tlacitek
    else {
      regulace_vlhk();             // zpracuje zavlazovani dle vhlkosti
      regulace_tepl();             // zpracuje zavlažování dle teploty
      cerpadlo();                  // zpracuj cerpadlo
      plneni_nadrze();             // zpracuj plneni vody do nadrze
    }
    
  //------------------------------------------
  // http://www.gammon.com.au/forum/?id=11504
  /*
// stop Timer 0 interrupts from throwing the count out
  byte oldTCCR0A = TCCR0A;
  byte oldTCCR0B = TCCR0B;
  TCCR0A = 0;    // stop timer 0
  TCCR0B = 0;    
  */
  startCounting (500);  // how many ms to count for
  while (!counterReady)
     {
      wdt_reset();
      } 
  // adjust counts by counting interval to give frequency in Hz
  frq = (timerCounts *  1000.0) / timerPeriod;

  //Serial.println(F("--"));
  //Serial.println ((unsigned long) frq);
  //Serial.println (" Hz.");
 /*
  // restart timer 0
  TCCR0A = oldTCCR0A;
  TCCR0B = oldTCCR0B;
*/
  //--------------------------------------------
  vlhkost = mapuj(frq,cidlo_fMIN,cidlo_fMAX,10,0); // prevod frekvence na vlhkost stupnice vlhka 0-10 s ohledem na min a max frekvenci z cidla
  if(vlhkost >= 10)  vlhkost = 10;
  if(vlhkost <= 1)   vlhkost = 0;    

  Alarm.delay(1);              // tik casovace  
  wdt_reset();
} // end loop

//**********************************************************************************************
// casovace *************************************************************************************
void nastav_casovace(){  
   Alarm.alarmRepeat(23,59,50, PeriodaVyp);             // 23:59:50 kazdy den pred pulnoci resetuje periodu
   Alarm.alarmRepeat(0,0,3, PeriodaDen);                // 00:00:03 kazdy den po pulnoci pricita periodu (den) 1-7 
   Alarm.timerRepeat(2, CasovacSec);                    // prepni text na LCD po 2 sec a mer cidla
   Alarm.alarmRepeat(cas1_hod,cas1_min,0, ZavlahaCasa); // h:m cas zavlazovani 1 
   Alarm.alarmRepeat(cas2_hod,cas2_min,0, ZavlahaCasb); // h:m cas zavlazovani 2 
   Alarm.alarmRepeat(cas3_hod,cas3_min,0, ZavlahaCasc); // h:m cas zavlazovani 3 
} //end void

void PeriodaVyp(){ // resetuje vzdy den 1 az 7 o pulnoci
   perioda_je_tu = false; // skoncil den nulujeme priznak perioda
   CAS1 = false;             // zakazeme cas1
   CAS2 = false;             // zakazeme cas2
   CAS3 = false;             // zakazeme cas2
   cerpadlo_makej = false;   // zakazeme cerpadlo
   blokuj_mereni_teploty = false;
   blokuj_mereni_vlhka = false;
   Serial_ende();
   switch(perioda_posun){ // na konci dne resetuje den (kdyz je napriklad den 3 tak ho vynuluje, ale az nakonci)
     case 1:
        if (perioda_den == 1) { // kdyz je v menu nstavena peri 1 den
          perioda_posun = 0;
          }
     break;
     case 2:
        if (perioda_den == 2) {
          perioda_posun = 0;
          }
     break;
     case 3:
        if (perioda_den == 3) {
          perioda_posun = 0;
          }
     break;
     case 4:
        if (perioda_den == 4) {
          perioda_posun = 0;
          }
     break;
     case 5:
        if (perioda_den == 5) {
          perioda_posun = 0;
          }
     break;
     case 6:
        if (perioda_den == 6) {
          perioda_posun = 0;
          }
     break;
     case 7:
        if (perioda_den == 7) {
          perioda_posun = 0;
          }
     break;
   } // end switch  
} // end void

void PeriodaDen(){ // odpocitava den 1 az 7
   perioda_posun++;  // 1-7 den
   if (perioda_posun >= 8){
      perioda_posun = 0; // cykl 1-7 den
      PeriodaDen();
      } 
   switch(perioda_posun){
     case 1:
        if (perioda_den == 1) { // kdyz je v menu nstavena peri 1 den
          perioda_je_tu = true; // zacal 1 den
          perioda_posun = 0;
          }
     break;
     case 2:
        if (perioda_den == 2) {
          perioda_je_tu = true; // zacal 2 den
          perioda_posun = 0;
          }
     break;
     case 3:
        if (perioda_den == 3) {
          perioda_je_tu = true; // zacal 3 den
          perioda_posun = 0;
          }
     break;
     case 4:
        if (perioda_den == 4) {
          perioda_je_tu = true; // zacal 4 den
          perioda_posun = 0;
          }
     break;
     case 5:
        if (perioda_den == 5) {
          perioda_je_tu = true; // zacal 5 den
          perioda_posun = 0;
          }
     break;
     case 6:
        if (perioda_den == 6) {
          perioda_je_tu = true; // zacal 6 den
          perioda_posun = 0;
          }
     break;
     case 7:
        if (perioda_den == 7) {
          perioda_je_tu = true; // zacal 7 den
          perioda_posun = 0;
          }
     break;
   } // end switch  
#ifdef DEBUG      
   Serial.println(F("DEN"));
#endif    
} // end void

void ZavlahaCasa(){ //
#ifdef DEBUG
   Serial.println(F("CAS1"));
#endif   
   CAS1 = true; // prave je cas 1 
   predchoziCasCerpadlo = millis(); // ulozime aktualni cas pro cerpadlo
   interval = 0;                    // interval cyklovani cerpadla 
   predchoziCasDoba = millis();     // interval doby zalevani (doba1)
   trvani_doby = doba1*1000;       // promena do casovace zalevani void regulace_vlhk() a teploty
} // end void

void ZavlahaCasb(){ //
#ifdef DEBUG
   Serial.println(F("CAS2"));
#endif    
   CAS2 = true;
   predchoziCasCerpadlo = millis(); // ulozime aktualni cas pro cerpadlo
   interval = 0;                    // interval cyklovani cerpadla 
   predchoziCasDoba = millis();      // interval doby zalevani (doba2)
   trvani_doby = doba2*1000;
} // end void

void ZavlahaCasc(){ //
#ifdef DEBUG
   Serial.println(F("CAS3"));
#endif    
   CAS3 = true;
   predchoziCasCerpadlo = millis(); // ulozime aktualni cas pro cerpadlo
   interval = 0;                    // interval cyklovani cerpadla 
   predchoziCasDoba = millis();     // interval doby zalevani (doba3)
   trvani_doby = doba3*1000;
} // end void

void CasovacSec(){ // posouva text na lcd a meri z cidel po 1 sec  
   if (menu==false){ 
     cti_teplotu();   // ds18b20
     cti_vlhkost();   // vstup frekvence na serial   
     posun_lcd++;
     vypisuj_info_lcd();
   }
} // end void 

//**********************************************************************************************
// eeprom pamet ********************************************************************************
void write_eeprom_long(int adres, long hodnota){ // zapis do pameti eeprom (long cislo)
  byte Hi = (hodnota & 0xFF);
  byte Lo = ((hodnota >> 8) & 0xFF);
  EEPROM.write(adres, Hi);
  EEPROM.write(adres + 1, Lo);
} // end void

void write_eeprom_byte(int adres, byte hodnota){ // zapis do pameti eeprom (byte cislo)
  EEPROM.write(adres, hodnota);
} // end void

long read_eeprom_long(long adr){ // cteni z pameti eeprom (long cislo)
  long Hig = EEPROM.read(adr);
  long Low = EEPROM.read(adr + 1);
  return ((Hig << 0) & 0xFF) + ((Low << 8) & 0xFFFF); // vraci long cislo ze dvou bunek eeprom
} // end void

long read_eeprom_byte(long adr){ // cteni z pameti eeprom (byte cislo)
  byte High = EEPROM.read(adr);
  return (High); // vraci byte cislo z eeprom
} // end void

void init_eeprom(){ // cteni z pameti eeprom a ulozeni defaultu pokud je nova eeprom v cpu prazdna
  init_read(); // nacteme z eeprom hodnoty 
  boolean edit = false;
  // pokud jsou mimo rozsah upravime a ulozime jako default
  if (perioda_den > 7 || perioda_den < 1)  {edit = true;write_eeprom_byte(adr_perioda_den, 1);}      // nastaveno na 1 den opakovani
  if (cas1_hod > 23 || cas1_hod < 0)       {edit = true;write_eeprom_byte(adr_cas1_hod, 6);}         // nastaveno na 6 hod (>6:00)
  if (cas1_min > 59 || cas1_min < 0)       {edit = true;write_eeprom_byte(adr_cas1_min, 0);}         // nastaveno na 0 min (6:>00)
  if (doba1 > 250 || doba1 < 5)            {edit = true;write_eeprom_byte(adr_doba1, 30);}           // nastaveno na 30 sec doba zalevani
  if (cas2_hod > 23 || cas2_hod < 0)       {edit = true;write_eeprom_byte(adr_cas2_hod, 18);}        // nastaveno na 18 hod (>18:00)
  if (cas2_min > 59 || cas2_min < 0)       {edit = true;write_eeprom_byte(adr_cas2_min, 0);}         // nastaveno na 0 min (18:>00)
  if (doba2 > 250 || doba2 < 5)            {edit = true;write_eeprom_byte(adr_doba2, 30);}           // nastaveno na 30 sec doba zalevani
  if (cas3_hod > 23 || cas3_hod < 0)       {edit = true;write_eeprom_byte(adr_cas3_hod, 9);}         // nastaveno na 9 hod (>9:00)
  if (cas3_min > 59 || cas3_min < 0)       {edit = true;write_eeprom_byte(adr_cas3_min, 0);}         // nastaveno na 0 min (9:>00)
  if (doba3 > 250 || doba3 < 5)            {edit = true;write_eeprom_byte(adr_doba3, 30);}           // nastaveno na 30 sec doba zalevani
  if (cerpadlo_on > 250 || cerpadlo_on < 0){edit = true;write_eeprom_byte(adr_cerpadlo_on, 5);}      // nastaveno na 5 sec cerpadlo bezi
  if (pauza_cerpadlo_off > 250 || pauza_cerpadlo_off < 0) {edit = true;write_eeprom_byte(adr_pauza_cerpadlo_off, 20);}  // nastaveno na 20 sec cerpadlo ceka
  if (use_teplota > 2 || use_teplota < 1)  {edit = true;write_eeprom_byte(adr_use_teplota, 1);}      // nepouzivat mereni teploty (2=on, 1=off)
  if (last_teplota > 7000 || last_teplota < 0) {edit = true;write_eeprom_long(adr_last_teplota, 0);} // posledni zmerena teplota vychozi 0 C
  if (use_vlhkost > 2 || use_vlhkost < 1)  {edit = true;write_eeprom_byte(adr_use_vlhkost, 1);}      // pouzivat mereni vlhkosti ano(2=on, 1=off)
  if (last_vlhkost > 10 || last_vlhkost < 0) {edit = true;write_eeprom_long(adr_last_vlhkost, 0);}   // posledni zmerena vlhkost vychozi 0 (0-10)
  if (pln_nadrz > 360 || pln_nadrz < 5)   {edit = true;write_eeprom_long(adr_pln_nadrz, 60);}        // nastaveno na 60 min max doba plneni nadrze
  if (edit) init_read();  // znovu nacteme ulozene z eeprom pokud se neco menilo   
} // end init_eeprom  

void init_read(){ // nacteni z eeprom po zapnuti zarizeni
  perioda_den=read_eeprom_byte(adr_perioda_den);            // nacte se 1-7 z eeprom 
  cas1_hod=read_eeprom_byte(adr_cas1_hod);                  // nacte se cas1 0-23 hod   
  cas1_min=read_eeprom_byte(adr_cas1_min);                  // nacte se cas1 0-59 min
  doba1=read_eeprom_byte(adr_doba1);                        // nacte se doba1 5-250 sec 
  cas2_hod=read_eeprom_byte(adr_cas2_hod);                  // nacte se cas2 0-23 hod 
  cas2_min=read_eeprom_byte(adr_cas2_min);                  // nacte se cas2 0-59 min
  doba2=read_eeprom_byte(adr_doba2);                        // nacte se doba2 5-250 sec 
  cas3_hod=read_eeprom_byte(adr_cas3_hod);                  // nacte se cas3 0-23 hod      
  cas3_min=read_eeprom_byte(adr_cas3_min);                  // nacte se cas3 0-59 min
  doba3=read_eeprom_byte(adr_doba3);                        // nacte se doba3 5-250 sec 
  cerpadlo_on=read_eeprom_byte(adr_cerpadlo_on);            // nacte se doba sepnuti cerpadla 0-250 sec
  pauza_cerpadlo_off=read_eeprom_byte(adr_pauza_cerpadlo_off); // nacte se doba cekani cerpadla 0-250 sec
  use_teplota=read_eeprom_byte(adr_use_teplota);               // nacte 1,2 pouzivat mereni teploty 1=off, 2=on  
  last_teplota=read_eeprom_long(adr_last_teplota);             // nacte posledni zmerenou teplotu (hodnota je float x1000) 
  use_vlhkost=read_eeprom_byte(adr_use_vlhkost);               // nacte 1,2 pouzivat mereni vlhkosti 1=off, 2=on
  last_vlhkost=read_eeprom_long(adr_last_vlhkost);             // nacte posledni zmerenou vlhkost (namerena hodnota z AD 0-1023) 
  pln_nadrz=read_eeprom_long(adr_pln_nadrz);                   // nacte 5-360 min

#ifdef DEBUG 
  Serial.println(F("INITOK"));
#endif   
} // end init_read  

void ee_read(){ // vypise eeprom hodnoty
   init_read(); // nacte ee
#ifdef DEBUG
   Serial.print(F("Perioda:"));Serial.println(perioda_den);
   Serial.print(F("Cas1:"));Serial.print(cas1_hod);Serial.print(cas1_min);Serial.println(doba1);
   Serial.print(F("Cas2:"));Serial.print(cas2_hod);Serial.print(cas2_min);Serial.println(doba2);
   Serial.print(F("Cas3:"));Serial.print(cas3_hod);Serial.print(cas3_min);Serial.println(doba3);
   Serial.print(F("CerpZAP:"));Serial.println(cerpadlo_on);
   Serial.print(F("CerpVYP:"));Serial.println(pauza_cerpadlo_off);
   Serial.print(F("PouzT:"));Serial.println(use_teplota);
   Serial.print(F("PoslT:"));Serial.println(last_teplota);
   Serial.print(F("PouzV:"));Serial.println(use_vlhkost);
   Serial.print(F("PoslV:"));Serial.println(last_vlhkost);
   Serial.print(F("Ventil:"));Serial.println(pln_nadrz);
#endif    
} // end void   

void clear_eeprom(){ // write 0 do vsech bytes EEPROM
    for (int i = 0; i < EEPROM.length(); i++){
      EEPROM.write(i, 0);  
      }
#ifdef DEBUG      
    Serial.println(F("CLREE"));  
#endif     
} // end void    

void save_eeprom() { // ulozi z lcd menu hodnoty
  write_eeprom_byte(adr_perioda_den, perioda_den);
  write_eeprom_byte(adr_cas1_hod, cas1_hod);
  write_eeprom_byte(adr_cas1_min, cas1_min);
  write_eeprom_byte(adr_doba1, doba1);
  write_eeprom_byte(adr_cas2_hod, cas2_hod);
  write_eeprom_byte(adr_cas2_min, cas2_min);
  write_eeprom_byte(adr_doba2, doba2);
  write_eeprom_byte(adr_cas3_hod, cas3_hod);
  write_eeprom_byte(adr_cas3_min, cas3_min);
  write_eeprom_byte(adr_doba3, doba3);
  write_eeprom_byte(adr_cerpadlo_on, cerpadlo_on);
  write_eeprom_byte(adr_pauza_cerpadlo_off, pauza_cerpadlo_off);
  write_eeprom_byte(adr_use_teplota, use_teplota);
  write_eeprom_byte(adr_use_vlhkost, use_vlhkost);
#ifdef DEBUG
  Serial.println(F("SAVE"));
#endif     

  save = false;
  lcd.noBlink(); // prestaneme blikat kurzorem
  lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
  lcd.print(F("Ulozeno...        "));  
  lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
  lcd.print(F("                  "));  
  delay(2000); // pockame na lcd text
  RTC_a_restart(); // pomocne funkce
} // end void  

//**********************************************************************************************
// klavesnice na analogovam vstupu *************************************************************
int read_LCD_buttons(){ // tato funkce cte hodnotu tlacitek
    adc_key_in = analogRead(button_pin);      // cteme hodnotu ze senzoru A0
    // stredni hodnota tlacitek je zhruba tato: 0, 144, 329, 504, 741
    // pridame cca 50 k temto hodnotam a zkontrolujeme zda jsme blizko
    if (adc_key_in > 1000) return btnNONE; // kontrolujeme nejprve tuto moznost, protoze to je nejcastejsi stav
    if (adc_key_in < 50)   return btnRIGHT;  
    if (adc_key_in < 195)  return btnUP;
    if (adc_key_in < 380)  return btnDOWN;
    if (adc_key_in < 555)  return btnLEFT;
    if (adc_key_in < 790)  return btnSELECT;  
    delay(1);
    return btnNONE;  // pokud nic z toho neodpovida, vracime tuto hodnotu
} //end read

void cti_klavesnici(){ // Rutina cte stiskle klavesy a meni hodnoty parametru
  lcd_key = read_LCD_buttons();  // precteme stav tlacitek   
  switch (lcd_key){               // dle toho, ktere je stisknuto vypiseme akci
   
   case btnRIGHT: // vpravo               
     if(btnPUSH) break; 
#ifdef DEBUG     
     Serial.println(F("VPRAVO"));  
#endif   
     if (menu_state==13) {LR_menu++; if (LR_menu >= 4) {LR_menu = 4;} lcd_menu(13); btnPUSH = true; break;} // posouvame vpravo v menu lcd cas a datum     
     if (menu_state==5) {LR_menu++; if (LR_menu >= 1) {LR_menu = 1;} lcd_menu(5); btnPUSH = true; break;} // posouvame vpravo v menu lcd cas 3     
     if (menu_state==3) {LR_menu++; if (LR_menu >= 1) {LR_menu = 1;} lcd_menu(3); btnPUSH = true; break;} // posouvame vpravo v menu lcd cas 2     
     if (menu_state==1) {LR_menu++; if (LR_menu >= 1) {LR_menu = 1;} lcd_menu(1); btnPUSH = true; break;} // posouvame vpravo v menu lcd cas 1        
     btnPUSH = true;                              
     break;
     
   case btnLEFT: // vlevo
     if(btnPUSH) break;
#ifdef DEBUG     
     Serial.println(F("VLEVO"));
#endif        
     if (menu_state==13) {LR_menu--; if (LR_menu <= 0) {LR_menu = 0;} lcd_menu(13); btnPUSH = true; break;} // posouvame vlevo v menu lcd cas a datum
     if (menu_state==5) {LR_menu--; if (LR_menu <= 0) {LR_menu = 0;} lcd_menu(5); btnPUSH = true; break;} // posouvame vlevo v menu lcd cas 3     
     if (menu_state==3) {LR_menu--; if (LR_menu <= 0) {LR_menu = 0;} lcd_menu(3); btnPUSH = true; break;} // posouvame vlevo v menu lcd cas 2     
     if (menu_state==1) {LR_menu--; if (LR_menu <= 0) {LR_menu = 0;} lcd_menu(1); btnPUSH = true; break;} // posouvame vlevo v menu lcd cas 1        
     btnPUSH = true;
     break;
     
   case btnUP: // nahoru
     if(btnPUSH) break;
#ifdef DEBUG     
     Serial.println(F("NAHORU"));
#endif        
     // save
     if (menu_state==20) {save = true; menu_state = -1; btnPUSHED=0; break;}  // blokujeme vypis na lcd  pouze do konce menu pak vyskocime a ulozime
     // smazat do default
     if (menu_state==19) {menu_state = -1;  clear_eeprom(); init_eeprom(); 
         vterina = 0; minuta = 0; hodina = 12; den = 1; mesic = 1; rok = 18;
         RTC_a_restart(); break;
         }  // blokujeme vypis na lcd  pouze do konce menu pak vyskocime a ulozime
     // datum cas
     if (menu_state==13) {if (LR_menu == 0) {if (den >= 31) {den = 1;} else {den++;} lcd_menu(13); btnPUSH = true; break;} }          // LR menu 0=den, 1=mesic, 2=rok, 3=hod, 4=min
     if (menu_state==13) {if (LR_menu == 1) {if (mesic >= 12) {mesic = 1;} else {mesic++;} lcd_menu(13); btnPUSH = true; break;} }    //         1=mesic
     if (menu_state==13) {if (LR_menu == 2) {if (rok >= 99) {rok = 18;} else {rok++;} lcd_menu(13); btnPUSH = true; break;} }         //         2=rok
     if (menu_state==13) {if (LR_menu == 3) {if (hodina >= 23) {hodina = 0;} else {hodina++;} lcd_menu(13); btnPUSH = true; break;} } //         3=hodina
     if (menu_state==13) {if (LR_menu == 4) {if (minuta >= 59) {minuta = 0;} else {minuta++;} lcd_menu(13); btnPUSH = true; break;} } //         4=minuta
     // use vlhkost
     if (menu_state==10) {if (use_vlhkost == 2) {use_vlhkost = 1;} lcd_menu(10); btnPUSH = true; break;} // true 2 /false 1
     // use teplota
     if (menu_state==9) {if (use_teplota  == 2) {use_teplota  = 1;} lcd_menu(9); btnPUSH = true; break;} // true 2 /false 1
     // preruseni zavlahy na dobu
     if (menu_state==8) {if (pauza_cerpadlo_off >= 250) {pauza_cerpadlo_off = 0;} else {pauza_cerpadlo_off++;} lcd_menu(8); btnPUSH = true; break;} // krok 1 sec
     // preruseni zavlahy kazde
     if (menu_state==7) {if (cerpadlo_on >= 250) {cerpadlo_on = 0;} else {cerpadlo_on++;} lcd_menu(7); btnPUSH = true; break;} // krok 1 sec
     // interval cas 3
     if (menu_state==6) {if (doba3 >= 250) {doba3 = 5;} else {doba3+=5;} lcd_menu(6); btnPUSH = true; break;} // krok 5 sec
     // cas spinani 3
     if (menu_state==5) {if (LR_menu == 0) {if (cas3_hod >= 23) {cas3_hod = 0;} else {cas3_hod++;} lcd_menu(5); btnPUSH = true; break;} }          // LR menu 0=hod,
     if (menu_state==5) {if (LR_menu == 1) {if (cas3_min >= 59) {cas3_min = 0;} else {cas3_min++;} lcd_menu(5); btnPUSH = true; break;} }          // LR menu 0=min,
     // interval cas 2
     if (menu_state==4) {if (doba2 >= 250) {doba2 = 5;} else {doba2+=5;} lcd_menu(4); btnPUSH = true; break;} // krok 5 sec
     // cas spinani 2
     if (menu_state==3) {if (LR_menu == 0) {if (cas2_hod >= 23) {cas2_hod = 0;} else {cas2_hod++;} lcd_menu(3); btnPUSH = true; break;} }          // LR menu 0=hod,
     if (menu_state==3) {if (LR_menu == 1) {if (cas2_min >= 59) {cas2_min = 0;} else {cas2_min++;} lcd_menu(3); btnPUSH = true; break;} }          // LR menu 0=min,
     // interval cas 1
     if (menu_state==2) {if (doba1 >= 250) {doba1 = 5;} else {doba1+=5;} lcd_menu(2); btnPUSH = true; break;} // krok 5 sec
     // cas spinani 1
     if (menu_state==1) {if (LR_menu == 0) {if (cas1_hod >= 23) {cas1_hod = 0;} else {cas1_hod++;} lcd_menu(1); btnPUSH = true; break;} }          // LR menu 0=hod,
     if (menu_state==1) {if (LR_menu == 1) {if (cas1_min >= 59) {cas1_min = 0;} else {cas1_min++;} lcd_menu(1); btnPUSH = true; break;} }          // LR menu 0=min,
     // perioda dnu 1-7
     if (menu_state==0) {if (perioda_den >= 7) {perioda_den = 1;} else {perioda_den++;} lcd_menu(0); btnPUSH = true; break;} // krok 1 den
     btnPUSH = true;
     break;
     
   case btnDOWN: // dolu
     if(btnPUSH) break;
#ifdef DEBUG     
     Serial.println(F("DOLU"));
#endif        
     // save
     if (menu_state==20) {menu = false; save = false; lcd.noBlink(); menu_state = -1; btnPUSHED=0; break;}  // blokujeme vypis na lcd  pouze do konce menu pak vyskocime  
     // smazat do default
     if (menu_state==19) {menu = false; save = false; lcd.noBlink(); menu_state = -1; btnPUSHED=0; break;}  // blokujeme vypis na lcd  pouze do konce menu pak vyskocime 
     // datum cas
     if (menu_state==13) {if (LR_menu == 0) {if (den <= 1) {den = 31;} else {den--;} lcd_menu(13); btnPUSH = true; break;} }          // LR menu 0=den, 1=mesic, 2=rok, 3=hod, 4=min
     if (menu_state==13) {if (LR_menu == 1) {if (mesic <= 1) {mesic = 12;} else {mesic--;} lcd_menu(13); btnPUSH = true; break;} }    //         1=mesic
     if (menu_state==13) {if (LR_menu == 2) {if (rok <= 18) {rok = 99;} else {rok--;} lcd_menu(13); btnPUSH = true; break;} }         //         2=rok
     if (menu_state==13) {if (LR_menu == 3) {if (hodina <= 0) {hodina = 23;} else {hodina--;} lcd_menu(13); btnPUSH = true; break;} } //         3=hodina
     if (menu_state==13) {if (LR_menu == 4) {if (minuta <= 0) {minuta = 59;} else {minuta--;} lcd_menu(13); btnPUSH = true; break;} } //         4=minuta
     // use vlhkost
     if (menu_state==10) {if (use_vlhkost == 1) {use_vlhkost = 2;} lcd_menu(10); btnPUSH = true; break;} // true 2 /false 1
     // use teplota
     if (menu_state==9) {if (use_teplota  == 1) {use_teplota  = 2;} lcd_menu(9); btnPUSH = true; break;} // true 2/false 1
     // preruseni zavlahy na dobu
     if (menu_state==8) {if (pauza_cerpadlo_off <= 0) {pauza_cerpadlo_off = 250;} else {pauza_cerpadlo_off--;} lcd_menu(8); btnPUSH = true; break;} // krok 1 sec
     // preruseni zavlahy kazde
     if (menu_state==7) {if (cerpadlo_on <= 0) {cerpadlo_on = 250;} else {cerpadlo_on--;} lcd_menu(7); btnPUSH = true; break;} // krok 1 sec
     // interval cas 3
     if (menu_state==6) {if (doba3 <= 5) {doba3 = 250;} else {doba3-=5;} lcd_menu(6); btnPUSH = true; break;} // krok 5 sec
     // cas spinani 3
     if (menu_state==5) {if (LR_menu == 0) {if (cas3_hod <= 0) {cas3_hod = 23;} else {cas3_hod--;} lcd_menu(5); btnPUSH = true; break;} }          // LR menu 0=hod,
     if (menu_state==5) {if (LR_menu == 1) {if (cas3_min <= 0) {cas3_min = 59;} else {cas3_min--;} lcd_menu(5); btnPUSH = true; break;} }          // LR menu 0=min,
     // interval cas 2
     if (menu_state==4) {if (doba2 <= 5) {doba2 = 250;} else {doba2-=5;} lcd_menu(4); btnPUSH = true; break;} // krok 5 sec
     // cas spinani 2
     if (menu_state==3) {if (LR_menu == 0) {if (cas2_hod <= 0) {cas2_hod = 23;} else {cas2_hod--;} lcd_menu(3); btnPUSH = true; break;} }          // LR menu 0=hod,
     if (menu_state==3) {if (LR_menu == 1) {if (cas2_min <= 0) {cas2_min = 59;} else {cas2_min--;} lcd_menu(3); btnPUSH = true; break;} }          // LR menu 0=min,
     // interval cas 1
     if (menu_state==2) {if (doba1 <= 5) {doba1 = 250;} else {doba1-=5;} lcd_menu(2); btnPUSH = true; break;} // krok 5 sec
     // cas spinani 1
     if (menu_state==1) {if (LR_menu == 0) {if (cas1_hod <= 0) {cas1_hod = 23;} else {cas1_hod--;} lcd_menu(1); btnPUSH = true; break;} }          // LR menu 0=hod,
     if (menu_state==1) {if (LR_menu == 1) {if (cas1_min <= 0) {cas1_min = 59;} else {cas1_min--;} lcd_menu(1); btnPUSH = true; break;} }          // LR menu 0=min,
     // perioda dnu 1-7
     if (menu_state==0) {if (perioda_den <= 1) {perioda_den = 7;} else {perioda_den--;} lcd_menu(0); btnPUSH = true; break;} // krok 1 den
     btnPUSH = true;
     break;
     
   case btnSELECT: // menu (select)
     if(btnPUSH) break;
#ifdef DEBUG     
     Serial.print(F("MENU "));     
     Serial.println(btnPUSHED);
#endif       
     menu = true;
     lcd_menu(btnPUSHED); // posouvame menu
     if (btnPUSHED == 1 || btnPUSHED == 3 || btnPUSHED == 5 || btnPUSHED == 13) {LR_menu = 0;  lcd_menu(btnPUSHED); } // vratime se na zacatek tj doleva
     btnPUSHED++;
     btnPUSH = true;
     break;
    
   case btnNONE:
     btnPUSH = false;
     break;
    
   } // end switch
 } //end void
 
void lcd_menu(byte krok) // Rutina tiskne texty menu na LCD)
{ 
  switch(krok){
    case 0: // interval spinani pudy
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(F("Interval spinani"));  
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      lcd.print(perioda_den);
      lcd.print(F(" [1-7 den]          "));    
      menu_state = 0;
      lcd.setCursor(0,1); lcd.blink();
      break;
    case 1: // cas spinani 1
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(F("Cas spinani 1   "));  
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      print_digits_lcd(cas1_hod);
      lcd.print(F(":"));
      print_digits_lcd(cas1_min);
      lcd.print(F(" [h:m]           "));
      menu_state = 1;
      if (LR_menu==0) {lcd.setCursor(1,1); lcd.blink();} // zacne blikat kurzor hod
      if (LR_menu==1) {lcd.setCursor(4,1); lcd.blink();} // zacne blikat kurzor min
      break; 
    case 2: // interval cas1
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(F("Interval cas 1 "));
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      print_digits_lcd(doba1);
      lcd.print(F(" [5-250 sec]     "));
      menu_state = 2;
      lcd.setCursor(1,1); lcd.blink();
      break;   
    case 3: // cas spinani 2
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(F("Cas spinani 2   "));  
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      print_digits_lcd(cas2_hod);
      lcd.print(F(":"));
      print_digits_lcd(cas2_min);
      lcd.print(F(" [h:m]           "));
      menu_state = 3;
      if (LR_menu==0) {lcd.setCursor(1,1); lcd.blink();} // zacne blikat kurzor hod
      if (LR_menu==1) {lcd.setCursor(4,1); lcd.blink();} // zacne blikat kurzor min
      break; 
    case 4: // interval cas 2
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(F("Interval cas 2 "));
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      print_digits_lcd(doba2);
      lcd.print(F(" [5-250 sec]     ")); 
      menu_state = 4;
      lcd.setCursor(1,1); lcd.blink();
      break;
    case 5: // cas spinani 3
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(F("Cas spinani 3   "));  
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      print_digits_lcd(cas3_hod);
      lcd.print(F(":"));
      print_digits_lcd(cas3_min);
      lcd.print(F(" [h:m]           "));
      menu_state = 5;
      if (LR_menu==0) {lcd.setCursor(1,1); lcd.blink();} // zacne blikat kurzor hod
      if (LR_menu==1) {lcd.setCursor(4,1); lcd.blink();} // zacne blikat kurzor min
      break; 
    case 6: // interval cas 3
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(F("Interval cas 3 "));
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      print_digits_lcd(doba3);
      lcd.print(F(" [5-250 sec]     ")); 
      menu_state = 6;
      lcd.setCursor(1,1); lcd.blink();
      break;
    case 7: // preruseni zavlahy kazde
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(F("Preruseni kazde "));
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      print_digits_lcd(cerpadlo_on);
      lcd.print(F(" [0-250 sec]     ")); 
      menu_state = 7;
      lcd.setCursor(1,1); lcd.blink();
      break; 
    case 8: // preruseni zavlahy na dobu
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(F("Preruseni doba "));
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      lcd.print(pauza_cerpadlo_off);
      lcd.print(F(" [0-250 sec]     ")); 
      menu_state = 8;
      lcd.setCursor(0,1); lcd.blink();
      break;    
    case 9: // pouzivat teplotu
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(F("Pouzit teplotu  "));
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      if (use_teplota==2){  
        lcd.print(F("ANO ["));
        lcd.write((byte) 5); // symbol sipka dolu
        lcd.print(F("=NE]         "));
        } 
        else {
          lcd.print(F("NE ["));
          lcd.write((byte) 4); // symbol sipka 
          lcd.print(F("=ANO]            "));
          }
      menu_state = 9;
      lcd.setCursor(0,1); lcd.blink();
      break;    
    case 10: // pouzivat vlhkost
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(F("Pouzit vlhkost "));
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      if (use_vlhkost==2){  
        lcd.print(F("ANO ["));
        lcd.write((byte) 5); // symbol sipka dolu
        lcd.print(F("=NE]         "));
        } 
        else {
          lcd.print(F("NE ["));
          lcd.write((byte) 4); // symbol sipka 
          lcd.print(F("=ANO]            "));
          }
      menu_state = 10;
      lcd.setCursor(0,1); lcd.blink();
      break;
// todo upravit
   case 13: // datum cas
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(F("Nastaveni casu  "));
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      lcd.print(F("                "));
      lcd.setCursor(0,1); 
      print_digits_lcd(den);
      lcd.setCursor(2,1); 
      lcd.write('.');
      print_digits_lcd(mesic);
      lcd.print(F("     "));
      lcd.setCursor(5,1); 
      lcd.write('.');
      print_digits_lcd(rok);  
      lcd.print(F("   "));
      lcd.setCursor(11,1);
      print_digits_lcd(hodina);
      lcd.setCursor(13,1); 
      lcd.write(':');
      print_digits_lcd(minuta);
      lcd.print(F("  ")); 
      menu_state = 13;
      vterina = 0;
      if (LR_menu==0) {lcd.setCursor(1,1); lcd.blink();} // zacne blikat kurzor den
      if (LR_menu==1) {lcd.setCursor(4,1); lcd.blink();} // zacne blikat kurzor mesic
      if (LR_menu==2) {lcd.setCursor(7,1); lcd.blink();} // zacne blikat kurzor rok
      if (LR_menu==3) {lcd.setCursor(12,1); lcd.blink();} // zacne blikat kurzor hod
      if (LR_menu==4) {lcd.setCursor(15,1); lcd.blink();} // zacne blikat kurzor min
      break; 
   case 19: // dotaz smazat
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(F("SMAZAT nastaveni "));
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      lcd.print(F("? [")); 
      lcd.write((byte) 4); // symbol sipka dolu
      lcd.print(F("=NE "));
      lcd.write((byte) 5); // symbol sipka 
      lcd.print(F("=ANO]        ")); 
      menu_state = 19;
      lcd.setCursor(0,1); lcd.blink();
      break;           
   case 20: // dotaz ulozit
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(F("ULOZIT nastaveni "));
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      lcd.print(F("? [")); 
      lcd.write((byte) 4); // symbol sipka dolu
      lcd.print(F("=NE "));
      lcd.write((byte) 5); // symbol sipka 
      lcd.print(F("=ANO]        ")); 
      menu_state = 20;
      lcd.setCursor(0,1); lcd.blink();
      break;        
   case 21: // znovu
      btnPUSHED = 0;  
      lcd_menu(btnPUSHED);
      break;  
    } // switch
} // end void    
  
//**********************************************************************************************
// pomocne funkce ******************************************************************************
void vypisuj_info_lcd(){ // informace na lcd
// vzor lcd.print("                "); 16 znaku
  if (menu == false){// budeme vypisovat jen kdyz neni aktivni menu a neni prace se serialem
       switch(posun_lcd){ // informacni texty na lcd pouze kdyz neni menu
         // mnozstvi vody       
         case 0:
         // teplota
           tisk_casu();
           if (blokuj_mereni_teploty){ // pokud prave bezi cas 1-3
              lcd.setCursor(0,1);
              lcd.print(F("Prave zalevam     "));
              } else { // else 1
                 lcd.setCursor(0,1);
                 lcd.write((byte) 1); // prvni symbol v pameti lcd - ikona teploty
                 lcd.print(F(" "));
                 if (celsius > -100){
                    lcd.print(celsius,1);
                    lcd.print(F(" ["));
                    lcd.write((byte) 0); // nulty symbol v pameti lcd - krouzek C
                    lcd.print(F("C]     "));
                    }//end if
                 else { lcd.print(F("Chyba cidla!    "));}
              }//end else   
           break;
         case 1:
         // vlhkost
           tisk_casu();
           if (blokuj_mereni_vlhka){ // pokud prave bezi cas 1-3
              lcd.setCursor(0,1);
              lcd.print(F("Prave zalevam      "));
              } else { // else 1
                lcd.setCursor(0,1);
                lcd.write((byte) 2); // druhy symbol v pameti lcd - ikona kapka
                lcd.print(F(" "));
                lcd.print(vlhkost);
                lcd.print(F(" ["));
                if(frq<1000)  {lcd.print(frq); lcd.print(F("Hz]    "));}
                if(frq>=1000) {lcd.print(frq/1000); lcd.print(F("kHz]    "));}
                  } 
           break;
         case 2:
         // cerpadlo
            if (cerpadlo_makej){ // kdyz je cerpadlo v provozu
              lcd.setCursor(0,1);
              lcd.print(F("Cerp "));
              if (cerpadlo_vystup_stav) {lcd.print(F("VYP "));} else {lcd.print(F("ZAP "));}
              long odpocet_cerpadlo = 0;
              odpocet_cerpadlo = ((predchoziCasCerpadlo+interval)-aktualniCasCerpadlo)/1000; // cas v sec
              lcd.print(odpocet_cerpadlo); lcd.print(F(" s       ")); // zobrazi cas v sec
              } else {
                      posun_lcd=3;
                      vypisuj_info_lcd();
                      }
           break;       
         case 3:
         // info odpocet do startu ventilu
         tisk_casu();
           if (ventil_odpocet){ // pokud je aktivni casovac na doplneni nadrze (na konci zalevani)
              lcd.setCursor(0,1);
              lcd.print(F("Plnim za ")); 
              long odpocet_plneni = 0;
              odpocet_plneni = ((predchoziCasVentil+(pln_nadrz*60000))-aktualniCasVentil)/1000; // cas v sec
              if (odpocet_plneni >= 60)  {
                lcd.print(odpocet_plneni/60); lcd.print(F(" min    ")); // zobrazi cas v min
                } else {
                   lcd.print(odpocet_plneni); lcd.print(F(" sec     ")); } // zobrazi cas v sec
              } else {
                      posun_lcd=4;
                      vypisuj_info_lcd();
                     } // end else      
           break;    
         case 4:
          // den 1-7
         tisk_casu();
           lcd.setCursor(0,1);
           lcd.print(F("Dnes je ")); 
           lcd.print(perioda_posun);
           lcd.print(F(" den         "));
           posun_lcd=-1; // vynulujeme posun na zacatek
           break;      
         } // end switch   
    } //end if not menu
} // end void

void cti_vlhkost(){
#ifdef DEBUG     
   Serial.print(F("FREK:"));
   Serial.print(frq);
   Serial.print(F(" VLHK:"));
   Serial.println(vlhkost);
#endif  
} // end void


long mapuj(long x, long in_min, long in_max, long out_min, long out_max) // vraci prevedenou hodnotu tj funkce map
 {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 } // end long

void inicializace(){ // inicializace knihoven
#ifdef DEBUG 
  Serial.begin(baud_rate);      // Inicializace Serialu
  delay(500);
#endif  
  
  lcd.begin(16, 2);             // Inicializace LCD 16x2 znaku
  lcd.createChar(0, stupen);    // vytvorime v lcd znak stupne
  lcd.createChar(1, teplomer);  // vytvorime v lcd ikonu teplomeru
  lcd.createChar(2, kapka);     // vytvorime v lcd ikonu vlhka
  lcd.createChar(3, jas);       // vytvorime v lcd ikonu jasu
  lcd.createChar(4, sipka_dolu);       // vytvorime v lcd ikonu sipky dolu
  lcd.createChar(5, sipka_nahoru);     // vytvorime v lcd ikonu sipky nahoru
  lcd.createChar(6, hladina);   // vytvorime v lcd ikonu hladiny
} // end void

void cti_cas_datum(){
  den = day();
  mesic = month();
  rok = year()-2000;  // aby bylo cislo 18 - xx misto 2018
  hodina = hour();
  minuta = minute();
} // end void

void init_piny(){ // nystavy vstupy a vystupy
  pinMode(cerpadlo_pin,OUTPUT);    // Vystup pro cerpadlo 
  digitalWrite(cerpadlo_pin,LOW);  // Vypneme vystup cerpadlo
  pinMode(ventil_pin,OUTPUT);      // Vystup pro ventil vody 
  digitalWrite(ventil_pin,LOW);    // Vypneme vystup ventilu
  pinMode(frek_pin,INPUT);         // Vstup pro citac frekvence na T1 cpu atmega
  pinMode(lcd_led,OUTPUT);         // LED LCD vystup
  digitalWrite(lcd_led,HIGH);      // zapneme LED 
} // end void 

void firmware(){ // vypise reklamu a verzi fw na serial a lcd
  lcd.setCursor(0,0);                 // 1 radek LCD displeje
  lcd.print(F("www.pihrt.com   "));   // Reklama
  lcd.setCursor(0,1);                 // 2 radek LCD displeje
  lcd.print(verze);                   // Verze programu zarizeni
#ifdef DEBUG   
  Serial.println(F("MENU NL&CR:"));
  Serial.println(F("? help")); 
  Serial.println(F("A run cas1"));
  Serial.println(F("B run cas2"));
  Serial.println(F("C run cas3"));
  Serial.println(F("D date"));
  Serial.println(F("E period off"));
  Serial.println(F("F EE default"));
  Serial.println(F("G EE print"));
  Serial.println(F("I frek"));
#endif  
  delay(2000);
} // end void 

void nacti_RTC(){ // nacte cas z RTC a vytiskne na lcd (serial) pripadne chybu   
   setSyncProvider(RTC.get);   // nastaveni syncu casu z RTC
   setSyncInterval(86400); // za jak dlouho se zavola sync z RTC
      //24h*60m*60s=86400
   if (timeStatus() != timeSet) { // skok do menu nastavit cas
#ifdef DEBUG        
      Serial.println(F("RTC?"));
#endif     
      menu = true;         // povolime menu
      btnPUSHED = 13;      // v menu je 13 cas a datum
      LR_menu = 0;         // vratime kruzor doleva
      lcd_menu(btnPUSHED); // vytiskneme polozku z menu
      } // end if    
} // end void

void tisk_casu(){ // tiskne na prvni radek datum a cas
  lcd.setCursor(0,0);
  print_digits_lcd(day());
  lcd.write('.');
  print_digits_lcd(month());
  lcd.write('.');
  lcd.print(year());  
  lcd.print(F("  "));
  lcd.setCursor(11,0);
  print_digits_lcd(hour());
  lcd.write(':');
  print_digits_lcd(minute());
  lcd.print(F(" "));
  serial_clock_display(); // tisk casu na serial
} // end void 

void serial_clock_display(){ // tisk aktualniho systemoveho casu na serial
#ifdef DEBUG  
  print_digits(hour());
  Serial.write(':');
  print_digits(minute());
  Serial.write(':');
  print_digits(second());
  Serial.println(); 
#endif    
} //end void

void print_digits(int number) { // dodava na serial nulu kdyz je mensi nez 10
#ifdef DEBUG  
  if (number >= 0 && number < 10) {
    Serial.write('0');}
  Serial.print(number);
#endif    
}// end void

void print_digits_lcd(int number) { // dodava na lcd nulu kdyz je mensi nez 10
  if (number >= 0 && number < 10) {
    lcd.write('0');}
  lcd.print(number);
}// end void

void RTC_a_restart(){ // ulozi do rtc a udela restart
#ifdef DEBUG 
   Serial.println(F("RBT"));
#endif     
   lcd.home();
   lcd.print(F("Restartuji se..."));
   lcd.setCursor(0,1);
   lcd.print("                ");
   setTime(hodina,minuta,vterina,den,mesic,rok);
   RTC.set(now());
   delay(4500); // pockame na watchdog az to sestreli
} // end void   

void plneni_nadrze(){ // po skonceni zalevani se ceka (v menu "nacerpat vodu" 60-360 min) a pak pokud je voda low sepne ventil do doby voda high
 if (ventil_odpocet){ // pokud skoncilo zalevani spusti se odpocet
    aktualniCasVentil = millis(); // nacteme aktualni cas v ms
    
    if (aktualniCasVentil - predchoziCasVentil >= (pln_nadrz*60000)){ // pokud ubehl cas pro doplneni nadrze *1000 = sec, *60 = min
       predchoziCasVentil = aktualniCasVentil;
       ventil_zap = true; // povolime plneni
       ventil_odpocet = false; // zakazeme casovac     
       } // end if 
    } // end if ventil_odpocet 

 if (ventil_zap){ // je povoleno plnit nadrz
      digitalWrite(ventil_pin,HIGH); 
#ifdef DEBUG        
       Serial.println(F("VENTIL ZAP"));
#endif          
   } // end ventil_zap   
} // end void

void cerpadlo(){ // rutina spinani cerpadla cas on a cas off
 if (cerpadlo_makej){
   // http://www.baldengineer.com/millis-ind-on-off-times.html
   aktualniCasCerpadlo = millis(); // nacteme aktualni cas v ms
   
   digitalWrite(cerpadlo_pin,cerpadlo_vystup_stav); // cerpadlo sepne nebo vypne dle stavu  
        
   if (aktualniCasCerpadlo - predchoziCasCerpadlo >= interval){ // pokud ubehl cas pro sepnuti nebo vypnuti cerpadla (60000 ms je minuta)
     if (cerpadlo_vystup_stav){
         interval = cerpadlo_on*1000;               // on time cerpadla
         } else {
         interval = pauza_cerpadlo_off*1000;        // off time cerpadla
                } // end else
#ifdef DEBUG                  
     Serial.print(F("CERPADLO: "));
     Serial.println(cerpadlo_vystup_stav);     
#endif       
     cerpadlo_vystup_stav = !(cerpadlo_vystup_stav);
     predchoziCasCerpadlo = aktualniCasCerpadlo;
     } // end if aktualniCasCerpdalo
   } // end if cerpadlo makej
    else {
        digitalWrite(cerpadlo_pin,LOW); // cerpadlo vypne protoze neni povolenej stav cerpadlo_makej
         } // end else
} // end void  

void regulace_vlhk(){ // reguluje dle vlhkosti
  if (perioda_je_tu && use_vlhkost==2){ // kdyz je jiz dany x-ty den (toto nastavuje casovac krok 86400 sec) a je povolena vlhkost
      if (vlhkost < 4){ // zmerena vlhkost je 1,2,3 (na stupnici 0-10)
         if (CAS1 || CAS2 || CAS3){     // je prave cas1, cas2, cas3 na casovaci
            blokuj_mereni_vlhka = true; // zakazeme merit vlhkost po dobu zalevani casu1 a na lcd vypisujeme ze bezi cas 1 namisto vlhkosti
            cerpadlo_makej = true;      // povolime cerpadlo    
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)  
            if (aktualniCasDoba - predchoziCasDoba >= trvani_doby+(trvani_doby/2)){ // pocitame dobu1 zalevani a pak vse vypneme + 50% casu zalevani
                predchoziCasDoba = aktualniCasDoba;                  
                CAS1 = false;             // zakazeme cas1
                CAS2 = false;             // zakazeme cas2
                CAS3 = false;             // zakazeme cas2
                cerpadlo_makej = false;   // zakazeme cerpadlo 
                ventil_odpocet = true;    // povolime plneni nadrze po skonceni zalevani
                predchoziCasVentil = millis(); // start casovace odpoctu plneni od aktualniho casu
                write_eeprom_long(adr_last_vlhkost, vlhkost); // ulozime vlhkost do eeprom na konci casu1
                Serial_ende();
                blokuj_mereni_vlhka = false; // povolime na lcd mereni vlhkosti a v casovaci mereni vlhka   
                blokuj_mereni_teploty = false;         
                } // end if casovac
            } // end if CAS 1 2 3 - delsi interval
         } // end vlhkost  
      if (vlhkost >= 4 && vlhkost < 6){ // zmerena vlhkost je 4,5 (na stupnici 0-10)
         if (CAS1 || CAS2 || CAS3){     // je prave cas1, cas2, cas3 na casovaci
            blokuj_mereni_vlhka = true; // zakazeme merit vlhkost po dobu zalevani casu1 a na lcd vypisujeme ze bezi cas 1 namisto vlhkosti
            cerpadlo_makej = true;      // povolime cerpadlo    
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)  
            if (aktualniCasDoba - predchoziCasDoba >= trvani_doby){ // pocitame dobu1 zalevani a pak vse vypneme
                predchoziCasDoba = aktualniCasDoba;                  
                CAS1 = false;             // zakazeme cas1
                CAS2 = false;             // zakazeme cas2
                CAS3 = false;             // zakazeme cas2
                cerpadlo_makej = false;   // zakazeme cerpadlo 
                ventil_odpocet = true;    // povolime plneni nadrze po skonceni zalevani
                predchoziCasVentil = millis(); // start casovace odpoctu plneni od aktualniho casu
                write_eeprom_long(adr_last_vlhkost, vlhkost); // ulozime vlhkost do eeprom na konci casu123
                Serial_ende();
                blokuj_mereni_vlhka = false; // povolime na lcd mereni vlhkosti a v casovaci mereni vlhka 
                blokuj_mereni_teploty = false;        
                } // end if casovac
            } // end if CAS 1 2 3
         } // end vlhkost  
      if (vlhkost >= 6 && vlhkost < 8){ // zmerena vlhkost je 6,7 (na stupnici 0-10)
         if (CAS1 || CAS2){             // je prave cas1 nebo cas 2 na casovaci
            blokuj_mereni_vlhka = true; // zakazeme merit vlhkost po dobu zalevani casu1 a na lcd vypisujeme ze bezi cas 1 namisto vlhkosti
            cerpadlo_makej = true;      // povolime cerpadlo    
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)  
            if (aktualniCasDoba - predchoziCasDoba >= trvani_doby){ // pocitame dobu1 zalevani a pak vse vypneme
                predchoziCasDoba = aktualniCasDoba;                  
                CAS1 = false;             // zakazeme cas1
                CAS2 = false;             // zakazeme cas2
                cerpadlo_makej = false;   // zakazeme cerpadlo 
                ventil_odpocet = true;    // povolime plneni nadrze po skonceni zalevani
                predchoziCasVentil = millis(); // start casovace odpoctu plneni od aktualniho casu
                write_eeprom_long(adr_last_vlhkost, vlhkost); // ulozime vlhkost do eeprom na konci casu12
                Serial_ende();
                blokuj_mereni_vlhka = false; // povolime na lcd mereni vlhkosti a v casovaci mereni vlhka  
                blokuj_mereni_teploty = false;       
                } // end if casovac
            } // end if CAS 1 a 2
         } // end vlhkost   
      if (vlhkost >= 8){                // zmerena vlhkost je > 8 (na stupnici 0-10)
         if (CAS1){                     // je prave cas1 na casovaci
            blokuj_mereni_vlhka = true; // zakazeme merit vlhkost po dobu zalevani casu1 a na lcd vypisujeme ze bezi cas 1 namisto vlhkosti
            cerpadlo_makej = true;      // povolime cerpadlo    
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)  
            if (aktualniCasDoba - predchoziCasDoba >= trvani_doby){ // pocitame dobu1 zalevani a pak vse vypneme
                predchoziCasDoba = aktualniCasDoba;                  
                CAS1 = false;             // zakazeme cas1
                cerpadlo_makej = false;   // zakazeme cerpadlo 
                ventil_odpocet = true;    // povolime plneni nadrze po skonceni zalevani
                predchoziCasVentil = millis(); // start casovace odpoctu plneni od aktualniho casu
                write_eeprom_long(adr_last_vlhkost, vlhkost); // ulozime vlhkost do eeprom na konci casu1
                Serial_ende();
                blokuj_mereni_vlhka = false; // povolime na lcd mereni vlhkosti a v casovaci mereni vlhka  
                blokuj_mereni_teploty = false;       
                } // end if casovac
            } // end if CAS1
         } // end vlhkost      
      } // end perioda   
} // end void  

void regulace_tepl(){ // reguluje dle teploty
  if (perioda_je_tu && use_teplota==2){ // kdyz je jiz dany x-ty den (toto nastavuje casovac krok 86400 sec) a je povolena teplota
      if (celsius >= 40.0){ // zmerena teplota je > 40 stupnu
         if (CAS1 || CAS2 || CAS3){     // je prave cas1, cas2, cas3 na casovaci
            blokuj_mereni_teploty = true; // zakazeme merit teplotu po dobu zalevani casu1 a na lcd vypisujeme ze bezi cas 1 namisto teploty
            cerpadlo_makej = true;      // povolime cerpadlo    
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)  
            if (aktualniCasDoba - predchoziCasDoba >= trvani_doby+(trvani_doby/2)){ // pocitame dobu1 zalevani a pak vse vypneme + 50% casu zalevani
                predchoziCasDoba = aktualniCasDoba;                  
                CAS1 = false;             // zakazeme cas1
                CAS2 = false;             // zakazeme cas2
                CAS3 = false;             // zakazeme cas2
                cerpadlo_makej = false;   // zakazeme cerpadlo 
                ventil_odpocet = true;    // povolime plneni nadrze po skonceni zalevani
                predchoziCasVentil = millis(); // start casovace odpoctu plneni od aktualniho casu
                write_eeprom_long(adr_last_teplota, int(celsius*1000)); // ulozime teplotu do eeprom na konci casu123 jako pr: 23.57C = 2357 cislo
                Serial_ende();
                blokuj_mereni_teploty = false; // povolime na lcd mereni teploty a v casovaci mereni teploty   
                blokuj_mereni_vlhka = false;     
                } // end if casovac
            } // end if CAS 1 2 3 - delsi interval
         } // end teplota  
      if (celsius >= 30.0 && celsius < 40.0){ // zmerena teplota je 30-40 stupnu
         if (CAS1 || CAS2 || CAS3){     // je prave cas1, cas2, cas3 na casovaci
            blokuj_mereni_teploty = true; // zakazeme merit teplotu po dobu zalevani casu1 a na lcd vypisujeme ze bezi cas 1 namisto teploty
            cerpadlo_makej = true;      // povolime cerpadlo    
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)  
            if (aktualniCasDoba - predchoziCasDoba >= trvani_doby){ // pocitame dobu1 zalevani a pak vse vypneme 
                predchoziCasDoba = aktualniCasDoba;                  
                CAS1 = false;             // zakazeme cas1
                CAS2 = false;             // zakazeme cas2
                CAS3 = false;             // zakazeme cas2
                cerpadlo_makej = false;   // zakazeme cerpadlo 
                ventil_odpocet = true;    // povolime plneni nadrze po skonceni zalevani
                predchoziCasVentil = millis(); // start casovace odpoctu plneni od aktualniho casu
                write_eeprom_long(adr_last_teplota, int(celsius*1000)); // ulozime teplotu do eeprom na konci casu123 jako pr: 23.57C = 2357 cislo
                Serial_ende();
                blokuj_mereni_teploty = false; // povolime na lcd mereni teploty a v casovaci mereni teploty    
                blokuj_mereni_vlhka = false;    
                } // end if casovac
            } // end if CAS 1 2 3 
         } // end teplota   
      if (celsius >= 20.0 && celsius < 30.0){ // zmerena teplota je 20-30 stupnu
         if (CAS1 || CAS2){     // je prave cas1, cas2
            blokuj_mereni_teploty = true; // zakazeme merit teplotu po dobu zalevani casu1 a na lcd vypisujeme ze bezi cas 1 namisto teploty
            cerpadlo_makej = true;      // povolime cerpadlo    
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)  
            if (aktualniCasDoba - predchoziCasDoba >= trvani_doby){ // pocitame dobu1 zalevani a pak vse vypneme 
                predchoziCasDoba = aktualniCasDoba;                  
                CAS1 = false;             // zakazeme cas1
                CAS2 = false;             // zakazeme cas2
                cerpadlo_makej = false;   // zakazeme cerpadlo 
                ventil_odpocet = true;    // povolime plneni nadrze po skonceni zalevani
                predchoziCasVentil = millis(); // start casovace odpoctu plneni od aktualniho casu
                write_eeprom_long(adr_last_teplota, int(celsius*1000)); // ulozime teplotu do eeprom na konci casu12 jako pr: 23.57C = 2357 cislo
                Serial_ende();
                blokuj_mereni_teploty = false; // povolime na lcd mereni teploty a v casovaci mereni teploty  
                blokuj_mereni_vlhka = false;      
                } // end if casovac
            } // end if CAS 1 2 
         } // end teplota   
      if (celsius >= 5.0 && celsius < 20.0){ // zmerena teplota je 5-20 stupnu
         if (CAS1){     // je prave cas1
            blokuj_mereni_teploty = true; // zakazeme merit teplotu po dobu zalevani casu1 a na lcd vypisujeme ze bezi cas 1 namisto teploty
            cerpadlo_makej = true;      // povolime cerpadlo    
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)  
            if (aktualniCasDoba - predchoziCasDoba >= trvani_doby){ // pocitame dobu1 zalevani a pak vse vypneme 
                predchoziCasDoba = aktualniCasDoba;                  
                CAS1 = false;             // zakazeme cas1
                cerpadlo_makej = false;   // zakazeme cerpadlo 
                ventil_odpocet = true;    // povolime plneni nadrze po skonceni zalevani
                predchoziCasVentil = millis(); // start casovace odpoctu plneni od aktualniho casu
                write_eeprom_long(adr_last_teplota, int(celsius*1000)); // ulozime teplotu do eeprom na konci casu1 jako pr: 23.57C = 2357 cislo
                Serial_ende();
                blokuj_mereni_teploty = false; // povolime na lcd mereni teploty a v casovaci mereni teploty   
                blokuj_mereni_vlhka = false;     
                } // end if casovac
            } // end if CAS 1 
         } // end teplota       
      } // end perioda   
} // end void 

void Serial_ende(){ // tiskne end
#ifdef DEBUG   
  Serial.println(F("END"));
#endif   
}//end void 

void init_teplota(){
  sensors.begin();    // pro DS18B20
}//end void
    
void cti_teplotu(){                      // #include <DallasTemperature.h> pomoci teto knihovny se neceka 1000ms ale meri se hned :-)
   sensors.requestTemperatures();        // pozadavek na zmereni teploty
   celsius = sensors.getTempCByIndex(0); // index 0 je prvni cidlo dalas
#ifdef DEBUG  
   Serial.print(F("TEPL:"));
   Serial.println(celsius,1);
#endif   
} // end void

//**********************************************************************************************
// serial rx tx ********************************************************************************
void cti_serial(){
#ifdef DEBUG  
    if (Serial.available()>0) {
      int znak = Serial.read(); 
      if (znak == 'F') { // klavesa F smazat a default hodnoty eeprom
         clear_eeprom(); init_eeprom(); 
         vterina = 0; minuta = 0; hodina = 12; den = 1; mesic = 1; rok = 15;
         RTC_a_restart(); 
         } // end if   
      if (znak == 'D') { // klavesa D nastaveni casu a datumu
         setTimeData();
         RTC_a_restart(); // pomocne funkce
         } // end if 
      if (znak == 'A') {  // spustit casovac cas1
         PeriodaDen();
         ZavlahaCasa();
         perioda_je_tu = true;
         } // end if 
      if (znak == 'B') {  // spustit casovac cas2
         PeriodaDen();
         ZavlahaCasb();
         perioda_je_tu = true;
         } // end if    
      if (znak == 'C') {  // spustit casovac cas3
         PeriodaDen();
         ZavlahaCasc();
         perioda_je_tu = true;
         } // end if 
       if (znak == 'E') {  // smazat den PeriodaVyp() jako 32:59:59
         PeriodaVyp();
         Serial_ende();
         } // end if 
       if (znak == 'G') {  // vypsat eeprom
         ee_read();
         } // end if     
       if (znak == 'I') {  // vypsat frekvenci a vlhkost
         Serial.print(F("FR:"));
         Serial.print(frq);
         Serial.print(F(" VLH:"));
         Serial.println(vlhkost);
         } // end if      
       if (znak == '?') {  // vypsat menu
         firmware();
         } // end if     
      } // end serial 
#endif       
} // end void  

void setTimeData(){ // vkladani udaju ze serialu
#ifdef DEBUG  
  Serial.print(F("Rok "));
  rok = readByte(); // musi byt 2x jinak se hned vlozi nula a skoci na mesic
  rok = readByte();
  Serial.println(rok);
  Serial.print(F("Den "));
  den = readByte();
  Serial.println(den);
  Serial.print(F("Mes "));
  mesic = readByte();
  Serial.println(mesic);
  Serial.print(F("Hod "));
  hodina = readByte();
  Serial.println(hodina);
  Serial.print(F("Min "));
  minuta = readByte();
  Serial.println(minuta);
  Serial.print(F("Sec "));
  vterina = readByte();
  Serial.println(vterina);
  Serial.print(hodina);Serial.print(":");Serial.print(minuta);Serial.print(":");
  Serial.print(vterina);Serial.print(" ");Serial.print(den);Serial.print(".");
  Serial.print(mesic);Serial.print(".");Serial.println(rok);
#endif  
  lcd.home();
  lcd.print(F("Restartuji se..."));
  lcd.setCursor(0,1);
  lcd.print(F("                "));
  delay(2000);
} // end void  

byte readByte(){ // vraci cislo zadane na serialu
#ifdef DEBUG  
  while (!Serial.available()) {delay(10);
  wdt_reset();
  } // cekame dokud neni zadano
  byte reading = 0;
  byte incomingByte = Serial.read();
  while (incomingByte != '\n') {
    if (incomingByte >= '0' && incomingByte <= '9')
      reading = reading * 10 + (incomingByte - '0');
    else;
    incomingByte = Serial.read();
  }
  Serial.flush();
  return reading;
#endif  
}// end byte



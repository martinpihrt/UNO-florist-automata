/*

*** MENU *************
? Napoveda
A Spustit program 1
B Spustit program 2
C Spustit program 3
D Nastavit datum a cas
E Smazat EEPROM
F Tisk EEPROM
G Vypsat frekvenci nebo napeti (zalezi na verzi FW)
H Vypsat teplotu
I Vypsat datum a cas
**********************

Vice na www.pihrt.com nebo github

Arduino IDE 1.8.8 www.arduino.cc
Deska UNO (Atmega 328)
verze: avr-gcc\\5.4.0-atmel3.6.1 AVR board 1.6.23
pojistky: efuse:0xFD hfuse:0xDE lfuse:0xFF

                           Rs, E,DB4,DB5,DB6,DB7
LCD:                    pin 8, 9,  2,  3,  6,  7
Serial:                 pin 0, 1
Cidlo vlhka IN:         pin 5 (meri se frekvence ze snimace vlhkosti)
LCD podsviceni:         pin 10
Tlacitka IN:            pin 14 (A0 delic 5 tlacitek)
Cidlo vlhka IN:         pin 15 (A1) meri se napeti ze snimace vlhkosti
Teplota:                pin 16  DS18B20 ruda +5v, cerna GND, modrá(bila) Data
Cerpadlo:               pin 17 (pro motor cerpadla)
I2C:                    pin 18 (A4 SDA), 19 (A5 SCL) pouziva DS1307

adresa I2C:
0x68  DS1307 RTC
*/

/*
 * verze 1.00 
   výchozí verze pouze měření frekvence
 * verze 1.10 
   přidaná možnost měřit vlhkost pomocí frekvence, nebo napětí na AD vstupu (čidlo: https://www.diymore.cc/products/2pcs-capacitive-soil-moisture-sensor-v1-2-analog-corrosion-resistant-dc-3-3-5-5v)
   new features Chris : https://pihrt.com/elektronika/400-uno-florist-automata#comment-370
 */

char verze[]="21.02.19-V1.10"; // Verze programu
#define DEBUG                  // 1/0 Povoleni, nebo zakazani ladeni na serialu
#define baud_rate 115200       // Rychlost serialu nastavit Both NL a CR

// mereni frekvence z cidla vlhkosti
//#define USE_FREQ_PROBE       // Pokud není zakomentované bude se používat měření vlhkosti pomocí frekvence
#define USE_AD_PROBE           // Pokud není zakomentované bude se používat měření vlhkosti pomocí napětí AD převodníku

// freq čidlo
#define cidlo_fMIN     5000    // Hz z cidla pri mokru ve vode
#define cidlo_fMAX     60      // Hz z cidla pri suchu na vzduchu
// napětí čidlo na AD
#define cidlo_AD_min   580     // Cislo 0-1023 na AD převodníku za sucha (wet sensor)
#define cidlo_AD_max   250     // Cislo 0-1023 na AD převodníku za mokra (dry sensor)

#define cerpadlo_pin   17      // Cislo pinu pro cerpadlo
#define temp_pin       16      // Cislo pinu cidla teploty
#define lcd_rs         8       // Cisla pinu pro lcd
#define lcd_en         9
#define lcd_db4        2
#define lcd_db5        3
#define lcd_db6        6
#define lcd_db7        7
#define lcd_led        10      // Cislo pinu podsviceni LCD
#define frek_pin       5       // Cislo pinu snimace vlhkosti (vlhkost na frekvenci) na casovaci T1 cpu
#define button_pin     14      // Cislo pinu tlacitek (A0)
#define AD_probe_pin   A1      // Cislo pinu snimace vlhkosti (vlhkost na napeti) vstup A1

#include "LiquidCrystal.h"     // Knihovna LCD displeje
#include "EEPROM.h"            // Knihovna EEPROM
#include "Time.h"              // Cas systemu
#include "Wire.h"              // Knihovna I2C
#include "DS1307RTC.h"         // Knihovna I2C DS1307
#include "OneWire.h"           // Knihovna pro Dallas Temperature OneWire DS18S20, DS18B20, DS1822
#include "DallasTemperature.h" // Knihovna pro rychle mereni teploty bez delay 1000
#include <avr/interrupt.h>     // Knihovna preruseni pro citace frekvence
#include <avr/wdt.h>           // Watchdog
/*watchdog
  wdt_disable();
  wdt_enable(WDTO_15MS); wdt_enable(WDTO_8S); etc
  wdt_reset();
*/

// CZ texty pro LCD
char msg_prazdno[]     ="                ";   // 16 znaku mezera
char msg_mezera[]      =" ";                  // 1 znak mezera
char msg_menu0[]       ="Zavlaha ve dnech";   // menu 0
char msg_menu1[]       ="Cas programu 1  ";   // menu 1
char msg_menu2[]       ="Beh programu 1  ";   // menu 2
char msg_menu3[]       ="Cas programu 2  ";   // menu 3
char msg_menu4[]       ="Beh programu 2  ";   // menu 4
char msg_menu5[]       ="Cas programu 3  ";   // menu 5
char msg_menu6[]       ="Beh programu 3  ";   // menu 6
char msg_menu7[]       ="Cerp vypnout za ";   // menu 7
char msg_menu8[]       ="Cekat vypnuty   ";   // menu 8
char msg_menu9[]       ="Reguluj teplotou";   // menu 9
char msg_menu10[]      ="Reguluj vlhkosti";   // menu 10
char msg_menu11[]      ="Datum a cas     ";   // menu 11
char msg_menu12[]      ="SMAZAT nastaveni";   // menu 12
char msg_menu13[]      ="ULOZIT nastaveni";   // menu 13
char msg_ZAPNUTO[]     ="ZAPNUTO";            // pouzito v menu 9, 10, 12, 13
char msg_VYPNUTO[]     ="VYPNUTO";            // pouzito v menu 9, 10, 12, 13
char msg_VYP[]         ="VYP";                // pouzito v menu 9, 10
char msg_ZAP[]         ="ZAP";                // pouzito v menu 9, 10
char msg_dvojtecka[]   =":";                  // pouzito v menu 1, 3, 5
char msg_rovnase[]     ="=";
char msg_procenta[]    ="%";
char msg_volt[]        ="V";
char msg_Hz[]          ="Hz";
char msg_kHz[]         ="kHz";
char msg_err_cidla[]   ="Chyba cidla";
char msg_tecka[]       =".";
char msg_zavL[]        ="[";
char msg_zavP[]        ="]";
char msg_otaznik[]     ="?";
char msg_minus[]       ="-";
char msg_lomitko[]     ="/";
char msg_ANO[]         ="ANO";
char msg_NE[]          ="NE";
char msg_po[]          ="Po";
char msg_ut[]          ="Ut";
char msg_st[]          ="St";
char msg_ct[]          ="Ct";
char msg_pa[]          ="Pa";
char msg_so[]          ="So";
char msg_ne[]          ="Ne";
char msg_dnes[]        ="Dnes je";
char msg_pondeli[]     ="pondeli";
char msg_utery[]       ="utery";
char msg_streda[]      ="streda";
char msg_ctvrtek[]     ="ctvrtek";
char msg_patek[]       ="patek";
char msg_sobota[]      ="sobota";
char msg_nedele[]      ="nedele";
char msg_hod_min[]     ="hod:min";
char msg_sec[]         ="sec";
char msg_interval_A[]  ="5-250";
char msg_interval_B[]  ="0-250";
char msg_ulozeno[]     ="ULOZENO";
char msg_rbt[]         ="RESTARTUJI SE";
char msg_run_now[]     ="BEZI PROGRAM...";
char msg_rtc[]         ="RTC NENALEZEN!";  // nenasel na I2C 0x68
char msg_rtc_run[]     ="CAS NENASTAVEN!"; // vraci 165:165:165 165.165.2165
char msg_cerpadlo[]    ="CERPADLO";
char msg_vloz[]        ="Vloz";
char msg_doba[]        ="doba";
char msg_chyba_dne[]   ="CHYBA RTC!";
char msg_cas[]         ="CAS";
char msg_jedna[]       ="1";
char msg_dva[]         ="2";
char msg_tri[]         ="3";
char msg_last_T[]      ="Posledni telota";
char msg_last_V[]      ="Posledni vlhkost";
char msg_EE_del[]      ="EEPROM smazana";
char msg_menu[]        ="MENU";
char msg_dwn[]         ="DOLU";
char msg_up[]          ="NAHORU";
char msg_left[]        ="VLEVO";
char msg_right[]       ="VPRAVO";
char msg_end[]         ="Konec";

#ifdef USE_FREQ_PROBE
   float frq;       // hodnota frekvence z čidla
#endif  
#ifdef USE_AD_PROBE
   int AD_in;      // hodnota napětí z čidla jako 0-1023
   float AD_volt;  // hodnota napětí z čidla jako 0-5V
#endif  
long odpocet_cerpadlo = 0;
byte get_day;

OneWire oneWire(temp_pin);
DallasTemperature sensors(&oneWire);
float celsius;              // Hodnota teploty v C
byte vlhkost = 0;           // Hodnota vlhkosti  0-100 na stupnici

tmElements_t tm;            // Systemovy cas

LiquidCrystal lcd(lcd_rs, lcd_en, lcd_db4, lcd_db5, lcd_db6, lcd_db7); // LCD displej (Rs,E,DB4,DB5,DB6,DB7)

// spinani cerpadla
boolean 
   cerpadlo_makej = false,                    // Pokud je true bude cerpadlo zapinat a vypinat v intervalech
   cerpadlo_vystup_stav = true;               // Pomocna pro vystup cerpadla

unsigned long 
   aktualniCasCerpadlo = millis(),            // Pomocna pro casovac cerpadla
   predchoziCasCerpadlo = 0,                  // Pomocna pro casovac
   interval = 0;                              // Pomocna pro casovac

// posun LCD
unsigned long LCDMillis   = 0;
unsigned int  LCDinterval = 3000;

// regulace zalevani dle casu, vlhkosti a teploty
boolean
   CAS1 = false,                      // Hodi se true v nastaveny cas 1
   CAS2 = false,                      // Hodi se true v nastaveny cas 2
   CAS3 = false;                      // Hodi se true v nastaveny cas 3
     
unsigned long 
   predchoziCasDoba = 0,             // Pomocna trvani casu 1-3 pro cerpadlo
   aktualniCasDoba = millis();       // Pomocna trvani casu 1-3 pro cerpadlo
   
unsigned int
   trvani_doby = 0;                  // Nastavuje dobu zalevani - pomocna

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
int 
   vterina = 0,
   minuta = 0,
   hodina = 0,
   den = 1,
   mesic = 1,
   rok = 18;

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
long
    last_teplota,
    last_vlhkost;

byte
    cas1_hod,
    cas1_min,
    doba1,
    cas2_hod,
    cas2_min,
    doba2,
    cas3_hod,
    cas3_min,
    doba3,
    cerpadlo_on,
    pauza_cerpadlo_off,
    use_teplota,
    use_vlhkost;

#define adr_day_po       20
#define adr_day_ut       21
#define adr_day_st       22
#define adr_day_ct       23
#define adr_day_pa       24
#define adr_day_so       25
#define adr_day_ne       26

#define adr_doba1        4         // doba1 5-60 min krok 5 min
#define adr_doba2        7         // doba2 5-60 min krok 5 min
#define adr_doba3        10        // doba3 5-60 min krok 5 min
#define adr_cas1_hod     2         // cas1 0-23 hod
#define adr_cas1_min     3         // cas1 0-59 min (max 23:30) krok 30 min
#define adr_cas2_hod     5         // cas2 0-23 hod
#define adr_cas2_min     6         // cas2 0-59 min (max 23:30) krok 30 min
#define adr_cas3_hod     8         // cas3 0-23 hod
#define adr_cas3_min     9         // cas3 0-59 min (max 23:30) krok 30 min
#define adr_cerpadlo_on  11        // doba sepnuti cerpadla na 0-10 min krok 1 min
#define adr_use_teplota  13        // pouzivat mereni teploty 1=off, 2=on
#define adr_use_vlhkost  16        // pouzivat mereni vlhkosti 1=off, 2=on
#define adr_last_teplota 14        // posledni zmerena teplota (hodnota je float x1000)
#define adr_last_vlhkost 17        // posledni zmerena vlhkost (namerena hodnota 0-100)
#define adr_pauza_cerpadlo_off  12 // doba cekani cerpadla na 0-5 min krok 1 min

byte
    day_po = 2,
    day_ut = 2,
    day_st = 2,
    day_ct = 2,
    day_pa = 2,
    day_so = 2,
    day_ne = 2;

//**********************************************************************************************
// citac frekvence na D5 ********************************************************************************
#ifdef USE_FREQ_PROBE
  volatile unsigned long timerCounts;
  volatile boolean counterReady;
  unsigned long overflowCount;
  unsigned int timerTicks;
  unsigned int timerPeriod;
#endif  

void startCounting (unsigned int ms){
#ifdef USE_FREQ_PROBE  
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
#endif  
}  // end of TIMER1_OVF_vect

//******************************************************************
//  Timer2 Interrupt Service is invoked by hardware Timer 2 every 1 ms = 1000 Hz
//  16Mhz / 128 / 125 = 1000 Hz

ISR (TIMER2_COMPA_vect){// grab counter value before it changes any more
#ifdef USE_FREQ_PROBE  
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
#endif  
} // end of TIMER2_COMPA_vect


//**********************************************************************************************
// setup - loop ********************************************************************************
void setup() {

    inicializace();     // pomocne funkce (inicializace serial, lcd)
    init_piny();        // pomocne funkce (nastavy vystupy a vypne je)
    init_eeprom();      // eeprom (nacte ulozene hodnoty, pripadne ulozi default do eeprom)
    firmware();         // pomocne funkce (init serial a lcd, fw na serial a lcd)
    setSyncProvider(RTC.get);   // nastaveni syncu casu z RTC
    setSyncInterval(300);       // za jak dlouho se zavola sync z RTC den=24h*60m*60s tj 86400

    if(timeStatus() != timeSet) { // skok do menu nastavit cas
      lcd.clear();
      lcd.print(msg_rtc_run);
      #ifdef DEBUG
        Serial.println(msg_rtc_run);
      #endif
      delay(4000);
      menu = true;         // povolime menu
      btnPUSHED = 11;      // v menu je 11 nastaveni casu a datumu
      LR_menu = 0;         // vratime kruzor doleva
      lcd_menu(btnPUSHED); // vytiskneme polozku z menu
    }

    cti_serial();       // serial_rx_tx (komunikace se serialem)
    init_teplota();     // pomocne funkce cidlo DS18B20
    Mereni();           // nacte teplotu a vlhkost
    wdt_enable(WDTO_4S);// povolime watchdog 4 sec
    wdt_reset();        // vynulujeme wdt tik
} // end setup

void loop() {

  casovacLCD();                // vypisuje na LCD stavova okna po xx sec
  cti_serial();                // serial_rx_tx (komunikace se serialem)
  cti_klavesnici();            // klavesnice (vyhodnocujeme klavesy)

  if (save) {save_eeprom();}   // ulozi do pameti z lcd menu tlacitek
  else if(!menu) {//neni aktivni menu

     cti_cas_datum();
     Mereni();              // nacte teplotu a vlhkost
     regulace_vlhk();       // zpracuje zavlazovani dle vhlkosti
     regulace_tepl();       // zpracuje zavlažování dle teploty
     cerpadlo();            // zpracuj cerpadlo

     if((hour()==cas1_hod)&&(minute()==cas1_min)&&(second()<5)&& !CAS1) ZavlahaCas(1); // pokud je cas povolime zavlazovani pgm1
     if((hour()==cas2_hod)&&(minute()==cas2_min)&&(second()<5)&& !CAS2) ZavlahaCas(2); // pokud je cas povolime zavlazovani pgm2
     if((hour()==cas3_hod)&&(minute()==cas3_min)&&(second()<5)&& !CAS3) ZavlahaCas(3); // pokud je cas povolime zavlazovani pgm3
  }//end else

  wdt_reset();
} // end loop

void ZavlahaCas(byte pgm){ // je
#ifdef DEBUG
   Serial.print(msg_cas);Serial.print(msg_mezera);Serial.print(pgm);Serial.print(msg_mezera);
#endif
   if(pgm==1) {trvani_doby = doba1; CAS1 = true;} // prave je cas 1
   else if(pgm==2) {trvani_doby = doba2; CAS2 = true;} // prave je cas 2
   else if(pgm==3) {trvani_doby = doba3; CAS3 = true;} // prave je cas 3
   else return;
   predchoziCasCerpadlo = millis(); // ulozime aktualni cas pro cerpadlo
   interval = 0;                    // interval cyklovani cerpadla
   predchoziCasDoba = millis();     // interval doby zalevani 
#ifdef DEBUG
   Serial.print(msg_doba);Serial.print(msg_mezera);Serial.print(trvani_doby);Serial.print(msg_mezera);Serial.println(msg_sec);
#endif      
} // end void

void casovacLCD(){         // posouva pomalu text na lcd
  if(!menu){               // kdyz neni menu
    if(!cerpadlo_makej){   // nejede program
      LCDinterval = 3000;  // 3 vteriny posun textu na LCD
      if (millis() - LCDMillis >= LCDinterval){
        LCDMillis = millis();
        posun_lcd++;
        vypisuj_info_lcd();
      }//end millis
    }//end makej
    else{ // jede program
      LCDinterval = 1000; // vterina posun textu s casem na LCD
        if (millis() - LCDMillis >= LCDinterval){
          LCDMillis = millis();
          lcd.setCursor(0,0);
          lcd.print(msg_cerpadlo);
          lcd.print(msg_mezera);
          if(cerpadlo_vystup_stav) {
            lcd.print(msg_VYP); lcd.print(msg_prazdno);lcd.setCursor(0,1);lcd.print(msg_doba);lcd.print(msg_mezera);
            }
          else{
            lcd.print(msg_ZAP); lcd.print(msg_prazdno);lcd.setCursor(0,1);lcd.print(msg_doba);lcd.print(msg_mezera);
            }
          odpocet_cerpadlo = ((predchoziCasCerpadlo+interval)-aktualniCasCerpadlo)/1000; // cas v sec misto ms
          // zobrazi na LCD cas v sec zap nebo vyp a celkovy cas programu (pr: 5/254sec)
          lcd.print(odpocet_cerpadlo);lcd.print(msg_lomitko);lcd.print(aktualniCasDoba/1000-predchoziCasDoba/1000);lcd.print(msg_sec); 
          lcd.print(msg_prazdno);
        }//end millis
    }//end makej
  }//end if
}//end void

void Mereni(){ // meri teplotu a frekvenci pokud neni aktivni menu a nebezi program 1-3
  if (!menu && !CAS1 && !CAS2 && !CAS3){
    cti_teplotu();   // ds18b20
    #ifdef USE_FREQ_PROBE
      cti_frekvenci();  // freq snimac vlhkosti
    #endif  
    
    #ifdef USE_AD_PROBE
      cti_napeti();    // napeti snimac vhlkosti
    #endif  
  }
} // end void

void cti_frekvenci(){
#ifdef USE_FREQ_PROBE  
  //------------------------------------------
  // http://www.gammon.com.au/forum/?id=11504

// stop Timer 0 interrupts from throwing the count out
  //byte oldTCCR0A = TCCR0A;
  //byte oldTCCR0B = TCCR0B;
  //TCCR0A = 0;    // stop timer 0
  //TCCR0B = 0;

  startCounting (500);  // how many ms to count for
  while (!counterReady){
    wdt_reset();
  }
  // adjust counts by counting interval to give frequency in Hz
  frq = (timerCounts * 1000.0) / timerPeriod;
  // restart timer 0
  //TCCR0A = oldTCCR0A;
  //TCCR0B = oldTCCR0B;
#endif
} // end void

void cti_napeti(){
    #ifdef USE_AD_PROBE
      AD_in = analogRead(AD_probe_pin);  // A1 vstup
      AD_volt = (AD_in * 5.0) / 1024.0;  // převod 0-1024 na 0.0-5.0V
      delay(1);
    #endif    
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
  if (last_vlhkost > 100 || last_vlhkost < 0) {edit = true;write_eeprom_long(adr_last_vlhkost, 0);}   // posledni zmerena vlhkost vychozi 0 (0-100)
  if (day_po > 2 || day_po==1) {edit = true;write_eeprom_byte(adr_day_po, 2);}
  if (day_ut > 2 || day_ut==1) {edit = true;write_eeprom_byte(adr_day_ut, 2);}
  if (day_st > 2 || day_st==1) {edit = true;write_eeprom_byte(adr_day_st, 2);}
  if (day_ct > 2 || day_ct==1) {edit = true;write_eeprom_byte(adr_day_ct, 2);}
  if (day_pa > 2 || day_pa==1) {edit = true;write_eeprom_byte(adr_day_pa, 2);}
  if (day_so > 2 || day_so==1) {edit = true;write_eeprom_byte(adr_day_so, 2);}
  if (day_ne > 2 || day_ne==1) {edit = true;write_eeprom_byte(adr_day_ne, 2);}
  if (edit) init_read();  // znovu nacteme ulozene z eeprom pokud se neco menilo
} // end init_eeprom

void init_read(){ // nacteni z eeprom po zapnuti zarizeni
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
  last_vlhkost=read_eeprom_long(adr_last_vlhkost);             // nacte posledni zmerenou vlhkost (namerena hodnota 0-100)
  day_po = read_eeprom_byte(adr_day_po);
  day_ut = read_eeprom_byte(adr_day_ut);
  day_st = read_eeprom_byte(adr_day_st);
  day_ct = read_eeprom_byte(adr_day_ct);
  day_pa = read_eeprom_byte(adr_day_pa);
  day_so = read_eeprom_byte(adr_day_so);
  day_ne = read_eeprom_byte(adr_day_ne);
#ifdef DEBUG
  Serial.println(F("EEPROM nacteni OK"));
#endif
} // end init_read

void ee_read(){ // vypise eeprom hodnoty
   init_read(); // nacte ee
#ifdef DEBUG
   Serial.print(msg_cas);Serial.print(msg_jedna);Serial.print(msg_mezera);Serial.print(cas1_hod);Serial.print(msg_dvojtecka);Serial.print(cas1_min);Serial.print(F("-"));Serial.println(doba1);
   Serial.print(msg_cas);Serial.print(msg_dva);Serial.print(msg_mezera);Serial.print(cas2_hod);Serial.print(msg_dvojtecka);Serial.print(cas2_min);Serial.print(F("-"));Serial.println(doba2);
   Serial.print(msg_cas);Serial.print(msg_tri);Serial.print(msg_mezera);Serial.print(cas3_hod);Serial.print(msg_dvojtecka);Serial.print(cas3_min);Serial.print(F("-"));Serial.println(doba3);
   Serial.print(msg_cerpadlo);Serial.print(msg_mezera);Serial.print(msg_ZAP);Serial.print(msg_mezera);Serial.println(cerpadlo_on);
   Serial.print(msg_cerpadlo);Serial.print(msg_mezera);Serial.print(msg_VYP);Serial.print(msg_mezera);Serial.println(pauza_cerpadlo_off);
   Serial.print(msg_menu9);Serial.print(msg_mezera);Serial.println(use_teplota);
   Serial.print(msg_last_T);Serial.print(msg_mezera);Serial.println(last_teplota);
   Serial.print(msg_menu10);Serial.print(msg_mezera);Serial.println(use_vlhkost);
   Serial.print(msg_last_V);Serial.print(msg_mezera);Serial.println(last_vlhkost);
   Serial.print(msg_po);Serial.print(msg_mezera);Serial.println(day_po);
   Serial.print(msg_ut);Serial.print(msg_mezera);Serial.println(day_ut);
   Serial.print(msg_st);Serial.print(msg_mezera);Serial.println(day_st);
   Serial.print(msg_ct);Serial.print(msg_mezera);Serial.println(day_ct);
   Serial.print(msg_pa);Serial.print(msg_mezera);Serial.println(day_pa);
   Serial.print(msg_so);Serial.print(msg_mezera);Serial.println(day_so);
   Serial.print(msg_ne);Serial.print(msg_mezera);Serial.println(day_ne);
#endif
} // end void

void clear_eeprom(){ // write 0 do vsech bytes EEPROM
    for (int i = 0; i < EEPROM.length(); i++){
      EEPROM.write(i, 0);
      }
#ifdef DEBUG
    Serial.println(msg_EE_del);
#endif
} // end void

void save_eeprom() { // ulozi z lcd menu hodnoty
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
  write_eeprom_byte(adr_day_po, day_po);
  write_eeprom_byte(adr_day_ut, day_ut);
  write_eeprom_byte(adr_day_st, day_st);
  write_eeprom_byte(adr_day_ct, day_ct);
  write_eeprom_byte(adr_day_pa, day_pa);
  write_eeprom_byte(adr_day_so, day_so);
  write_eeprom_byte(adr_day_ne, day_ne);
#ifdef DEBUG
  Serial.println(msg_ulozeno);
#endif
  save = false;
  lcd.noBlink(); // prestaneme blikat kurzorem
  lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
  lcd.print(msg_ulozeno);
  lcd.print(msg_prazdno);
  lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
  lcd.print(msg_prazdno);
  delay(2000); // pockame na lcd text
  RTC_a_restart(); // pomocne funkce
} // end void

//**********************************************************************************************
// klavesnice na analogovam vstupu *************************************************************
int read_LCD_buttons(){ // tato funkce cte hodnotu tlacitek
    adc_key_in = analogRead(button_pin);      // cteme hodnotu ze senzoru A0
    //Serial.print("adc:"); Serial.print(adc_key_in);Serial.print(" ");
    // stredni hodnota tlacitek je zhruba tato: 0, 144, 329, 504, 741
    // pridame cca 50 k temto hodnotam a zkontrolujeme zda jsme blizko
    if (adc_key_in<50)   return btnRIGHT;
    if (adc_key_in<195)  return btnUP;
    if (adc_key_in<380)  return btnDOWN;
    if (adc_key_in<555)  return btnLEFT;
    if (adc_key_in<790)  return btnSELECT;
    if (adc_key_in>1000) return btnNONE; // kontrolujeme nejprve tuto moznost, protoze to je nejcastejsi stav
    return btnNONE;  // pokud nic z toho neodpovida, vracime tuto hodnotu
} //end read

void cti_klavesnici(){ // Rutina cte stiskle klavesy a meni hodnoty parametru
  lcd_key = read_LCD_buttons();  // precteme stav tlacitek
  switch (lcd_key){              // dle toho, ktere je stisknuto vypiseme akci
    case btnRIGHT: // vpravo
     //if(btnPUSH) break;
     delay(400);
#ifdef DEBUG
     Serial.println(msg_right);
#endif
     if (menu_state==11) {LR_menu++; if (LR_menu >= 4) {LR_menu = 4;} lcd_menu(11); btnPUSH = true; break;} // posouvame vpravo v menu lcd cas a datum
     if (menu_state==5)  {LR_menu++; if (LR_menu >= 1) {LR_menu = 1;} lcd_menu(5); btnPUSH = true; break;} // posouvame vpravo v menu lcd cas 3
     if (menu_state==3)  {LR_menu++; if (LR_menu >= 1) {LR_menu = 1;} lcd_menu(3); btnPUSH = true; break;} // posouvame vpravo v menu lcd cas 2
     if (menu_state==1)  {LR_menu++; if (LR_menu >= 1) {LR_menu = 1;} lcd_menu(1); btnPUSH = true; break;} // posouvame vpravo v menu lcd cas 1
     if (menu_state==0)  {LR_menu++; if (LR_menu >= 6) {LR_menu = 6;} lcd_menu(0); btnPUSH = true; break;} // posouvame vlevo v menu lcd den
     btnPUSH = true;
     break;

   case btnLEFT: // vlevo
     delay(400);
     //if(btnPUSH) break;
#ifdef DEBUG
     Serial.println(msg_left);
#endif
     if (menu_state==11) {LR_menu--; if (LR_menu <= 0) {LR_menu = 0;} lcd_menu(11); btnPUSH = true; break;} // posouvame vlevo v menu lcd cas a datum
     if (menu_state==5)  {LR_menu--; if (LR_menu <= 0) {LR_menu = 0;} lcd_menu(5); btnPUSH = true; break;} // posouvame vlevo v menu lcd cas 3
     if (menu_state==3)  {LR_menu--; if (LR_menu <= 0) {LR_menu = 0;} lcd_menu(3); btnPUSH = true; break;} // posouvame vlevo v menu lcd cas 2
     if (menu_state==1)  {LR_menu--; if (LR_menu <= 0) {LR_menu = 0;} lcd_menu(1); btnPUSH = true; break;} // posouvame vlevo v menu lcd cas 1
     if (menu_state==0)  {LR_menu--; if (LR_menu <= 0) {LR_menu = 0;} lcd_menu(0); btnPUSH = true; break;} // posouvame vlevo v menu lcd den
     //btnPUSH = true;
     break;

   case btnUP: // nahoru
     delay(200);
     //if(btnPUSH) break;
#ifdef DEBUG
     Serial.println(msg_up);
#endif
     // save
     if (menu_state==13) {save = true; menu_state = -1; btnPUSHED=0; break;}  // blokujeme vypis na lcd  pouze do konce menu pak vyskocime a ulozime
     // smazat do default
     if (menu_state==12) {menu_state = -1;  clear_eeprom(); init_eeprom();
         vterina = 0; minuta = 0; hodina = 0; den = 1; mesic = 1; rok = 18;
         RTC_a_restart(); break;
         }  // blokujeme vypis na lcd  pouze do konce menu pak vyskocime a ulozime
     // datum cas
     if (menu_state==11) {
        if (LR_menu == 0) {if (den >= 31) {den = 1;} else {den++;} lcd_menu(11); btnPUSH = true; break;}          // LR menu 0=den, 1=mesic, 2=rok, 3=hod, 4=min
        if (LR_menu == 1) {if (mesic >= 12) {mesic = 1;} else {mesic++;} lcd_menu(11); btnPUSH = true; break;}    //         1=mesic
        if (LR_menu == 2) {if (rok >= 50) {rok = 19;} else {rok++;} lcd_menu(11); btnPUSH = true; break;}         //         2=rok
        if (LR_menu == 3) {if (hodina >= 23) {hodina = 0;} else {hodina++;} lcd_menu(11); btnPUSH = true; break;} //         3=hodina
        if (LR_menu == 4) {if (minuta >= 59) {minuta = 0;} else {minuta++;} lcd_menu(11); btnPUSH = true; break;} //
     }//end menu
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
     if (menu_state==5) {
       if (LR_menu == 0) {if (cas3_hod >= 23) {cas3_hod = 0;} else {cas3_hod++;} lcd_menu(5); btnPUSH = true; break;}          // LR menu 0=hod,
       if (LR_menu == 1) {if (cas3_min >= 59) {cas3_min = 0;} else {cas3_min++;} lcd_menu(5); btnPUSH = true; break;}
     }//end menu
     // interval cas 2
     if (menu_state==4) {if (doba2 >= 250) {doba2 = 5;} else {doba2+=5;} lcd_menu(4); btnPUSH = true; break;} // krok 5 sec
     // cas spinani 2
     if (menu_state==3) {
       if (LR_menu == 0) {if (cas2_hod >= 23) {cas2_hod = 0;} else {cas2_hod++;} lcd_menu(3); btnPUSH = true; break;}           // LR menu 0=hod,
       if (LR_menu == 1) {if (cas2_min >= 59) {cas2_min = 0;} else {cas2_min++;} lcd_menu(3); btnPUSH = true; break;}
     }//end menu
     // interval cas 1
     if (menu_state==2) {if (doba1 >= 250) {doba1 = 5;} else {doba1+=5;} lcd_menu(2); btnPUSH = true; break;} // krok 5 sec
     // cas spinani 1
     if (menu_state==1) {
       if (LR_menu == 0) {if (cas1_hod >= 23) {cas1_hod = 0;} else {cas1_hod++;} lcd_menu(1); btnPUSH = true; break;}           // LR menu 0=hod,
       if (LR_menu == 1) {if (cas1_min >= 59) {cas1_min = 0;} else {cas1_min++;} lcd_menu(1); btnPUSH = true; break;}
     }//end menu
     // dny po-ne
     if (menu_state==0) {
       if (LR_menu == 0) {day_po=2; lcd_menu(0); btnPUSH = true; break;} // den po
       if (LR_menu == 1) {day_ut=2; lcd_menu(0); btnPUSH = true; break;} // den ut
       if (LR_menu == 2) {day_st=2; lcd_menu(0); btnPUSH = true; break;} // den st
       if (LR_menu == 3) {day_ct=2; lcd_menu(0); btnPUSH = true; break;} // den ct
       if (LR_menu == 4) {day_pa=2; lcd_menu(0); btnPUSH = true; break;} // den pa
       if (LR_menu == 5) {day_so=2; lcd_menu(0); btnPUSH = true; break;} // den so
       if (LR_menu == 6) {day_ne=2; lcd_menu(0); btnPUSH = true; break;} // den ne
       //btnPUSH = true;
     }//end menu
     break;

   case btnDOWN: // dolu
     delay(200);
     //if(btnPUSH) break;
#ifdef DEBUG
     Serial.println(msg_dwn);
#endif
     // save
     if (menu_state==13) {menu = false; save = false; lcd.noBlink(); menu_state = -1; btnPUSHED=0; break;}  // blokujeme vypis na lcd  pouze do konce menu pak vyskocime
     // smazat do default
     if (menu_state==12) {menu = false; save = false; lcd.noBlink(); menu_state = -1; btnPUSHED=0; break;}  // blokujeme vypis na lcd  pouze do konce menu pak vyskocime
     // datum cas
     if (menu_state==11) {
       if (LR_menu == 0) {if (den <= 1) {den = 31;} else {den--;} lcd_menu(11); btnPUSH = true; break;}           // LR menu 0=den, 1=mesic, 2=rok, 3=hod, 4=min
       if (LR_menu == 1) {if (mesic <= 1) {mesic = 12;} else {mesic--;} lcd_menu(11); btnPUSH = true; break;}     //         1=mesic
       if (LR_menu == 2) {if (rok <= 19) {rok = 50;} else {rok--;} lcd_menu(11); btnPUSH = true; break;}          //         2=rok
       if (LR_menu == 3) {if (hodina <= 0) {hodina = 23;} else {hodina--;} lcd_menu(11); btnPUSH = true; break;}  //         3=hodina
       if (LR_menu == 4) {if (minuta <= 0) {minuta = 59;} else {minuta--;} lcd_menu(11); btnPUSH = true; break;}
     } //end menu
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
     if (menu_state==5) {
       if (LR_menu == 0) {if (cas3_hod <= 0) {cas3_hod = 23;} else {cas3_hod--;} lcd_menu(5); btnPUSH = true; break;}           // LR menu 0=hod,
       if (LR_menu == 1) {if (cas3_min <= 0) {cas3_min = 59;} else {cas3_min--;} lcd_menu(5); btnPUSH = true; break;}
     }//end menu
     // interval cas 2
     if (menu_state==4) {if (doba2 <= 5) {doba2 = 250;} else {doba2-=5;} lcd_menu(4); btnPUSH = true; break;} // krok 5 sec
     // cas spinani 2
     if (menu_state==3) {
       if (LR_menu == 0) {if (cas2_hod <= 0) {cas2_hod = 23;} else {cas2_hod--;} lcd_menu(3); btnPUSH = true; break;}           // LR menu 0=hod,
       if (LR_menu == 1) {if (cas2_min <= 0) {cas2_min = 59;} else {cas2_min--;} lcd_menu(3); btnPUSH = true; break;}
     }//end menu
     // interval cas 1
     if (menu_state==2) {if (doba1 <= 5) {doba1 = 250;} else {doba1-=5;} lcd_menu(2); btnPUSH = true; break;} // krok 5 sec
     // cas spinani 1
     if (menu_state==1) {
       if (LR_menu == 0) {if (cas1_hod <= 0) {cas1_hod = 23;} else {cas1_hod--;} lcd_menu(1); btnPUSH = true; break;}           // LR menu 0=hod,
       if (LR_menu == 1) {if (cas1_min <= 0) {cas1_min = 59;} else {cas1_min--;} lcd_menu(1); btnPUSH = true; break;}
     }//end menu
     // dny po-ne
     if (menu_state==0) {
       if (LR_menu == 0) {day_po=0; lcd_menu(0); btnPUSH = true; break;} // den po
       if (LR_menu == 1) {day_ut=0; lcd_menu(0); btnPUSH = true; break;} // den ut
       if (LR_menu == 2) {day_st=0; lcd_menu(0); btnPUSH = true; break;} // den st
       if (LR_menu == 3) {day_ct=0; lcd_menu(0); btnPUSH = true; break;} // den ct
       if (LR_menu == 4) {day_pa=0; lcd_menu(0); btnPUSH = true; break;} // den pa
       if (LR_menu == 5) {day_so=0; lcd_menu(0); btnPUSH = true; break;} // den so
       if (LR_menu == 6) {day_ne=0; lcd_menu(0); btnPUSH = true; break;} // den ne
     }//end menu
     btnPUSH = true;
     break;

   case btnSELECT: // menu (select)
     //if(btnPUSH) break;
#ifdef DEBUG
     Serial.print(msg_menu);
     Serial.println(btnPUSHED);
#endif
     menu = true;
     lcd_menu(btnPUSHED); // posouvame menu
     if (btnPUSHED == 0 || btnPUSHED == 1 || btnPUSHED == 3 || btnPUSHED == 5 || btnPUSHED == 11) {LR_menu = 0;  lcd_menu(btnPUSHED); } // vratime se na zacatek tj doleva
     btnPUSHED++;
     btnPUSH = true;
     delay(800);
     break;

   case btnNONE:
     //btnPUSH = false;
     break;

   } // end switch
} //end void

void lcd_menu(byte krok) {// Rutina tiskne texty menu na LCD)
  switch(krok){
    case 0: // menu 0
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(msg_menu0);
      lcd.setCursor(14,1); lcd.print(msg_mezera);lcd.print(msg_otaznik);
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      if(day_po==2)lcd.print(msg_po); else {lcd.print(msg_minus);lcd.print(msg_minus);}
      lcd.setCursor(2,1);
      if(day_ut==2)lcd.print(msg_ut); else {lcd.print(msg_minus);lcd.print(msg_minus);}
      lcd.setCursor(4,1);
      if(day_st==2)lcd.print(msg_st); else {lcd.print(msg_minus);lcd.print(msg_minus);}
      lcd.setCursor(6,1);
      if(day_ct==2)lcd.print(msg_ct); else {lcd.print(msg_minus);lcd.print(msg_minus);}
      lcd.setCursor(8,1);
      if(day_pa==2)lcd.print(msg_pa); else {lcd.print(msg_minus);lcd.print(msg_minus);}
      lcd.setCursor(10,1);
      if(day_so==2)lcd.print(msg_so); else {lcd.print(msg_minus);lcd.print(msg_minus);}
      lcd.setCursor(12,1);
      if(day_ne==2)lcd.print(msg_ne); else {lcd.print(msg_minus);lcd.print(msg_minus);}
      menu_state = 0;
      if (LR_menu==0) {lcd.setCursor(0,1); lcd.blink(); }
      if (LR_menu==1) {lcd.setCursor(2,1); lcd.blink();}
      if (LR_menu==2) {lcd.setCursor(4,1); lcd.blink();}
      if (LR_menu==3) {lcd.setCursor(6,1); lcd.blink();}
      if (LR_menu==4) {lcd.setCursor(8,1); lcd.blink();}
      if (LR_menu==5) {lcd.setCursor(10,1); lcd.blink();}
      if (LR_menu==6) {lcd.setCursor(12,1); lcd.blink();}
      break;
    case 1: // menu 1
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(msg_menu1);
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      print_digits_lcd(cas1_hod);
      lcd.print(msg_dvojtecka);
      print_digits_lcd(cas1_min);
      lcd.print(msg_mezera);
      lcd.print(msg_hod_min);
      lcd.print(msg_prazdno);
      menu_state = 1;
      if (LR_menu==0) {lcd.setCursor(1,1); lcd.blink();} // zacne blikat kurzor hod
      if (LR_menu==1) {lcd.setCursor(4,1); lcd.blink();} // zacne blikat kurzor min
      break;
    case 2: // menu 2
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(msg_menu2);
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      print_digits_lcd(doba1);
      lcd.print(msg_mezera);
      lcd.print(msg_zavL);
      lcd.print(msg_interval_A);
      lcd.print(msg_mezera);
      lcd.print(msg_sec);
      lcd.print(msg_zavP);
      lcd.print(msg_prazdno);
      menu_state = 2;
      lcd.setCursor(1,1); lcd.blink();
      break;
    case 3: // menu 3
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(msg_menu3);
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      print_digits_lcd(cas2_hod);
      lcd.print(msg_dvojtecka);
      print_digits_lcd(cas2_min);
      lcd.print(msg_mezera);
      lcd.print(msg_hod_min);
      lcd.print(msg_prazdno);
      menu_state = 3;
      if (LR_menu==0) {lcd.setCursor(1,1); lcd.blink();} // zacne blikat kurzor hod
      if (LR_menu==1) {lcd.setCursor(4,1); lcd.blink();} // zacne blikat kurzor min
      break;
    case 4: // menu 4
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(msg_menu4);
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      print_digits_lcd(doba2);
      lcd.print(msg_mezera);
      lcd.print(msg_zavL);
      lcd.print(msg_interval_A);
      lcd.print(msg_mezera);
      lcd.print(msg_sec);
      lcd.print(msg_zavP);
      lcd.print(msg_prazdno);
      menu_state = 4;
      lcd.setCursor(1,1); lcd.blink();
      break;
    case 5: // menu 5
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(msg_menu5);
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      print_digits_lcd(cas3_hod);
      lcd.print(msg_dvojtecka);
      print_digits_lcd(cas3_min);
      lcd.print(msg_mezera);
      lcd.print(msg_hod_min);
      lcd.print(msg_prazdno);
      menu_state = 5;
      if (LR_menu==0) {lcd.setCursor(1,1); lcd.blink();} // zacne blikat kurzor hod
      if (LR_menu==1) {lcd.setCursor(4,1); lcd.blink();} // zacne blikat kurzor min
      break;
    case 6: // menu 6
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(msg_menu6);
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      print_digits_lcd(doba3);
      lcd.print(msg_mezera);
      lcd.print(msg_zavL);
      lcd.print(msg_interval_A);
      lcd.print(msg_mezera);
      lcd.print(msg_sec);
      lcd.print(msg_zavP);
      lcd.print(msg_prazdno);
      menu_state = 6;
      lcd.setCursor(1,1); lcd.blink();
      break;
    case 7: // menu 7
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(msg_menu7);
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      print_digits_lcd(cerpadlo_on);
      lcd.print(msg_mezera);
      lcd.print(msg_zavL);
      lcd.print(msg_interval_B);
      lcd.print(msg_mezera);
      lcd.print(msg_sec);
      lcd.print(msg_zavP);
      lcd.print(msg_prazdno);
      menu_state = 7;
      lcd.setCursor(1,1); lcd.blink();
      break;
    case 8: // menu 8
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(msg_menu8);
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      lcd.print(pauza_cerpadlo_off);
      lcd.print(msg_mezera);
      lcd.print(msg_zavL);
      lcd.print(msg_interval_B);
      lcd.print(msg_mezera);
      lcd.print(msg_sec);
      lcd.print(msg_zavP);
      lcd.print(msg_prazdno);
      menu_state = 8;
      lcd.setCursor(0,1); lcd.blink();
      break;
    case 9: // menu 9
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(msg_menu9);
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      if (use_teplota==2){
        lcd.print(msg_ZAPNUTO);
        lcd.print(msg_mezera);
        lcd.print(msg_zavL);
        lcd.write((byte)2);          // symbol sipka
        lcd.print(msg_rovnase);
        lcd.print(msg_VYP);
        lcd.print(msg_zavP);
        }
        else {
        lcd.print(msg_VYPNUTO);
        lcd.print(msg_mezera);
        lcd.print(msg_zavL);
        lcd.write((byte)1);          // symbol sipka
        lcd.print(msg_rovnase);
        lcd.print(msg_ZAP);
        lcd.print(msg_zavP);
          }
      menu_state = 9;
      lcd.setCursor(0,1); lcd.blink();
      break;
    case 10: // menu 10
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(msg_menu10);
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      if (use_vlhkost==2){
        lcd.print(msg_ZAPNUTO);
        lcd.print(msg_mezera);
        lcd.print(msg_zavL);
        lcd.write((byte)2);          // symbol sipka
        lcd.print(msg_rovnase);
        lcd.print(msg_VYP);
        lcd.print(msg_zavP);
        }
        else {
        lcd.print(msg_VYPNUTO);
        lcd.print(msg_mezera);
        lcd.print(msg_zavL);
        lcd.write((byte)1);          // symbol sipka
        lcd.print(msg_rovnase);
        lcd.print(msg_ZAP);
        lcd.print(msg_zavP);
          }
      menu_state = 10;
      lcd.setCursor(0,1); lcd.blink();
      break;
    case 11: // datum cas
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(msg_menu11);
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      lcd.print(msg_prazdno);
      lcd.setCursor(0,1);
      print_digits_lcd(den);
      lcd.setCursor(2,1);
      lcd.print(msg_tecka);
      print_digits_lcd(mesic);
      lcd.print(msg_prazdno);
      lcd.setCursor(5,1);
      lcd.print(msg_tecka);
      print_digits_lcd(rok);
      lcd.print(msg_prazdno);
      lcd.setCursor(11,1);
      print_digits_lcd(hodina);
      lcd.setCursor(13,1);
      lcd.print(msg_dvojtecka);
      print_digits_lcd(minuta);
      lcd.print(msg_prazdno);
      menu_state = 11;
      vterina = 0;
      if (LR_menu==0) {lcd.setCursor(1,1); lcd.blink();} // zacne blikat kurzor den
      if (LR_menu==1) {lcd.setCursor(4,1); lcd.blink();} // zacne blikat kurzor mesic
      if (LR_menu==2) {lcd.setCursor(7,1); lcd.blink();} // zacne blikat kurzor rok
      if (LR_menu==3) {lcd.setCursor(12,1); lcd.blink();} // zacne blikat kurzor hod
      if (LR_menu==4) {lcd.setCursor(15,1); lcd.blink();} // zacne blikat kurzor min
      break;
   case 12: // dotaz smazat
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(msg_menu12);
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      lcd.print(msg_otaznik);
      lcd.print(msg_mezera);
      lcd.print(msg_zavL);
      lcd.write((byte) 2); // symbol sipka
      lcd.print(msg_rovnase);
      lcd.print(msg_ANO);
      lcd.print(msg_mezera);
      lcd.write((byte) 1); // symbol sipka
      lcd.print(msg_rovnase);
      lcd.print(msg_NE);
      lcd.print(msg_zavP);
      lcd.print(msg_prazdno);
      menu_state = 12;
      lcd.setCursor(0,1); lcd.blink();
      break;
   case 13: // dotaz ulozit
      lcd.setCursor(0,0);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 1 radek
      lcd.print(msg_menu13);
      lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
      lcd.print(msg_otaznik);
      lcd.print(msg_mezera);
      lcd.print(msg_zavL);
      lcd.write((byte) 2); // symbol sipka
      lcd.print(msg_rovnase);
      lcd.print(msg_ANO);
      lcd.print(msg_mezera);
      lcd.write((byte) 1); // symbol sipka
      lcd.print(msg_rovnase);
      lcd.print(msg_NE);
      lcd.print(msg_zavP);
      lcd.print(msg_prazdno);
      menu_state = 13;
      lcd.setCursor(0,1); lcd.blink();
      break;
   case 14: // znovu
      btnPUSHED = 0;
      lcd_menu(btnPUSHED);
      break;
    } // switch
} // end void

//**********************************************************************************************
// pomocne funkce ******************************************************************************
void vypisuj_info_lcd(){ // informace na lcd
// vzor lcd.print("                "); 16 znaku
  if (!menu){// budeme vypisovat jen kdyz neni aktivni menu a neni prace se serialem
       switch(posun_lcd){ // informacni texty na lcd pouze kdyz neni menu
         case 0:
             lcd.setCursor(0,0);
             tisk_casu();
             lcd.setCursor(0,1);
             if (celsius > -100){
               lcd.print(celsius,1);
               lcd.write((byte) 0); // nulty symbol v pameti lcd - krouzek C
               lcd.print(F("C"));
               lcd.print(msg_prazdno);
             }//end if
             else {
              lcd.print(msg_err_cidla);lcd.print(msg_otaznik);lcd.print(msg_prazdno);
              }
           break;
         case 1:
           // vlhkost
           lcd.setCursor(0,1);
           #ifdef USE_FREQ_PROBE
              if(frq==0) {lcd.print(msg_err_cidla);lcd.print(msg_otaznik);lcd.print(msg_prazdno);}
              else {lcd.print(vlhkost);lcd.print(msg_procenta);lcd.print(msg_mezera);}
              if(frq<1000 && frq>0)  {lcd.print(frq);lcd.print(msg_mezera);lcd.print(msg_Hz);lcd.print(msg_prazdno);}
              else{lcd.print(frq/1000); lcd.print(msg_mezera);lcd.print(msg_kHz);lcd.print(msg_prazdno);}
              lcd.setCursor(0,0);
           #endif   
           #ifdef USE_AD_PROBE
              lcd.print(vlhkost);lcd.print(msg_procenta);lcd.print(msg_mezera);
              lcd.print(AD_volt);lcd.print(msg_volt);lcd.print(msg_prazdno);
           #endif 
           tisk_casu();
           break;
         case 2:
           // den po-ne
           lcd.setCursor(0,0);
           tisk_casu();
           lcd.setCursor(0,1);
           switch(weekday()){ // weekday() vraci 1-7 dle dne v tydnu
             case 1:
               lcd.print(msg_dnes); // dnes je
               lcd.print(msg_mezera);
               lcd.print(msg_nedele);
             break;
             case 2:
               lcd.print(msg_dnes); // dnes je
               lcd.print(msg_mezera);
               lcd.print(msg_pondeli);
             break;
             case 3:
               lcd.print(msg_dnes); // dnes je
               lcd.print(msg_mezera);
               lcd.print(msg_utery);
             break;
             case 4:
               lcd.print(msg_dnes); // dnes je
               lcd.print(msg_mezera);
               lcd.print(msg_streda);
             break;
             case 5:
               lcd.print(msg_dnes); // dnes je
               lcd.print(msg_mezera);
               lcd.print(msg_ctvrtek);
             break;
             case 6:
               lcd.print(msg_dnes); // dnes je
               lcd.print(msg_mezera);
               lcd.print(msg_patek);
             break;
             case 7:
               lcd.print(msg_dnes); // dnes je
               lcd.print(msg_mezera);
               lcd.print(msg_sobota);
             break;
             default:
               lcd.print(msg_chyba_dne);
             break;
           }//end switch
           lcd.print(msg_prazdno);
           break;
         case 3:
           lcd.setCursor(0,0);
           lcd.print(msg_menu0); // zavlaha bude
           lcd.print(msg_prazdno);
           lcd.setCursor(14,1); lcd.print(msg_prazdno);
           lcd.setCursor(0,1);             // Nastaveni kurzoru LCD na pozici 0 sloupec, 2 radek
           if(day_po==2)lcd.print(msg_po); else {lcd.print(msg_minus);lcd.print(msg_minus);}
           lcd.setCursor(2,1);
           if(day_ut==2)lcd.print(msg_ut); else {lcd.print(msg_minus);lcd.print(msg_minus);}
           lcd.setCursor(4,1);
           if(day_st==2)lcd.print(msg_st); else {lcd.print(msg_minus);lcd.print(msg_minus);}
           lcd.setCursor(6,1);
           if(day_ct==2)lcd.print(msg_ct); else {lcd.print(msg_minus);lcd.print(msg_minus);}
           lcd.setCursor(8,1);
           if(day_pa==2)lcd.print(msg_pa); else {lcd.print(msg_minus);lcd.print(msg_minus);}
           lcd.setCursor(10,1);
           if(day_so==2)lcd.print(msg_so); else {lcd.print(msg_minus);lcd.print(msg_minus);}
           lcd.setCursor(12,1);
           if(day_ne==2)lcd.print(msg_ne); else {lcd.print(msg_minus);lcd.print(msg_minus);}
           lcd.print(msg_prazdno);
           posun_lcd=-1; // vynulujeme posun na zacatek
           break;
         } // end switch
    } //end if not menu
} // end void

long mapuj(long x, long in_min, long in_max, long out_min, long out_max){ // vraci prevedenou hodnotu tj funkce map
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
} // end long

void inicializace(){ // inicializace knihoven
#ifdef DEBUG
  Serial.begin(baud_rate);      // Inicializace Serialu
  delay(500);
#endif
  Wire.begin();
  lcd.begin(16, 2);                    // Inicializace LCD 16x2 znaku
  lcd.clear();
  lcd.createChar(0, stupen);           // vytvorime v lcd znak stupne
  lcd.createChar(1, sipka_dolu);       // vytvorime v lcd ikonu sipky dolu
  lcd.createChar(2, sipka_nahoru);     // vytvorime v lcd ikonu sipky nahoru
} // end void

void cti_cas_datum(){
  den = day();
  mesic = month();
  rok = year()-2000;  // aby bylo cislo roku 18 misto 2018
  hodina = hour();
  minuta = minute();
  vterina = second();
} // end void

void init_piny(){ // nystavy vstupy a vystupy
  pinMode(cerpadlo_pin,OUTPUT);    // Vystup pro cerpadlo
  digitalWrite(cerpadlo_pin,LOW);  // Vypneme vystup cerpadlo
  #ifdef USE_FREQ_PROBE
     pinMode(frek_pin,INPUT);      // Vstup pro citac frekvence na T1 cpu atmega
  #endif
  #ifdef USE_AD_PROBE
     pinMode(AD_probe_pin,INPUT);     // Vstup pro měření vlhkosti pomocí AD převodníku (napětí na 0-1023 číslo)
  #endif
  pinMode(lcd_led,OUTPUT);         // LED LCD vystup
  digitalWrite(lcd_led,HIGH);      // zapneme LED
} // end void

void firmware(){ // vypise reklamu a verzi fw na serial a lcd
  lcd.setCursor(0,0);                 // 1 radek LCD displeje
  lcd.print(F("www.pihrt.com"));      // Reklama
  lcd.setCursor(0,1);                 // 2 radek LCD displeje
  lcd.print(verze);                   // Verze programu zarizeni
#ifdef DEBUG
  Serial.println(F("*** MENU *************"));
  Serial.println(F("? Napoveda"));
  Serial.println(F("A Spustit program 1"));
  Serial.println(F("B Spustit program 2"));
  Serial.println(F("C Spustit program 3"));
  Serial.println(F("D Nastavit datum a cas"));
  Serial.println(F("E Smazat EEPROM"));
  Serial.println(F("F Tisk EEPROM"));
  #ifdef USE_FREQ_PROBE
     Serial.println(F("G Vypsat frekvenci ze sondy"));
  #endif   
  #ifdef USE_AD_PROBE
     Serial.println(F("G Vypsat napeti ze sondy"));
  #endif   
  Serial.println(F("H Vypsat teplotu"));
  Serial.println(F("I Vypsat datum a cas"));
  Serial.println(F("**********************"));
#endif
  delay(2000);
} // end void

void tisk_casu(){ // tiskne na prvni radek datum a cas
  print_digits_lcd(day());
  lcd.print(msg_tecka);
  print_digits_lcd(month());
  lcd.print(msg_tecka);
  lcd.print(year());
  lcd.print(msg_mezera);
  lcd.print(msg_mezera);
  lcd.setCursor(11,0);
  print_digits_lcd(hour());
  lcd.print(msg_dvojtecka);
  print_digits_lcd(minute());
  lcd.print(msg_mezera);
} // end void

void serial_clock_display(){ // tisk aktualniho systemoveho casu na serial
#ifdef DEBUG
  print_digits(hour());
  Serial.print(msg_dvojtecka);
  print_digits(minute());
  Serial.print(msg_dvojtecka);
  print_digits(second());
  Serial.print(msg_mezera);
  print_digits(day());
  Serial.print(msg_tecka);
  print_digits(month());
  Serial.print(msg_tecka);
  Serial.print(year());
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
    lcd.print(F("0"));
    }
  lcd.print(number);
}// end void

void RTC_a_restart(){ // ulozi do rtc a udela restart
#ifdef DEBUG
   Serial.println(msg_rbt);
#endif
   lcd.home();
   lcd.print(msg_rbt);
   lcd.print(msg_prazdno);
   lcd.setCursor(0,1);
   lcd.print(msg_prazdno);
   setTime(hodina,minuta,vterina,den,mesic,rok); // nastavime system cas
   RTC.set(now());                               // ulozime system cas do RTC
   while(1); // pockame na watchdog az to sestreli
} // end void

void cerpadlo(){ // rutina spinani cerpadla cas on a cas off
 if (cerpadlo_makej){
   // http://www.baldengineer.com/millis-ind-on-off-times.html
   aktualniCasCerpadlo = millis(); // nacteme aktualni cas v ms
   digitalWrite(cerpadlo_pin,cerpadlo_vystup_stav); // cerpadlo sepne nebo vypne dle stavu
   if (aktualniCasCerpadlo - predchoziCasCerpadlo >= interval){ // pokud ubehl cas pro sepnuti nebo vypnuti cerpadla (60000 ms je minuta)
     if (cerpadlo_vystup_stav){
       interval = cerpadlo_on*1000;               // on time cerpadla
       }
       else
         {
         interval = pauza_cerpadlo_off*1000;      // off time cerpadla
         } // end else
#ifdef DEBUG
     Serial.print(msg_cerpadlo);Serial.print(msg_mezera);
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

bool is_this_day(){ // vraci cislo true pokud je aktualni den jako je nastaven v menu
  switch(weekday()){ // weekday() vraci cislo dne 1-7
     case 1: // nedele
       if(day_ne==2) return true; // ano je nedele
     break;
     case 2: // pondeli
       if(day_po==2) return true; // ano je pondeli
     break;
     case 3: // utery
       if(day_ut==2) return true;
     break;
     case 4: // streda
       if(day_st==2) return true;
     break;
     case 5: // ctvrtek
       if(day_ct==2) return true;
     break;
     case 6: // patek
       if(day_pa==2) return true;
     break;
     case 7: // sobota
       if(day_so==2) return true;
     break;
  }//end switch
  return false; // neni zadny den tak false
} //end bool

void regulace_vlhk(){ // reguluje dle vlhkosti
  #ifdef USE_FREQ_PROBE
     vlhkost = mapuj(frq,cidlo_fMIN,cidlo_fMAX,100,0); // prevod frekvence na vlhkost stupnice vlhka 0-100 s ohledem na min a max frekvenci z cidla
  #endif

  #ifdef USE_AD_PROBE
     vlhkost = mapuj(AD_in,cidlo_AD_min,cidlo_AD_max,100,0); // prevod napeti na vlhkost stupnice vlhka 0-100 s ohledem na min a max napeti z cidla
  #endif
      
  if(vlhkost >= 100)  vlhkost = 100;
  if(vlhkost <1)   vlhkost = 0;

  if (is_this_day() && use_vlhkost==2 && use_teplota==1){ // kdyz je konkretni den a je povolena vlhkost ale neni povolena regulace teplotou
      if (vlhkost < 40){ // zmerena vlhkost je 10,20,30 (na stupnici 0-100)
         if (CAS1 || CAS2 || CAS3){     // je prave cas1, cas2, cas3 na casovaci
            cerpadlo_makej = true;      // povolime cerpadlo
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)
            if (aktualniCasDoba/1000 - predchoziCasDoba/1000 >= trvani_doby+(trvani_doby/2)){ // pocitame dobu1 zalevani a pak vse vypneme + 50% casu zalevani
                predchoziCasDoba = aktualniCasDoba/1000;
                CAS1 = false;             // zakazeme cas1
                CAS2 = false;             // zakazeme cas2
                CAS3 = false;             // zakazeme cas2
                cerpadlo_makej = false;   // zakazeme cerpadlo
                write_eeprom_long(adr_last_vlhkost, vlhkost); // ulozime vlhkost do eeprom na konci casu1
                Serial_ende();
                } // end if casovac
            } // end if CAS 1 2 3 - delsi interval
         } // end vlhkost
      if (vlhkost >= 40 && vlhkost < 60){ // zmerena vlhkost je 40,50 (na stupnici 0-100)
         if (CAS1 || CAS2 || CAS3){     // je prave cas1, cas2, cas3 na casovaci
            cerpadlo_makej = true;      // povolime cerpadlo
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)
            if (aktualniCasDoba/1000 - predchoziCasDoba/1000 >= trvani_doby){ // pocitame dobu1 zalevani a pak vse vypneme
                predchoziCasDoba = aktualniCasDoba/1000;
                CAS1 = false;             // zakazeme cas1
                CAS2 = false;             // zakazeme cas2
                CAS3 = false;             // zakazeme cas2
                cerpadlo_makej = false;   // zakazeme cerpadlo
                write_eeprom_long(adr_last_vlhkost, vlhkost); // ulozime vlhkost do eeprom na konci casu123
                Serial_ende();
                } // end if casovac
            } // end if CAS 1 2 3
         } // end vlhkost
      if (vlhkost >= 60 && vlhkost < 80){ // zmerena vlhkost je 60,70 (na stupnici 0-100)
         if (CAS1 || CAS2){             // je prave cas1 nebo cas 2 na casovaci
            cerpadlo_makej = true;      // povolime cerpadlo
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)
            if (aktualniCasDoba/1000 - predchoziCasDoba/1000 >= trvani_doby){ // pocitame dobu1 zalevani a pak vse vypneme
                predchoziCasDoba = aktualniCasDoba/1000;
                CAS1 = false;             // zakazeme cas1
                CAS2 = false;             // zakazeme cas2
                cerpadlo_makej = false;   // zakazeme cerpadlo
                write_eeprom_long(adr_last_vlhkost, vlhkost); // ulozime vlhkost do eeprom na konci casu12
                Serial_ende();
                } // end if casovac
            } // end if CAS 1 a 2
         } // end vlhkost
      if (vlhkost >= 80){                // zmerena vlhkost je > 80 (na stupnici 0-100)
         if (CAS1){                     // je prave cas1 na casovaci
            cerpadlo_makej = true;      // povolime cerpadlo
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)
            if (aktualniCasDoba/1000 - predchoziCasDoba/1000 >= trvani_doby){ // pocitame dobu1 zalevani a pak vse vypneme
                predchoziCasDoba = aktualniCasDoba/1000;
                CAS1 = false;             // zakazeme cas1
                cerpadlo_makej = false;   // zakazeme cerpadlo
                write_eeprom_long(adr_last_vlhkost, vlhkost); // ulozime vlhkost do eeprom na konci casu1
                Serial_ende();
                } // end if casovac
            } // end if CAS1
         } // end vlhkost
      } // end thisday
} // end void

void regulace_tepl(){ // reguluje dle teploty
  if (is_this_day() && use_teplota==2 && use_vlhkost==1){ // kdyz je konkretni den a je povolena teplota ale neni povolena vlhkost
    if (celsius >= 40.0){ // zmerena teplota je > 40 stupnu
         if (CAS1 || CAS2 || CAS3){     // je prave cas1 nebo cas2 nebo cas3 na casovaci
            cerpadlo_makej = true;      // povolime cerpadlo
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)
            if (aktualniCasDoba/1000 - predchoziCasDoba/1000 >= trvani_doby+(trvani_doby/2)){ // pocitame dobu1 zalevani a pak vse vypneme + 50% casu zalevani
                predchoziCasDoba = aktualniCasDoba/1000;
                CAS1 = false;             // zakazeme cas1
                CAS2 = false;             // zakazeme cas2
                CAS3 = false;             // zakazeme cas2
                cerpadlo_makej = false;   // zakazeme cerpadlo
                write_eeprom_long(adr_last_teplota, int(celsius*1000)); // ulozime teplotu do eeprom na konci casu123 jako pr: 23.57C = 2357 cislo
                Serial_ende();
                } // end if casovac
            } // end if CAS 1 2 3 - delsi interval
         } // end teplota
      if (celsius >= 30.0 && celsius < 40.0){ // zmerena teplota je 30-40 stupnu
         if (CAS1 || CAS2 || CAS3){     // je prave cas1 nebo cas2 nebo cas3 na casovaci
            cerpadlo_makej = true;      // povolime cerpadlo
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)
            if (aktualniCasDoba/1000 - predchoziCasDoba/1000 >= trvani_doby){ // pocitame dobu1 zalevani a pak vse vypneme
                predchoziCasDoba = aktualniCasDoba/1000;
                CAS1 = false;             // zakazeme cas1
                CAS2 = false;             // zakazeme cas2
                CAS3 = false;             // zakazeme cas2
                cerpadlo_makej = false;   // zakazeme cerpadlo
                write_eeprom_long(adr_last_teplota, int(celsius*1000)); // ulozime teplotu do eeprom na konci casu123 jako pr: 23.57C = 2357 cislo
                Serial_ende();
                } // end if casovac
            } // end if CAS 1 2 3
         } // end teplota
      if (celsius >= 20.0 && celsius < 30.0){ // zmerena teplota je 20-30 stupnu
         if (CAS1 || CAS2){     // je prave cas1 nebo cas2
            cerpadlo_makej = true;      // povolime cerpadlo
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)
            if (aktualniCasDoba/1000 - predchoziCasDoba/1000 >= trvani_doby){ // pocitame dobu1 zalevani a pak vse vypneme
                predchoziCasDoba = aktualniCasDoba/1000;
                CAS1 = false;             // zakazeme cas1
                CAS2 = false;             // zakazeme cas2
                cerpadlo_makej = false;   // zakazeme cerpadlo
                write_eeprom_long(adr_last_teplota, int(celsius*1000)); // ulozime teplotu do eeprom na konci casu12 jako pr: 23.57C = 2357 cislo
                Serial_ende();
                } // end if casovac
            } // end if CAS 1 2
         } // end teplota
      if (celsius >= 5.0 && celsius < 20.0){ // zmerena teplota je 5-20 stupnu
         if (CAS1){     // je prave cas1
            cerpadlo_makej = true;      // povolime cerpadlo
            aktualniCasDoba = millis();   // aktualni cas doby zalevani (doba1)
            if (aktualniCasDoba/1000 - predchoziCasDoba/1000 >= trvani_doby){ // pocitame dobu1 zalevani a pak vse vypneme
                predchoziCasDoba = aktualniCasDoba/1000;
                CAS1 = false;             // zakazeme cas1
                cerpadlo_makej = false;   // zakazeme cerpadlo
                write_eeprom_long(adr_last_teplota, int(celsius*1000)); // ulozime teplotu do eeprom na konci casu1 jako pr: 23.57C = 2357 cislo
                Serial_ende();
                } // end if casovac
            } // end if CAS 1
         } // end teplota
      } // end thisday
} // end void

void Serial_ende(){ // tiskne msg end
#ifdef DEBUG
  Serial.println(msg_end);
#endif
}//end void

void init_teplota(){
  sensors.begin();    // pro DS18B20
}//end void

void cti_teplotu(){                      // #include <DallasTemperature.h> pomoci teto knihovny se neceka 1000ms ale meri se hned :-)
   sensors.requestTemperatures();        // pozadavek na zmereni teploty
   celsius = sensors.getTempCByIndex(0); // index 0 je prvni cidlo dalas
} // end void

//**********************************************************************************************
// serial rx tx ********************************************************************************
void cti_serial(){
#ifdef DEBUG
    if (Serial.available()>0) {
      int znak = Serial.read();
      if (znak == 'A') {  // spustit casovac cas1
         ZavlahaCas(1);
         } // end if
      if (znak == 'B') {  // spustit casovac cas2
         ZavlahaCas(2);
         } // end if
      if (znak == 'C') {  // spustit casovac cas3
         ZavlahaCas(3);
         } // end if
      if (znak == 'D') { // klavesa D nastaveni casu a datumu
         setTimeData();
         RTC_a_restart(); // pomocne funkce
         } // end if
      if (znak == 'E') { // smazat a default hodnoty eeprom
         clear_eeprom(); init_eeprom();
         vterina = 0; minuta = 0; hodina = 0; den = 1; mesic = 1; rok = 18;
         RTC_a_restart();
         } // end if
      if (znak == 'F') {  // vypsat eeprom
         ee_read();
         } // end if
      #ifdef USE_FREQ_PROBE   
          if (znak == 'G') {  // vypsat frekvenci a vlhkost
            Mereni();
            Serial.print(frq);
            Serial.print(msg_mezera);
            Serial.println(vlhkost);
          } // end if
      #endif    
      #ifdef USE_AD_PROBE
          if (znak == 'G') {  // vypsat napětí a vstup
            Serial.print(AD_in);
            Serial.print(msg_mezera);
            Serial.print(AD_volt);
            Serial.println(msg_volt);
          } // end if
      #endif       
      if (znak == 'H') {  // vypsat teplotu
         Mereni();
         Serial.println(celsius);
         } // end if
      if (znak == 'I') {  // vypsat datum a cas
         serial_clock_display();
         } // end if
      if (znak == '?') {  // vypsat menu
         firmware();
         } // end if
      } // end serial
#endif
} // end void

void setTimeData(){ // vkladani udaju ze serialu
#ifdef DEBUG
  Serial.print(msg_vloz); Serial.print(F(" Rok 19-99: "));
  rok = readByte(); // musi byt 2x jinak se hned vlozi nula a skoci na mesic
  rok = readByte();
  Serial.println(rok);
  Serial.print(msg_vloz); Serial.print(F(" Den 1-31: "));
  den = readByte();
  Serial.println(den);
  Serial.print(msg_vloz); Serial.print(F(" Mes 1-12: "));
  mesic = readByte();
  Serial.println(mesic);
  Serial.print(msg_vloz); Serial.print(F(" Hod 0-23: "));
  hodina = readByte();
  Serial.println(hodina);
  Serial.print(msg_vloz); Serial.print(F(" Min 0-59: "));
  minuta = readByte();
  Serial.println(minuta);
  Serial.print(msg_vloz); Serial.print(F(" Sec 0-59: "));
  vterina = readByte();
  Serial.println(vterina);
#endif
  RTC_a_restart();
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

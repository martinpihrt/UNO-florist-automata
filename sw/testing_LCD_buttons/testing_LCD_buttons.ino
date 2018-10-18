#include <LiquidCrystal.h>

LiquidCrystal lcd( 8, 9, 4, 5, 6, 7 );

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

int lcd_key = 0;
int adc_key_in = 0;

int read_LCD_buttons() {

    adc_key_in = analogRead( 0 );

    Serial.print( F( "\nadc_key: " ) );
    Serial.print( adc_key_in );

    if ( adc_key_in < 50 ) return btnRIGHT;
    if ( adc_key_in < 195 ) return btnUP;
    if ( adc_key_in < 380 ) return btnDOWN;
    if ( adc_key_in < 555 ) return btnLEFT;
    if ( adc_key_in < 790 ) return btnSELECT;

    return btnNONE;
}

void setup() {
    lcd.begin( 16, 2 );
    lcd.setCursor( 0, 0 );
    lcd.print( "  www.pihrt.com  " );
    lcd.setCursor( 0, 1 );
    lcd.print( "1602 LCD  Keypad" );
    delay( 5000 );
    lcd.clear();
    lcd.setCursor( 0, 0 );
    lcd.print( "Press buttons" );
    Serial.begin( 9600 );
}

void loop() {
    lcd.setCursor( 12, 1 );
    lcd.print( millis() / 1000 );
    lcd.setCursor( 0, 1 );
    lcd_key = read_LCD_buttons();

    switch ( lcd_key ) {
        case btnRIGHT: {
            Serial.print( F( "RIGHT" ) );
            lcd.print( "RIGHT" );
            break;
        }
        case btnLEFT: {
            Serial.print( F( "LEFT" ) );
            lcd.print( "LEFT" );
            break;
        }
        case btnUP: {
            Serial.print( F( "UP" ) );
            lcd.print( "UP" );
            break;
        }
        case btnDOWN: {
            Serial.print( F( "DOWN" ) );
            lcd.print( "DOWN" );
            break;
        }
        case btnSELECT: {
            Serial.print( F( "SELECT" ) );
            lcd.print( "SELECT  " );
            break;
        }
        case btnNONE: {
            lcd.print( "-NONE-" );
            break;
        }
    }
}

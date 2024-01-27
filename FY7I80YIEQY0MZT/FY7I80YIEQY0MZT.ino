/////                   ARDUINO MPPT SOLAR CHARGE CONTROLLER  V-3.1          //
//    ------------------------------------------------------------------------

//  Author: Debasish Dutta, Keith Hungerford 
//          www.opengreenenergy.in
//  This code is for an arduino Nano based Solar MPPT charge controller ( V-3.1)
// last  update 20 September 2015

//  Specifications:
//  - Solar Panel Wattage= 150W or more if some power is not used
//  - Solar Panel Voltage = 39V max OC voltage 
//  - Battery Charging Current= 10A max
//  - developed for Lead Acid 12 volt battery (6 cells in series)

//----------------------------------------------------------------------------------------------------
// Arduino pin Connections----------------------------------------------------------------------------

  // A0 - Voltage divider to measure solar panel voltage
  // A1 - ACS 712 Out ( solar panel current measurement )
  // A2 - Voltage divider to measure battery voltage
  // A3 - ACS 712 Out ( battery current measurement )
  // A4 - LCD SDA
  // A5 - LCD SCL
  // A6 - ACS 712 Out ( load current measurement )

  // D0- reserved for USB port
  // D1- reserved for USB port
  // D2 - WiFi ESP8266 Tx
  // D3 - WiFi ESP8266 Rx through the voltage divider
  // D4 - temperature from DS18B20 probe 
  // D5 - LCD back light control button
  // D6 - Load Control 
  // D7 - not used
  // D8 - not used
  // D9 - PWM MOSFET driver IR2104-1 IN & SD +  IR2104-2 IN
  // D10- PWM MOSFET driver IR2104-2 SD
  // D11- Green LED
  // D12- Yellow LED
  // D13- Red LED

#include <Timer1Fast.h>
#include <LiquidCrystal_I2C.h>      // using the LCD I2C Library from https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
#include <Wire.h>  
//  #include<avr/io.h>


  // Definitions :

  // Analogue pins  
  #define solar_volt_sense 0                    // defining the analog pin A0 to read solar panel Voltage
  #define solar_current_sense 1                 // defining the analog pin A1 to read solar panel current
  #define bat_volt_sense 2                      //defining the analog pin A2 to read battery voltage
  #define bat_current_sense 3                 // defining analog pin A3 to read battery current
  #define load_current_sense 6                 // defining analog pin A6 to read load current

  //Defining ESP8266 Tx and Rx pin
  #define ESP_Tx 2
  #define ESP_Rx 3

  // Digital pins
  // Defining lcd back light pin
  #define back_light_pin 5  

  // Defining load control pin
  #define load_pin 6
  // Defining IR2104 pins used for PWM
  #define SD_pin 10                     // pin 10 is used to control shutoff function of MOSFET driver IR2104-2  
  #define IN_pin 9                       // pin 9 is used for IR2104-2 IN and both IN and SD of IR2104-1 

  //Defining led pins for indication
  #define green_led 11 
  #define yellow_led 12 
  #define red_led 13  
   
//    #define ACS712_sensitivity 0.185          // ACS712 5A module has sensitivity 185mV/A // We have to change it for other ACS712 module
//    #define ACS712_offset 2.5                 // ACS712 has offset of 2.5V //  The current value is zero when sensor read 2.5V

  // Defining the scaling factors
  // To calibrate we assume arduino Vcc is 5V
 //  scaling for floating point implementation
 //  #define battery_voltage_scale 0.029296875          // (5V/1024)* ( ( 100+20)/20 ) // R1=100k and R2 =20K                                  
 //  #define solar_voltage_scale 0.04150390625  // (5V/1024)* ( ( 150+20)/20 ) // R1=150k and R2 =20K                                  
 //  # define current_scale 0.026393581                  // 5V/(0.185*1024) - amps per step 
 //  # define current_offset 13.5135                         // 2.5V * amps per step = offset 

//  scaling for integer implementation
   #define battery_voltage_scale 30000                   // (5V * ( 100+20)/20 ) // R1=100k and R2 =20K Full scale reading in millivolts                                 
   #define solar_voltage_scale 42500                       // (5V * ( ( 150+20)/20 ) // R1=150k and R2 =20K Full scale reading in millivolts                                 
   #define current_scale 27027                              // 5V/0.185*1000 - full scale reading in milli amps 
   #define current_offset 5090                              // setting to get zero measured current when current is zero  
   #define DCMCCM 5                 // fraction of maximum current for changing between DCM and CCM (eg 20% for 5)

// define macros for direct PWM on pins
#define output_low(port,pin) port &= ~(1<<pin)
#define output_high(port,pin) port |= (1<<pin) 


//-------------------------------------------BIT MAP ARRAY-------------------------------------------------


  byte battery_icons[6][8]=
  {{
    0b01110,
    0b11011,
    0b10001,
    0b10001,
    0b10001,
    0b10001,
    0b10001,
    0b11111,
  },
  {
    0b01110,
    0b11011,
    0b10001,
    0b10001,
    0b10001,
    0b10001,
    0b11111,
    0b11111,
  },
  {
    0b01110,
    0b11011,
    0b10001,
    0b10001,
    0b10001,
    0b11111,
    0b11111,
    0b11111,
  },
  {
    0b01110,
    0b11011,
    0b10001,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
  },
  {
    0b01110,
    0b11011,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
  },
  {
    0b01110,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
  }};
  #define SOLAR_ICON 6
  byte solar_icon[8] = //icon for termometer
  {
    0b11111,
    0b10101,
    0b11111,
    0b10101,
    0b11111,
    0b10101,
    0b11111,
    0b00000
  };
  #define PWM_ICON 7
  byte _PWM_icon[8]=
  {
    0b11101,
    0b10101,
    0b10101,
    0b10101,
    0b10101,
    0b10101,
    0b10101,
    0b10111,
  };
  byte backslash_char[8]= 
  {
    0b10000,
    0b10000,
    0b01000,
    0b01000,
    0b00100,
    0b00100,
    0b00010,
    0b00010,
  };
//------------------------------------------------------------------------------------------------------


  //Declaring global variables

  //  serial monitor options
  String MONITOR = "DEBUG" ; // "INPUT" ;
  int first_time =  0 ;


  //  measurement variables
  long  solar_volt;                      // solar voltage long
  long  solar_amps;                   // solar current  signed long
  // int solar_amps_balance ;        // filtering long term value of solar_amps
  //  int solar_amps_decay = 100 ;
  long bat_volt ;                    // battery voltage long
  long bat_amps;                     // battery current signed long
  long solar_watts;                  // solar watts signed long
  long old_solar_watts; 
  unsigned long old_millis ;                    // millis time on previous pass unsigned long
  unsigned long old_micros ;                  // micros time on previous pass unsigned long
  int temperature ;                           // probably in centi-degrees Celsius, ie value 100 = 0.1 C; 25C = 2500
  int back_light_pin_State = 0;         // variable for storing the state of the backlight button
  String battery_type = "Flooded";   //Declare a String variable to hold battery type
  String command;
  String load_mode = "ALL" ;
 
  // battery management values in millivolts
  unsigned int  float_voltage1 = 13800 ;
  unsigned int  float_voltage ;
  unsigned int  boost_voltage1 = 14200 ;
  unsigned int  boost_voltage ;
  unsigned int  boost_reconnect_voltage1 = 13200 ;
  unsigned int  boost_reconnect_voltage ;
  unsigned int  low_voltage_disconnect1 = 11100 ;
  unsigned int  low_voltage_disconnect ;
  unsigned int  low_voltage_reconnect1 = 12600 ;
  unsigned int  low_voltage_reconnect ;
  unsigned long boost_duration = 1000 ; // for testing only 3600000  ;  // 1 hour in milliseconds, allowed boost in 1 day
  int temperature_compensation = -18 ; // millivolts per degree Celsius
  long max_current =10000 ; // maximum design current in milliamps
  

  unsigned long boost_today ;
  // startup thresholds in millivolts
  unsigned int startup_threshold = 12000 ;
  int startup_margin = 500 ;
  long pwm1 ; // this variable is used for PWM ratio calculations for IN-pin. 
  long pwm2 ; // this variable is used for PWM ratio calculations for SD_pin. 
  int track_direction ; // tracking direction for MPPT
  long sample4 = 0 ;
  
  // Enumerated variable to manage the charging state machine  
  enum charger_state {off, // controller is off waiting for changed external conditions 
        test_nil, // test state
        test,     // test state
        startup,   // initialise current flowing the right way
        float_DCM, // manages for battery voltage at float_voltage
        boost_float_DCM, // manages for battery voltage at boost_voltage
        bulk_DCM,  // MPPT with limited solar output
        boost_DCM, // MPPT with limited solar output and battery above Float for a limited time
        boost_CCM, // MPPT with battery above Float for a limited time
        bulk_CCM,  // MPPT with battery below Float
        current_limited_CCM, // manages for limited current capacity under float_voltage 
        current_limited_boost_CCM // manages for limited current capacity under boost_voltage 
        } charger_state ;  
  
 unsigned long charger_millis ;
 int off_timer ; 
 int counter = 0 ;

  // set the LCD address to 0x27 for a 20 chars 4 line display
  // Set the pins on the I2C chip used for LCD connections:
  //                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
  LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
  unsigned long time = 0;               // variable to store time the back light control button was pressed in millis     

 
//---------------------------------------- Set Up Function----------------------------------------------------------------------------------------------------------------------------------

  void setup() {
  
  Serial.begin(9600);                  // open the serial port at 9600 bps:
  lcd.begin(20,4);                     // initialize the lcd for 16 chars 2 lines, turn on backlight

  // create the LCD special characters. Characters 0-5 are the various battery fullness icons
  // icon 7 is for the PWM icon, and icon 8 is for the solar array
  lcd.backlight();
  for (int batchar = 0; batchar <   6; ++batchar) {
    lcd.createChar(batchar, battery_icons[batchar]);
  }
  lcd.createChar(PWM_ICON,_PWM_icon);
  lcd.createChar(SOLAR_ICON,solar_icon);
  lcd.createChar('\\', backslash_char);
  pinMode(red_led, OUTPUT);            
  pinMode(green_led, OUTPUT);          
  pinMode(yellow_led, OUTPUT);        
  pinMode(IN_pin, OUTPUT);            
  pinMode(SD_pin,OUTPUT);
  pinMode(back_light_pin, INPUT);      // backlight control button
  pinMode(load_pin,OUTPUT);            // output for the LOAD MOSFET (LOW = on, HIGH = off)
  old_millis =  millis() ;
  old_micros = micros() ;
  charger_state = off ;
  charger_millis = millis() ;
  track_direction = 1 ;
   // display the constant stuff on the LCD
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("SOL");
  lcd.setCursor(4, 0);
  lcd.write(SOLAR_ICON);
  lcd.setCursor(8, 0);
  lcd.print("BAT");
  lcd.setCursor(15,0);
  lcd.print("PWM");
  lcd.setCursor(19,0);
  lcd.write(PWM_ICON);
   //delay(1000);
  }  // end of setup

  
 
  //-----------------------------------------------------------Main Loop-------------------------------------------------------------------------------------------------------------------

  void loop() {
  
  read_sensors_data();         // for measuring voltage and currents from sensors
  run_charger();                  // Charge controller state machine and associated algorithms
  load_control();                  // control the connected load
  led_indication();               // led indication for battery SOC
  lcd_display();                   // display important parameters  
  wifi_datalogging();          // sends data to thingspeak through ESP8266 
  serial_print_data();           // print data on serial monitor
  
}
  
  
//----------------------------------------------------------------Function Definition-------------------------------------------------------------------------------------------------------


//---------------------------------------------------------------Function Definition for all sensors reading -------------------------------------------------------------------------------

/* 
   This function reads the voltage dividers and current sensors ( ACS712),gives row adc values in between 0-1023
   Then adc value is calibrated to get the actual voltages and currents
   
   Current Status : Working 
*/

  void read_sensors_data() // function for reading analog inputs
  {
 int sample0=0; // temporary variable to store adc value from solar_volt_sense (A0)
 int sample1=0; // temporary variable to store adc value from solar_current_sense (A1)
 int sample2=0; // temporary variable to store adc value from bat_volt_sense (A2)
 
   for(int i=0;i<10;i++){
     sample0 += analogRead(solar_volt_sense);            // read and accumulate the solar panel voltage
     sample1 += analogRead(solar_current_sense);      // read and accumulate the solar panel current
     sample2 += analogRead(bat_volt_sense);               // read and accumulate the battery voltage
     delayMicroseconds(10);                                         // pauses for 50 microseconds  
  } 
   
   solar_volt = sample0  ;         // multiply the averaged ADC value by the scaling factor to get actual solar panel voltage in millivolts
   solar_volt = solar_volt   * solar_voltage_scale ; 
   solar_volt = solar_volt   / 10240;
   bat_volt = sample2  ;           // multiply the averaged ADC value by the scaling factor to get actual battery voltage in millivolts
   bat_volt = bat_volt * battery_voltage_scale  ;  
   bat_volt = bat_volt / 10240 ;  
   
   solar_amps = sample1- current_offset  ;    // remove offset first (511) then multiply by current scale to get solar amps in milliamps
   solar_amps = solar_amps  * current_scale ;
   solar_amps = solar_amps   / 10240 ;
   
   bat_amps = solar_amps * solar_volt ;   // scale up solar amps by PWM factor
   bat_amps = bat_amps / bat_volt ;
   
   solar_watts = solar_amps * solar_volt  ; // 
   solar_watts = solar_watts / 1000 ; // 

  // read temperature from DS18B20 probe via D4 
  // temperature stub
  temperature = 2500 ; // nominal temperature assumption for zero adjustment
  // ToDo - add smoothing or range for temp to stop noise
  
  float_voltage = float_voltage1 + ((temperature - 2500) * temperature_compensation / 100) ;
  boost_voltage = boost_voltage1 + ((temperature - 2500) * temperature_compensation / 100) ;
  boost_reconnect_voltage = boost_reconnect_voltage1 + ((temperature - 2500) * temperature_compensation / 100) ;
  low_voltage_disconnect = low_voltage_disconnect1 + ((temperature - 2500) * temperature_compensation / 100) ;
  low_voltage_reconnect = low_voltage_reconnect1 + ((temperature - 2500) * temperature_compensation / 100) ;
  
  
   
  // Check time
  if (millis() < old_millis) {  // we have to wait
  unsigned long delaytime = 1000 - micros() + old_micros ;
  delaytime = max(delaytime, 0) ;  
  delayMicroseconds(delaytime); 
  old_micros = micros() ;
  old_millis = millis() ; 
  } //  end of if  millis
} // end of read sensors data

// ------------------------------------------------ functions used by run_charger ---------
void startup_part_2 () {
  output_high (PORTB, 2);  // sets pin D10 (PortB pin 2) high
  output_high (PORTB, 1);  // sets pin D9 (PortB pin 1) high
  delayMicroseconds(17);

  // turn off Q2 and turn on Q3
  output_low (PORTB, 1);  //  sets pin D9 (PortB pin 1) low 
  delayMicroseconds(2);
  // turn off Q3
  output_low (PORTB, 2);  //  sets pin D10 (PortB pin 2) low 
  
 // startup section 3: to start PWM on IR2104-2 turning on Q2 only
  // we want to run this 10 times
  for (int i = 0; i<10; i++) {
  // First turn on Q2 (may not turn it on if Vb is not pumped up)
  output_high (PORTB, 2);  // sets pin D10 (PortB pin 2) high
  output_high (PORTB, 1);  // sets pin D9 (PortB pin 1) high
  delayMicroseconds(15);
  
  // turn off Q2 (but not turn on Q3) for 5 microseconds minus guard time of 0.5 us)
  output_low (PORTB, 2);  //  sets pin D10 (PortB pin 2) low 
  output_low (PORTB, 1);  //  sets pin D9 (PortB pin 1) low 
  delayMicroseconds(5);
  sample4 += analogRead(solar_current_sense);      // read and accumulate the solar panel current
  }  //end of For loop

  sample4 = sample4-current_offset ;  //  ((current_offset * 4) / 10) ;
  sample4 = sample4 * current_scale ;
  sample4 = sample4 / 10240 ; // use 4096 for 4 samples ;  

        Serial.print (millis()) ;
        Serial.print (" Startup current1 =");
        Serial.print (sample4);
        Serial.print (" Bat_amps =");
        Serial.print (bat_amps);
        Serial.print (" Solar_amps =");
        Serial.print (solar_amps);
        Serial.print (" Counter =");
        Serial.println (counter);
   }


//-------------------------------------------------------Function definition for main MPPT------------------------------------------------------------------------------------------------
  void  run_charger(){

  switch (charger_state) {
// ______________________________________________________________   
   case off : 
     // Check time
  
     if (millis() > charger_millis+3600000) { // guard time 1 hour to make sure it is night 
       if (((bat_volt > 12000) && (solar_volt > bat_volt + 500)) || ((bat_volt <= 12000) && (solar_volt > 12500))) { 
          charger_state = startup ; 
          if (bat_volt < boost_reconnect_voltage) {
         boost_today = 0 ; // reset counter for boost today which limits daily boost time
        }
       }
     }
     else if (millis() > charger_millis+1000) {  // make sure we have been in state for 1 second 
       if (((bat_volt > 12000) && (solar_volt > bat_volt + 500)) || ((bat_volt <= 12000) && (solar_volt > 12500))) { 
       charger_state = startup ; }
     } // end of else part of if millis() 1 second limit 

   break; // end of case off condition
   
// ______________________________________________________________   
   case startup :
   // during this state, the controller starts the current flowing from the panel towards the battery
   // startup section 1: to initialise the IR2104-1 and turn on Q1
  
  output_high (PORTB, 1);  // sets pin D9 (PortB pin 1) high
  delayMicroseconds(3);
  output_low (PORTB, 1);  //  sets pin D9 (PortB pin 1) low 
  delayMicroseconds(2);
  output_high (PORTB, 1);  // sets pin D9 (PortB pin 1) high
  delayMicroseconds(3);
  output_low (PORTB, 1);  //  sets pin D9 (PortB pin 1) low 
  delayMicroseconds(2);
// ******************************
  startup_part_2 () ;
  // startup section 2: to turn on Q3 once
  
  if (sample4 < 100) { // minimum current 100 mA average over 4 pulses
    // charger_state = off ;
    // charger_millis = millis() ;
    // break ;
    // } // end of current (Amps) test
     sample4 = 0 ;
    
// ******************************************

// ******************************
  startup_part_2 () ;
  if (sample4 < 100) { // minimum current 100 mA average over 4 pulses
     charger_state = off ;
     charger_millis = millis() ;
     sample4 = 0 ;
     break ; 
    } // end of current (Amps) test
  } 

// ******************************************
  // startup section 4: start PWM for DCM at 100 microseconds and 10% duty cycle
  // suitable for float_DCM and other low current DCM states 

   Timer1Fast.initializeFast(100); // set a period of length 100 uS =1600 clock cycles
   // Start PWM with 10% duty cycle on IN pin and SD pin synchronised together to give Asynchronous operation (Q3 OFF)
   pwm1 = 13106 ; // // 6553 = 65536 / 10 not enough
   pwm2 = 13106 ; // // 6553 = 65536 / 10
   // pwm1 = 12451 ; // 19%
   // pwm2 = 14417 ; // 22%
   Timer1Fast.startPwm(IN_pin, pwm1, 0);  
   Timer1Fast.startPwm(SD_pin, pwm2, 0);  

  if (boost_today < boost_duration) {
    if (bat_volt > boost_voltage - 15) charger_state = boost_float_DCM ;
    else if (bat_volt > float_voltage - 15) charger_state = boost_DCM ;
    else charger_state = bulk_DCM ;
    } else { 
    if (bat_volt > float_voltage - 15) charger_state = float_DCM ;
    else charger_state = bulk_DCM ; 
    }
        sample4 = 0 ;
//       charger_state = test  ; // KH
    
   break; // end of case startup condition
// ______________________________________________________________
   case test_nil : // does nothing
        


   break ; // end of test_nil state
// ______________________________________________________________
   case test : // 
 
      delayMicroseconds(100) ;
      counter ++ ;
  if (solar_amps < 0) { // minimum current 0 mA normal reading
     charger_state = off ;
     charger_millis = millis() ;
     Timer1Fast.disablePwm (SD_pin); // cease PWM on pin 10
     Timer1Fast.disablePwm (IN_pin); // cease PWM on pin 9
     break ; 
  } // end of current (Amps) test



   break ; // end of test state
// ______________________________________________________________   
   case float_DCM : // manages for battery voltage to stay at float voltage 
  if ((solar_amps < 0) || (solar_volt < bat_volt + 500) || (bat_volt > boost_voltage + 1000)) { 
    // minimum current 0 mA normal reading
    // not enough sunlight; or
    // too much current so battery exceeds allowed float voltage + margin for transition from boost_float
     charger_state = off ;
     charger_millis = millis() ;
     Timer1Fast.disablePwm (SD_pin); // cease PWM on pin 10
     Timer1Fast.disablePwm (IN_pin); // cease PWM on pin 9
     break ; 
  } // end of state off tests 
  else if (bat_volt < float_voltage-105) { // large enough gap to go to bulk_DCM 
      charger_state = bulk_DCM ;
      old_solar_watts = solar_watts ;
      break ;
      }

   // cycle to maintain Vb at float_voltage 
   if (bat_volt > float_voltage+15) { // Bat_volt step size is 30; decrease PWM so as to reduce charger current
     
     if (pwm1 > 1310) {
      pwm1 = pwm1 - 41  ; 
      pwm2 = pwm1 * (solar_volt - bat_volt) ;
      pwm2 = pwm2 / solar_volt;
      pwm2 = pwm2 / 2 ;
      pwm2 = pwm2 + pwm1 ;
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
      Timer1Fast.setPwmDuty(SD_pin, pwm2) ;
    }
    }
   else if (bat_volt < float_voltage-15) { // Bat_volt step size is 30; increase PWM so as to increase charger current
   
      if (pwm1 < 64215) {
      pwm1 = pwm1 + 41 * 16 ; 
      pwm2 = pwm1 * (solar_volt - bat_volt) ;
      pwm2 = pwm2 / solar_volt;
      pwm2 = pwm2 / 2 ;
      pwm2 = pwm2 + pwm1 ;
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
      Timer1Fast.setPwmDuty(SD_pin, pwm2) ;
     }
   
    }
   
   break ; // end of case float_DCM condition

// ________________________________________________________
   case boost_float_DCM : // manages for battery voltage to stay at boost voltage 
     if ((solar_amps < 0) || (solar_volt < bat_volt + 500) || (bat_volt > boost_voltage + 1000)) { 
      // minimum current 0 mA normal reading; or
      // not enough sunlight ; or
      // too much current so battery exceeds allowed boost voltage
     charger_state = off ;
     charger_millis = millis() ;
     Timer1Fast.disablePwm (SD_pin); // cease PWM on pin 10
     Timer1Fast.disablePwm (IN_pin); // cease PWM on pin 9
     break ; 
  } // end of current (Amps) test

   //  test for boost duration
    boost_today = boost_today + millis() - charger_millis ;
    charger_millis = millis();
    if (boost_today > boost_duration) {
      charger_state = float_DCM ;
      break ;
    }

   if (bat_volt < boost_voltage-105) { // large enough gap to go to boost_DCM 
      charger_state = boost_DCM ;
      old_solar_watts = solar_watts ;
      break ;
      }

   // maintain float at boost_voltage 
   if (bat_volt > boost_voltage+15) { // Bat_volt step size is 30; decrease PWM so as to reduce charger current
      if (pwm1 > 1310) {
      pwm1 = pwm1 - 41  ; 
      pwm2 = pwm1 * (solar_volt - bat_volt) ;
      pwm2 = pwm2 / solar_volt;
      pwm2 = pwm2 / 2 ;
      pwm2 = pwm2 + pwm1 ;
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
      Timer1Fast.setPwmDuty(SD_pin, pwm2) ;
    }
   }
   else if (bat_volt < boost_voltage-15) { // Bat_volt step size is 30; increase PWM so as to increase charger current
     if (pwm1 < 64215) {
      pwm1 = pwm1 + 41 * 16 ; 
      pwm2 = pwm1 * (solar_volt - bat_volt) ;
      pwm2 = pwm2 / solar_volt;
      pwm2 = pwm2 / 2 ;
      pwm2 = pwm2 + pwm1 ;
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
      Timer1Fast.setPwmDuty(SD_pin, pwm2) ;
     }
   }

   break ; // end of case boost_float_DCM condition

// ______________________________________________________________   
   case bulk_DCM : // MPPT with limited solar output
  if (solar_amps < 0) { // minimum current 0 mA normal reading
     charger_state = off ;
     charger_millis = millis() ;
     Timer1Fast.disablePwm (SD_pin); // cease PWM on pin 10
     Timer1Fast.disablePwm (IN_pin); // cease PWM on pin 9
     break ; 
  } // end of current (Amps) test

    if (solar_volt < bat_volt + 500) {    // not enough sunlight 
    charger_state = off ; 
    charger_millis = millis() ;
    Timer1Fast.disablePwm (SD_pin); // cease PWM on pin 10
    Timer1Fast.disablePwm (IN_pin); // cease PWM on pin 9
    break ;
    } 
    if (bat_volt > float_voltage) { // Battery has reached Float voltage 
      if (boost_today < boost_duration) { // Boost allowed
        charger_state = boost_DCM ;
        charger_millis = millis() ;
        break ;
      }
      else {  
      charger_state = float_DCM ;
      break ;
      }
    }
    // change state if current exceeds 10% of rating
    if (bat_amps > max_current/ DCMCCM)  { 
        charger_state = bulk_CCM ;
        Timer1Fast.startPwm(IN_pin, pwm1, 12);  
        digitalWrite(SD_pin, HIGH); // constant high level to SD
        break ;
    }

    // MPPT tracking section
      if (track_direction = 1) {
      if (pwm1 < 64215) {
      pwm1 = pwm1 + 41 * 16 ; 
      pwm2 = pwm1 * (solar_volt - bat_volt) ;
      pwm2 = pwm2 / solar_volt;
      pwm2 = pwm2 / 2 ;
      pwm2 = pwm2 + pwm1 ;
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
      Timer1Fast.setPwmDuty(SD_pin, pwm2) ;
     }
    } else if (pwm1 > 1310) {
      pwm1 = pwm1 - 41  ; 
      pwm2 = pwm1 * (solar_volt - bat_volt) ;
      pwm2 = pwm2 / solar_volt;
      pwm2 = pwm2 / 2 ;
      pwm2 = pwm2 + pwm1 ;
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
      Timer1Fast.setPwmDuty(SD_pin, pwm2) ;
    }
    if (solar_watts < old_solar_watts) {
      track_direction = track_direction * -1 ;
      } 
    old_solar_watts = solar_watts ;
   
   break ; // end of case bulk_DCM condition

// ______________________________________________________________   
   case boost_DCM : // MPPT with limited solar output and battery above Float for a limited time
  if (solar_amps < 0) { // minimum current 0 mA normal reading
     charger_state = off ;
     charger_millis = millis() ;
     Timer1Fast.disablePwm (SD_pin); // cease PWM on pin 10
     Timer1Fast.disablePwm (IN_pin); // cease PWM on pin 9
     break ; 
  } // end of current (Amps) test

    boost_today = boost_today + millis() - charger_millis ;
    charger_millis = millis();
    if (boost_today > boost_duration) {
      if (bat_volt > float_voltage) charger_state = float_DCM ;
      else charger_state = bulk_DCM ;
      break ;
    }

    if (solar_volt < bat_volt + 500) {    // not enough sunlight 
     charger_state = off ; 
     Timer1Fast.disablePwm (SD_pin); // cease PWM on pin 10
     Timer1Fast.disablePwm (IN_pin); // cease PWM on pin 9
     break ; 
    } 
    
    if (bat_volt > boost_voltage) { // Battery has reached Boost voltage 
     charger_state = boost_float_DCM ;
     break ;
    }
    // change state if current exceeds 10% of rating
    if (bat_amps > max_current/DCMCCM) {
        charger_state = boost_CCM ;
        Timer1Fast.startPwm(IN_pin, pwm1, 12);  
        digitalWrite(SD_pin, HIGH); // constant high level to SD
        break ;
       }

    // MPPT tracking section
      if (track_direction = 1) {
      if (pwm1 < 64215) {
      pwm1 = pwm1 + 41 ; 
      pwm2 = pwm1 * (solar_volt - bat_volt) ;
      pwm2 = pwm2 / solar_volt;
      pwm2 = pwm2 / 2 ;
      pwm2 = pwm2 + pwm1 ;
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
      Timer1Fast.setPwmDuty(SD_pin, pwm2) ;
     }
    } else if (pwm1 > 1310) {
      pwm1 = pwm1 - 41 ; 
      pwm2 = pwm1 * (solar_volt - bat_volt) ;
      pwm2 = pwm2 / solar_volt;
      pwm2 = pwm2 / 2 ;
      pwm2 = pwm2 + pwm1 ;
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
      Timer1Fast.setPwmDuty(SD_pin, pwm2) ;
    }
    if (solar_watts < old_solar_watts) {
      track_direction = track_direction * -1 ;
      } 
    old_solar_watts = solar_watts ;    

   break ; // end of case boost_DCM condition

// ______________________________________________________________   
   case bulk_CCM :  // MPPT with battery below Float
  if (solar_amps < 0) { // minimum current 0 mA normal reading
     charger_state = off ;
     charger_millis = millis() ;
     Timer1Fast.disablePwm (SD_pin); // cease PWM on pin 10
     Timer1Fast.disablePwm (IN_pin); // cease PWM on pin 9
     break ; 
  } // end of current (Amps) test

    if (bat_volt > float_voltage) { // Battery has reached Float voltage 
      if (boost_today < boost_duration) { // Boost allowed
        charger_state = boost_CCM ;
        charger_millis = millis() ;
        break ; 
      }
      else {  
      charger_state = float_DCM ;
      Timer1Fast.initializeFast(100); // set a period of 100 uS =1600 clock cycles
      // Start PWM with 2% duty cycle on IN pin and SD pin synchronised together to give Asynchronous operation (Q3 OFF)
      pwm1 = 1310 ; // 1310 = 65536 / 50
      pwm2 = 1310 ; // 1310 = 65536 / 50
      Timer1Fast.startPwm(IN_pin, pwm1, 0);  
      Timer1Fast.startPwm(SD_pin, pwm2, 0);  // check
      break ;
      }
    }
    // change state if current falls below 10% of rating
  if (bat_amps < max_current/DCMCCM ) {
        charger_state = bulk_DCM ;
        Timer1Fast.initializeFast(100); // set a period of 100 uS =1600 clock cycles
       // Start PWM with CCM same duty cycle on IN pin and SD pin increased (synchronous operation)
        pwm2 = pwm1 * (solar_volt - bat_volt) ;
        pwm2 = pwm2 / solar_volt;
        pwm2 = pwm2 / 2 ;
        pwm2 = pwm2 + pwm1 ;
        Timer1Fast.startPwm(IN_pin, pwm1, 0);  
        Timer1Fast.startPwm(SD_pin, pwm2, 0);  // check if this over-rides digitalWrite (pin, High)
        break ;
      }
    // change state if current rises above 100% of rating
    if (bat_amps > max_current+100) {
        charger_state = current_limited_CCM ;
        break ;
        }

    // MPPT tracking section for CCM
      if (track_direction = 1) {
      if (pwm1 < 54314) { // corresponds to 10 microseconds out of 12
      pwm1 = pwm1 + 342 ; // corresponds to 1 CPU clock cycle out of 192
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
     }
    } else if (pwm1 < 11222) { // corresponds to 2 microseconds out of 12
      pwm1 = pwm1 - 342 ; // corresponds to 1 CPU clock cycle out of 192
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
    }
    if (solar_watts < old_solar_watts) {
      track_direction = track_direction * -1 ;
      } 
    old_solar_watts = solar_watts ;

   break ; // end of case bulk_CCM condition

// ______________________________________________________________   
   case boost_CCM : // MPPT with battery above Float for a limited time
  if (solar_amps < 0) { // minimum current 0 mA normal reading
     charger_state = off ;
     charger_millis = millis() ;
     Timer1Fast.disablePwm (SD_pin); // cease PWM on pin 10
     Timer1Fast.disablePwm (IN_pin); // cease PWM on pin 9
     break ; 
  } // end of current (Amps) test

    boost_today = boost_today + millis() - charger_millis ;
    charger_millis = millis();
    if (boost_today > boost_duration) {
      charger_state = bulk_CCM ;
      break ;
    }
    if (bat_volt > boost_voltage) { // Battery has reached Boost voltage 
     charger_state = boost_float_DCM ;
     Timer1Fast.initializeFast(100); // set a period of 100 uS =1600 clock cycles
     // Start PWM with 2% duty cycle on IN pin and SD pin synchronised together to give Asynchronous operation (Q3 OFF)
     pwm1 = 1310 ; // 1310 = 65536 / 50
     pwm2 = 1310 ; // 1310 = 65536 / 50
     Timer1Fast.startPwm(IN_pin, pwm1, 0);  
     Timer1Fast.startPwm(SD_pin, pwm2, 0);  // check if this over-rides digitalWrite (pin, High)
     break ;
     }

    if (bat_volt < float_voltage - 15) { // charging rate does not need Boost voltage 
     charger_state = bulk_CCM ;
     break ;
    }
    // change state if current rises above 100% of rating
    if (bat_amps > max_current) {
        charger_state = current_limited_boost_CCM ;
        break ;
        }
    // change state if current falls below 10% of rating
    if (bat_amps < max_current/DCMCCM) {
        charger_state = boost_DCM ;
        Timer1Fast.initializeFast(100); // set a period of 100 uS =1600 clock cycles
        // Start PWM with CCM same duty cycle on IN pin and SD pin increased (synchronous operation)
        pwm2 = pwm1 * (solar_volt - bat_volt) ;
        pwm2 = pwm2 / solar_volt;
        pwm2 = pwm2 / 2 ;
        pwm2 = pwm2 + pwm1 ;
        Timer1Fast.startPwm(IN_pin, pwm1, 0);  
        Timer1Fast.startPwm(SD_pin, pwm2, 0);  // check if this over-rides digitalWrite (pin, High)
        break ;
       }

    // MPPT tracking section for CCM
      if (track_direction = 1) {
      if (pwm1 < 54314) { // corresponds to 10 microseconds out of 12
      pwm1 = pwm1 + 342 ; // corresponds to 1 CPU clock cycle out of 192
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
     }
    } else if (pwm1 > 11222) { // corresponds to 2 microseconds out of 12
      pwm1 = pwm1 - 342 ; // corresponds to 1 CPU clock cycle out of 192
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
    }
    if (solar_watts < old_solar_watts) {
      track_direction = track_direction * -1 ;
      } 
    old_solar_watts = solar_watts ;

   break ; // end of case boost_CCM condition

// ______________________________________________________________   
   case current_limited_CCM : // manages for limited current capacity under float_voltage

  if (solar_amps < 0) { // minimum current 0 mA normal reading
     charger_state = off ;
     charger_millis = millis() ;
     Timer1Fast.disablePwm (SD_pin); // cease PWM on pin 10
     Timer1Fast.disablePwm (IN_pin); // cease PWM on pin 9
     break ; 
  } // end of current (Amps) test

   if (bat_amps < max_current-100) { // 
    charger_state = bulk_CCM ;
    break ;
   }
   if (bat_volt > float_voltage) {
    charger_state = float_DCM ;
    break ;
   }

   // control bat_amps to max_current
   if (bat_amps > max_current+200) { // decrease PWM so as to reduce charger current
     if (pwm1 > 11222) { 
      pwm1 = pwm1 - 342 ;
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
     } 
   }
   else if (bat_amps < max_current) { // increase PWM so as to increase charger current
     if (pwm1 < 54314) {
      pwm1 = pwm1 + 342 ; 
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
     }        
   }

   break ; // end of case current_limited_CCM condition
   
// ______________________________________________________________   
   case current_limited_boost_CCM : // manages for limited current capacity under boost_voltage

      if (solar_amps < 0) { // minimum current 0 mA normal reading
     charger_state = off ;
     charger_millis = millis() ;
     Timer1Fast.disablePwm (SD_pin); // cease PWM on pin 10
     Timer1Fast.disablePwm (IN_pin); // cease PWM on pin 9
     break ; 
  } // end of current (Amps) test
     boost_today = boost_today + millis() - charger_millis ;
    charger_millis = millis();
    if (boost_today > boost_duration) {
      charger_state = current_limited_CCM ;
      break ;
    }
   if (bat_amps < max_current-100) { // 
    charger_state = boost_CCM ;
    break ;
   }

   if (bat_volt > boost_voltage) {
     charger_state = boost_float_DCM ;
     Timer1Fast.initializeFast(100); // set a period of 100 uS =1600 clock cycles
     // Start PWM with 2% duty cycle on IN pin and SD pin synchronised together to give Asynchronous operation (Q3 OFF)
     pwm1 = 1310 ; // 1310 = 65536 / 50
     pwm2 = 1310 ; // 1310 = 65536 / 50
     Timer1Fast.startPwm(IN_pin, pwm1, 0);  
     Timer1Fast.startPwm(SD_pin, pwm2, 0);  // check if this over-rides digitalWrite (pin, High)
     break ;
     }
   
   // control bat_amps to max_current
   if (bat_amps > max_current+200) { // decrease PWM so as to reduce charger current
     if (pwm1 > 11222) { 
      pwm1 = pwm1 - 342 ;
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
     } 
   }
   else if (bat_amps < max_current) { // increase PWM so as to increase charger current
     if (pwm1 < 54314) {
      pwm1 = pwm1 + 342 ; 
      Timer1Fast.setPwmDuty(IN_pin, pwm1) ;
     }        
   }
    
   break ; // end of case current_limited_boost_CCM condition
   
// ______________________________________________________________   
   default:
   // programming error
   Serial.print ("State machine error");
   Serial.println (charger_state);
   charger_state = off ;
   Timer1Fast.disablePwm (SD_pin); // cease PWM on pin 10
   Timer1Fast.disablePwm (IN_pin); // cease PWM on pin 9
   break; // end of case default condition
   } // end of switch / case package
 
  } // end of run_charger


//   Current Status : Code written, needs to be tested.

//-------------------------------------------------------Function definition for LCD display------------------------------------------------------------------------------------------------
/*
 This function is used to display the various parameters(solar panel, battery and load releted )of the controller on a 20X4 char, I2C LCD.
 Display looks like this                                           
                                         | Sol "icon"   Bat "icon"       PWM "icon"      | 
                                         | xx V         xx V             XX %           | 
                                         | xx A      "Charger Status"    Load           | 
                                         | xx W       " SOC %"           "Load Status"  | 
                                         |----------------------------------------------|
     
 The first column parameters are solar panel voltage,current and power
 The second column parameters are battery voltage,Charger Status (On or Off) and SOC in percentage.
 The third column 2nd row display PWM in percentage and 4th row display load staus ( ON or OFF )
 
 Back Light Control : A button to control the back light. By default the back light will be in off condition.
 If the user press the switch then it will on for 15 secs and again goes off.
 
 Current Status : Working 
 
 */
  
   void lcd_display()
  {
    static bool current_backlight_state = -1;
    back_light_pin_State = digitalRead(back_light_pin);
    if (current_backlight_state != back_light_pin_State) {
      current_backlight_state = back_light_pin_State;
      if (back_light_pin_State == HIGH)
        lcd.backlight();// finish with backlight on
      else
        lcd.noBacklight();
    }
  
    if (back_light_pin_State == HIGH)
    {
      time = millis();                        // If any of the buttons are pressed, save the time in millis to "time"
    }
   
   lcd.setCursor(0, 1);
   lcd.print(float(solar_volt/1000));               // dividing 1000 to convrt mV to V
   lcd.print("V ");
   lcd.setCursor(0, 2);
   lcd.print(float(solar_amps/1000));              // dividing 1000 to convrt mA to A
   lcd.print("A");  
   lcd.setCursor(0, 3);
   lcd.print(float(solar_watts/1000));
   lcd.print("W "); 
   lcd.setCursor(8, 1);
   lcd.print(float(bat_volt/1000));
   lcd.print("V ");
   lcd.setCursor(8,2);
   
   if(charger_state == off){
   lcd.print("Off ");
   }
   else{
   lcd.print("On ");
   }
   //-----------------------------------------------------------
   //--------------------Battery State Of Charge ---------------
   //-----------------------------------------------------------
   int pct = 100.0*(bat_volt - 11300)/(13800 - 11300);
   if (pct < 0)
       pct = 0;
   else if (pct > 100)
       pct = 100;
  
   lcd.setCursor(12,0);
   lcd.print((char)(pct*5/100));
  
   lcd.setCursor(8,3);
   pct = pct - (pct%10);
   lcd.print(pct);
   lcd.print("%  ");

  //--------------------------------------------------------------------- 
  //------------------Duty Cycle-----------------------------------------
  //---------------------------------------------------------------------
   
   lcd.setCursor(15,1);
   lcd.print("   ");
   if( charger_state == off){
   lcd.setCursor(15,1);
   lcd.print(0);
   lcd.print("% ");
   lcd.setCursor(15,2);
   lcd.print(0);
   lcd.print("% ");
   }
   else
   {
   lcd.setCursor(15,1);
   lcd.print(pwm1/1000,DEC);
   lcd.print("% ");
   lcd.setCursor(15,2); 
   lcd.print(pwm2/1000,DEC);
   lcd.print("% ");
   }
   
   
   /*
   //----------------------------------------------------------------------
   //------------------------Load Status-----------------------------------
   //----------------------------------------------------------------------
   lcd.setCursor(15,2);
   lcd.print("Load");
   lcd.setCursor(15,3);
   if (load_status)
   {
      lcd.print("On  ");
   }
   else
   {
     lcd.print("Off ");
   } 
 */
   lcd.setCursor(15,3);
   spinner();
   backLight_timer();                      // call the backlight timer function in every loop 
  }
  
  void backLight_timer(){
    if((millis() - time) <= 15000)         // if it's been less than the 15 secs, turn the backlight on
        lcd.backlight();                   // finish with backlight on  
    else 
        lcd.noBacklight();                 // if it's been more than 15 secs, turn the backlight off
  }
  void spinner(void) {
    static int cspinner;
    static char spinner_chars[] = { '*','*', '*', ' ', ' '};
    cspinner++;
    lcd.print(spinner_chars[cspinner%sizeof(spinner_chars)]);
  }








//-------------------------------------------------------Function definition for Serial monitor printing------------------------------------------------------------------------------------
  
/*
   This function prints parameters on the serial monitor.
   This is very useful during debugging
  
   Current Status : Working  
*/
  
  void serial_print_data(){
if (MONITOR == "DEBUG") {
  
  Serial.print(millis() ) ;
  Serial.print(" =");

  Serial.print(" C= "); 
  Serial.print(counter);
  Serial.print("\t");
  
  Serial.print("State = ");
  if (charger_state == off) Serial.print("off       ");
  else if (charger_state == startup) Serial.print("startup   ");
  else if (charger_state == float_DCM) Serial.print("float_DCM "); 
  else if (charger_state == bulk_DCM) Serial.print("bulk_DCM  "); 
  else if (charger_state == boost_DCM) Serial.print("boost_DCM "); 
  else if (charger_state == boost_CCM) Serial.print("boost_CCM ");
  else if (charger_state == bulk_CCM) Serial.print("bulk_CCM  ");
  else if (charger_state == boost_float_DCM) Serial.print("boost_float_DCM  ");
  else if (charger_state == current_limited_CCM) Serial.print("current_limited_CCM");
  else if (charger_state == current_limited_boost_CCM) Serial.print("cur_ltd_boost_CCM");
  else if (charger_state == test_nil) Serial.print("test_nil   ");
  else if (charger_state == test) Serial.print("test       ");

  else Serial.print ("unknown state");
  Serial.print("\t");

  Serial.print(" Boost today= "); 
  Serial.print(boost_today);
  Serial.print("\t");

  Serial.print("pwm1,2 = ");
  Serial.print(pwm1/1000,DEC);
  Serial.print("\t");
  Serial.print(pwm2/1000,DEC);
  Serial.print("\t");

  Serial.print(" Ip= \t"); 
  Serial.print(float(solar_amps/1000));
  Serial.print("\t");

  Serial.print("Vp="); 
  Serial.print(float(solar_volt/1000));
  Serial.print("\t");

  Serial.print("Wp="); 
   Serial.print(float(solar_watts/1000));
  Serial.print("\t");

  Serial.print("Vb=");  
  Serial.print(float(bat_volt/1000));
  Serial.print("\t");

  Serial.print("\n\r"); // carriage return and line feed
//  } // counter test
}
//  end of debug serial print section

// This is config control section 
   else {
    
   Serial.println(" Set end of line at no line ending  ");
   Serial.println("Commands: LIST START LOAD STOP ");
   Serial.println(" Enter a command: ");
    
    while(Serial.available()==0){
    }
    command = Serial.readString();
    Serial.print("You have entered: ");
    Serial.println(command);
    
    if ( command == "list" or command == "LIST" ){
    set_parameters();
    }
    if ( command == "start" or command == "START"){
    Serial.println(" All parameters ");
    // function to display all parameters
    
    }
    
    if ( command == "load" or command == "LOAD"){
    
      load_control_mode();
    }
    
    if ((command == "stop") || (command == "STOP")) {
    Serial.println(" Go back to main program ");
    MONITOR = "DEBUG" ;
    }
   }  // END OF else condition
  } // end of serial_print_data 

//------------------------------------------------------ Function for setting battery management parameters---------------------------------------------------------------------------------

 void set_parameters(void){
    //  ToDo print these with a decimal point
   
/*    

Serial.println("******** Present Battery management parameters  ********");
    Serial.print("Battery Type: ");
    Serial.println(battery_type);
    Serial.print("Float voltage: ");
    Serial.println(float_voltage1);
    Serial.print("Boost voltage: ");
    Serial.println(boost_voltage1);  
    Serial.print("Boost reconnect voltage: ");
    Serial.println(boost_reconnect_voltage);
    Serial.print("Low Voltage Disconnect: ");
    Serial.println(low_voltage_disconnect1);
    Serial.print("Low Voltage Reconnect: ");
    Serial.println(low_voltage_reconnect1); 
    Serial.print("Boost duration: ");
    Serial.println(boost_duration); 
   
  
    Serial.println(" Set end of line at no line ending  ");
    Serial.println("******** Set The Battery management parameters  ********");  //Prompt User for input
    Serial.println("Enter Battery Type: ");         //Prompt User for input
    while (Serial.available()==0) {             //Wait for user input
    }
    String temp = Serial.readString();        //Read user input into Battery_Type
    if (temp != "") {
    battery_type = temp ;                      //Read user input into Battery_Type
    }
    Serial.print("Battery Type: ");
    Serial.println(battery_type);
    
    Serial.println(" Set end of line at Both NL & CR ");
    Serial.println("Enter the Float voltage :");         //Prompt User for input
    while (Serial.available()==0)  {
    }
    long val =Serial.parseFloat() * 1000;
    if (val != 0) {
    float_voltage1 = val;                               //Read user input into age 
    }
    Serial.print("Float voltage :");
    Serial.println(float_voltage1);
  
  
    Serial.println("Enter the Boost voltage :");        
    while (Serial.available()==0)  {
    }
    val =Serial.parseFloat() * 1000;
    if (val != 0) {
    boost_voltage1= val ;
    }
    Serial.print("Boost voltage :");
    Serial.println(boost_voltage1);
  
  
    Serial.println("Enter the Enter the Boost reconnect voltage :");        
    while (Serial.available()==0)  {
    }
    val =Serial.parseFloat() * 1000;
    if (val != 0) {
    boost_reconnect_voltage1= val ;
    }
    Serial.print("Boost reconnect voltage :");
    Serial.println(boost_reconnect_voltage1);
  
  
    Serial.println("Enter the Low Voltage Disconnect :");        
    while (Serial.available()==0)  {
    }
    val =Serial.parseFloat() * 1000;
    if (val != 0) {
    low_voltage_disconnect1 = val ;
    }
    Serial.print("Low Voltage Disconnect :");
    Serial.println(low_voltage_disconnect1);
  
    Serial.println("Enter the Low Voltage Reconnect :");        
    while (Serial.available()==0)  {
    }
    val =Serial.parseFloat() * 1000;
    if (val != 0) {
    low_voltage_reconnect1= val ;
    }
    Serial.print("Low Voltage Reconnect :");
    Serial.println(low_voltage_reconnect1);
  
    Serial.println("Enter the Boost duration :");        
    while (Serial.available()==0)  {
    }
    val =Serial.parseFloat() * 1000;
    if (val != 0) {
    boost_duration= val ; 
    }
    Serial.print("Boost duration :");
    Serial.println(boost_duration);

    Serial.println("Enter the temperature compensation factor: ");
    while (Serial.available()==0)  {
    }
    val =Serial.parseFloat() ;
    if (val != 0) {
    temperature_compensation = val ; 
    }
    Serial.print("temperature compensation factor: ");
    Serial.println(temperature_compensation);
*/
   }  // end of set_parameters
   
//------------------------------------------------------Function definition for load control mode selection----------------------------------------------------------------------------------

   void  load_control_mode(void){
/*   
   Serial.println(" Enter the load control mode command: ALL or DARK ");
   while(Serial.available()==0){
   }
   String temp = Serial.readString();
   if (temp != "") {
     load_mode = temp ;
   }
   Serial.print("load control mode: ");
   Serial.println(load_mode);
   first_time = 1  ;  
*/    
   }  // end of load_control_mode



//-------------------------------------------------------Function definition for led indication----------------------------------------------------------------------------------------------

/* 
   This function display the current state of charge of the battery via LED as follows:
   
   1. YELLOW means overvoltage (over 14.1 volts)
   2. RED means undervoltage (under 11.9 volts)
   3. GREEN is between 11.9 and 14.1 volts
   
   Current Status : Working 

*/
  
 // light an individual LED
 // we remember which one was on before in last_lit and turn it off if different
 void light_led(char pin)
 {
  static char last_lit;
  if (last_lit == pin)
      return;
  if (last_lit != 0)
      digitalWrite(last_lit, LOW);
  digitalWrite(pin, HIGH);
  last_lit = pin;
}
  // display the current state via LED as follows:
  // YELLOW means overvoltage (over 14.1 volts)
  // RED means undervoltage (under 11.9 volts)
  // GREEN is between 11.9 and 14.1 volts
  void led_indication(void)
  {
    static char last_lit;
    if(bat_volt > 14100 )
      light_led(yellow_led);
    else if(bat_volt > 11900)
      light_led(green_led);
    else
        light_led(red_led);
  }


//-------------------------------------------------------Function definition for load control------------------------------------------------------------------------------------------------

/* 
  This function controls the load MOSFET(Q4).The switching logic is as follows :
  There are two modes of operation
  1.During the Night/dark
  2.During whole day/continuous

 1. ON : Turn on loads at night ( when the solar panel is not producing power ) and the battery voltage is above LVD ( low voltage disconnect)
 2. OFF : Turn off loads at day and when the battery voltage is below LVD

  Current Status : To be checked
*/
  void load_control(){
    if ( load_mode == "dark" or load_mode == "DARK"){
      
    // write the logic ToDo
    if (first_time != 0) {
    Serial.println(" DARK (Night time only)");
    first_time = 0 ;
    } // end of first_time test
    }
    
    else if ( load_mode == " all" or load_mode == "ALL"){
     
     // write the logic  ToDo
    if (first_time != 0 ) {
      Serial.println(" ALL (Continuous) ");
    first_time = 0 ;
    } // end of first_time test
    
     }
  
    else {
    if (first_time != 0 ) {
       Serial.print (" load mode not recognised:  ");
       Serial.print (load_mode);
       Serial.println (" use ALL or DARK  ");
    first_time = 0 ;
    } // end of first_time test
       
     }
 
 }  // end of load_control

   


//-------------------------------------------------------Function definition for WiFi data logging-------------------------------------------------------------------------------------------

/* 
   This function send parameters ( sol voltage,current,power & battery voltage,charging current etc )to the www.thingspeak.com via a ESP8266 wifi module
   First the wifi module will connect to the wifi router automatically ( User have to set the SSID,Router Password and Thingspeak API key )
   If the Wifi module connect succsessfully,it will transmit datas to wifi @ 15sec interval
   
   Current Status : Working but wifi module has to be first connect to the router by using a separte program and using AT command from serial monitor
   
*/
 
  void  wifi_datalogging(){


  } // end of wifi_datalogging

// END


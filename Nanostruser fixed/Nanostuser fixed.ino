#include <Arduino.h>
#include <U8g2lib.h>
#include <thermistor.h>
#include <AccelStepper.h>
#include <U8x8lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

const char DEGREE_SYMBOL[] = { 0xB0, '\0' };

    /*ST7920 (ender 3 lcd pinout)
        
        | GND    5V   |
        | CS     DT   |
          ENCA   SCK  |   LCD EXP3
        | ENCB        |
        | BEEPER ENCSW|

        LCD SPI MODE
    */  U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* CS=*/ 10, /* reset=*/ U8X8_PIN_NONE);
        //U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* reset=*/ 10);
        //U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 13SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display

// IO Setup
  //Thermistor
    thermistor therm1(A0,0);
  //Inputs Outputs
    //Output PWM
       int mosfet_pin = PD5; // D5
       int fan_pin = PD6; // D6 
    //Output Stepper
       int EN =  PD2; // D2
       int STEP = PD3; // D3 
       int DIR = PD4; // D4
       int LED = 12; // D10
    //Button n Encoder
       int stepper_speed = PC1; // A1
       int enca =  PB0; // D8
       int encb = PD7; // D7
       int encsw = PB1; // D9

       bool encsw_state = true;
       bool activate_stepper = false;
       unsigned long lastButtonPress = 0;

    //Define Stepper Control
    AccelStepper stepper1(1, STEP, DIR); //(Type of driver: with 2 pins, STEP, DIR)

    //Heater Variables
    //Default temperature setpoint. Leave it 0 and control it with rotary encoder
      float set_temperature = 0;
      float temperature_read = 0.0;
      float PID_error = 0;
      float previous_error = 0;
      float elapsedTime, Time, timePrev;
      float PID_value = 0;
      int button_pressed = 0;
      int menu_activated=0;
      float last_set_temperature = 0;
      int max_PWM = 255;
  
    //Stepper Variables
      int max_speed = 1000;
      int main_speed = 0;
      int rotating_speed = 0;
    

    //PID constants
    //////////////////////////////////////////////////////////
    int kp = 90;   int ki = 30;   int kd = 80;
    //////////////////////////////////////////////////////////
    int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
    float last_kp = 0;
    float last_ki = 0;
    float last_kd = 0;
  
    int PID_values_fixed = 0;

void setup(void) 
{
  Serial.begin(115200);
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setFont(u8g2_font_helvB10_tf); 
  u8g2.setColorIndex(1);

  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);  //Stepper drier is disabled
  stepper1.setMaxSpeed(max_speed);
  
  pinMode(encsw, INPUT_PULLUP);
  pinMode(enca, INPUT);
  pinMode(encb, INPUT);
  pinMode(stepper_speed, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  pinMode(mosfet_pin,OUTPUT);
    TCCR0B = TCCR0B & B11111000 | B00000010;    // D5 and D6 PWM frequency of 7812.50 Hz
    Time = millis();

  TCCR1A = 0;             //Reset entire TCCR1A register
  TCCR1B = 0;             //Reset entire TCCR1B register
  TCCR1A |= B00000010;    //   /8
  TCNT1 = 0;              //Reset Timer 1 value to 0
}

void loop(void) 
{
  //stepper dependancy
  steppercontrol();
  //thermocouple temp read
    temperature_read = therm1.analog2temp();
    PID_error = set_temperature - temperature_read + 6;
    PID_p = 0.01*kp * PID_error;
    PID_i = 0.01*PID_i + (ki * PID_error);
    //real time calculate speed change rate

    timePrev = Time;                            // the previous time is stored before the actual time read
    Time = millis();                            // actual time read
    elapsedTime = (Time - timePrev) / 1000;     //Now we can calculate the D calue
    PID_d = 0.01*kd*((PID_error - previous_error)/elapsedTime);  //Final total PID value is the sum of P + I + D
    PID_value = PID_p + PID_i + PID_d;
    //define PWM range
    if(PID_value < 0)
    {
      PID_value = 0;
    }
    if(PID_value > max_PWM)
    {
      PID_value = max_PWM;
    }
  
  analogWrite(mosfet_pin,PID_value);
  previous_error = PID_error;

  u8g2.firstPage();
  do {draw();}
  while( u8g2.nextPage() );
}

ISR(TIMER1_COMPA_vect)
{
  TCNT1 = 0;
  stepper1.runSpeed();
}

void steppercontrol(void)
{
  int encstate = digitalRead(encsw);
    if (encstate == LOW)
      {
        if (millis() - lastButtonPress > 50)
        {
        encsw_state = false;
        activate_stepper = !activate_stepper;
        delay(10);
        }
      }
    
      else if(encstate == HIGH)
      {
      encsw_state = true;
      }
  
    if(activate_stepper)
    {
    digitalWrite(LED, HIGH);
    digitalWrite(EN, LOW);    //We activate stepper driver
    rotating_speed = map(analogRead(stepper_speed),0,1024,main_speed,max_speed);
    stepper1.setSpeed(rotating_speed);
    stepper1.runSpeed();
    }

    else
    {
    digitalWrite(EN, HIGH);    //We deactivate stepper driver
    digitalWrite(LED, LOW);
    stepper1.setSpeed(0);
    stepper1.runSpeed();
    }

}

void draw(void)
{
  u8g2.setCursor(35, 28);
  u8g2.print(temperature_read);   
  u8g2.setCursor(37, 60);         
  u8g2.print(rotating_speed);
  u8g2.updateDisplay();
}
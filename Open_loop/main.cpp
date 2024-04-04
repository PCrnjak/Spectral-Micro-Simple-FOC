#include "Arduino.h"
#include "SimpleFOC.h"

// Driver PWM pins
#define PWM_CH1 PA_2
#define PWM_CH2 PA_1 
#define PWM_CH3 PA_0 

// Encoder pins
#define MOSI PB_5
#define MISO PB_4
#define CLK PB_3
#define CSN PA_15

#define SENSE1 PA_4 // Inline current sense
#define SENSE2  PA_3 // Inline current sense

#define TX_COM PB_6
#define RX_COM PB_7

// Current sense resistors
#define SENSE_RESISTOR 0.025f
#define CURRENT_AMP_GAIN 20

// Serial pins
#define Serial Serialx
HardwareSerial Serialx(RX_COM, TX_COM); // PA3, PA2 RX,TX

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number, phase resistance (optional) );
BLDCMotor motor = BLDCMotor(11);
//  BLDCDriver3PWM( pin_pwmA, pin_pwmB, pin_pwmC )
BLDCDriver3PWM driver = BLDCDriver3PWM(PWM_CH1, PWM_CH2, PWM_CH3);

//target variable
float target_velocity = 0.1;

void Init_pins();

void setup() {  

  // Init driver enable pins and LED
  Init_pins();

  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 5;
  // driver init
  driver.init();
    // link the motor and the driver
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  motor.init();
    // use monitoring with the BLDCMotor
  Serial.begin(115200);
  Serial.println("Motor ready!");
  _delay(1000);
  // monitoring port
  motor.useMonitoring(Serial);

}

void loop() {
  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  motor.move(target_velocity);
  // monitoring function outputting motor variables to the serial terminal 
  motor.monitor();

}

void Init_pins(){
  pinMode(PA_9, OUTPUT);
	pinMode(PA_8, OUTPUT);
	pinMode(PA_10, OUTPUT);
	pinMode(PB_15, OUTPUT);
	pinMode(PB_14, OUTPUT);
	pinMode(PB_13, OUTPUT);

  digitalWrite(PA_9,HIGH);
  digitalWrite(PA_8,HIGH);
  digitalWrite(PA_10,HIGH);
  digitalWrite(PB_14, HIGH);
  digitalWrite(PB_15, HIGH);
  digitalWrite(PB_13, HIGH);
}
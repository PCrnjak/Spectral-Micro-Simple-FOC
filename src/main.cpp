#include "Arduino.h"
#include "SimpleFOC.h"
#include <encoders/mt6816/MagneticSensorMT6816.h>

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

// Create SPI instance (if using a specific SPI bus)
SPIClass SPI_2(MOSI, MISO, CLK);

// Create MagneticSensorMT6816 instance
MagneticSensorMT6816 sensor = MagneticSensorMT6816(CSN);

// Serial pins
#define Serial Serialx
HardwareSerial Serialx(RX_COM, TX_COM); // PA3, PA2 RX,TX

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number, phase resistance (optional) );
BLDCMotor motor = BLDCMotor(11);
//  BLDCDriver3PWM( pin_pwmA, pin_pwmB, pin_pwmC )
BLDCDriver3PWM driver = BLDCDriver3PWM(PWM_CH1, PWM_CH2, PWM_CH3);

// target voltage,speed or positon
float target = 6;

void Init_pins();

void setup() {  

  // Init driver enable pins and LED
  Init_pins();

  // Initialize SPI for the encoder
  sensor.init(&SPI_2); // Use specified SPI instance (e.g., SPI_2)
  motor.linkSensor(&sensor);

  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 8;
  // driver init
  driver.init();
    // link the motor and the driver
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = 3;

  // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set torque mode
  motor.torque_controller = TorqueControlType::voltage;
  
  // set motion control loop to be used
  //motor.controller = MotionControlType::torque;
  //motor.controller = MotionControlType::angle;
  motor.controller = MotionControlType::velocity;
  //motor.controller = MotionControlType::velocity_openloop;


  // init motor hardware
  motor.init();
  // align sensor and start FOC
  motor.initFOC();
  // use monitoring with the BLDCMotor
  Serial.begin(115200);
  Serial.println("Motor ready!");
  _delay(1000);
  
  // monitoring port
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_VOLT_Q | _MON_CURR_Q;
  motor.useMonitoring(Serial);


}

void loop() {

  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move(target);
  
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
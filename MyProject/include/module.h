/*
 * module.h
 *
 *  Created on: 2024. 12. 17.
 *      Author: suchan
 */

#ifndef MODULE_H_
#define MODULE_H_
#include "msp.h"
#include "Clock.h"
#define OL &fsm[0]
#define L1 &fsm[1]
#define L2 &fsm[2]
#define L3 &fsm[3]
#define R1 &fsm[4]
#define R2 &fsm[5]
#define R3 &fsm[6]
#define LLC &fsm[7]
#define RLC &fsm[8]
#define LC1 &fsm[9]
#define LC2 &fsm[10]
#define LF &fsm[12]
#define FL &fsm[11]
#define RLLC &fsm[13]
#define RRLC &fsm[14]
#define dtGoodGood 40
#define dtGood 25
#define dt 15
#define dtSlightLost 400
#define dtLost 700
#define MAX 14998/3  // 100%
#define HIGH 13000/3 // 80%
#define MED 10000/3   // 50%
#define NMED -10000/3 // -50% (Reverse Medium)
#define LOW 5000/3   // 20%
#define LED_RED 1
#define LED_GREEN (LED_RED << 1)
#define LED_BLUE (LED_RED << 2)

/*************************************/
/*************************************/

// Linked data structure
struct State {
  int16_t leftDuty;              // Duty Cycle %
  int16_t rightDuty;             // [-100 to 100]
  uint32_t delay;                // Delay in ms
  const struct State *next[8];   // 3-bit input -> 8 next states
};
typedef const struct State State_t;

/*************************************/
/*************************************/




// proto

void led_init();
void turn_on_led(int);
void turn_off_led();
void switch_init();
int  read_switch();
void Motor_Init(void);
void Motor_Stop(void);
void Motor_Forward(uint16_t leftDuty, uint16_t rightDuty);
void Motor_Right(uint16_t leftDuty, uint16_t rightDuty);
void Motor_Left(uint16_t leftDuty, uint16_t rightDuty);
void Motor_Backward(uint16_t leftDuty, uint16_t rightDuty);
void my_motor_init(void);
void my_move(uint16_t leftDuty, uint16_t rightDuty);
void my_stop(void);
void my_left_forward();
void my_left_backward();
void my_right_forward();
void my_right_backward();
void Reflectance_Init(void);
void Reflectance_Start(void);
uint8_t Reflectance_End(void);
uint8_t AverageSensor (uint8_t data);
void call_motor(int16_t leftDuty, int16_t rightDuty);
void start_fsm();
void PWM_Init1(uint16_t period, uint16_t duty);
void PWM_Init12(uint16_t period, uint16_t duty1, uint16_t duty2);
void my_pwm_init34(uint16_t period, uint16_t duty3, uint16_t duty4);
uint16_t PWM_RobotArmGetDuty2(void);
void PWM_RobotArmDuty1(uint16_t duty1);
uint16_t PWM_RobotArmGetDuty0(void);
uint16_t PWM_RobotArmGetDuty1(void);
void PWM_RobotArmDuty2(uint16_t duty2);
void PWM_RobotArmDuty0(uint16_t duty0);
void PWM_RobotArmInit(uint16_t period, uint16_t duty0, uint16_t duty1, uint16_t duty2);
void PWM_Duty4(uint16_t duty4);
void PWM_Duty3(uint16_t duty3);
void PWM_Init34(uint16_t period, uint16_t duty3, uint16_t duty4);
void PWM_Duty2(uint16_t duty2);
void PWM_Duty1(uint16_t duty1);
void SysTick_Handler(void); // Periodically called to handle sensor process
uint8_t ReadSensorData(void); // Returns the last read data from the sensor
uint8_t getColFlag();
void BumpInt_Init(void(*task)(void));
void HandleCollision(uint8_t bumpSensor);
void SysTick_Init(uint32_t period, uint32_t priority);
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// ------------Reflectance_Init------------
// Initialize the GPIO pins associated with the QTR-8RC
// reflectance sensor.  Infrared illumination LEDs are
// initially off.
// Input: none
// Output: none
// ------------Reflectance_Start------------
// Begin the process of reading the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// Input: none
// Output: none
// Assumes: Reflectance_Init() has been called
// ------------Reflectance_End------------
// Finish reading the eight sensors
// Read sensors
// Turn off the 8 IR LEDs
// Input: none
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
// Assumes: Reflectance_Start() was called 1 ms ago
// AverageSensor
// Takes sensor data as input and returns
// 3 bit average location of low reflectivity
// 0 -> No data
//
/* Motor Translation Function
 * Motor cannot accept negative inputs, only PWM (0 to 14,998)
 * Decides which function to call based on signs of R/L
 */
// -> void call_motor(int16_t leftDuty, int16_t rightDuty); and start_fsm();
// Initializes Finite State Machine and continues looping indefinitely


#endif /* MODULE_H_ */

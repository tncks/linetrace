/*
 * module.c
 *
 *  Created on: 2024. 12. 17.
 *      Author: suchan
 */

#include "module.h"





// globals

uint8_t READ_DELAY = 1;
uint8_t CollisionData, CollisionFlag;
uint8_t SensorInput = 0;
uint8_t SensorInput_F = 0;
uint8_t Count = 0;
State_t *state;
State_t fsm[15]={
  {MAX, MAX,  dtGoodGood, {LC1, L3, L2, L1, OL, R1, R2, R3}},  // On Line
  {MAX, HIGH, dtGood, {LLC, L3, L2, L1, OL, R1, R2, R3}},  // Left 1
  {MAX, MED,  dt, {LLC, L3, L2, L1, OL, R1, R2, R3}},  // Left 2
  {MAX, LOW,  dt, {LLC, L3, L2, L1, OL, R1, R2, R3}},  // Left 3
  {HIGH,MAX,  dtGood, {RLC, L3, L2, L1, OL, R1, R2, R3}},  // Right 1
  {MED, MAX,  dt, {RLC, L3, L2, L1, OL, R1, R2, R3}},  // Right 2
  {LOW, MAX,  dt, {RLC, L3, L2, L1, OL, R1, R2, R3}},  // Right 3
  {MED, NMED, dtSlightLost, {RLLC, L3, L2, L1, OL, R1, R2, R3}},  // Left Lost Check
  {NMED, MED, dtSlightLost, {RRLC, L3, L2, L1, OL, R1, R2, R3}},  // Right Lost Check
  {MED, NMED, dtLost, {LC2, L3, L2, L1, OL, R1, R2, R3}},  // Lost Check 1
  {NMED, MED, dtLost, {LF,  L3, L2, L1, OL, R1, R2, R3}},  // Lost Check 2
  {0, 0,    1000, {FL,  FL, FL, FL, FL, FL, FL, FL}},   // Fully Lost, Stop
  {MED, MED, 650, {FL, L3, L2, L1, OL, R1, R2, R3}}, // Lost forward check //was 700
  {NMED, MED, dtSlightLost, {LC1, L3, L2, L1, OL, R1, R2, R3}},  // Left Lost Check REV
  {MED, NMED, dtSlightLost, {LC1, L3, L2, L1, OL, R1, R2, R3}}  // Right Lost Check REV
};
// Abbreviations:
// OL -> On Line
// L1 -> Off Left 1 (smallest deviation from line)
// NL -> No Line
// LLC -> Left Lost Check (Lost but know we are left of line)
// LC1 -> Lost Check 1 (Very lost)
// LC2 -> Lost Check 2
// FL -> Fully Lost
// State order in &fsm






// fun body

void led_init() {
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;
    P2->DIR |= 0x07;
    P2->OUT &= ~0x07;
}
void turn_on_led(int color){
    P2->OUT &= ~0x07;
    P2->OUT |= color;
}
void turn_off_led(){
    P2->OUT &= ~0x07;
}
void switch_init() {
    P1->SEL0 &= ~0x12;
    P1->SEL1 &= ~0x12;
    P1->DIR &= ~0x12;
    P1->REN |= 0x12;
    P1->OUT |= 0x12;
}
int read_switch() {
    return (P1->IN & 0x02) == 0;
}


// ------------Motor_Init------------
/*
 * motor explain
// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)
 * */
// Initialize GPIO pins for output, which will be
// used to control the direction of the motors and
// to enable or disable the drivers.
// The motors are initially stopped, the drivers
// are initially powered down, and the PWM speed
// control is uninitialized.
// Input: none
// Output: none
void Motor_Init(void){
    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;      // configure P2.6 P2.7
    P2->DIR |= 0xC0;        // identify as output

    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;      // configure P3.6 P3.7
    P3->DIR |= 0xC0;        // identify as output

    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;      // configure P5.5 P5.4
    P5->DIR |= 0x30;        // identify as output

    PWM_Init34(15000, 0, 0);
    P3->OUT &= ~0xC0;   // low current sleep mode
}

// ------------Motor_Stop------------
// Stop the motors, power down the drivers, and
// set the PWM speed control to 0% duty cycle.
// Input: none
// Output: none
void Motor_Stop(void){
    P1->OUT &= ~0xC0;
    PWM_Duty3(0);
    PWM_Duty4(0);
    P3->OUT &= ~0xC0;   // low current sleep mode
}

// ------------Motor_Forward------------
// Drive the robot forward by running left and
// right wheels forward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Forward(uint16_t leftDuty, uint16_t rightDuty){

    P5->OUT = (P5->OUT&0xCF)|0x00;
    P3->OUT = (P3->OUT&0x3F)|0xC0;
    P2->OUT = (P2->OUT&0x3F)|0x60;

    PWM_Duty3(rightDuty);
    PWM_Duty4(leftDuty);
}

// ------------Motor_Right------------
// Turn the robot to the right by running the
// left wheel forward and the right wheel
// backward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Right(uint16_t leftDuty, uint16_t rightDuty){
    P5->OUT = (P5->OUT&0xCF)|0x20;
    P3->OUT = (P3->OUT&0x3F)|0xC0;
    P2->OUT = (P2->OUT&0x3F)|0x60;

    PWM_Duty3(rightDuty);
    PWM_Duty4(leftDuty);
}

// ------------Motor_Left------------
// Turn the robot to the left by running the
// left wheel backward and the right wheel
// forward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Left(uint16_t leftDuty, uint16_t rightDuty){
    P5->OUT = (P5->OUT&0xCF)|0x10;
    P3->OUT = (P3->OUT&0x3F)|0xC0;
    P2->OUT = (P2->OUT&0x3F)|0x60;

    PWM_Duty3(rightDuty);
    PWM_Duty4(leftDuty);
}

// ------------Motor_Backward------------
// Drive the robot backward by running left and
// right wheels backward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Backward(uint16_t leftDuty, uint16_t rightDuty){
    P5->OUT = (P5->OUT&0xCF)|0x30;
    P3->OUT = (P3->OUT&0x3F)|0xC0;
    P2->OUT = (P2->OUT&0x3F)|0x60;

    PWM_Duty3(rightDuty);
    PWM_Duty4(leftDuty);
}

/* new */
void my_motor_init(void) {
    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;
    P3->DIR |= 0xC0;
    P3->OUT &= ~0xC0;

    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR |= 0x30;
    P5->OUT &= ~0x30;

    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;
    P2->OUT &= ~0xC0;
    // PWM 초기화
    //pwm_init34(7500, 0, 0); // 듀티 사이클 초기화
}
void my_move(uint16_t leftDuty, uint16_t rightDuty) {
    P3->OUT |= 0xC0; // 모터 전원 켜기
    TIMER_A0->CCR[3] = leftDuty; // 왼쪽 모터 듀티 사이클 설정
    TIMER_A0->CCR[4] = rightDuty; // 오른쪽 모터 듀티 사이클 설정
}

void my_stop(void) {
    P3->OUT &= ~0xC0; // 모터 전원 끄기
}

void my_left_forward() {
    P5->OUT &= ~0x10;
 }

void my_left_backward() {
P5->OUT |= 0x10;

 }

void my_right_forward() {
    P5->OUT &= ~0x20;
 }

void my_right_backward() {
    P5->OUT |= 0x20;
 }



//



//
// ------------Reflectance_Init------------
/* reflectance explain:
// reflectance even LED illuminate connected to P5.3
// reflectance odd LED illuminate connected to P9.2
// reflectance sensor 1 connected to P7.0 (robot's right, robot off road to left)
// reflectance sensor 2 connected to P7.1
// reflectance sensor 3 connected to P7.2
// reflectance sensor 4 connected to P7.3 center
// reflectance sensor 5 connected to P7.4 center
// reflectance sensor 6 connected to P7.5
// reflectance sensor 7 connected to P7.6
// reflectance sensor 8 connected to P7.7 (robot's left, robot off road to right)
 * */
// Initialize the GPIO pins associated with the QTR-8RC
// reflectance sensor.  Infrared illumination LEDs are
// initially off.
// Input: none
// Output: none
void Reflectance_Init(void){
    // Set up EVEN P5.3
    P5->SEL0 &= ~(0b1 << 3);
    P5->SEL1 &= ~(0b1 << 3);
    P5->DIR |= 0b1 << 3;        // Set as Output
    P5->OUT &= ~(0b1 << 3);     // Initially off

    // Set up ODD P9.2
    P9->SEL0 &= ~(0b1 << 2);
    P9->SEL1 &= ~(0b1 << 2);
    P9->DIR |= 0b1 << 2;        // Set as Output
    P9->OUT &= ~(0b1 << 2);     // Initially off

    // Set up sensor lines P7.0-P7.7
    P7->SEL0 &= ~(0xFF);
    P7->SEL1 &= ~(0xFF);
    P7->DIR &= ~(0xFF);         // Initially inputs
    P7->REN &= ~(0xFF);         // No pull resistor
}

// ------------Reflectance_Start------------
// Begin the process of reading the eight sensors
// Turn on the 8 IR LEDs
// Pulse the 8 sensors high for 10 us
// Make the sensor pins input
// Input: none
// Output: none
// Assumes: Reflectance_Init() has been called
void Reflectance_Start(void){
    // Changing sensors to output
    P7->DIR |= 0xFF;
    P7->OUT &= ~(0xFF);     // Initially off

    // 1) Turn on the IR LEDs
    P5->OUT |= 0b1 << 3;
    P9->OUT |= 0b1 << 2;

    // 2) Initiate a charge the 8 capacitors
    P7->OUT |= 0xFF;

    // 3) Wait for 10us for charging to complete
    Clock_Delay1us(10);

    // 4) Change the directionality of the sensor line ports to make them inputs
    P7->DIR &= ~(0xFF);
}


// ------------Reflectance_End------------
// Finish reading the eight sensors
// Read sensors
// Turn off the 8 IR LEDs
// Input: none
// Output: sensor readings
// Assumes: Reflectance_Init() has been called
// Assumes: Reflectance_Start() was called 1 ms ago
uint8_t Reflectance_End(void){
    uint8_t result;

    // Read the binary interpretations of the sensor line voltages
    result = P7->IN & 0xFF;

    // Turn off the IR LEDs
    P5->OUT &= ~(0b1 << 3);
    P9->OUT &= ~(0b1 << 2);

    return AverageSensor(result);
}

// AverageSensor
// Takes sensor data as input and returns
// 3 bit average location of low reflectivity
// 0 -> No data
uint8_t AverageSensor (uint8_t data) {
    uint8_t data_mut = data;
    uint32_t result = 0;
    uint32_t bits = 0;
    uint32_t i;

    for(i = 0; i < 8; i++) {
        result += (data_mut & 1) * (i + 1);
        bits += data_mut & 1;
        data_mut =  data_mut >> 1;
    }

    if (bits == 0) {
        return 0;
    } else {
        return (uint8_t) (result / bits) % 8;
    }
}

//



//
// Motor Translation Function
// Motor cannot accept negative inputs, only PWM (0 to 14,998)
void call_motor(int16_t leftDuty, int16_t rightDuty){

    uint8_t leftForward = leftDuty >= 0;
    uint8_t rightForward = rightDuty >= 0;

    // Forward
    if (leftForward && rightForward) {
        Motor_Forward((uint16_t) leftDuty, (uint16_t) rightDuty);

    // Left
    } else if (!leftForward && rightForward) {
        Motor_Left((uint16_t) (-1 * leftDuty), (uint16_t) rightDuty);

    // Right
    } else if (leftForward && !rightForward) {
        Motor_Right((uint16_t) leftDuty, (uint16_t) (-1 * rightDuty));

    // Backward
    } else {
        Motor_Backward((uint16_t) (-1 * leftDuty), (uint16_t) (-1 * rightDuty));
    }
}




// Starts fsm and will loop continuously
void start_fsm(){
    // Assume initially on line
    state = OL;

    while(1){
        if(!getColFlag())
        {
            call_motor(state->leftDuty, state->rightDuty);  // Send state output to motor
            Clock_Delay1ms(state->delay);                   // wait
            uint8_t test = ReadSensorData();
            state = state->next[ReadSensorData()];          // next depends on input and state
        }
        else
        {
            state = FL;
        }
    }
}

//



//
//***************************PWM_Init1*******************************
/**
 * pwm explain:
// PWM on P2.4 using TimerA0 TA0.CCR1
// PWM on P2.5 using TimerA0 TA0.CCR2
// PWM on P2.6 using TimerA0 TA0.CCR3
// PWM on P2.7 using TimerA0 TA0.CCR4
// MCLK = SMCLK = 3MHz DCO; ACLK = 32.768kHz
// TACCR0 generates a square wave of freq ACLK/1024 =32Hz
 */
// PWM outputs on P2.4
// Inputs:  period (166.67ns)
//          duty (0<=duty<period-1)
// Outputs: none
// SMCLK = 48MHz/4 = 12 MHz, 83.33ns
// Counter counts up to TA0CCR0 and back down
// Let Timerclock period T = 1/12MHz = 83.33ns
// P2.4=1 when timer equals TA0CCR1 on way down, P2.4=0 when timer equals TA0CCR1 on way up
// Period of P2.4 is period*166.67ns, duty cycle is duty/period
void PWM_Init1(uint16_t period, uint16_t duty){
  if(duty >= period) return;     // bad input
  P2->DIR |= 0x10;               // P2.4 output
  P2->SEL0 |= 0x10;              // P2.4 Timer0A functions
  P2->SEL1 &= ~0x10;             // P2.4 Timer0A functions
  TIMER_A0->CCTL[0] = 0x0080;    // CCI0 toggle
  TIMER_A0->CCR[0] = period;     // Period is 2*period*8*83.33ns is 1.333*period
  TIMER_A0->EX0 = 0x0000;        //    divide by 1
  TIMER_A0->CCTL[1] = 0x0040;    // CCR1 toggle/reset
  TIMER_A0->CCR[1] = duty;       // CCR1 duty cycle is duty1/period
  TIMER_A0->CTL = 0x0230;        // SMCLK=12MHz, divide by 1, up-down mode
// bit  mode
// 9-8  10    TASSEL, SMCLK=12MHz
// 7-6  00    ID, divide by 1
// 5-4  11    MC, up-down mode
// 2    0     TACLR, no clear
// 1    0     TAIE, no interrupt
// 0          TAIFG
}
//***************************PWM_Init12*******************************
// PWM outputs on P2.4, P2.5
// Inputs:  period (1.333us)
//          duty1
//          duty2
// Outputs: none
// SMCLK = 48MHz/4 = 12 MHz, 83.33ns
// Counter counts up to TA0CCR0 and back down
// Let Timerclock period T = 8/12MHz = 666.7ns
// P2.4=1 when timer equals TA0CCR1 on way down, P2.4=0 when timer equals TA0CCR1 on way up
// P2.5=1 when timer equals TA0CCR2 on way down, P2.5=0 when timer equals TA0CCR2 on way up
// Period of P2.4 is period*1.333us, duty cycle is duty1/period
// Period of P2.5 is period*1.333us, duty cycle is duty2/period
void PWM_Init12(uint16_t period, uint16_t duty1, uint16_t duty2){
  if(duty1 >= period) return; // bad input
  if(duty2 >= period) return; // bad input
  P2->DIR |= 0x30;          // P2.4, P2.5 output
  P2->SEL0 |= 0x30;         // P2.4, P2.5 Timer0A functions
  P2->SEL1 &= ~0x30;        // P2.4, P2.5 Timer0A functions
  TIMER_A0->CCTL[0] = 0x0080;      // CCI0 toggle
  TIMER_A0->CCR[0] = period;       // Period is 2*period*8*83.33ns is 1.333*period
  TIMER_A0->EX0 = 0x0000;        //    divide by 1
  TIMER_A0->CCTL[1] = 0x0040;      // CCR1 toggle/reset
  TIMER_A0->CCR[1] = duty1;        // CCR1 duty cycle is duty1/period
  TIMER_A0->CCTL[2] = 0x0040;      // CCR2 toggle/reset
  TIMER_A0->CCR[2] = duty2;        // CCR2 duty cycle is duty2/period
  TIMER_A0->CTL = 0x02F0;        // SMCLK=12MHz, divide by 8, up-down mode
// bit  mode
// 9-8  10    TASSEL, SMCLK=12MHz
// 7-6  11    ID, divide by 8
// 5-4  11    MC, up-down mode
// 2    0     TACLR, no clear
// 1    0     TAIE, no interrupt
// 0          TAIFG
}

//***************************PWM_Duty1*******************************
// change duty cycle of PWM output on P2.4
// Inputs:  duty1
// Outputs: none
// period of P2.4 is 2*period*666.7ns, duty cycle is duty1/period
void PWM_Duty1(uint16_t duty1){
  if(duty1 >= TIMER_A0->CCR[0]) return; // bad input
  TIMER_A0->CCR[1] = duty1;        // CCR1 duty cycle is duty1/period
}

//***************************PWM_Duty2*******************************
// change duty cycle of PWM output on P2.5
// Inputs:  duty2
// Outputs: none// period of P2.5 is 2*period*666.7ns, duty cycle is duty2/period
void PWM_Duty2(uint16_t duty2){
  if(duty2 >= TIMER_A0->CCR[0]) return; // bad input
  TIMER_A0->CCR[2] = duty2;        // CCR2 duty cycle is duty2/period
}

//***************************PWM_Init34*******************************
// PWM outputs on P2.6, P2.7
// Inputs:  period (1.333us)
//          duty3
//          duty4
// Outputs: none
// SMCLK = 48MHz/4 = 12 MHz, 83.33ns
// Counter counts up to TA0CCR0 and back down
// Let Timerclock period T = 8/12MHz = 666.7ns
// period of P7.3 squarewave is 4*period*666.7ns
// P2.6=1 when timer equals TA0CCR3 on way down, P2.6=0 when timer equals TA0CCR3 on way up
// P2.7=1 when timer equals TA0CCR4 on way down, P2.7=0 when timer equals TA0CCR4 on way up
// Period of P2.6 is period*1.333us, duty cycle is duty3/period
// Period of P2.7 is period*1.333us, duty cycle is duty4/period
void PWM_Init34(uint16_t period, uint16_t duty3, uint16_t duty4){
    if(duty3 >= period) return; // bad input
    if(duty4 >= period) return; // bad input
    P2->DIR |= 0xC0;          // P2.6, P2.7 output
    P2->SEL0 |= 0xC0;         // P2.6, P2.7 Timer0A functions
    P2->SEL1 &= ~0xC0;        // P2.6, P2.7 Timer0A functions
    TIMER_A0->CCTL[0] = 0x0080;      // CCI0 toggle
    TIMER_A0->CCR[0] = period;       // Period is 2*period*8*83.33ns is 1.333*period
    TIMER_A0->EX0 = 0x0000;        //    divide by 1
    TIMER_A0->CCTL[3] = 0x0040;      // CCR3 toggle/reset
    TIMER_A0->CCR[3] = duty3;        // CCR3 duty cycle is duty1/period
    TIMER_A0->CCTL[4] = 0x0040;      // CCR4 toggle/reset
    TIMER_A0->CCR[4] = duty4;        // CCR4 duty cycle is duty2/period
    TIMER_A0->CTL = 0x02F0;        // SMCLK=12MHz, divide by 8, up-down mode
    // bit  mode
    // 9-8  10    TASSEL, SMCLK=12MHz
    // 7-6  11    ID, divide by 8
    // 5-4  11    MC, up-down mode
    // 2    0     TACLR, no clear
    // 1    0     TAIE, no interrupt
    // 0          TAIFG

}

//***************************PWM_Duty3*******************************
// change duty cycle of PWM output on P2.6
// Inputs:  duty3
// Outputs: none
// period of P2.6 is 2*period*666.7ns, duty cycle is duty3/period
void PWM_Duty3(uint16_t duty3){
    if(duty3 >= TIMER_A0->CCR[0]) return; // bad input
    TIMER_A0->CCR[3] = duty3;        // CCR3 duty cycle is duty3/period

}

//***************************PWM_Duty4*******************************
// change duty cycle of PWM output on P2.7
// Inputs:  duty4
// Outputs: none// period of P2.7 is 2*period*666.7ns, duty cycle is duty2/period
void PWM_Duty4(uint16_t duty4){
    if(duty4 >= TIMER_A0->CCR[0]) return; // bad input
    TIMER_A0->CCR[4] = duty4;        // CCR3 duty cycle is duty3/period
}

//***************************PWM_RobotArmInit*******************************
// PWM outputs on P2.4/PM_TA0.1 (PMAP from TA1.1), P3.5/PM_UCB2CLK (PMAP from TA1.2), and P5.7/TA2.2/VREF-/VeREF-/C1.6
// Inputs:  period (333.33ns)
//          duty0 (0<=duty0<period-1)
//          duty1 (0<=duty1<period-1)
//          duty2 (0<=duty2<period-1)
// Outputs: none
// SMCLK = 48MHz/4 = 12 MHz, 83.33ns
// Use clock divider of 2 to get timer clock period 166.67ns
// Counter counts up to TAnCCR0 and back down
// Let Timerclock period T = 2/12MHz = 166.67nsns
// P2.4=1 when timer equals TA1CCR1 on way down, P2.4=0 when timer equals TA1CCR1 on way up
// P3.5=1 when timer equals TA1CCR2 on way down, P3.5=0 when timer equals TA1CCR2 on way up
// P5.7=1 when timer equals TA2CCR2 on way down, P5.7=0 when timer equals TA2CCR2 on way up
// Period of P2.4 is period*333.33ns, duty cycle is duty0/period
// Period of P3.5 is period*333.33ns, duty cycle is duty1/period
// Period of P5.7 is period*333.33ns, duty cycle is duty2/period
void PWM_RobotArmInit(uint16_t period, uint16_t duty0, uint16_t duty1, uint16_t duty2){
  if(duty0 >= period) return;    // bad input
  if(duty1 >= period) return;    // bad input
  if(duty2 >= period) return;    // bad input
  TIMER_A1->CTL &= ~0x0030;      // halt TimerA1 while port mapping is in progress
  PMAP->KEYID = 0x2D52;          // write key to unlock write access to PMAP registers
  PMAP->CTL = 0x0002;            // allow reconfiguration of port mapping (in case needed in another module)
  P2MAP->PMAP_REGISTER4 = PMAP_TA1CCR1A;// configure P2.4 as TA1.1
  P2->DIR |= 0x10;               // P2.4 output
  P2->SEL0 |= 0x10;              // P2.4 PMAP functions
  P2->SEL1 &= ~0x10;             // P2.4 PMAP functions
  P3MAP->PMAP_REGISTER5 = PMAP_TA1CCR2A;// configure P3.5 as TA1.2
  P3->DIR |= 0x20;               // P3.5 output
  P3->SEL0 |= 0x20;              // P3.5 PMAP functions
  P3->SEL1 &= ~0x20;             // P3.5 PMAP functions
  PMAP->KEYID = 0x0000;          // write incorrect key to lock write access to PMAP registers
  TIMER_A1->CCTL[0] = 0x0080;    // CCI0 toggle
  TIMER_A1->CCR[0] = period - 1; // Period is 2*period*8*166.67ns is 2.666*period
  TIMER_A1->EX0 = 0x0000;        //    divide by 1
  TIMER_A1->CCTL[1] = 0x0040;    // CCR1 toggle/reset
  TIMER_A1->CCR[1] = duty0;      // CCR1 duty cycle is duty0/period
  TIMER_A1->CCTL[2] = 0x0040;    // CCR2 toggle/reset
  TIMER_A1->CCR[2] = duty1;      // CCR2 duty cycle is duty1/period
  TIMER_A1->CTL = 0x0270;        // SMCLK=12MHz, divide by 2, up-down mode
// bit  mode
// 9-8  10    TASSEL, SMCLK=12MHz
// 7-6  01    ID, divide by 2
// 5-4  11    MC, up-down mode
// 2    0     TACLR, no clear
// 1    0     TAIE, no interrupt
// 0          TAIFG
  P5->DIR |= 0x80;               // P5.7 output
  P5->SEL0 |= 0x80;              // P5.7 TimerA2.2 functions
  P5->SEL1 &= ~0x80;             // P5.7 TimerA2.2 functions
  TIMER_A2->CCTL[0] = 0x0080;    // CCI0 toggle
  TIMER_A2->CCR[0] = period - 1; // Period is 2*period*8*166.67ns is 2.666*period
  TIMER_A2->EX0 = 0x0000;        //    divide by 1
  TIMER_A2->CCTL[2] = 0x0040;    // CCR2 toggle/reset
  TIMER_A2->CCR[2] = duty2;      // CCR2 duty cycle is duty2/period
  TIMER_A2->CTL = 0x0270;        // SMCLK=12MHz, divide by 2, up-down mode
// bit  mode
// 9-8  10    TASSEL, SMCLK=12MHz
// 7-6  01    ID, divide by 2
// 5-4  11    MC, up-down mode
// 2    0     TACLR, no clear
// 1    0     TAIE, no interrupt
// 0          TAIFG
}

//***************************PWM_RobotArmDuty0*******************************
// change duty cycle of PWM output on P2.4
// Inputs:  duty0
// Outputs: none
// Period of P2.4 is period*333.33ns, duty cycle is duty0/period
void PWM_RobotArmDuty0(uint16_t duty0){
  if(duty0 >= TIMER_A1->CCR[0]) return; // bad input
  TIMER_A1->CCR[1] = duty0;      // CCR1 duty cycle is duty0/period
}

//***************************PWM_RobotArmGetDuty0*******************************
// get the duty cycle of PWM output on P2.4
// Inputs: none
// Outputs: duty0
// Period of P2.4 is period*333.33ns, duty cycle is duty0/period
uint16_t PWM_RobotArmGetDuty0(void){
  return TIMER_A1->CCR[1];       // CCR1 duty cycle is duty0/period
}

//***************************PWM_RobotArmDuty1*******************************
// change duty cycle of PWM output on P3.5
// Inputs:  duty1
// Outputs: none
// Period of P3.5 is period*333.33ns, duty cycle is duty1/period
void PWM_RobotArmDuty1(uint16_t duty1){
  if(duty1 >= TIMER_A1->CCR[0]) return; // bad input
  TIMER_A1->CCR[2] = duty1;      // CCR2 duty cycle is duty1/period
}

//***************************PWM_RobotArmGetDuty1*******************************
// get the duty cycle of PWM output on P3.5
// Inputs: none
// Outputs: duty1
// Period of P3.5 is period*333.33ns, duty cycle is duty1/period
uint16_t PWM_RobotArmGetDuty1(void){
  return TIMER_A1->CCR[2];       // CCR2 duty cycle is duty1/period
}

//***************************PWM_RobotArmDuty2*******************************
// change duty cycle of PWM output on P5.7
// Inputs:  duty2
// Outputs: none
// Period of P5.7 is period*333.33ns, duty cycle is duty2/period
void PWM_RobotArmDuty2(uint16_t duty2){
  if(duty2 >= TIMER_A2->CCR[0]) return; // bad input
  TIMER_A2->CCR[2] = duty2;      // CCR2 duty cycle is duty2/period
}

//***************************PWM_RobotArmGetDuty2*******************************
// get the duty cycle of PWM output on P5.7
// Inputs: none
// Outputs: duty2
// Period of P5.7 is period*333.33ns, duty cycle is duty2/period
uint16_t PWM_RobotArmGetDuty2(void){
  return TIMER_A2->CCR[2];       // CCR2 duty cycle is duty2/period
}

/* new */
void my_pwm_init34(uint16_t period, uint16_t duty3, uint16_t duty4) {

    TIMER_A0->CCR[0] = period; // CCR0 period

    TIMER_A0->EX0 = 0x0000; // divide by 1

    // toggle ? reset ?
    TIMER_A0->CCTL[3] = 0x0040;
    TIMER_A0->CCR[3] = duty3;
    TIMER_A0->CCTL[4] = 0x0040;
    TIMER_A0->CCR[4] = duty4;

    // 0x200 -> SMCLK
    // 0b1100 0000 -> input divider /8
    // 0b0011 0000 -> up and down mode set.
    TIMER_A0->CTL = 0x02F0;

    // set alternative  // with set pin mode
    P2->DIR |= 0xC0;
    P2->SEL0 |= 0xC0;
    P2->SEL1 &= ~0xC0;
 }


//



//
void SysTick_Handler(void){

    if (Count % 10 == 0) {
        Reflectance_Start();
    } else if (Count % 10 == READ_DELAY) {
        SensorInput_F = 0;                  // Set Semaphore
        SensorInput = Reflectance_End();    // Write to SensorInput
        SensorInput_F = 1;                  // Reset Semaphore
    }

    Count += 1;
}

uint8_t ReadSensorData(void){

    return SensorInput;
}


//Accessor Function for FSM Controller to stop operating if collision has occured.
uint8_t getColFlag(){
    return CollisionFlag;
}


void HandleCollision(uint8_t bumpSensor){
   CollisionData = bumpSensor;
   CollisionFlag = 1;
   Motor_Stop();

   /**
    *
    * if(CollisionData == 0x1f || CollisionData == 0x2f || CollisionData == 0x37) {
        Motor_Backward(5000, 5000);
        Clock_Delay1ms(300);
        Motor_Left(7000, 7000);
        Clock_Delay1ms(300);
        Motor_Stop();
        }
       else if(CollisionData == 0x3b || CollisionData == 0x3d || CollisionData == 0x3e) {
        Motor_Backward(5000, 5000);
        Clock_Delay1ms(300);
        Motor_Right(7000, 7000);
        Clock_Delay1ms(300);
        Motor_Stop();
        }
    */
}
//void HandleCollision(uint8_t bumpSensor) {
//Motor_Stop();
//CollisionData = bumpSensor;
//CollisionFlag = 1;
//}


//
void BumpInt_Init(void(*task)(void)){
    P4->DIR &= ~0xED;
    P4->OUT |= 0xED;
    P4->REN |= 0xED;
    P4->IES |= 0xED;
    P4->IFG &= ~0xED;
    P4->IE |= 0xED;
    NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF)|0x00400000;
    NVIC->ISER[1] = 1 << ((PORT4_IRQn) & 31);
}




void SysTick_Init(uint32_t period, uint32_t priority){
  SysTick->CTRL = 0;              // 1) disable SysTick during setup
  SysTick->LOAD = period - 1;     // 2) reload value sets period
  SysTick->VAL = 0;               // 3) any write to current clears it
  SCB->SHP[11] = priority<<5;     // set priority into top 3 bits of 8-bit register
  SysTick->CTRL = 0x00000007;     // 4) enable SysTick with core clock and interrupts
}


//

//*********** DisableInterrupts ***************
// disable interrupts
// inputs:  none
// outputs: none
void DisableInterrupts(void){
  __asm ("    CPSID  I\n"
         "    BX     LR\n");
}

//*********** EnableInterrupts ***************
// enable interrupts
// inputs:  none
// outputs: none
void EnableInterrupts(void){
  __asm  ("    CPSIE  I\n"
          "    BX     LR\n");
}

// REFERENCE CODE COMMENT
/* MAIN  BODY */

    //Clock_Init48MHz();
    //systick_init();
    //motor_init();
    //ir_sensor_init();
    //main body 코드 흐름
    //if (sensor & 0x10) { // 중앙 센서가 검은 선을 감지
    //      P2->OUT |= 0x01;
    //    left_forward();
    //    right_forward();
    //    move(2000, 2000); // 직진
    //    systick_wait1s();
    //} else if (sensor & 0x08) { // 좌측 센서가 검은 선을 감지
    //    P2->OUT |= 0x01;
    //left_forward();
    //    right_forward();
    //    move(1000, 2000); // 좌회전 (좌측으로 틀기)
    //    systick_wait1s();
    //} else if (sensor & 0x20) { // 우측 센서가 검은 선을 감지
    //    P2->OUT |= 0x01;
    //    left_forward();
    //    right_forward();
    //    move(2000, 1000); // 우회전 (우측으로 틀기)
    //    systick_wait1s();
    //} else {
    //    P2->OUT &= ~0x07;
    //    stop(); // 선이 감지되지 않으면 정지
    //}
    //P5->OUT &= ~0x08;
    //P9->OUT &= ~0x04;
    //Clock_Delay1ms(10);
    //Clock_Delay1ms(100); // 안정화 대기

/* MAIN  BODY */




/* IR SENSOR SNIPPET */
/*
 *
 *
 *
 *
void my_ir_sensor_init(void) {

    // IR 센서 초기화
    P5->SEL0 &= ~0x08;
    P5->SEL1 &= ~0x08;
    P5->DIR |= 0x08;
    P5->OUT &= ~0x08;

    P9->SEL0 &= ~0x04;
    P9->SEL1 &= ~0x04;
    P9->DIR |= 0x04;
    P9->OUT &= ~0x04;

    P7->SEL0 &= ~0xFF;
    P7->SEL1 &= ~0xFF;
    P7->DIR &= ~0xFF;
}
uint8_t my_read_ir_sensor(void) {

    P5->OUT |= 0x08; P9->OUT |= 0x04;
    P7->DIR = 0xFF;
    P7->OUT = 0xFF;
    Clock_Delay1us(10); // 충전 대기
    // 입력으로 전환
    P7->DIR = 0x00;
    // 잠시 대기
    Clock_Delay1us(1000);
    // 입력값 읽기
    return (P7->IN);
}

 *
 *
 *
 */

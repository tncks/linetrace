
#include "msp.h"
#include "Clock.h"
#include <stdio.h>
#include "module.h"

void main(void)
{
    Clock_Init48MHz();
    SysTick_Init(48000, 2);
    Reflectance_Init();
    EnableInterrupts();
    Motor_Init();
    //BumpInt_Init(&HandleCollision);
    //start_fsm();
}


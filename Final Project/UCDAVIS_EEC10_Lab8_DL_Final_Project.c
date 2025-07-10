/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>

#include "../inc/SysTick.h"
#include "../inc/Clock.h"

///////////// Variable Definitions /////////////

uint32_t Size;
uint32_t I;
uint32_t qp;

uint16_t c;
uint16_t cc;
uint16_t j;
uint16_t i;

int start = 0;

float avgMax_old_P6_0 = 0;
float avgMax_old_P6_1 = 0;

float x[1024];
float y[1024];
float z[1024];
float z2[1024];

float alpha;
float alpha_new;

float avgMax_P6_0 = 0, avgMax_P6_1 = 0;
float maxP6_0 = 0, maxP6_1 = 0;

float threshold = 0.3; // Lowered threshold for increased sensitivity
float soundLevel = 0;
float signalDifference = 0;

float avgmax1;
float avgmax2;
float avgmax1_old;
float avgmax2_old;
float offset;
float offset_old;

int32_t INPUT_P6_1[1024];
float Real_INPUT_P6_1[1024];

float max1[10] = {0};  // Find 10 local maximums in the array of Real_Input
float max2[10] = {0};  // Find 10 local maximums in the array of Real_Input

int32_t INPUT_P6_0[1024];
float Real_INPUT_P6_0[1024];

float x1[1024];
float y1[1024];

float x2[1024];
float y2[1024];

uint8_t DIRECTION;

#define FORWARD    1
#define BACKWARD   2
#define LEFT       3
#define RIGHT      4
#define STOP       5
#define ROTATE_180 6

uint8_t MODE;

#define SAMPLING_MODE  1
#define RUNNING_MODE   2

/////////////////////////////////////////////////

/* Timer Configuration */
#define PERIOD  100

/* Timer_A UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig = {
    TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source 12MHz
    TIMER_A_CLOCKSOURCE_DIVIDER_12,         // SMCLK/12 = 1MHz Timer clock
    PERIOD,                                 // Period of 100 timer clocks => 10 KHz Frequency
    TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,     // Enable CCR0 interrupt
    TIMER_A_DO_CLEAR                        // Clear value
};

/* PWM Configuration */
#define TIMER_PERIOD 15000  // 10 ms PWM Period
#define DUTY_CYCLE1  0
#define DUTY_CYCLE2  0

/* Timer_A UpDown Configuration Parameter */
Timer_A_UpDownModeConfig upDownConfig = {
    TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
    TIMER_A_CLOCKSOURCE_DIVIDER_4,          // SMCLK/4 = 1.5MHz
    TIMER_PERIOD,                           // 15000 period
    TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
    TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
    TIMER_A_DO_CLEAR                        // Clear value
};

/* Timer_A Compare Configuration Parameter (PWM3) */
Timer_A_CompareModeConfig compareConfig_PWM1 = {
    TIMER_A_CAPTURECOMPARE_REGISTER_1,          // Use CCR3
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
    TIMER_A_OUTPUTMODE_TOGGLE_RESET,            // Toggle output mode
    DUTY_CYCLE1
};

/* Timer_A Compare Configuration Parameter (PWM4) */
Timer_A_CompareModeConfig compareConfig_PWM2 = {
    TIMER_A_CAPTURECOMPARE_REGISTER_2,          // Use CCR4
    TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
    TIMER_A_OUTPUTMODE_TOGGLE_RESET,            // Toggle output mode
    DUTY_CYCLE2
};

/////////////////////////////////////////////////////////////

/* Function Prototypes */
void TimerA2_Init(void);
void PWM_Init12(void);
void PWM_duty1(uint16_t duty1, Timer_A_CompareModeConfig* data);
void PWM_duty2(uint16_t duty1, Timer_A_CompareModeConfig* data);
void MotorInit(void);
void motor_forward(uint16_t leftDuty, uint16_t rightDuty);
void motor_right(uint16_t leftDuty, uint16_t rightDuty);
void motor_left(uint16_t leftDuty, uint16_t rightDuty);
void motor_backward(uint16_t leftDuty, uint16_t rightDuty);
void motor_stop(void);
void ADC_Ch14Ch15_Init(void);

//////////////////////// MAIN FUNCTION /////////////////////////////////////

int main(void) {
    Size = 1000;
    I = Size - 1;

    // Set Microcontroller Clock to 48 MHz
    Clock_Init48MHz();

    // Initialize PWM
    PWM_Init12();

    // Initialize SysTick
    SysTick_Init();

    // Motor Configuration
    MotorInit();

    // Set up GPIO Port 6, Pin 4 as Output
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN4);

    // Setup ADC for Channel A14 and A15
    ADC_Ch14Ch15_Init();

    // Timer A2 Configuration
    TimerA2_Init();

    // Set Default Mode and Direction
    DIRECTION = FORWARD;
    MODE = SAMPLING_MODE;

    while (1) {
        // Infinite loop for continuous operation
    }
}

//////////////////////// FUNCTIONS /////////////////////////////////////



void TA2_0_IRQHandler(void) {
    GPIO_toggleOutputOnPin(GPIO_PORT_P6, GPIO_PIN4);

    // IN SAMPLING MODE
    if (MODE == SAMPLING_MODE) {
        motor_stop(); // Stop motors to avoid interference from motor noise
        ADC14_toggleConversionTrigger(); // Trigger ADC conversion

        while (ADC14_isBusy()) {} // Wait for ADC conversion to finish

        // Read raw ADC data from both microphones
        INPUT_P6_0[I] = ADC14_getResult(ADC_MEM1); // Microphone 1 (P6.1)
        INPUT_P6_1[I] = ADC14_getResult(ADC_MEM0); // Microphone 2 (P6.0)

        // Convert raw ADC data to voltage
        Real_INPUT_P6_0[I] = (INPUT_P6_0[I] * 3.3) / 16384; // Microphone 1
        Real_INPUT_P6_1[I] = (INPUT_P6_1[I] * 3.3) / 16384; // Microphone 2

        if (I == 0) {
            I = Size - 1;
            MODE = RUNNING_MODE;

            ///// MAKE DIRECTION DECISION BASED ON SAMPLING RESULTS /////

            // Design a high-pass filter
            for (c = 0; c < 1000; c++) {
                x[c] = Real_INPUT_P6_1[c];
                x2[c] = Real_INPUT_P6_0[c];
            }

            alpha = 0.8888; // Cutoff frequency to 200 Hz
            y[0] = x[0];
            y2[0] = x2[0];

            // High-pass filtering
            for (c = 1; c < 1000; c++) {
                y[c] = alpha * y[c - 1] + alpha * (x[c] - x[c - 1]);
                y2[c] = alpha * y2[c - 1] + alpha * (x2[c] - x2[c - 1]);
            }

            // Low-pass filtering
            alpha_new = 0.65337; //3k frequency cutoff
            z[0] = alpha_new * y[0];
            z2[0] = alpha_new * y2[0];

            for (c = 1; c < 1000; c++) {
                z[c] = alpha_new * y[c] + (1 - alpha_new) * z[c - 1];
                z2[c] = alpha_new * y2[c] + (1 - alpha_new) * z2[c - 1];
            }

            // Find max values for both microphones (P6.0 and P6.1)
            for (i = 0; i < 10; i++) {
                start = i * 100;
                maxP6_0 = 0;
                maxP6_1 = 0;

                for (qp = start; qp < start + 100 && qp < Size; qp++) {
                    if (z2[qp] > maxP6_0) {  // Max value from microphone 1 (P6.0)
                        maxP6_0 = z2[qp];
                    }
                    if (z[qp] > maxP6_1) {  // Max value from microphone 2 (P6.1)
                        maxP6_1 = z[qp];
                    }
                }

                avgMax_P6_0 += maxP6_0; // Add the maximums up, to divide later
                avgMax_P6_1 += maxP6_1;
            }

            // Compute average max values
            avgMax_P6_0 /= 10;
            avgMax_P6_1 /= 10;

            // Make direction decision based on filtered audio signals
            soundLevel = (avgMax_P6_0 + avgMax_P6_1) / 2.0; //average of the average, for overall sound
            signalDifference = fabs(avgMax_P6_0 - avgMax_P6_1); //Difference between right and left sound, for direction

            if (soundLevel > threshold) { //There is sound
                if (signalDifference > 0.03) { //More sound on one side than the other
                    DIRECTION = (avgMax_P6_0 > avgMax_P6_1) ? LEFT : RIGHT; //If left is higher, go left else go right
                } else { //they are approximately equal, so go straight
                    DIRECTION = FORWARD;
                }
            } else {
                DIRECTION = STOP; // Stop the robot since there's no sound
            }

            avgMax_old_P6_0 = avgMax_P6_0; //update the averages, meant to be used for 180 turn
            avgMax_old_P6_1 = avgMax_P6_1;

        } else {
            I--;
        }
    }

    // IN RUNNING MODE
    if (MODE == RUNNING_MODE) {
        uint16_t turn_speed = 2000;
        uint16_t turn_speed_slow = 500;

        if (DIRECTION == FORWARD) {
            motor_forward(turn_speed, turn_speed);
            SysTick_Wait10ms(300); // Run for 3s
        } else if (DIRECTION == LEFT) {
            motor_forward(turn_speed_slow, turn_speed); //Right wheel goes faster than left, turning left
            SysTick_Wait10ms(300);
        } else if (DIRECTION == RIGHT) {
            motor_forward(turn_speed, turn_speed_slow); //Left wheel goes faster than right, turning right
            SysTick_Wait10ms(300);
        } else if (DIRECTION == ROTATE_180) { //not used, but implemented
            motor_right(turn_speed, turn_speed);
            SysTick_Wait10ms(450);
        } else if (DIRECTION == STOP) {
            motor_stop();
            SysTick_Wait10ms(300);
        }

        MODE = SAMPLING_MODE; //Go back to sampling mode after running
    }

    Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}

void TimerA2_Init(void) {
    Timer_A_configureUpMode(TIMER_A2_BASE, &upConfig);
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableInterrupt(INT_TA2_0);
    Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);
    Interrupt_setPriority(INT_TA2_0, 0x20);
    Interrupt_enableMaster();
}

void PWM_Init12(void) {
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4 + GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
    Timer_A_configureUpDownMode(TIMER_A0_BASE, &upDownConfig);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UPDOWN_MODE);
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM1);
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM2);
}

void PWM_duty1(uint16_t duty1, Timer_A_CompareModeConfig* data) {
    if (duty1 >= TIMER_PERIOD) return;
    data->compareValue = duty1;
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM1);
}

void PWM_duty2(uint16_t duty2, Timer_A_CompareModeConfig* data) {
    if (duty2 >= TIMER_PERIOD) return;
    data->compareValue = duty2;
    Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM2);
}

void MotorInit(void) {
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN6); // P3.0 and P3.6 as outputs
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5 | GPIO_PIN7); // P3.5 and P3.7 as outputs
}

void motor_forward(uint16_t leftDuty, uint16_t rightDuty) {
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5 | GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0 | GPIO_PIN6);
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty, &compareConfig_PWM2);
}

void motor_right(uint16_t leftDuty, uint16_t rightDuty) {
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty, &compareConfig_PWM2);
}

void motor_left(uint16_t leftDuty, uint16_t rightDuty) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty, &compareConfig_PWM2);
}

void motor_backward(uint16_t leftDuty, uint16_t rightDuty) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
    PWM_duty1(rightDuty, &compareConfig_PWM1);
    PWM_duty2(leftDuty, &compareConfig_PWM2);
}

void motor_stop(void) {
    PWM_duty1(0, &compareConfig_PWM1);
    PWM_duty2(0, &compareConfig_PWM2);
}

void ADC_Ch14Ch15_Init(void) {
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0 | GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

    ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, false);
    ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A15, false);

    ADC14_disableInterrupt(ADC_INT1);
    Interrupt_disableInterrupt(INT_ADC14);

    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
    ADC14_enableConversion();
}


///////////////////////////////////////END/////////////////////////////////////////////


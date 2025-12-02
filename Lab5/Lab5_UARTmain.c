// RSLK
#include "msp.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "..\inc\UART0.h"
#include "..\inc\EUSCIA0.h"
#include "..\inc\FIFO0.h"
#include "..\inc\Clock.h"
//#include "..\inc\SysTick.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\TimerA1.h"
//#include "..\inc\Bump.h"
#include "..\inc\BumpInt.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Motor.h"
#include "../inc/IRDistance.h"
#include "../inc/ADC14.h"
#include "../inc/LPF.h"
#include "..\inc\Reflectance.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Tachometer.h"

#define P2_4 (*((volatile uint8_t *)(0x42098070)))
#define P2_3 (*((volatile uint8_t *)(0x4209806C)))
#define P2_2 (*((volatile uint8_t *)(0x42098068)))
#define P2_1 (*((volatile uint8_t *)(0x42098064)))
#define P2_0 (*((volatile uint8_t *)(0x42098060)))

void AppMotorTest(void);
void AppIRTest(void);
void AppBumperTest(void);
void AppRefTest(void);
void AppTachTest(void);
void AppAdvBumpTest(void);
void AppLineFollowerTest(void);

//#define DEBUG

/********* APP UTIL ***********/
const char* conv_int2byte(uint8_t x)
{
    static char b[9];
    b[0] = '\0';

    uint8_t z;
    for (z = 128; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}


static void fsm_led(uint8_t v)
{
    LaunchPad_Output((v == 0) << 1 | (v == 1) << 0 | (v == 2) << 2);
}

typedef enum
{
    SEND, REPLACE
} eUART_SEND;

static void AppUARTSend(char* str, eUART_SEND t)
{
    if (t == REPLACE)
        UART0_OutString("\033[A\33[2K\r");
    UART0_OutString(str);
    EUSCIA0_OutChar(CR);
    EUSCIA0_OutChar(LF);
}

/*****************************/
// =============================================================================
// LED Helper Functions for Visual Feedback
// Uses LaunchPad RGB LED: P2.0=Red, P2.1=Green, P2.2=Blue
// =============================================================================

// Turn off all LEDs
static void LED_Off(void) {
    LaunchPad_Output(0); // 0b000 = all off
}

// Set LED to a specific color (use with COLOR_ constants below)
static void LED_SetColor(uint8_t color) {
    LaunchPad_Output(color);
}

/*
Predefined color constants (match LaunchPad hardware):
- COLOR_OFF: 0x00 (0b000) - All colors off
- COLOR_RED: 0x01 (0b001) - Red (P2.0)
- COLOR_GREEN: 0x02 (0b010) - Green (P2.1)
- COLOR_YELLOW: 0x03 (0b011) - Red + Green
- COLOR_BLUE: 0x04 (0b100) - Blue (P2.2)
- COLOR_PURPLE: 0x05 (0b101) - Red + Blue
- COLOR_CYAN: 0x06 (0b110) - Green + Blue
- COLOR_WHITE: 0x07 (0b111) - All colors on
*/


/*****************************/

/* [7] FSM */

typedef enum
{
    STATE_OK = 0,
    STATE_L,
    STATE_R,
    STATE_ERR = -1
} eStateFsm;
typedef struct
{
    eStateFsm state;
    uint32_t iterations;
    void(*fp_led)(uint8_t);

} fsm_t;
static void _proc_fsm(fsm_t* fsm, int32_t sig);
static const uint32_t STATE_PERSIST_DUR_MS = 25;
static uint32_t MOTOR_SPEED = 1200;

/***********/


void RSLK_Reset(void)
{
    DisableInterrupts();

    LaunchPad_Init();

    EnableInterrupts();
}

void TimedPause(uint32_t time)
{
    Clock_Delay1ms(time);          // run for a while and stop
    Motor_Stop();
}

// variables for ir sensors
volatile uint32_t ADCvalue;
volatile uint32_t ADCflag;
volatile uint32_t nr, nc, nl;


// variables for bumper
static volatile uint8_t bumpIntData, isCollided;
void handlerGpio4Bump(uint8_t v)
{
    Motor_Stop();
    bumpIntData = v;
    isCollided = 1;
    P4->IFG &= ~0xED; // clear interrupt flags

}
uint8_t bump_data;
bool bump[6] = { 0 };
char bumperBuf[30];


void handlerAdcRead(void)
{  // runs at 2000 Hz
    uint32_t raw17, raw12, raw16;
    P1OUT ^= 0x01;         // profile
    P1OUT ^= 0x01;         // profile
    ADC_In17_12_16(&raw17, &raw12, &raw16);  // sample
    nr = LPF_Calc(raw17);  // right is channel 17 P9.0
    nc = LPF_Calc2(raw12);  // center is channel 12, P4.1
    nl = LPF_Calc3(raw16);  // left is channel 16, P9.1
    ADCflag = 1;           // semaphore
    P1OUT ^= 0x01;         // profile
}





// tachometer variables
uint16_t Period0;              // (1/SMCLK) units = 83.3 ns units
uint16_t First0 = 0;             // Timer A3 first edge, P10.4
uint32_t Done0 = 0;              // set each rising

uint16_t Period2;              // (1/SMCLK) units = 83.3 ns units
uint16_t First2 = 0;             // Timer A3 first edge, P8.2
uint32_t Done2 = 0;              // set each rising
uint32_t LeftRPM = 0, RightRPM = 0;

// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure0(uint16_t time)
{
    Period0 = (time - First0) & 0xFFFF; // 16 bits, 83.3 ns resolution
    First0 = time;                    // setup for next
    Done0++;
}

// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure2(uint16_t time)
{
    Period2 = (time - First2) & 0xFFFF; // 16 bits, 83.3 ns resolution
    First2 = time;                    // setup for next
    Done2++;
}



// RSLK Self-Test
// Sample program of how the text based menu can be designed.
// Only one entry (RSLK_Reset) is coded in the switch case. Fill up with other menu entries required for Lab5 assessment.
// Init function to various peripherals are commented off.  For reference only. Not the complete list.

int main(void)
{
    uint32_t cmd = 0xDEAD, menu = 0;

    DisableInterrupts();
    Clock_Init48MHz();  // makes SMCLK=12 MHz
    //SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts
    Motor_Init();
    //Motor_Stop();
    LaunchPad_Init();
    BumpInt_Init(&handlerGpio4Bump);
    //Bumper_Init();
    //IRSensor_Init();
    //Tachometer_Init();
    EUSCIA0_Init();     // initialize UART
    EnableInterrupts();

    while (1)
    {                     // Loop forever
        // write this as part of Lab 5
        EUSCIA0_OutChar(CR);
        EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("RSLK Self-Test (Basic + Adv) UI");
        EUSCIA0_OutChar(CR);
        EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[0] RSLK Reset");
        EUSCIA0_OutChar(CR);
        EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[1] Motor Test");
        EUSCIA0_OutChar(CR);
        EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[2] IR Sensor Test");
        EUSCIA0_OutChar(CR);
        EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[3] Bumper Test");
        EUSCIA0_OutChar(CR);
        EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[4] Reflectance Sensor Test");
        EUSCIA0_OutChar(CR);
        EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[5] Tachometer Test");
        EUSCIA0_OutChar(CR);
        EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[6] All Bump Switches Detection");
        EUSCIA0_OutChar(CR);
        EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[7] FSM Line follower test not working");
        EUSCIA0_OutChar(CR);
        EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[8] Turn Left 90 degrees");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[9] Turn 90 Left when black line detected");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[10] Turn Right 90 degrees");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[11] Turn 90 Right when black line detected");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[12] Line Following");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[13] Obstacle Avoidance");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[14] Bump Binary Converter");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[15] The L task Switch Control LED Demo");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[16] Bumper Recovery Mode to Turn Robot Away from Obstacle");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[17] IR distance Led detection with LED");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[18] Forward with Safety Stop");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[19] H task Sequenced Wall Follow");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[20] Cup Navigation not working");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[21] LF case 1 need to check not working");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[22] LF case 2 need to check not working ");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString("[23] bump sensor counter");
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

        EUSCIA0_OutString("CMD: ");
        cmd = EUSCIA0_InUDec();
        EUSCIA0_OutChar(CR);
        EUSCIA0_OutChar(LF);
        isCollided = 0;
        // \033[A\33[2K\r VT100 code to clear prev line
        switch (cmd)
        {
        case 0:
            RSLK_Reset();
            menu = 1;
            cmd = 0xDEAD;
            AppUARTSend("SYSTEM RESET", SEND);
            break;

        case 1:
            // Motor Test
            AppUARTSend("STARTING MOTOR TEST", SEND);
            AppMotorTest();
            AppUARTSend("MOTOR TEST DONE. Returning.", SEND);
            break;

        case 2:
            // IR Sensor test
            AppUARTSend("STARTING IR SENSOR TEST", SEND);
            AppIRTest();
            AppUARTSend("IR SENSOR TEST DONE. Returning.", SEND);
            break;

        case 3:
            // Bumpers
            AppUARTSend("STARTING BUMPER TEST", SEND);
            AppBumperTest();
            AppUARTSend("BUMPER TEST DONE. Returning.", SEND);

            break;

        case 4:
            // reflectance
            AppUARTSend("STARTING REFLECTANCE SENSOR TEST", SEND);
            AppRefTest();
            AppUARTSend("REFLECTANCE SENSOR TEST DONE. Returning.", SEND);
            break;

        case 5:

            // Tachometer test
            AppUARTSend("STARTING TACHOMETER SENSOR TEST", SEND);

            AppTachTest();
            AppUARTSend("TACHOMETER SENSOR TEST DONE. Returning.", SEND);
            break;

        case 6:
            // Bumpers

            AppUARTSend(" Simultaneous bump switches detection", SEND);
            AppUARTSend(" ", SEND);
            AppAdvBumpTest();

            AppUARTSend(
                "Simultaneous bump switches detection TEST DONE. Returning.",
                SEND);
            break;
        case 7:
            // Bumpers

            AppUARTSend("Line follower test FSM ", SEND);
            AppUARTSend(" ", SEND);
            AppLineFollowerTest();

            AppUARTSend(
                "Line follower TEST DONE. Returning.",
                SEND);
            break;
        case 8:
            AppUARTSend("Starting LEFT 90 turn", SEND);
            AppTurnLeft90();
            AppUARTSend("Turned Left done", SEND);
            break;
        case 9:
            AppUARTSend("Starting BLACK LINE TURN 90 LEFT test", SEND);
            AppTurnLeftOnBlackLine();
            AppUARTSend("BLACK LINE TURN 90 left test done. Returning.", SEND);
            break;
        case 10:
            AppUARTSend("Starting RIGHT 90 turn", SEND);
            AppTurnRight90();
            AppUARTSend("Turned Right done.", SEND);
            break;
        case 11:
            AppUARTSend("Starting BLACK LINE TURN 90 RIGHT test", SEND);
            AppTurnRightOnBlackLine();
            AppUARTSend("BLACK LINE TURN 90 Right test done. Returning.", SEND);
            break;
        case 12:
            AppUARTSend("Starting LINE FOLLOWING test", SEND);
            Line_Following();
            AppUARTSend("LINE FOLLOWING test done. Returning.", SEND);
            break;
        case 13:
            AppUARTSend("Starting OBSTACLE AVOIDANCE", SEND);
            Obstacle_Avoidance();
            AppUARTSend("OBSTACLE AVOIDANCE DONE. Returning.", SEND);
            break;
        case 14:
            AppUARTSend("Starting Bump Binary Converter", SEND);
            AppBumpBinaryConverter();
            AppUARTSend("Bump Binary Converter DONE. Returning.", SEND);
            break;
        case 15:
             AppUARTSend("Starting Switch Control Demo", SEND);
             AppSwitchControlLED();
             AppUARTSend("Switch Control Demo finished", SEND);
             break;
        case 16:
            AppUARTSend("Starting Bumper Recovery Mode", SEND);
//            AppBumper_Recovery();
            AppBumper_Recoveryy();
            AppUARTSend("Bumper Recovery finished", SEND);
            break;
        case 17:
            AppUARTSend("Starting IR distance LED detection", SEND);
            AppIR_Distance_Warning();
            AppUARTSend("IR distance Led detection DONE. Returning. ", SEND);
            break;
        case 18:
            AppUARTSend("Starting Sequenced Wall Follow", SEND);
            AppForward_With_Safety_Stop();
            AppUARTSend("Sequenced Wall Follow DONE. Returning. ", SEND);
            break;
        case 19:
            AppUARTSend("Starting Sequenced Wall Follow", SEND);
            AppSequenced_Wall_Follow();
            AppUARTSend("Sequenced Wall Follow DONE. Returning. ", SEND);
            break;
        case 20:
            AppUARTSend("Starting Cup Navigation", SEND);
            AppCupNavigation();
            AppUARTSend("Cup Navigation DONE. Returning.", SEND);
            break;
        case 21:
            AppUARTSend("Line following with switch cases to input stuff", SEND);
            AppAdvancedLineFollowing();
            AppUARTSend("End the thing DONE. Returning.", SEND);
            break;
        case 22:
            AppUARTSend("customize line following with scenarios not sure if working", SEND);
            Flexible_Line_Following();
            AppUARTSend("End the thing DONE. Returning.", SEND);
            break;
        case 23:
            AppUARTSend("BUmp sensor count", SEND);
            Bump_Sensor_Counter();
            AppUARTSend("End the thing DONE. Returning.", SEND);
            break;

            menu = 1;
            break;
        }

        if (!menu)
            Clock_Delay1ms(1000);

        else
        {
            menu = 0;
        }

    }
}

/*
 * This function uses timer PWM to control the motor speed.
 * It stops when all tests are complete.
 */
void AppMotorTest(void)
{
    AppUARTSend("DIRECTION: FORWARD", REPLACE);
    Motor_Forward(3000, 3000);
    TimedPause(2000);
    AppUARTSend("DIRECTION: BACKWARD", REPLACE);
    Motor_Backward(3000, 3000);
    TimedPause(2000);
    AppUARTSend("DIRECTION: RIGHT", REPLACE);
    Motor_Right(3000, 3000);
    TimedPause(2000);
    AppUARTSend("DIRECTION: LEFT", REPLACE);
    Motor_Left(3000, 3000);
    TimedPause(2000);
    AppUARTSend("RIGHT MOTOR ONLY", REPLACE);
    Motor_Forward(3000, 0);
    TimedPause(2000);
    AppUARTSend("LEFT MOTOR ONLY", REPLACE);
    Motor_Forward(0, 3000);
    TimedPause(2000);
    Motor_Stop();
}

/*
 * This function initializes ADC, runs a single sample to initialize lpf ringbuf,
 * then init timer a1 with fp to handlerAdcRead() to be called at 2000Hz. this will acquire
 * the raw ADC values and set the flag high.
 * the spinloop in the program will monitor the adc flag and print the values if it is set,
 * and exit the program if bumpswitch collision flag is set.
 */
void AppIRTest(void)
{
    isCollided = 0;

    char buf[100];
    uint32_t raw17, raw12, raw16;
    uint32_t s, n;

    ADCflag = 0;
    s = 256; // replace with your choice

    ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16

    ADC_In17_12_16(&raw17, &raw12, &raw16);  // sample

    LPF_Init(raw17, s);     // P9.0/channel 17
    LPF_Init2(raw12, s);     // P4.1/channel 12
    LPF_Init3(raw16, s);     // P9.1/channel 16

    //UART0_Init();          // initialize UART0 115,200 baud rate

    TimerA1_Init(&handlerAdcRead, 250);    // 2000 Hz sampling
    EnableInterrupts();

    while (1)
    {
        for (n = 0; n < 250; n++)
        {
            while (ADCflag == 0)
                ;
            ADCflag = 0; // show every nth point
        }
        if (isCollided)
        {

            isCollided = 0;
            return;
        }
        sprintf(buf, "%5dcm,%5dcm,%5dcm", LeftConvert(nl), CenterConvert(nc),
            RightConvert(nr));
        AppUARTSend(buf, REPLACE);
    }

}

/*
 * This function uses reads the status of the bump switches from Bump_Read()
 * on demand. this function is also asynchronously called by the gpio4handler.
 * It exits when all bump switches have been pressed.
 */
void AppBumperTest(void)
{
    isCollided = 0;

    while (!bump[0] || !bump[1] || !bump[2] || !bump[3] || !bump[4] || !bump[5])
    {
        bump_data = Bump_Read();

        for (uint8_t i = 0; i < 6; i++)
        {
            if ((((bump_data >> i & 0xF) & 0x1)) == 0 && bump[i] == 0)
            {
                bump[i] = true;
                char buf[30];
                snprintf(buf, 30, "BUMPER %d PRESSED", i);
                AppUARTSend(buf, REPLACE);

            }
        }
    }
    for (uint8_t i = 0; i < 6; i++)  bump[i] = false; // rst state

}

/*
 * This function reads from the reflectance sensors and prints out their value
 * It exits when isCollided is set from gpio4 interrupt.
 */
void AppRefTest(void)
{
    isCollided = 0;

    uint8_t Data; // QTR-8RC
    int32_t Position; // 332 is right, and -332 is left of center

    char refBuf[20] = "";

    while (1)
    {

        if (isCollided)
        {

            isCollided = 0;
            LaunchPad_Output(0); // turn off leds
            break;
        }
        Data = Reflectance_Read(2000);
        Position = Reflectance_Position(Data);

        // >>>>>>>>>> ADD THESE TWO LINES FOR LED FEEDBACK <<<<<<<<<<
                if (Data != 0x00)
                    LaunchPad_Output(0x02); // GREEN (P2.1)
                else
                    LaunchPad_Output(0x01); // RED (P2.0)
                // >>>>>>>>>> END OF ADDITION <<<<<<<<<<

        sprintf(refBuf, "%s, %d mm", conv_int2byte(Data), Position / 10);
        AppUARTSend(refBuf, REPLACE);


        Clock_Delay1ms(100);

    }
}

/*
 * This function init timer a3 in capture mode, starts motor, places the cpu in WFI, prints tachometer
 * info for each 100 exits from WFI, and exits if collided.
 */
void AppTachTest(void)
{
    isCollided = 0;
    EnableInterrupts();
    uint32_t count = 0;
    char buf[100];
    TimerA3Capture_Init(&PeriodMeasure0, &PeriodMeasure2);
    TimedPause(500);
    Motor_Forward(1500, 1500);
    EnableInterrupts();
    while (1)
    {
        //WaitForInterrupt();
        __WFI();
        count++;
        if (count % 100 == 0)
        {
            memset(buf, 0, sizeof(buf));
            // (pulse per rot * pulse dur * interval between pulse / 1k) = time taken for 1 rot in ms
            // ms in a minute / ^ = RPM
            LeftRPM = ((60 * 1000 * 1000) / (360 * 83.3 * Period0 / 1000));
            RightRPM = ((60 * 1000 * 1000) / (360 * 83.3 * Period2 / 1000));

            sprintf(buf, "Right RPM = %2d, Left RPM = %2d, Right count = %2d, Left count = %2d", RightRPM, LeftRPM, Period2, Period0);

            AppUARTSend(buf, REPLACE);
            count = 0;
        }
        if (isCollided)
        {

            isCollided = 0;
            return;
        }
    }
}

/*
 * This function also tests bump switches but allows for multiple simultaneous presses.
 * Exits when all are pressed
 */
void AppAdvBumpTest(void)
{
    while (1)
    {

        int pressed = 0;
        strcpy(bumperBuf, "Pressed on switches: ");
        bump_data = Bump_Read();
        if ((((bump_data >> 0) & 0x1)) == 0)
        {
            strcat(bumperBuf, "0 ");
            pressed = 1;
        }
        if ((((bump_data >> 1) & 0x1)) == 0)
        {
            strcat(bumperBuf, "1 ");
            pressed = 1;
        }
        if ((((bump_data >> 2) & 0x1)) == 0)
        {
            strcat(bumperBuf, "2 ");
            pressed = 1;
        }
        if ((((bump_data >> 3) & 0x1)) == 0)
        {
            strcat(bumperBuf, "3 ");
            pressed = 1;
        }
        if ((((bump_data >> 4) & 0x1)) == 0)
        {
            strcat(bumperBuf, "4 ");
            pressed = 1;
        }
        if ((((bump_data >> 5) & 0x1)) == 0)
        {
            strcat(bumperBuf, "5 ");
            pressed = 1;
        }

        if (!pressed)
        {
            AppUARTSend("Waiting for press", REPLACE);
        }
        else
            AppUARTSend(bumperBuf, REPLACE);

        if ((bump_data & 0b111111) == 0)
        {

            return;
        }

        Clock_Delay1ms(100);

    }
}



void AppTurnLeft90(void) {
    AppUARTSend("TURNING LEFT 90", SEND);
    Motor_Left(3000, 3000);
    Clock_Delay1ms(250); // Calibrate this delay for your RSLK
    Motor_Stop();
    AppUARTSend("TURN LEFT COMPLETE", REPLACE);
}

void AppTurnRight90(void) {
    AppUARTSend("TURNING Right 90", SEND);
    Motor_Right(3000, 3000);
    Clock_Delay1ms(250); // Calibrate this delay for your RSLK
    Motor_Stop();
    AppUARTSend("TURN RIGHT COMPLETE", REPLACE);
}

void AppTurnLeftOnBlackLine(void) {
    AppUARTSend("Move forward until black line detected", SEND);
    uint8_t Data;
    int32_t Position;

    Motor_Forward(1500, 1500); // start moving forward
    Clock_Delay1ms(100);

    while (1) {
        Data = Reflectance_Read(2000);
        Position = Reflectance_Position(Data);

        if (Data != 0) { // black line detected
            Motor_Stop();
            AppUARTSend("Black line detected!", SEND);
            break;
        }
    }

    // Now perform a 90 turn to the left
    AppUARTSend("Turning LEFT 90", SEND);
    AppTurnLeft90();


   // Clock_Delay1ms(750); // adjust this for your actual 90 turn
    Motor_Stop();

    AppUARTSend("Turn complete", REPLACE);
}


void AppTurnRightOnBlackLine(void) {
    AppUARTSend("Move forward until black line detected", SEND);
    uint8_t Data;
    int32_t Position;

    Motor_Forward(1500, 1500); // start moving forward
    Clock_Delay1ms(100);

    while (1) {
        Data = Reflectance_Read(2000);
        Position = Reflectance_Position(Data);

        if (Data != 0) { // black line detected
            Motor_Stop();
            AppUARTSend("Black line detected!", SEND);
            break;
        }
    }

    // Now perform a 90 turn to the left
    AppUARTSend("Turning LEFT 90", SEND);
    AppTurnRight90();


 //   Clock_Delay1ms(750); // adjust this for your actual 90 turn
    Motor_Stop();

    AppUARTSend("Turn complete", REPLACE);
}


// My Line following implementation
void Line_Following(void) {
    uint8_t data = 0;
    uint8_t temp = 0;
    int Position, lastPosition = 0;

    AppUARTSend("Starting Line Following...", SEND);



    while (1) {
        // Read reflectance sensors
        data = Reflectance_Read(2000);
        temp = data;
        int Position = Reflectance_Position(data);

        // Debug print
        for (int i = 0; i < 8; i++) {
            EUSCIA0_OutUDec(data % 2);
            EUSCIA0_OutString("-");
            data /= 2;
        }
        EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);


        // Line following control
        if (Position >= -47 && Position <= 47) {
            Motor_Forward(2000, 2000);
        }
        else if (Position > 47 && Position <= 238) {
            Motor_Forward(750, 3000);
        }
        else if (Position < -47 && Position >= -238) {
            Motor_Forward(3000, 750);
        }
        else if (Position > 238 && Position <= 334) {
            Motor_Right(2000, 2000);
        }
        else if (Position < -238 && Position >= -334) {
            Motor_Left(2000, 2000);
        }
        else if (Position > 334 || Position < -334) {
            // Instead of stopping immediately, try recovery
                       EUSCIA0_OutString("Off-line detected. Attempting recovery...\r\n");
                       if (Position > 0) {
                           Motor_Right(2000, 2000);
                       } else {
                           Motor_Left(2000, 2000);
                       }
                       Clock_Delay1ms(150);
                       continue;

        }

        // Parking condition
        if (temp == 0xFF) {
            EUSCIA0_OutString("PARKING\r\n");
            Motor_Stop();
            break;
        }

        Clock_Delay1ms(50); // small delay to prevent overload
    }

    EUSCIA0_OutString("LINE FOLLOWING DONE\r\n");
}

// Obstacle avoidance implementation
void Obstacle_Avoidance(void) {
    AppUARTSend("Starting OBSTACLE AVOIDANCE", SEND);
    isCollided = 0;

    uint32_t raw17,raw12,raw16;
           int32_t n; uint32_t s;
           ADCflag = 0;
           s = 256; // replace with your choice
           ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16
           ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
           LPF_Init(raw17,s);     // P9.0/channel 17
           LPF_Init2(raw12,s);     // P4.1/channel 12
           LPF_Init3(raw16,s);     // P9.1/channel 16
             //UART0_Init();          // initialize UART0 115,200 baud rate
            LaunchPad_Init();
           TimerA1_Init(&handlerAdcRead, 250);    // 2000 Hz sampling
           //UART0_OutString("GP2Y0A21YK0F test\nValvano Oct 2017\nConnect analog signals to P9.0,P4.1,P9.1\n");
           //EnableInterrupts();
           //int32_t c = CenterConvert(nc);

           isCollided = 0;
           while(!isCollided){
             for(n=0; n<2000; n++){
               while(ADCflag == 0){};
               ADCflag = 0; // show every 2000th point
             }

             UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" mm\r\n");


             if (CenterConvert(nc) < 150) {  // obstacle ahead
                         AppUARTSend("Obstacle detected! Turning right", REPLACE);
                         Motor_Stop();
                        // Clock_Delay1ms(700);

                         // Detour
                         Motor_Right(3000, 3000);
                         Clock_Delay1ms(750);  // ~90� right turn
                         Motor_Forward(2000, 2000);
                         Clock_Delay1ms(1500);

                         // Resume
                         AppUARTSend("Resuming path", REPLACE);
                         Motor_Left(3000, 3000);
                         Clock_Delay1ms(750);  // return to original direction
                     }
                     else {
                         Motor_Forward(1500, 1500);
                     }

                     if (isCollided) break;
                     Clock_Delay1ms(50);
 }

    Motor_Stop();
    AppUARTSend("OBSTACLE AVOIDANCE COMPLETE", SEND);
}



//  Binary, Decimal, Hex converter for bump switches
void AppBumpBinaryConverter(void) {
    AppUARTSend("Press any bump switches to see Binary, Decimal, Hex", SEND);

    uint8_t prevData = 0xFF;
    while (1) {
        uint8_t data = Bump_Read();   // active-low
        data = (~data) & 0x3F;        // mask to 6 bits

        if (data != prevData) {
            prevData = data;

            uint32_t decimal = 0;
            for (int i = 0; i < 6; i++)
                if (data & (1 << i))
                    decimal += (1 << i);

            // manual hex conversion
            char hex[3];
            uint8_t hi = (decimal >> 4) & 0xF;
            uint8_t lo = decimal & 0xF;
            hex[0] = (hi < 10) ? ('0' + hi) : ('A' + hi - 10);
            hex[1] = (lo < 10) ? ('0' + lo) : ('A' + lo - 10);
            hex[2] = '\0';

            char buf[80];
            sprintf(buf, "Binary: %s | Decimal: %u | Hex: 0x%s", conv_int2byte(data), decimal, hex);
            AppUARTSend(buf, REPLACE);
        }
        Clock_Delay1ms(100);
    }
}

void AppSwitchControlLED(void)
{
    AppUARTSend("Switch Control Demo: Connect switches to P1.1 and P1.4", SEND);
    AppUARTSend("Press any bumper to exit", SEND);


    P1->DIR &= ~0x12;
    P1->REN |= 0x12;
    P1->OUT |= 0x12;


    isCollided = 0;
    while (!isCollided) {

        uint8_t sw1_pressed = (P1->IN & 0x02) == 0;
        uint8_t sw2_pressed = (P1->IN & 0x10) == 0;


        if (sw1_pressed && !sw2_pressed) {

            Motor_Forward(3000, 3000);
            LaunchPad_Output(0x01); // red
            Clock_Delay1ms(1000);

        }
      //  else if (sw1_pressed && !sw2_pressed) {

     //       Motor_Left(2500, 2500);

     //       LaunchPad_Output(0x02); // green
    //    }
        else if (!sw1_pressed && sw2_pressed) {

            Motor_Backward(3000, 3000);
            LaunchPad_Output(0x04); // blue
           // Motor_Stop();
            Clock_Delay1ms(1000);
        }
        else {

            Motor_Stop();
            LED_Off();
        }

        Clock_Delay1ms(100);
    }


    Motor_Stop();
    LED_Off();

}


// AppBumper_Recovery
// Robot drives forward until a bumper is pressed, then executes a recovery maneuver.


void AppBumper_Recovery(void)
{
    AppUARTSend("Bumper Recovery Mode: Driving forward until collision", SEND);
    AppUARTSend("After hit: 90-degree left turn + short move + stop", SEND);

    // Ensure we start fresh
    isCollided = 0;
    Motor_Stop();
    Clock_Delay1ms(200);

    // Drive forward until bumper is hit
    Motor_Forward(2000, 2000);
    LaunchPad_Output(0x02); // green

    while (!isCollided) {
        Clock_Delay1ms(10);
    }

    // --- Collision detected! ---
    AppUARTSend("Bumper pressed! Executing recovery...", SEND);
    LaunchPad_Output(0x01);

    // Stop motors
    Motor_Stop();
    Clock_Delay1ms(200);


    // === RECOVERY MANEUVER ===
        // Option 1: 90-degree LEFT turn
    const uint32_t TURN_DELAY_MS = 750;

    // Option 2: 180-degree turn (uncomment below if preferred)
    // const uint32_t TURN_DELAY_MS = 1500;

    Motor_Backward(3000, 3000);
    Clock_Delay1ms(1000);
    AppTurnLeft90();
    // Move forward to clear obstacle
    Motor_Forward(2500, 2500);
    Clock_Delay1ms(1000);

    // Final stop
    Motor_Stop();
    LED_Off();

    AppUARTSend("Recovery complete. Robot stopped.", SEND);
}

void AppBumper_Recoveryy(void)
{
    AppUARTSend("Bumper Recovery Mode: Driving forward until collision", SEND);
    AppUARTSend("After hit: 90-degree left turn + short move + stop", SEND);

    // Ensure we start fresh
    isCollided = 0;
    Motor_Stop();
    Clock_Delay1ms(200);


    uint32_t raw17,raw12,raw16;
              int32_t n; uint32_t s;
              ADCflag = 0;
              s = 256; // replace with your choice
              ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16
              ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
              LPF_Init(raw17,s);     // P9.0/channel 17
              LPF_Init2(raw12,s);     // P4.1/channel 12
              LPF_Init3(raw16,s);     // P9.1/channel 16
                //UART0_Init();          // initialize UART0 115,200 baud rate
               LaunchPad_Init();
              TimerA1_Init(&handlerAdcRead, 250);    // 2000 Hz sampling
              //UART0_OutString("GP2Y0A21YK0F test\nValvano Oct 2017\nConnect analog signals to P9.0,P4.1,P9.1\n");
              //EnableInterrupts();
              //int32_t c = CenterConvert(nc);

              isCollided = 0;
              while(!isCollided){
                for(n=0; n<2000; n++){
                  while(ADCflag == 0){};
                  ADCflag = 0; // show every 2000th point
                }

                UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" mm\r\n");

                // Drive forward until bumper is hit
                    Motor_Forward(2000, 2000);
                    LaunchPad_Output(0x02); // green

                    while (!isCollided) {
                        Clock_Delay1ms(10);
                    }

                    // --- Collision detected! ---
                    AppUARTSend("Bumper pressed! Executing recovery...", SEND);
                    LaunchPad_Output(0x01);

                    // Stop motors
                    Motor_Stop();
                    Clock_Delay1ms(200);

                if ( CenterConvert(nc) < 100 ){
                    // Blink LED or something
                    LaunchPad_Output(0x01);       // Red
                    Clock_Delay1ms(100);

                    // === RECOVERY MANEUVER ===
                    Motor_Backward(3000, 3000);
                    Clock_Delay1ms(1000);


                    // Final stop
                    Motor_Stop();
                    LED_Off();
                } else if(LeftConvert(nl) < 100){
                    LaunchPad_Output(0x06);       // cyan
                    Clock_Delay1ms(100);
                    // === RECOVERY MANEUVER ===
                    Motor_Backward(3000, 3000);
                    Clock_Delay1ms(1000);
                    AppTurnLeft90();
                    // Move forward to clear obstacle
                    Motor_Forward(2500, 2500);
                    Clock_Delay1ms(1000);
                }else if(RightConvert(nr) < 100){
                    LaunchPad_Output(0x05);       // purple
                    Clock_Delay1ms(100);
                    // === RECOVERY MANEUVER ===
                                       Motor_Backward(3000, 3000);
                                       Clock_Delay1ms(500);
                                       AppTurnRight90();
                                       // Move forward to clear obstacle
                                       Motor_Forward(2500, 2500);
                                       Clock_Delay1ms(1000);
                }else {
                    LaunchPad_Output(0x04);       // cyan
                    Clock_Delay1ms(100);
                }
    }



    AppUARTSend("Recovery complete. Robot stopped.", SEND);
}

// AppLED_IR_Distance_Indicator
// Uses center IR sensor to indicate proximity via LED color.
// - RED:    < 10 cm (very close)
// - cyan: 10�20 cm
// - purple:  20�30 cm
// - BLUE:   > 30 cm (far)
void AppIR_Distance_Warning(void)
{
    AppUARTSend("IR Distance LED Indicator (center sensor)", SEND);
    uint32_t raw17,raw12,raw16;
          int32_t n; uint32_t s;
          ADCflag = 0;
          s = 256; // replace with your choice
          ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16
          ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
          LPF_Init(raw17,s);     // P9.0/channel 17
          LPF_Init2(raw12,s);     // P4.1/channel 12
          LPF_Init3(raw16,s);     // P9.1/channel 16
            //UART0_Init();          // initialize UART0 115,200 baud rate
           LaunchPad_Init();
          TimerA1_Init(&handlerAdcRead, 250);    // 2000 Hz sampling
          //UART0_OutString("GP2Y0A21YK0F test\nValvano Oct 2017\nConnect analog signals to P9.0,P4.1,P9.1\n");
          //EnableInterrupts();
          //int32_t c = CenterConvert(nc);

          isCollided = 0;
          while(!isCollided){
            for(n=0; n<2000; n++){
              while(ADCflag == 0){};
              ADCflag = 0; // show every 2000th point
            }

            UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" mm\r\n");

            if ( CenterConvert(nc) < 100 ){
                // Blink LED or something
                LaunchPad_Output(0x01);       // Red
                Clock_Delay1ms(100);
            } else if(CenterConvert(nc) < 200){
                LaunchPad_Output(0x06);       // cyan
                Clock_Delay1ms(100);
            }else if(CenterConvert(nc) < 300){
                LaunchPad_Output(0x05);       // purple
                Clock_Delay1ms(100);
            }else {
                LaunchPad_Output(0x04);       // cyan
                Clock_Delay1ms(100);
            }
}
          AppUARTSend("IR LED Indicator stopped (bumper pressed)", SEND);
}

void AppForward_With_Safety_Stop(void)
{
  //  while(LaunchPad_Input()==0);  // wait for touch
    //while(LaunchPad_Input());     // wait for release
   // TimedPause(1000);

    // IR init /////////////////////////////////////////


          uint32_t raw17,raw12,raw16;
          int32_t n; uint32_t s;
          ADCflag = 0;
          s = 256; // replace with your choice
          ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16
          ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
          LPF_Init(raw17,s);     // P9.0/channel 17
          LPF_Init2(raw12,s);     // P4.1/channel 12
          LPF_Init3(raw16,s);     // P9.1/channel 16
            //UART0_Init();          // initialize UART0 115,200 baud rate
           LaunchPad_Init();
          TimerA1_Init(&handlerAdcRead, 250);    // 2000 Hz sampling
          //UART0_OutString("GP2Y0A21YK0F test\nValvano Oct 2017\nConnect analog signals to P9.0,P4.1,P9.1\n");
          //EnableInterrupts();
          while(1){
            for(n=0; n<2000; n++){
              while(ADCflag == 0){};
              ADCflag = 0; // show every 2000th point
            }

            Motor_Forward(1000,1000);
            TimedPause(1000);

            UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" mm\r\n");

            if ( CenterConvert(nc) < 150 ){
                Motor_Forward(2500,2500);
                TimedPause(100);
                Motor_Stop();
                return;

            }
          }

    }


void AppSequenced_Wall_Follow(void)
{

    // place wall perpendicular to wall
    // turn left & check that right sensor is around 15cm away
    // motor forward
    // after set distance 30cm, turn right & adjust distance from wall

    // after 100cm, stop only if theres obstacle

    // IR Init

 //   while(LaunchPad_Input()==0);  // wait for touch
 //   TimedPause(800);

     while(LaunchPad_Input()==0);  // wait for touch
       while(LaunchPad_Input());     // wait for release
        TimedPause(1000);



     uint32_t raw17,raw12,raw16;
     int32_t n; uint32_t s;
     ADCflag = 0;
     s = 256; // replace with your choice
     ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16
     ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
     LPF_Init(raw17,s);     // P9.0/channel 17
     LPF_Init2(raw12,s);     // P4.1/channel 12
     LPF_Init3(raw16,s);     // P9.1/channel 16
       //UART0_Init();          // initialize UART0 115,200 baud rate
      LaunchPad_Init();
     TimerA1_Init(&handlerAdcRead, 250);    // 2000 Hz sampling
     //UART0_OutString("GP2Y0A21YK0F test\nValvano Oct 2017\nConnect analog signals to P9.0,P4.1,P9.1\n");
     //EnableInterrupts();

     bool firstStart = true;
     int32_t turnCounter = 0;
     bool firstTurnLeft = false;


     while(1){
       for(n=0; n<2000; n++){
         while(ADCflag == 0){};
         ADCflag = 0; // show every 2000th point
       }
       // STARTING , ADJUST DISTANCE AND TURN LEFT
         while ( CenterConvert(nc) > 150 && firstStart)
         {
             // ADJUST DONE
             if ( CenterConvert(nc) < 130 )
             {
                 firstStart = false;
             }
             Motor_Forward(1000,1000);
             TimedPause(1000);
         }
         // IF INITIAL DIST IS TOO NEAR
         while (CenterConvert(nc) < 80 && firstStart)
         {
             firstStart = false;
             Motor_Backward(800,800);
             TimedPause(1000);

         }
         // TURN LEFT
         if (!firstTurnLeft){
         Motor_Left(1400,1400);
         TimedPause(800);
         firstTurnLeft = true;
         }



         while (turnCounter < 5){



         // MOVE FORWARD
         Motor_Forward(1500,1500);
         TimedPause(1000);

         // TURN RIGHT & ADJUST DISTANCE

         Motor_Right(1400,1400);
         TimedPause(800);
         while ( CenterConvert(nc) > 60 )
         {

              // ADJUST DONE
              if ( CenterConvert(nc) < 100 )
              {
                  break;
              }
              Motor_Forward(800,800);
              TimedPause(500);
         }
          // IF DIST IS TOO NEAR
          while (CenterConvert(nc) < 80 )
          {
              Motor_Backward(800,800);
              TimedPause(1000);

          }
         turnCounter++;
         // RETURN TO PATH AND CONTINUE
         Motor_Left(1400,1400);
         TimedPause(800);


         }                                              // end of turnCounter while loop

         while (CenterConvert(nc) > 150)
         {
             Motor_Forward(1000,1000);
             TimedPause(800);
                     Motor_Stop();



         } // end of continue after 100cm while loop

         LaunchPad_Output(0x04);       // B
         Clock_Delay1ms(1000);
         return;
} // end of while loop
   //  Motor_Stop();
} // end of function


// Cup Navigation Function
// Robot spins to detect 4 cups using IR sensors, then navigates to closest and second closest
void AppCupNavigation(void)
{
    AppUARTSend("Starting Cup Navigation", SEND);

    // Variables to store cup distances
    uint32_t cup_distances[4];
    uint32_t min_distance1 = 0xFFFFFFFF, min_distance2 = 0xFFFFFFFF;
    uint8_t closest_cup = 0, second_closest_cup = 0;
//    uint32_t min_distance1 = 0xFFFFFFFF, max_distance = 0;  // Change min_distance2 to max_distance
//    uint8_t closest_cup = 0, farthest_cup = 0;  // Change second_closest_cup to farthest_cup

    // Initialize ADC for IR sensors
    uint32_t raw17, raw12, raw16;
    uint32_t s = 256;
    ADCflag = 0;

    ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16
    ADC_In17_12_16(&raw17, &raw12, &raw16);  // sample
    LPF_Init(raw17, s);     // P9.0/channel 17
    LPF_Init2(raw12, s);     // P4.1/channel 12
    LPF_Init3(raw16, s);     // P9.1/channel 16
    TimerA1_Init(&handlerAdcRead, 250);    // 2000 Hz sampling
    EnableInterrupts();

    // Spin robot to detect 4 cups
    AppUARTSend("Spinning to detect cups...", SEND);
    Motor_Right(3000, 0);

    uint8_t cup_count = 0;
    uint32_t last_detection_time = 0;
    uint32_t current_time = 0;

    // Detect 4 cups while spinning
    while (cup_count < 4) {
        // Wait for ADC data
        while (ADCflag == 0);
        ADCflag = 0;

        // Check if we detect a cup (sudden change in distance readings)
        // For simplicity, we'll assume a cup is detected when all sensors show a significant distance
        uint32_t avg_distance = (LeftConvert(nl) + CenterConvert(nc) + RightConvert(nr)) / 3;

        current_time++;
        // Check if enough time has passed since last detection to avoid duplicate detections
        if (current_time - last_detection_time > 50) {
            // Simple cup detection: if average distance is significantly large
            if (avg_distance > 125) { // Adjust threshold as needed
                cup_distances[cup_count] = avg_distance;
                char buf[50];
                sprintf(buf, "Cup %d detected at distance %d", cup_count+1, avg_distance);
                AppUARTSend(buf, REPLACE);
                cup_count++;
                last_detection_time = current_time;
            }
        }

        // Safety break after spinning for too long
        if (current_time > 1000) {
            break;
        }
    }

    Motor_Stop();

    // Find closest and second closest cups
    for (uint8_t i = 0; i < 4 && i < cup_count; i++) {
        if (cup_distances[i] < min_distance1) {
            min_distance2 = min_distance1;
            second_closest_cup = closest_cup;
            min_distance1 = cup_distances[i];
            closest_cup = i;
        } else if (cup_distances[i] < min_distance2) {
            min_distance2 = cup_distances[i];
            second_closest_cup = i;
        }
    }

    /* // Find closest and farthest cups
    for (uint8_t i = 0; i < 4 && i < cup_count; i++) {
        if (cup_distances[i] < min_distance1) {
        min_distance1 = cup_distances[i];
        closest_cup = i;
        }
    // Remove the else if for second closest and add condition for farthest
        if (cup_distances[i] > max_distance) {
        max_distance = cup_distances[i];
        farthest_cup = i;
        }
    } */

    char buf[100];
    sprintf(buf, "Closest cup: %d at distance %d", closest_cup+1, min_distance1);
    AppUARTSend(buf, SEND);
    sprintf(buf, "Second closest cup: %d at distance %d", second_closest_cup+1, min_distance2);
    AppUARTSend(buf, SEND);

    // Navigate to closest cup
    AppUARTSend("Navigating to closest cup...", SEND);
    // Move forward until close to cup
    Motor_Forward(1500, 1500);
    while (CenterConvert(nc) > 100) { // Stop when close to cup (adjust distance as needed)
        while (ADCflag == 0);
        ADCflag = 0;
    }
    Motor_Stop();
    AppUARTSend("Reached closest cup", SEND);

    // Navigate to second closest cup
    AppUARTSend("Navigating to second closest cup...", SEND);
    // Turn towards second cup (simplified - in practice would need more precise turning)
    Motor_Left(1500, 1500);
    Clock_Delay1ms(1000); // Adjust timing for 90-degree turn
    Motor_Stop();

    // Move forward until close to second cup
    Motor_Forward(1500, 1500);
    while (nc > 100) { // Stop when close to cup (adjust distance as needed)
        while (ADCflag == 0);
        ADCflag = 0;
    }
    Motor_Stop();
    AppUARTSend("Reached second closest cup", SEND);
    AppUARTSend("Cup Navigation Complete", SEND);
}


void Flexible_Line_Following(void) {
    uint8_t Data;
    int32_t Position;
    int lastPosition = 0;
    int smoothedPosition = 0;
    uint8_t all_black_count = 0;
    static uint8_t behavior_counter = 0;  // Tracks which behavior to execute next

    // Proportional control constants for smooth movement
    const int Kp = 8;  // Proportional gain - adjust as needed
    const int baseSpeed = 2000;
    int leftSpeed, rightSpeed;

    AppUARTSend("Starting Flexible Line Following", SEND);

    while (1) {
        Data = Reflectance_Read(2000);
        Position = Reflectance_Position(Data);

        // Check if all sensors detect black (end of line)
        if (Data == 0xFF) {
            all_black_count++;
            if (all_black_count > 5) { // Confirm it's not a glitch
                AppUARTSend("All black detected - Executing next behavior", SEND);
                Motor_Stop();
                Clock_Delay1ms(500);

                // Execute different behaviors based on the counter
                // You can modify these cases later according to your needs
                switch (behavior_counter % 3) {  // 3 different behaviors, you can add more
                    case 0:
                        AppUARTSend("Behavior 0: Turning right 90 degrees", SEND);
                        AppTurnRight90();  // Call existing function
                        break;
                    case 1:
                        AppUARTSend("Behavior 1: Turning left 90 degrees", SEND);
                        AppTurnLeft90();   // Call existing function
                        break;
                    case 2:
                        AppUARTSend("Behavior 2: Moving forward", SEND);
                        Motor_Forward(1500, 1500);
                        Clock_Delay1ms(1000);
                        Motor_Stop();
                        break;
                }

                behavior_counter++;  // Increment for next time
                all_black_count = 0;

                // Continue line following after behavior
                AppUARTSend("Resuming line following", SEND);
                continue;
            }
        } else {
            all_black_count = 0; // Reset counter if not all black
        }

        // Smooth line following behavior using proportional control
        // Simple moving average to smooth position readings
        smoothedPosition = (Position + lastPosition) / 2;
        lastPosition = Position;

        // Proportional control - adjust motor speeds based on position
        // The further from center, the more we turn
        int turnAdjustment = Kp * smoothedPosition / 100;  // Scale down for smoother control

        leftSpeed = baseSpeed + turnAdjustment;
        rightSpeed = baseSpeed - turnAdjustment;

        // Ensure speeds stay within valid range
        if (leftSpeed > 3000) leftSpeed = 3000;
        if (leftSpeed < 500) leftSpeed = 500;
        if (rightSpeed > 3000) rightSpeed = 3000;
        if (rightSpeed < 500) rightSpeed = 500;

        // Apply motor speeds for smooth movement
        Motor_Forward(leftSpeed, rightSpeed);

        // LED feedback
        if (Data != 0x00)
            LaunchPad_Output(0x02); // GREEN (P2.1) when line detected
        else
            LaunchPad_Output(0x01); // RED (P2.0) when no line

        Clock_Delay1ms(20); // Reduced delay for more responsive control
    }
}


// Advanced Line Following Function
// Follows black line, handles turns, detects end of line and continues
void AppAdvancedLineFollowing(void)
{
    AppUARTSend("Starting Advanced Line Following", SEND);

    uint8_t Data;
    int32_t Position;
    uint8_t all_black_count = 0;
    uint8_t search_mode = 0;

    while (1) {
        Data = Reflectance_Read(2000);
        Position = Reflectance_Position(Data);

        // Check if all sensors detect black (end of line)
        if (Data == 0xFF) {
            all_black_count++;
            if (all_black_count > 5) { // Confirm it's not a glitch
                AppUARTSend("All black detected - End of line", SEND);

                // Take a right turn
                Motor_Stop();
                Clock_Delay1ms(500);
                AppUARTSend("Turning right", SEND);
                Motor_Right(2000, 2000);
                Clock_Delay1ms(750); // Adjust for 90-degree turn

                // Move forward a bit
                Motor_Forward(1500, 1500);
                Clock_Delay1ms(1000);

                // Enter search mode for black line
                AppUARTSend("Searching for black line", SEND);
                search_mode = 1;
                all_black_count = 0;
            }
        } else {
            all_black_count = 0; // Reset counter if not all black
        }

        if (search_mode) {
            // Search for black line by spiraling outward
            static uint8_t search_step = 0;

            if (Data != 0x00) { // Found black line again
                AppUARTSend("Black line found - Resuming follow", SEND);
                search_mode = 0;
                search_step = 0;
            } else {
                // Continue search pattern
                switch (search_step % 4) {
                    case 0: // Forward
                        Motor_Forward(1000, 1000);
                        Clock_Delay1ms(500);
                        break;
                    case 1: // Right turn
                        Motor_Right(1500, 1500);
                        Clock_Delay1ms(300);
                        break;
                    case 2: // Forward
                        Motor_Forward(1000, 1000);
                        Clock_Delay1ms(500);
                        break;
                    case 3: // Left turn
                        Motor_Left(1500, 1500);
                        Clock_Delay1ms(300);
                        break;
                }
                search_step++;

                // Prevent infinite search
                if (search_step > 20) {
                    AppUARTSend("Line not found - Stopping", SEND);
                    Motor_Stop();
                    return;
                }
            }
        } else {
            // Normal line following
            if (Position >= -47 && Position <= 47) {
                // Centered on line
                Motor_Forward(2000, 2000);
                LaunchPad_Output(0x02); // Green LED
            }
            else if (Position > 47 && Position <= 238) {
                // Need to turn right
                Motor_Forward(750, 3000);
                LaunchPad_Output(0x06); // Cyan LED
            }
            else if (Position < -47 && Position >= -238) {
                // Need to turn left
                Motor_Forward(3000, 750);
                LaunchPad_Output(0x05); // Purple LED
            }
            else if (Position > 238 || Position < -238) {
                // Sharp turn needed
                if (Position > 0) {
                    Motor_Right(2000, 2000);
                } else {
                    Motor_Left(2000, 2000);
                }
                LaunchPad_Output(0x01); // Red LED
            }
            else {
                // No line detected
                Motor_Forward(1500, 1500);
                LaunchPad_Output(0x04); // Blue LED
            }
        }

        Clock_Delay1ms(50); // Small delay
    }
}


// Bump Sensor Counter Function
// Counts how many times each bump sensor is pressed and displays the counts
void Bump_Sensor_Counter(void) {
    uint8_t bump_data;
    uint8_t prev_bump_data = 0xFF;  // All bump sensors are active-low
    uint8_t press_count[6] = {0};   // Count for each of the 6 bump sensors
    char buf[50];

    AppUARTSend("Starting Bump Sensor Counter", SEND);
    AppUARTSend("Press any bump sensor to count presses", SEND);
    AppUARTSend("Press all sensors simultaneously to exit", SEND);

    while (1) {
        bump_data = Bump_Read();

        // Check each bump sensor for press/release transitions
        for (uint8_t i = 0; i < 6; i++) {
            // Check if sensor i is pressed (active-low) and was not pressed before
            if ((((bump_data >> i) & 0x01) == 0) && (((prev_bump_data >> i) & 0x01) == 1)) {
                press_count[i]++;
                sprintf(buf, "Bump %d pressed %d times", i, press_count[i]);
                AppUARTSend(buf, REPLACE);

                // Visual feedback with LED
                LaunchPad_Output(0x02); // Green LED
                Clock_Delay1ms(100);
                LaunchPad_Output(0x00); // LED off
            }
        }

        prev_bump_data = bump_data;

        // Exit condition: all bump sensors pressed simultaneously
        if ((bump_data & 0x3F) == 0x00) {
            AppUARTSend("All bump sensors pressed - Exiting", SEND);
            break;
        }

        Clock_Delay1ms(50); // Small delay to prevent excessive polling
    }

    // Display final counts
    AppUARTSend("Final Bump Sensor Press Counts:", SEND);
    for (uint8_t i = 0; i < 6; i++) {
        sprintf(buf, "Bump %d: %d presses", i, press_count[i]);
        AppUARTSend(buf, SEND);
    }
}
/*
 * This function runs a simple FSM which contain 3 states: left, right, center. The motor direction is determined by the state.
 * The motor speed is determined by the IR sensor readings; under some threshold, i.e. 20cm, it will slow down slightly,
 * if no line is detected, the function will reverse the robot back onto the line
 * the fsm state is indicated by the led color through the fp passed to fsm instance, and
 * this function terminates on bump switch collision.
 */
void AppLineFollowerTest(void)
{
    AppUARTSend("Starting FSM LINE FOLLOWER", SEND);
    isCollided = 0;

    fsm_t fsm;
    fsm.state = STATE_OK;
    fsm.fp_led = fsm_led;
    uint8_t Data;
    int32_t Position;

    while (!isCollided) {
        Data = Reflectance_Read(2000);
        Position = Reflectance_Position(Data);





        // FSM logic
        switch (fsm.state) {
        case STATE_OK:
            Motor_Forward(1500, 1500);
            if (Position > 100) fsm.state = STATE_L;
            else if (Position < -100) fsm.state = STATE_R;
            break;

        case STATE_L:
            Motor_Forward(800, 1500);
            if (Position <= 100) fsm.state = STATE_OK;
            break;

        case STATE_R:
            Motor_Forward(1500, 800);
            if (Position >= -100) fsm.state = STATE_OK;
            break;


        default:
            Motor_Stop();
            fsm.state = STATE_OK;
            break;
        }

        // LED indication for states
        fsm.fp_led(fsm.state);
        Clock_Delay1ms(30);
    }

    Motor_Stop();
    AppUARTSend("FSM LINE FOLLOWER STOPPED", SEND);
}

static void _proc_fsm(fsm_t* fsm, int32_t sig)
{

    fsm->fp_led(fsm->state);
    switch (fsm->state)
    {
        char buf[50];
#ifdef DEBUG
        sprintf(buf, "%d, %d", sig, (uint8_t)fsm->state);
#endif
        AppUARTSend(buf, SEND);
    case STATE_OK:
    {
        Motor_Forward(MOTOR_SPEED, MOTOR_SPEED);

        if (sig > 100)
        {
            fsm->state = STATE_L;
        }
        else if (sig < -100)
        {
            fsm->state = STATE_R;
        }
        break;

    }
    case STATE_L:
    {
        Motor_Forward(MOTOR_SPEED / 3, MOTOR_SPEED);
        if (sig <= 100)
        {
            fsm->state = STATE_OK;
        }
        break;

    }
    case STATE_R:
    {
        Motor_Forward(MOTOR_SPEED, MOTOR_SPEED / 3);
        if (sig >= -100)
        {
            fsm->state = STATE_OK;
        }
        break;

    }


    default:
    {
        fsm->state = STATE_ERR;
        break;
    }

    }

}

#if 0
//Sample program for using teh UART related functions.
int Program5_4(void)
{
    //int main(void){
        // demonstrates features of the EUSCIA0 driver
    char ch;
    char string[20];
    uint32_t n;
    DisableInterrupts();
    Clock_Init48MHz();// makes SMCLK=12 MHz
    EUSCIA0_Init();// initialize UART
    EnableInterrupts();
    EUSCIA0_OutString("\nLab 5 Test program for EUSCIA0 driver\n\rEUSCIA0_OutChar examples\n");
    for (ch = 'A'; ch <= 'Z'; ch = ch + 1)
    {     // print the uppercase alphabet
        EUSCIA0_OutChar(ch);
    }
    EUSCIA0_OutChar(LF);
    for (ch = 'a'; ch <= 'z'; ch = ch + 1)
    {     // print the lowercase alphabet
        EUSCIA0_OutChar(ch);
    }
    while (1)
    {
        EUSCIA0_OutString("\n\rInString: ");
        EUSCIA0_InString(string, 19); // user enters a string
        EUSCIA0_OutString(" OutString="); EUSCIA0_OutString(string); EUSCIA0_OutChar(LF);

        EUSCIA0_OutString("InUDec: "); n = EUSCIA0_InUDec();
        EUSCIA0_OutString(" OutUDec="); EUSCIA0_OutUDec(n); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString(" OutUFix1="); EUSCIA0_OutUFix1(n); EUSCIA0_OutChar(LF);
        EUSCIA0_OutString(" OutUFix2="); EUSCIA0_OutUFix2(n); EUSCIA0_OutChar(LF);

        EUSCIA0_OutString("InUHex: "); n = EUSCIA0_InUHex();
        EUSCIA0_OutString(" OutUHex="); EUSCIA0_OutUHex(n); EUSCIA0_OutChar(LF);
    }
}
#endif

//#############################################################################
// FILE:   LAB7_main.c
//
// TITLE:  Lab 7
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200
void setupSpib(void);
float readEncLeft(void);
float readEncRight(void);
void init_eQEPs(void);
void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);
// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// Lab7 NRW ADK
__interrupt void ADCA_ISR(void);

//*Ex1 NRW ADK - Setting up the SPIB Interrupt
__interrupt void SPIB_isr(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numTimerSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
uint32_t SPIBint = 0;

// *Ex1 NRW ADK - Global variables set (SPIVALUE)
int16_t spivalue1 = 0;
int16_t spivalue2 = 0;

float PWM1 = 0;
float PWM2 = 0;
uint16_t Flip1 = 0;
uint16_t Flip2 = 0;

// *Ex2 NRW ADK - defining the adc values & converted values
float adc1 = 0;
float adc2 = 0;
float ADC1 = 0;
float ADC2 = 0;

//LAB7
uint16_t adcd0result = 0;
uint16_t adcd1result = 0;
uint32_t adcd1count = 0;
float adcd0volt = 0;
float adcd1volt = 0;
float xkarray[22]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float ykadd =  0.0;
float xk2 = 0;
float xk3 = 0;
float yk2add = 0;
float yk3add = 0;
uint16_t adca2result = 0;
uint16_t adca3result = 0;
float xk2array[22]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
float xk3array[22]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

float barray[22]={   -2.3890045153263611e-03,
                     -3.3150057635348224e-03,
                     -4.6136191242627002e-03,
                     -4.1659855521681268e-03,
                     1.4477422497795286e-03,
                     1.5489414225159667e-02,
                     3.9247886844071371e-02,
                     7.0723964095458614e-02,
                     1.0453473887246176e-01,
                     1.3325672639406205e-01,
                     1.4978314227429904e-01,
                     1.4978314227429904e-01,
                     1.3325672639406205e-01,
                     1.0453473887246176e-01,
                     7.0723964095458614e-02,
                     3.9247886844071371e-02,
                     1.5489414225159667e-02,
                     1.4477422497795286e-03,
                     -4.1659855521681268e-03,
                     -4.6136191242627002e-03,
                     -3.3150057635348224e-03,
                     -2.3890045153263611e-03};


// *Ex3 NRW ADK - scaling the accel, gyro values
int16_t ax = 0;
int16_t ay = 0;
int16_t az = 0;
int16_t temp = 0;
int16_t gx = 0;
int16_t gy = 0;
int16_t gz = 0;

// *Ex3 NRW ADK - Conversion Var
float accelx = 0;
float accely = 0;
float accelz = 0;
float TEMP = 0;
float gyrox = 0;
float gyroy = 0;
float gyroz = 0;

float RightWheel = 0;
float LeftWheel = 0;

float radfoot = 5.166;
float Rightdist = 0;
float Leftdist = 0;

float controleff = 0;
float uLeft = 5.0;
float uRight = 5.0;

float PosLeft_K = 0;
float PosLeft_K_1 = 0;
float VLeftK = 0;

float PosRight_K = 0;
float PosRight_K_1 = 0;
float VRightK = 0;


//Controller Gains
float ki = 25;
float kp = 3;
float Vref = 1;
float errL_1 = 0;
float errR_1 = 0;
float errL = 0;
float errR = 0;
float LI_1 = 0;
float RI_1 = 0;
float LI = 0;
float RI = 0;
float Ltrap = 0;
float Rtrap = 0;

float KPturn = 3;
float eturn = 0;
float turn = 0;

//LAB7 Needed global Variables
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;
float accelzBalancePoint = -.68; // For ROBOT#9
int16 IMU_data[9];
int16_t doneCal = 0;
float tilt_value = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};

//LAB7 Kalman Filter vars
float T = 0.001; //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000;//50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t SpibNumCalls = -1;
float pred_P = 0;
float kalman_K = 0;
int32_t timecount = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;


/*
 WR = 0.56759; // (ft) Robot Width
    RWh = 1/radfoot; // (ft) Radius of the Wheel (OUR values)
    phiR = RWh/WR*(RightWheel-LeftWheel);
    thetaavg = 0.5 * (RightWheel-LeftWheel);

    AvRightK = (RightWheel - RightWheel_1)/0.004;
    AvLeftK = (LeftWheel - LeftWheel_1)/0.004;

    thetaavgD = 0.5 * (RightWheelD-LeftWheelD);

    xRD =   ;
    yRD =

 */
// Ex6 NRW ADK
float WR = 0;
float RWh = 0;
float phiR = 0;
float thetaavg = 0;
float thetaavgD = 0;
float RightWheel_1 = 0;
float LeftWheel_1 = 0;
float AvRightK = 0;
float AvLeftK = 0;
float xRD_1 = 0;
float yRD_1 = 0;
float xRD = 0;
float yRD = 0;
float xR_1 = 0;
float yR_1 = 0;
float xR = 0;
float yR = 0;

float velLeft = 0;
float velRight = 0;
float velLeft_1 = 0;
float velRight_1 = 0;
float gyro_value_1 = 0;
float gyro_valuedot = 0;
float gyro_valuedot_1 = 0;
float tilt_value_1 = 0;

float K1 = -60;
float K2 = -4.5;
float K3 = -1.1;
float K4 = -0.1;

float ubal = 0;

//ex4 NRW ADK
float WhlDiff = 0;
float WhlDiff_1 = 0;
float vel_WhlDiff = 0;
float vel_WhlDiff_1 = 0;

float turnref = 0;
float turnref_1 = 0;
float errorDiff = 0;
float errorDiff_1 = 0;
float intDiff = 0;
float intDiff_1 = 0;

float turnrate = 0;
float turnrate_1 = 0;

float Kp = 3.0;
float Ki = 20.0;
float Kd = 0.08;

//Ex5
float KpSpeed = 0.35;
float KiSpeed = 1.5;

float ForwardBackwardCommand = 0.0;
float Segbot_refSpeed = 0.0;
float eSpeed = 0.0;
float eSpeed_1 = 0.0;
float IK_eSpeed = 0.0;
float IK_eSpeed_1 = 0.0;

//Ex5 P2

float printLV3 = 0;
float printLV4 = 0;
float printLV5 = 0;
float printLV6 = 0;
float printLV7 = 0;
float printLV8 = 0;
float x = 0;
float y = 0;
float bearing = 0;
extern uint16_t NewLVData;
extern float fromLVvalues[LVNUM_TOFROM_FLOATS];
extern LVSendFloats_t DataToLabView;
extern char LVsenddata[LVNUM_TOFROM_FLOATS*4+2];
extern uint16_t newLinuxCommands;
extern float LinuxCommands[CMDNUM_FROM_FLOATS];

float angeff = 0;
uint16_t Flip = 0;
uint16_t AFlip = 0;


//State vars
int myStateVar = 10; // Initialize global State Variable to desired initial state
long timeint = 0;





void setDACA(float dacouta0)
{
    int16_t DACOutInt = 0;
    DACOutInt = dacouta0*4096.0/3.0; // perform scaling of 0 � almost 3V to 0 - 4095
    if (DACOutInt > 4095) DACOutInt = 4095;
    if (DACOutInt < 0) DACOutInt = 0;
    DacaRegs.DACVALS.bit.DACVALS = DACOutInt;
}

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    //############
    //## Pinmux ##
    //############

    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 1);

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    //*Ex1 NRW ADK -
    PieVectTable.SPIB_RX_INT = &SPIB_isr;
    // LAB7 NRW ADK
    PieVectTable.ADCA1_INT = &ADCA_ISR;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000); // 10000 -> 10ms
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 4000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);

    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // For EPWM8A
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; // For EPWM8B
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1; // For EPWM9A
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPWM12A
    EDIS;


    //#####################
    //## EPwm2 Registers ##
    //#####################

    //EPwm2
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBCTL.bit.FREE_SOFT= 3;
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.PHSEN = 0;
    EPwm2Regs.TBCTR = 0;
    EPwm2Regs.TBPRD = 2500; // NRW ADK
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.CMPB.bit.CMPB = 0;
    EPwm2Regs.AQCTLA.bit.ZRO = 2;
    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.AQCTLB.bit.ZRO = 2;
    EPwm2Regs.AQCTLB.bit.CBU = 1;
    EPwm2Regs.TBPHS.bit.TBPHS = 0;

    //#####################
    //## EPwm5 Registers ##
    //#####################

    // NRW ADK - Command the ADCD peripheral to sample ADCIND0 and ADCIND1 every 1ms
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (�pulse� is the same as �trigger�)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode

    //#####################
    //## EPwm8 Registers ##
    //#####################

    EPwm8Regs.TBCTL.bit.CLKDIV = 4;// NRW ADK - Setting to Clock divide by 16 (bin:100)
    EPwm8Regs.TBCTL.bit.FREE_SOFT= 3; // NRW ADK - Setting to Free Soft emulation mode to Free Run
    EPwm8Regs.TBCTL.bit.CTRMODE = 0; // NRW ADK - Setting to phase loading disabled
    EPwm8Regs.TBCTL.bit.PHSEN = 0; // NRW ADK - Setting to Starting the timer at zero
    EPwm8Regs.TBCTR = 0; // NRW ADK - Setting to Starting the timer at zero
    EPwm8Regs.TBPRD = 62500; // NRW ADK - Setting the period of the PWM signal to 50Hz, Which is calculated 50e6/50/16 = 62500
    // Note that the value initially obtained 50e6/50 = 100000 is out of range (65535) so that the number is divided by 16 (CLKDIV)
    EPwm8Regs.CMPA.bit.CMPA = 5000; // NRW ADK - Setting the initial start duty cycle to be at 8%, 62500*0.08= 5000
    EPwm8Regs.CMPB.bit.CMPB = 5000; // NRW ADK - Setting the initial start duty cycle to be at 8%, 62500*0.08= 5000
    EPwm8Regs.AQCTLA.bit.ZRO = 2; // NRW ADK - Setting pin to be set when TBCTR register is zero
    EPwm8Regs.AQCTLA.bit.CAU = 1; // NRW ADK - Setting signal pin be cleared when TBCTR reaches the value in CMPA
    EPwm8Regs.AQCTLB.bit.ZRO = 2; // NRW ADK - Setting pin to be set when TBCTR register is zero
    EPwm8Regs.AQCTLB.bit.CBU = 1; // NRW ADK - Setting signal pin be cleared when TBCTR reaches the value in CMPB
    EPwm8Regs.TBPHS.bit.TBPHS = 0; // NRW ADK - Setting the phase to zero

    //###################
    //## ADC Registers ##
    //###################

    // NRW ADK - Set up ADCD so that it uses 2 of its 16 SOCs (Start of Conversions)
    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB

    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0x2; //SOC0 will convert Channel you choose Does not have to be A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0xD;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0x3; //SOC1 will convert Channel you choose Does not have to be A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0xD;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0x1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    // NRW ADK - SOC1A&B what they mean.
    EDIS;

    setupSpib();


    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    // *Ex1 NRW ADK - Allocating interrupt number
    IER |=M_INT6; // SPIB_RX at group 6


    //
    // PIE GROUP
    //

    // *EX1 NRW ADK - Allocating PIE group
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1; // SPIB_RX at group 6 , column 3
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    //LAB07
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

//    init_serialSCIB(&SerialB,115200);
//    init_serialSCIC(&SerialC,115200);
//    init_serialSCID(&SerialD,115200);

    // Enable global Interrupts and higher priority real-time debug events
    init_eQEPs();

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 )
        {
            serial_printf(&SerialA,"Tilt:%.4f Gyro:%.4f LeftWheel:%.4f RightWheel:%.4f\r\n",tilt_value,gyro_value,LeftWheel,RightWheel);
            UARTPrint = 0;

        }
    }
}
// END OF MAIN **************************


void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}
float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (-raw*(2*PI/12000.0));
}
float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw*(2*PI/12000.0));
}

//
// ROBOT WHEEL CONTROL
//

void setEPWM2A(float controleffort)
{
    uint16_t output;

    if (controleffort < -10)
    {
        controleffort = -10;
        RI = RI_1;
    }
    if (controleffort > 10)
    {
        controleffort = 10;
        RI = RI_1;
    }

    output = (((controleffort) + 10.0)/20.0) * EPwm2Regs.TBPRD;

    EPwm2Regs.CMPA.bit.CMPA = output;
    return;
}


void setEPWM2B(float controleffort)
{
    uint16_t output;

    if (controleffort < -10)
    {
        controleffort = -10;
        LI = LI_1;
    }
    if (controleffort > 10)
    {
        controleffort = 10;
        LI = LI_1;
    }

    output = (((controleffort) + 10.0)/20.0) * EPwm2Regs.TBPRD;

    EPwm2Regs.CMPB.bit.CMPB = output;
    return;
}

//
// ROBOT MECHANISM CONTROL
//

void setEPWM8A(float angle)
{
    uint16_t angout;
    //NRW ADK - Saturating the float variable angle within the range of (-90,90)
    if (angle < -90)
    {
        angle = -90;
    }
    if (angle > 90)
    {
        angle = 90;
    }
    //NRW ADK - "Normalizing" the variable so that -90degrees is 4% DC, 0degrees is 8% DC and 90degrees is 12% DC
    angout = (((angle) + 90.0)/180.0) * EPwm8Regs.TBPRD*.08 + .04*EPwm8Regs.TBPRD;

    EPwm8Regs.CMPA.bit.CMPA = angout;

    return;
}

void setEPWM8B(float angle)
{
    uint16_t angout;
    //NRW ADK - Saturating the float variable angle within the range of (-90,90)
    if (angle < -90)
    {
        angle = -90;
    }
    if (angle > 90)
    {
        angle = 90;
    }
    //NRW ADK - "Normalizing" the variable so that -90degrees is 4% DC, 0degrees is 8% DC and 90degrees is 12% DC
    angout = (((angle) + 90.0)/180.0) * EPwm8Regs.TBPRD*.08 + .04*EPwm8Regs.TBPRD;

    EPwm8Regs.CMPB.bit.CMPB = angout;

    return;
}

// new functions: activate front arm
//                deactivate front arm
//                activate back arm
//                deactivate back arm
//                activate balancing
//                purposeful fall backward
//                purposeful fall forward
//                
//new variables:
// balance commman
// standing tipped command
// rover command
// 

void myPeriodicFunction(void) { // Most of the state machines we will be writing
// in this class will be inside a function that
// is called at a periodic rate. Like say every 1ms.

//increment a global long integer keeping track of a sample count for timing
timeint++;
etc.;
switch(myStateVar) { // state machine

case 10: // standing

if (Recieve balance command) {
activate arm
myStateVar = 15; //Tipped
//initialize any variables needed for state 20. Very important.
}
break;

case 15: //Tipped case

if (angleout = tipped angle) {
myStateVar = 20; // Balancing
activate balancing
Deactivate front arm
}

case 20;//balancing

activate balancing
if(go to tipped/standing) {
    Purposeful fall Forward
    myStateVar = 10/15;
} else if (go to rover) {
    purposedull fall Backward 
    myStateVar = 30;
}

case 30;//rover

if (recieve balance command) {
    extended arm 
    if (angle = balance angle){
    activate balancing
     myStateVar = 20;
    reset back arm 
    }

}

}
}


// *Ex3 NRW ADK
void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    // Step 1.
    /* cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in
    between each transfer to 0. Also don�t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65,
    66 which are also a part of the SPIB setup. */
    SpibRegs.SPICCR.bit.SPISWRESET = 0x0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 0x1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16-bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 0x1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0x0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHz. And this setting divides that base clock to create SCLK�s period //NRW ADK - Dividing Base Clock by 50 is needed -> 0x31
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 0x1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    // NRW ADK - "Pull out of reset"
    SpibRegs.SPIFFTX.bit.SPIFFENA = 0x1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 0x1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 0x1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 0x10; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
    SpibRegs.SPICCR.bit.SPISWRESET = 0x1; // Pull the SPI out of reset NRW ADK
    SpibRegs.SPIFFTX.bit.TXFIFO = 0x1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don�t think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL =0x10; //Interrupt Level to 16 words or more received into FIFO causes
    // interrupt. This is just the initial setting for the register. Will be changed below
    SpibRegs.SPICCR.bit.SPICHAR = 0xF;
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00;
    // *Ex1 NRW ADK
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB //CHECK IF HEX
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;
    //-----------------------------------------

    // Step 2.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending
    // 16-bit transfers, so two registers at a time after the first 16-bit transfer.
    // To address 00x13 write 0x00
    SpibRegs.SPITXBUF = 0x1300;
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    SpibRegs.SPITXBUF = 0x0000;
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    SpibRegs.SPITXBUF = 0x0000;
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    SpibRegs.SPITXBUF = 0x0013;
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    SpibRegs.SPITXBUF = 0x0200;
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    SpibRegs.SPITXBUF = 0x0806;
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00
    SpibRegs.SPITXBUF = 0x0000;
    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=7); // 1 address + 6 Register values
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    SpibRegs.SPIRXBUF;
    SpibRegs.SPIRXBUF;
    SpibRegs.SPIRXBUF;
    SpibRegs.SPIRXBUF;
    SpibRegs.SPIRXBUF;
    SpibRegs.SPIRXBUF;

    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending
    // 16-bit transfers, so two registers at a time after the first 16-bit transfer.
    // To address 00x13 write 0x00
    SpibRegs.SPITXBUF = 0x7600;
    // *EX3 NRW ADK - XA OFFSETs
    SpibRegs.SPITXBUF = 0x0000;
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    SpibRegs.SPITXBUF = 0x0B1E;
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    SpibRegs.SPITXBUF = 0x0DC1;

    while(SpibRegs.SPIFFRX.bit.RXFFST !=4); // 1 address + 6 Register values
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    SpibRegs.SPIRXBUF;
    SpibRegs.SPIRXBUF;
    SpibRegs.SPIRXBUF;

    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.



    // Step 3.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = 0x2300;
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = 0x408C;
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    SpibRegs.SPITXBUF = 0x0288;
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    SpibRegs.SPITXBUF = 0x0C0A;
    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    SpibRegs.SPIRXBUF;
    SpibRegs.SPIRXBUF;
    SpibRegs.SPIRXBUF;

    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.

    // Step 4.
    // perform a single 16-bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = 0x2A81;
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);

    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;



    //SpibRegs.SPITXBUF = (0x7700 | 0x00EB); // 0x7700
    SpibRegs.SPITXBUF = (0x7700 | 0x00E8); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    //SpibRegs.SPITXBUF = (0x7800 | 0x0012); // 0x7800
    SpibRegs.SPITXBUF = (0x7800 | 0x0070); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    //SpibRegs.SPITXBUF = (0x7A00 | 0x0011); // 0x7A00
    SpibRegs.SPITXBUF = (0x7A00 | 0x000E); // 0x7A00 YA OFFSET UPPER
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    //SpibRegs.SPITXBUF = (0x7B00 | 0x00FA); // 0x7B00
    SpibRegs.SPITXBUF = (0x7B00 | 0x002C); // 0x7B00 YA OFFSET Lower
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0026); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    //SpibRegs.SPITXBUF = (0x7E00 | 0x0050); // 0x7E00
    SpibRegs.SPITXBUF = (0x7E00 | 0x0038); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);
    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}


// *Ex1 NRW ADK - SPIB_ISR Function
__interrupt void SPIB_isr(void)
{

    // *Ex3 Setting up offset
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // *Ex3 NRW ADK - slave select high
    SpibRegs.SPIRXBUF; // *Ex3 NRW ADK assigning dummies in address + INT_STATUS
    ax = SpibRegs.SPIRXBUF;
    ay = SpibRegs.SPIRXBUF;
    az = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    gx = SpibRegs.SPIRXBUF;
    gy = SpibRegs.SPIRXBUF;
    gz = SpibRegs.SPIRXBUF;

    accelx = ax/32767.0*4.0;
    accely = ay/32767.0*4.0;
    accelz = az/32767.0*4.0;

    gyrox = gx/32767.0*250.0;
    gyroy = gy/32767.0*250.0;
    gyroz = gz/32767.0*250.0;

    //    LeftWheel = readEncLeft();
    //    RightWheel = readEncRight();

    //    setEPWM2A(uRight);
    //    setEPWM2B(-uLeft);

    //LAB7
    //Code to be copied into SPIB_ISR interrupt function after the IMU measurements have been collected.
    if(calibration_state == 0){
        calibration_count++;

        //
        if (calibration_count == 2000) {
            calibration_state = 1;
            calibration_count = 0;
        }
    } else if(calibration_state == 1){
        accelx_offset+=accelx;
        accely_offset+=accely;
        accelz_offset+=accelz;
        gyrox_offset+=gyrox;
        gyroy_offset+=gyroy;
        gyroz_offset+=gyroz;
        calibration_count++;
        // Taking average
        if (calibration_count == 2000) {
            calibration_state = 2;
            accelx_offset/=2000.0;
            accely_offset/=2000.0;
            accelz_offset/=2000.0;
            gyrox_offset/=2000.0;
            gyroy_offset/=2000.0;
            gyroz_offset/=2000.0;
            calibration_count = 0;
            doneCal = 1;
        }
    } else if(calibration_state == 2){
        accelx -=(accelx_offset);
        accely -=(accely_offset);
        accelz -=(accelz_offset-accelzBalancePoint);
        gyrox -= gyrox_offset;
        gyroy -= gyroy_offset;
        gyroz -= gyroz_offset;
        /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
        float tiltrate = (gyrox*PI)/180.0; // rad/s
        float pred_tilt, z, y, S;
        // Prediction Step
        pred_tilt = kalman_tilt + T*tiltrate;
        pred_P = kalman_P + Q;
        // Update Step
        z = -accelz; // Note the negative here due to the polarity of AccelZ
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P/S;
        kalman_tilt = pred_tilt + kalman_K*y;
        kalman_P = (1 - kalman_K)*pred_P;
        SpibNumCalls++;
        // Kalman Filter used
        tilt_array[SpibNumCalls] = kalman_tilt;
        gyro_array[SpibNumCalls] = tiltrate;
        LeftWheelArray[SpibNumCalls] = readEncLeft();
        RightWheelArray[SpibNumCalls] = readEncRight();
        if (SpibNumCalls >= 3) { // should never be greater than 3
            tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3])/4.0;
            gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0;
            LeftWheel=(LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
            RightWheel=(RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
            SpibNumCalls = -1;
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
        }
    }
    timecount++;
    if((timecount%200) == 0)
    {
        if(doneCal == 0) {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
        UARTPrint = 1; // Tell While loop to print
    }

    //###

    SPIBint++;
    if ((SPIBint % 200) == 0)
    {
        UARTPrint = 1;
    }
    // NRW ADK - The Offset only accounts for the upper 15 bits,
    // The actual offset can be determined by right-shifting the value by 1

    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt

}



// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void)
{
    // These three lines of code allow SWI_isr, to be i0nterrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts

    // Insert SWI ISR Code here.......

    //############################
    //### BALANCING THE SEGBOT ###
    //############################

    velLeft = 0.6*velLeft_1+100*(LeftWheel-LeftWheel_1);
    velRight = 0.6*velRight_1+100*(RightWheel-RightWheel_1);
    gyro_valuedot = 0.6*gyro_valuedot_1+100*(gyro_value-gyro_value_1);
    ubal = -K1*tilt_value - K2*gyro_value - K3*(velLeft+velRight)/2.0 - K4*gyro_valuedot;

    //#############################
    //### TURN CONTROL OF ROBOT ###
    //#############################

    WhlDiff = (LeftWheel - RightWheel);

    vel_WhlDiff = .333*vel_WhlDiff_1 + 166.667*(WhlDiff - WhlDiff_1);
    errorDiff = turnref - WhlDiff;
    intDiff = intDiff_1 + (errorDiff + errorDiff_1)*.004/2;
    turn = Kp*errorDiff + Ki*intDiff - Kd*vel_WhlDiff;
    turnref = turnref_1 + (turnrate + turnrate_1)/2*.004;
    eSpeed = (Segbot_refSpeed - (velLeft+velRight)/2);
    IK_eSpeed = IK_eSpeed + (eSpeed + eSpeed_1)/2 * 0.004;

    //Ex5
    ForwardBackwardCommand = KpSpeed*eSpeed+KiSpeed*IK_eSpeed;

    //no windup
    if (fabs(turn)>3)
        intDiff = intDiff_1;

    //saturation of turn
    if (turn > 4)
    {
        turn = 4;
    }
    if (turn < -4)
    {
        turn = -4;
    }

    if (fabs(ForwardBackwardCommand)>3)
        IK_eSpeed = IK_eSpeed_1;

    if (ForwardBackwardCommand > 4)
    {
        ForwardBackwardCommand = 4;
    }
    if (ForwardBackwardCommand <-4)
    {
        ForwardBackwardCommand = -4;
    }

    //Ex5
    uLeft = ubal/2 + turn - ForwardBackwardCommand;
    uRight = ubal/2 - turn - ForwardBackwardCommand;

    setEPWM2A(uRight);
    setEPWM2B(-uLeft);

    if (NewLVData == 1) {
        NewLVData = 0;
        Segbot_refSpeed = fromLVvalues[0];
        turnrate = fromLVvalues[1];
        printLV3 = fromLVvalues[2];
        printLV4 = fromLVvalues[3];
        printLV5 = fromLVvalues[4];
        printLV6 = fromLVvalues[5];
        printLV7 = fromLVvalues[6];
        printLV8 = fromLVvalues[7];
    }
    if((numSWIcalls%62) == 0) { // change to the counter variable of you selected 4ms. timer
        DataToLabView.floatData[0] = xR;
        DataToLabView.floatData[1] = yR;
        DataToLabView.floatData[2] = phiR;
        DataToLabView.floatData[3] = 2.0*((float)numTimerSWIcalls)*.001;
        DataToLabView.floatData[4] = 3.0*((float)numTimerSWIcalls)*.001;
        DataToLabView.floatData[5] = (float)numTimerSWIcalls;
        DataToLabView.floatData[6] = (float)numTimerSWIcalls*4.0;
        DataToLabView.floatData[7] = (float)numTimerSWIcalls*5.0;
        LVsenddata[0] = '*'; // header for LVdata
        LVsenddata[1] = '$';
        for (int i=0;i<LVNUM_TOFROM_FLOATS*4;i++) {
            if (i%2==0) {
                LVsenddata[i+2] = DataToLabView.rawData[i/2] & 0xFF;
            } else {
                LVsenddata[i+2] = (DataToLabView.rawData[i/2]>>8) & 0xFF;
            }
        }
        serial_sendSCID(&SerialD, LVsenddata, 4*LVNUM_TOFROM_FLOATS + 2);
    }

    numSWIcalls++;

    //LAB7 Saving past states
    LeftWheel_1 = LeftWheel;
    RightWheel_1 = RightWheel;
    gyro_value_1 = gyro_value;
    tilt_value_1 = tilt_value;
    velLeft_1 = velLeft;
    velRight_1 = velRight;
    turnrate_1 = turnrate;
    turnref_1 = turnref;

    //Ex4 LAB7 NRW ADK
    WhlDiff_1 = WhlDiff;
    vel_WhlDiff_1 = vel_WhlDiff;
    errorDiff_1 = errorDiff;
    intDiff_1 = intDiff;


    //Ex5
    eSpeed_1 = eSpeed;
    IK_eSpeed_1 = IK_eSpeed;

    DINT;
}


__interrupt void ADCA_ISR (void)
{
    adca2result = AdcaResultRegs.ADCRESULT0;
    adca3result = AdcaResultRegs.ADCRESULT1;
    xk2 = adca2result * 3.0 / 4096.0; // storing the voltages
    xk3 = adca3result * 3.0 / 4096.0;

    uint16_t adcd1add = 0;

    yk2add =  0.0;
    xk2array[0] = xk2;
    yk3add =  0.0;
    xk3array[0] = xk3;

    //dot product of arrays
    for (adcd1add=0;adcd1add<22;adcd1add++)
    {
        yk2add += barray[adcd1add]*xk2array[adcd1add];
        yk3add += barray[adcd1add]*xk3array[adcd1add];
    }


    // saving past states
    for (int i=21;i>0;i--)
    {
        xk2array[i] = xk2array[i - 1];
        xk3array[i] = xk3array[i - 1];
    }


    // Here write yk to DACA channel
    setDACA(yk2add);
  
    //LAB7
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPIFFRX.bit.RXFFIL = 8; // *Ex3 NRW ADK: 1 address + 3 gyro + 1 temp + 3 accelerometer
    // Issue the SPIB_RX_INT when two values are in the RX FIFO
    SpibRegs.SPITXBUF = 0xBA00; // First Value to send !!!Note that this is not DA *** since READ is 1, 1011 -> BA
    SpibRegs.SPITXBUF = 0x0000; //ACC_X
    SpibRegs.SPITXBUF = 0x0000; //ACC_Y
    SpibRegs.SPITXBUF = 0x0000; //ACC_Z
    SpibRegs.SPITXBUF = 0x0000; //TEMP
    SpibRegs.SPITXBUF = 0x0000; //gyroyRO_X
    SpibRegs.SPITXBUF = 0x0000; //gyroyRO_Y
    SpibRegs.SPITXBUF = 0x0000; //gyroyRO_Z

    //###

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    if ((numTimer0calls%25) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    if ((numTimer0calls%50) == 0) {
        // Blink LaunchPad Red LED
        //        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{


    //##################
    //##Dead Reckoning##
    //##################

    WR = 0.56759; // (ft) Robot Width
    RWh = 1/radfoot; // (ft) Radius of the Wheel (OUR values)
    phiR = RWh/WR*(RightWheel-LeftWheel);
    thetaavg = 0.5 * (RightWheel+LeftWheel);

    AvRightK = (RightWheel - RightWheel_1)/0.004;
    AvLeftK = (LeftWheel - LeftWheel_1)/0.004;

    thetaavgD = 0.5 * (AvRightK+AvLeftK);

    xRD = RWh*thetaavgD*cos(phiR);
    yRD = RWh*thetaavgD*sin(phiR);

    xR = xR_1+(xRD+xRD_1)/2*0.004;
    yR = yR_1+(yRD+yRD_1)/2*0.004;

}


// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 10) == 0) {
        UARTPrint = 1;
    }

    // NRW ADK - flipping the incrementation so that the interrupt function gradually increases the command
    // until 90 is reached and then switch gradually decreasing the command until the -90 is reached

    if (angeff <= -90.0)
    {
        AFlip = 0;
    }
    if (angeff >= 90.0)
    {
        AFlip = 1;
    }
    if (AFlip == 0)
    {
        angeff+=.15;

        setEPWM8A(angeff);
        setEPWM8B(angeff);
    }
    if (AFlip == 1)
    {
        angeff-=.15;
        setEPWM8A(angeff);
        setEPWM8B(angeff);
    }
} 
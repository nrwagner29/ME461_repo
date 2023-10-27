//#############################################################################
// FILE:   LABHW3_main.c
//
// TITLE:  HW3
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


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

void I2CB_Init(void);
int16_t WriteDAN777RCServo(uint16_t RC1, uint16_t RC2);
int16_t ReadDAN777ADC(uint16_t *ADC1, uint16_t *ADC2);

uint16_t ReadTwo16BitValuesFromBQ32000(uint16_t *Rvalue1,uint16_t *Rvalue2,uint16_t *Rvalue3,uint16_t *Rvalue4,uint16_t *Rvalue5,uint16_t *Rvalue6,uint16_t *Rvalue7);
uint16_t WriteTwo16BitValuesToBQ32000(uint16_t Rvalue1,uint16_t Rvalue2,uint16_t Rvalue3,uint16_t Rvalue4,uint16_t Rvalue5,uint16_t Rvalue6,uint16_t Rvalue7);
int16_t I2C_CheckIfTX(uint16_t timeout);
int16_t I2C_CheckIfRX(uint16_t timeout);
uint16_t ReadTwo16BitValuesFromDAN777(uint16_t *Rvalue1,uint16_t *Rvalue2);
uint16_t WriteTwo16BitValuesToDAN777(uint16_t Cmd16bit_1, uint16_t Cmd16bit_2);
uint16_t RunI2C = 0; // Flag variable to indicate when to run I2C commands
int16_t I2C_OK = 0;
int32_t num_WriteDAN777_Errors = 0;
int32_t num_WriteBQ32000_Errors = 0;
int32_t num_ReadDAN777_Errors = 0;
int32_t num_ReadBQ32000_Errors = 0;

int16_t AFlip = 0;
int16_t angeff = 0;

uint16_t DANRCS1 = 0;
uint16_t DANRCS2 = 0;
uint16_t DANADC1 = 0;
uint16_t DANADC2 = 0;

int16_t RStatus = 0;
int16_t BStatus = 0;
int16_t WStatus = 0;
int16_t BWStatus = 0;
int16_t Rsec = 0;
int16_t Rmin = 0;
int16_t Rcenthr = 0;
int16_t Rday = 0;
int16_t Rdate = 0;
int16_t Rmonth = 0;
int16_t Ryears = 0;
//Interface Still needs to be set so writing the date is entirely matching the registers
int16_t Wsec = 0;
int16_t Wmin = 0;
int16_t Wcenthr = 0;
int16_t Wday = 0;
int16_t Wdate = 0;
int16_t Wmonth = 0;
int16_t Wyears = 0;

int16_t secone = 0;
int16_t minone = 0;
int16_t hourone = 0;
int16_t day = 0;
int16_t dateone = 0;
int16_t monthone = 0;
int16_t yearone = 0;

int16_t secten = 0;
int16_t minten = 0;
int16_t hourten = 0;
int16_t cent = 0;
int16_t dateten = 0;
int16_t monthten = 0;
int16_t yearten = 0;
char dayname;

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
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 20000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    init_serialSCIB(&SerialB,115200);
    init_serialSCIC(&SerialC,115200);
    init_serialSCID(&SerialD,115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // I2c Function Calls NRW
    I2CB_Init();

    BWStatus = WriteTwo16BitValuesToBQ32000(Wsec,Wmin,Wcenthr,Wday,Wdate,Wmonth,Wyears);
    BStatus = ReadTwo16BitValuesFromBQ32000(Rsec,Rmin,Rcenthr,Rday,Rdate,Rmonth,Ryears);
    RStatus = ReadTwo16BitValuesFromDAN777(DANADC1, DANADC2);
    WStatus = WriteTwo16BitValuesToDAN777(DANRCS1, DANRCS2);



    // IDLE loop. Just sit and loop forever (optional): new i2c while loop NRM
    while(1)
    {
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA,"DANRCS1: %d, DANRCS2: %d, DANADC1: %d, DANADC2: %d\r\n  %s %d%d/%d%d/%d%d %d%d:%d%d:%d%d \r\n", DANRCS1, DANRCS2, DANADC1, DANADC2, dayname, monthten, monthone, dateten, dateone,yearten, yearone, hourten, hourone, minten, minone, secten, secone);
            UARTPrint = 0;
        }
        if (RunI2C == 1) {
            RunI2C = 0;
            // Write to CHIPXYZ
            I2C_OK = WriteTwo16BitValuesToDAN777(DANRCS1, DANRCS2);
            num_WriteDAN777_Errors = 0;
            while(I2C_OK != 0) {
                num_WriteDAN777_Errors++;
                if (num_WriteDAN777_Errors > 2) {
                    serial_printf(&SerialA,"WriteTwo16BitValuesToDAN777 Error: %d\r\n",I2C_OK);
                    I2C_OK = 0;
                } else {
                    I2CB_Init();
                    DELAY_US(100000);
                    I2C_OK = WriteTwo16BitValuesToDAN777(DANRCS1, DANRCS2);
                }
            }
            // Read CHIPXYZ
            I2C_OK = ReadTwo16BitValuesFromDAN777(DANADC1, DANADC2);
            num_ReadDAN777_Errors = 0;
            while(I2C_OK != 0) {
                num_ReadDAN777_Errors++;
                if (num_ReadDAN777_Errors > 2) {
                    serial_printf(&SerialA,"ReadTwo16BitValuesFromDAN777 Error: %d\r\n",I2C_OK);
                    I2C_OK = 0;
                } else {
                    I2CB_Init();
                    DELAY_US(100000);
                    I2C_OK = ReadTwo16BitValuesFromDAN777(DANADC1, DANADC2);
                }
            }
            // Write to CHIPXYZ
                       I2C_OK = WriteTwo16BitValuesToBQ32000(Wsec,Wmin,Wcenthr,Wday,Wdate,Wmonth,Wyears);
                       num_WriteBQ32000_Errors = 0;
                       while(I2C_OK != 0) {
                           num_WriteBQ32000_Errors++;
                           if (num_WriteBQ32000_Errors > 2) {
                               serial_printf(&SerialA,"WriteTwo16BitValuesToBQ32000 Error: %d\r\n",I2C_OK);
                               I2C_OK = 0;
                           } else {
                               I2CB_Init();
                               DELAY_US(100000);
                               I2C_OK = WriteTwo16BitValuesToBQ32000(Wsec,Wmin,Wcenthr,Wday,Wdate,Wmonth,Wyears);
                           }
                       }
            // Read CHIPXYZ
            I2C_OK = ReadTwo16BitValuesFromBQ32000(Rsec,Rmin,Rcenthr,Rday,Rdate,Rmonth,Ryears);
            num_ReadBQ32000_Errors = 0;
            while(I2C_OK != 0) {
                num_ReadBQ32000_Errors++;
                if (num_ReadBQ32000_Errors > 2) {
                    serial_printf(&SerialA,"ReadTwo16BitValuesFromBQ32000 Error: %d\r\n",I2C_OK);
                    I2C_OK = 0;
                } else {
                    I2CB_Init();
                    DELAY_US(100000);
                    I2C_OK = ReadTwo16BitValuesFromBQ32000(Rsec,Rmin,Rcenthr,Rday,Rdate,Rmonth,Ryears);
                }
            }
        }
    }
}
//i2C Functions
void I2CB_Init(void) {
    // Setting up EPWM 7 to use as a timer for error handling
    EALLOW;
    EPwm7Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm7Regs.TBCTL.bit.CTRMODE = 3;// Freeze counter
    EPwm7Regs.TBCTR = 0x0;// Clear counter
    EPwm7Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm7Regs.TBCTL.bit.PHSEN = 0;   // Disable phase loading
    EPwm7Regs.TBCTL.bit.CLKDIV = 7; // divide by 1  50Mhz Clock
    EPwm7Regs.TBPRD = 0xFFFF; // PRD not used for timer
    // Notice here we are not using the PWM signal, so CMPA/CMPB are not set
    EDIS;
    /* If an I2C device is holding the SDA Line Low you need to tell the device
     * to reinitialize by clocking 9 clocks to the device to reset it. */
    GpioDataRegs.GPBSET.bit.GPIO41 = 1; // Here make sure SCL clk is high
    GPIO_SetupPinMux(41, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(41, GPIO_OUTPUT, GPIO_PUSHPULL); // Set SDA as GPIO input pin for now to check if SDA is being held low.
    GPIO_SetupPinMux(40, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(40, GPIO_INPUT, GPIO_PULLUP);
    /* Check if SDA is low.  If it is manually set the SCL pin high and low to
     * create nine clock periods. For more reading see the I2C specification linked
     * in this homework document and search for "nine clock pulses" */
    if (GpioDataRegs.GPBDAT.bit.GPIO40 == 0) {  // If SDA low
        // Pulse CLK 9 Times if SDA pin Low
        for (int i = 0; i<9; i++) {
            GpioDataRegs.GPBSET.bit.GPIO41 = 1;
            DELAY_US(30);
            GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1;
            DELAY_US(30);

        }

    }
    // Now setup GPIO40 as SDAB and GPIO41 and SCLB
    EALLOW;     /* Enable internal pull-up for the selected I2C pins */
    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 1;      /* Set qualification for the selected I2C pins */
    GpioCtrlRegs.GPBQSEL1.bit.GPIO40 = 3;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO41 = 3;      /* Configure which of the possible GPIO pins will be I2C_B pins using GPIO regs*/
    GpioCtrlRegs.GPBGMUX1.bit.GPIO40 = 1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 2;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO41 = 1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 2;
    EDIS;      // At breakpoint, allow I2C to continue operating
    I2cbRegs.I2CMDR.bit.FREE = 1;      // Initialize I2C
    I2cbRegs.I2CMDR.bit.IRS = 0;      // 200MHz / 20 = 10MHz
    I2cbRegs.I2CPSC.all = 19;      // 10MHz/40 = 250KHz
    I2cbRegs.I2CCLKL = 15*3;  //psc > 2 so d = 5  See Usersguide
    I2cbRegs.I2CCLKH = 15*3;  //psc > 2 so d = 5  See Usersguide
    I2cbRegs.I2CIER.all = 0x00;
    I2cbRegs.I2CMDR.bit.IRS = 1;
    DELAY_US(2000);
}

int16_t I2C_CheckIfTX(uint16_t timeout) {
    int16_t Xrdy = 0;
    EPwm7Regs.TBCTR = 0x0; // Clear counter
    EPwm7Regs.TBCTL.bit.CTRMODE = 0; // unfreeze, and enter up count mode
    while(!Xrdy) {
        if (EPwm7Regs.TBCTR > timeout) {  // if it has been 100ms
            EPwm7Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
            return -1;
        }
        Xrdy = I2cbRegs.I2CSTR.bit.XRDY;
    }
    return 0;
}



int16_t I2C_CheckIfRX(uint16_t timeout) {
    int16_t Rrdy = 0;
    EPwm7Regs.TBCTR = 0x0; // Clear counter
    EPwm7Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    while(!Rrdy) {
        if (EPwm7Regs.TBCTR > timeout) {  // if we have been in this function for 100ms
            EPwm7Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
            return -1;
        }
        Rrdy = I2cbRegs.I2CSTR.bit.RRDY;
    }
    return 0;
}


// Write 2 16-bit commands (LSB then MSB) to I2C Slave CHIPXYZ starting at CHIPXYZ's register 4
uint16_t WriteTwo16BitValuesToBQ32000(uint16_t Rvalue1,uint16_t Rvalue2,uint16_t Rvalue3,uint16_t Rvalue4,uint16_t Rvalue5,uint16_t Rvalue6,uint16_t Rvalue7) {
    uint16_t Cmd1LSB = 0;
    uint16_t Cmd1MSB = 0;
    uint16_t Cmd2LSB = 0;
    uint16_t Cmd2MSB = 0;
    uint16_t Cmd3LSB = 0;
    uint16_t Cmd3MSB = 0;
    uint16_t Cmd4LSB = 0;
    uint16_t Cmd4MSB = 0;
    uint16_t Cmd5LSB = 0;
    uint16_t Cmd5MSB = 0;
    uint16_t Cmd6LSB = 0;
    uint16_t Cmd6MSB = 0;
    uint16_t Cmd7LSB = 0;
    uint16_t Cmd7MSB = 0;
    int16_t I2C_Xready = 0;
    Cmd1LSB = Rvalue1 & 0xFF; //Bottom 8 bits of command
    Cmd1MSB = (Rvalue1 >> 8) & 0xFF; //Top 8 bits of command
    Cmd2LSB = Rvalue2 & 0xFF; //Bottom 8 bits of command
    Cmd2MSB = (Rvalue2 >> 8) & 0xFF; //Top 8 bits of command
    Cmd3LSB = Rvalue3 & 0xFF; //Bottom 8 bits of command
    Cmd3MSB = (Rvalue3 >> 8) & 0xFF; //Top 8 bits of command
    Cmd4LSB = Rvalue4 & 0xFF; //Bottom 8 bits of command
    Cmd4MSB = (Rvalue4 >> 8) & 0xFF; //Top 8 bits of command
    Cmd5LSB = Rvalue5 & 0xFF; //Bottom 8 bits of command
    Cmd5MSB = (Rvalue5 >> 8) & 0xFF; //Top 8 bits of command
    Cmd6LSB = Rvalue6 & 0xFF; //Bottom 8 bits of command
    Cmd6MSB = (Rvalue6 >> 8) & 0xFF; //Top 8 bits of command
    Cmd7LSB = Rvalue7 & 0xFF; //Bottom 8 bits of command
    Cmd7MSB = (Rvalue7 >> 8) & 0xFF; //Top 8 bits of command

    // Allow time for I2C to finish up previous commands.
    DELAY_US(200);
    if (I2cbRegs.I2CSTR.bit.BB == 1) { // Check if I2C busy. If it is, it's better
        return 2; // to exit and try again next sample.
    } // This should not happen too often.
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C is ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CSAR.all = 0x25; // Set I2C address to that of BQ32000s
    I2cbRegs.I2CCNT = 15; // Number of values to send plus start register: 4 + 1
    I2cbRegs.I2CDXR.all = 0; // First need to transfer the register value to start writing data [4 is rcservo address]
    I2cbRegs.I2CMDR.all = 0x6E20; // I2C in master mode (MST), I2C is in transmit mode (TRX) with start and stop
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd1LSB; // Write Command 1 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd1MSB; // Write Command 1 MSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd2LSB; // Write Command 2 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd2MSB; // Write Command 2 MSB
    // Since I2CCNT = 0 at this point, a stop condition will be issued
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd3LSB; // Write Command 1 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd3MSB; // Write Command 1 MSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd4LSB; // Write Command 2 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd4MSB; // Write Command 2 MSB
    // Since I2CCNT = 0 at this point, a stop condition will be issued
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd5LSB; // Write Command 1 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd5MSB; // Write Command 1 MSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd6LSB; // Write Command 2 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd6MSB; // Write Command 2 MSB
    // Since I2CCNT = 0 at this point, a stop condition will be issued
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd7LSB; // Write Command 1 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd7MSB; // Write Command 1 MSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }

    return 0;
}

uint16_t WriteTwo16BitValuesToDAN777(uint16_t Cmd16bit_1, uint16_t Cmd16bit_2) {
    uint16_t Cmd1LSB = 0;
    uint16_t Cmd1MSB = 0;
    uint16_t Cmd2LSB = 0;
    uint16_t Cmd2MSB = 0;
    int16_t I2C_Xready = 0;
    Cmd1LSB = Cmd16bit_1 & 0xFF; //Bottom 8 bits of command
    Cmd1MSB = (Cmd16bit_1 >> 8) & 0xFF; //Top 8 bits of command
    Cmd2LSB = Cmd16bit_2 & 0xFF; //Bottom 8 bits of command
    Cmd2MSB = (Cmd16bit_2 >> 8) & 0xFF; //Top 8 bits of command
    // Allow time for I2C to finish up previous commands.
    DELAY_US(200);
    if (I2cbRegs.I2CSTR.bit.BB == 1) { // Check if I2C busy. If it is, it's better
        return 2; // to exit and try again next sample.
    } // This should not happen too often.
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C is ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CSAR.all = 0xD0; // Set I2C address to that of DAN777's
    I2cbRegs.I2CCNT = 5; // Number of values to send plus start register: 4 + 1
    I2cbRegs.I2CDXR.all = 4; // First need to transfer the register value to start writing data [4 is rcservo address]
    I2cbRegs.I2CMDR.all = 0x6E20; // I2C in master mode (MST), I2C is in transmit mode (TRX) with start and stop
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd1LSB; // Write Command 1 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd1MSB; // Write Command 1 MSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd2LSB; // Write Command 2 LSB
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CDXR.all = Cmd2MSB; // Write Command 2 MSB
    // Since I2CCNT = 0 at this point, a stop condition will be issued
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    return 0;
}

/* Read Two 16 Bit values from I2C Slave CHIPXYZ starting at CHIPXYZ's register 10.
 * Notice the Rvalue1 and Rvalue2 passed as pointers (passed by reference). So pass
 * address of the uint16_t variable when using this function. For example:
 * uint16_t Rval1 = 0;
 * uint16_t Rval2 = 0;
 * err = ReadTwo16BitValuesFromCHIPXYZ(&Rval1,&Rval2);
 * This allows Rval1 and Rval2 to be changed inside the function and return the
 * values read inside the function. */
uint16_t ReadTwo16BitValuesFromDAN777(uint16_t *Rvalue1,uint16_t *Rvalue2) {
    uint16_t Val1LSB = 0;
    uint16_t Val1MSB = 0;
    uint16_t Val2LSB = 0;
    uint16_t Val2MSB = 0;

    int16_t I2C_Xready = 0;
    int16_t I2C_Rready = 0;
    // Allow time for I2C to finish up previous commands.
    DELAY_US(200);
    if (I2cbRegs.I2CSTR.bit.BB == 1) { // Check if I2C busy. If it is, it's better
        return 2; // to exit and try again next sample.
    } // This should not happen too often.
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CSAR.all = 0x25; // I2C address of DAN777
    I2cbRegs.I2CCNT = 1; // Just sending address to start reading from
    I2cbRegs.I2CDXR.all = 0; // Start reading at this register location
    I2cbRegs.I2CMDR.all = 0x6620; // I2C in master mode (MST), I2C is in transmit mode (TRX) with start
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    // Reissuing another start command to begin reading the values we want
    I2cbRegs.I2CSAR.all = 0x25; // I2C address of DAN777
    I2cbRegs.I2CCNT = 4; // Receive count
    I2cbRegs.I2CMDR.all = 0x6C20; // I2C in master mode (MST), TRX=0 (receive mode) with start & stop
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val1LSB = I2cbRegs.I2CDRR.all; // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val1MSB = I2cbRegs.I2CDRR.all; // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val2LSB = I2cbRegs.I2CDRR.all; // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val2MSB = I2cbRegs.I2CDRR.all; // Read DAN777
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    // Since I2CCNT = 0 at this point, a stop condition will be issued
    *Rvalue1 = (Val1MSB << 8) | (Val1LSB & 0xFF);
    *Rvalue2 = (Val2MSB << 8) | (Val2LSB & 0xFF);
    return 0;
}


uint16_t ReadTwo16BitValuesFromBQ32000(uint16_t *Rvalue1,uint16_t *Rvalue2,uint16_t *Rvalue3,uint16_t *Rvalue4,uint16_t *Rvalue5,uint16_t *Rvalue6,uint16_t *Rvalue7) {
    uint16_t Val1LSB = 0;
    uint16_t Val1MSB = 0;
    uint16_t Val2LSB = 0;
    uint16_t Val2MSB = 0;
    uint16_t Val3LSB = 0;
    uint16_t Val3MSB = 0;
    uint16_t Val4LSB = 0;
    uint16_t Val4MSB = 0;
    uint16_t Val5LSB = 0;
    uint16_t Val5MSB = 0;
    uint16_t Val6LSB = 0;
    uint16_t Val6MSB = 0;
    uint16_t Val7LSB = 0;
    uint16_t Val7MSB = 0;

    int16_t I2C_Xready = 0;
    int16_t I2C_Rready = 0;
    // Allow time for I2C to finish up previous commands.
    DELAY_US(200);
    if (I2cbRegs.I2CSTR.bit.BB == 1) { // Check if I2C busy. If it is, it's better
        return 2; // to exit and try again next sample.
    } // This should not happen too often.
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    I2cbRegs.I2CSAR.all = 0xD1; // I2C address of BQ32000
    I2cbRegs.I2CCNT = 1; // Just sending address to start reading from
    I2cbRegs.I2CDXR.all = 0; // Start reading at this register location
    I2cbRegs.I2CMDR.all = 0x6620; // I2C in master mode (MST), I2C is in transmit mode (TRX) with start
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Xready = I2C_CheckIfTX(39062); // Poll until I2C ready to transmit
    if (I2C_Xready == -1) {
        return 4;
    }
    // Reissuing another start command to begin reading the values we want
    I2cbRegs.I2CSAR.all = 0xD1; // I2C address of BQ32000
    I2cbRegs.I2CCNT = 14; // Receive count
    I2cbRegs.I2CMDR.all = 0x6C20; // I2C in master mode (MST), TRX=0 (receive mode) with start & stop
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val1MSB = I2cbRegs.I2CDRR.all; // Read BQ32000
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val1LSB = I2cbRegs.I2CDRR.all; // Read BQ32000
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val2MSB = I2cbRegs.I2CDRR.all; // Read BQ32000
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val2LSB = I2cbRegs.I2CDRR.all; // Read BQ32000
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val3MSB = I2cbRegs.I2CDRR.all; // Read BQ32000
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val3LSB = I2cbRegs.I2CDRR.all; // Read BQ32000
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val4MSB = I2cbRegs.I2CDRR.all; // Read BQ32000
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val4LSB = I2cbRegs.I2CDRR.all; // Read BQ32000
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val5MSB = I2cbRegs.I2CDRR.all; // Read BQ32000
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val5LSB = I2cbRegs.I2CDRR.all; // Read BQ32000
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val6MSB = I2cbRegs.I2CDRR.all; // Read BQ32000
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val6LSB = I2cbRegs.I2CDRR.all; // Read BQ32000
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val7MSB = I2cbRegs.I2CDRR.all; // Read BQ32000
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    I2C_Rready = I2C_CheckIfRX(39062); //Poll until I2C has received 8-bit value
    if (I2C_Rready == -1) {
        return -1;
    }
    Val7LSB = I2cbRegs.I2CDRR.all; // Read BQ32000
    if (I2cbRegs.I2CSTR.bit.NACK == 1) { // Check for No Acknowledgement
        return 3; // This should not happen
    }
    // Since I2CCNT = 0 at this point, a stop condition will be issued
    *Rvalue1 = (Val1MSB << 8) | (Val1LSB & 0xFF);
    *Rvalue2 = (Val2MSB << 8) | (Val2LSB & 0xFF);
    *Rvalue3 = (Val3MSB << 8) | (Val3LSB & 0xFF);
    *Rvalue4 = (Val4MSB << 8) | (Val4LSB & 0xFF);
    *Rvalue5 = (Val5MSB << 8) | (Val5LSB & 0xFF);
    *Rvalue6 = (Val6MSB << 8) | (Val6LSB & 0xFF);
    *Rvalue7 = (Val7MSB << 8) | (Val7LSB & 0xFF);


    return 0;
}
//RCServo Value Functions

uint16_t setDAN777RCServo1(float angle)
{
    uint16_t angout;

    if (angle < -90)
    {
        angle = -90;
    }
    if (angle > 90)
    {
        angle = 90;
    }

    angout = (((angle) + 90.0)/180.0) *4000 + 1200; //NRW ADK - Normalizing


    return angout;
}
uint16_t setDAN777RCServo2(float angle)
{
    uint16_t angout;

    if (angle < -90)
    {
        angle = -90;
    }
    if (angle > 90)
    {
        angle = 90;
    }

    angout = (((angle) + 90.0)/180.0) * 4000 + 1200;


    return angout;
}



// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......


    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }

    if ((numTimer0calls%25) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    if ((numTimer0calls%50) == 0) {
        // Blink LaunchPad Red LED
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{

    CpuTimer1.InterruptCount++;
}



// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    CpuTimer2.InterruptCount++;
    RunI2C = 1;
    if ((CpuTimer2.InterruptCount % 5) == 0) {
        UARTPrint = 1;
    }

    // NRW
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
        angeff+=.05;
        DANRCS1 = setDAN777RCServo1(angeff);
        DANRCS2 = setDAN777RCServo2(angeff);
    }
    if (AFlip == 1)
    {
        angeff-=.05;
        DANRCS1 =  setDAN777RCServo1(angeff);
        DANRCS2 = setDAN777RCServo2(angeff);
    }
    BStatus = ReadTwo16BitValuesFromBQ32000(Rsec,Rmin,Rcenthr,Rday,Rdate,Rmonth,Ryears);
    RStatus = ReadTwo16BitValuesFromDAN777(DANADC1, DANADC2);
    WStatus = WriteTwo16BitValuesToDAN777(DANRCS1, DANRCS2);
    secone = (Rsec&0x0F);
    secten = (Rsec&0x60);

    minone = (Rmin&0x0F);
    minten = (Rmin&0x60);

    hourone = (Rcenthr&0x0F);
    hourten = (Rcenthr&0x02);
    cent = (Rcenthr&0x40);

    dateone = (Rdate&0x0F);
    dateten = (Rdate&0x30);

    monthone = (Rmonth&0x0F);
    monthten = (Rmonth&0x10);

    yearone = (Ryears&0x0F);
    yearten = (Ryears&0xF0);

    day = (Rday&0x07);
    if (day == 1){
        dayname = "Sunday";
    }
    if (day == 2){
        dayname = "Monday";
    }
    if (day == 3){
        dayname = "Tuesday";
    }
    if (day == 4){
        dayname = "Wednesday";
    }
    if (day == 5){
        dayname = "Thursday";
    }
    if (day == 6){
        dayname = "Friday";
    }
    if (day == 7){
        dayname = "Saturday";
    }

}



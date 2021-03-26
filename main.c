// voltage-sensor  --> ADCA1
// xuat theta      --> ADAA0

#define RED                20U
#define FED                20U
#define DB_UP          1

#include "F28x_Project.h"
#include "Solar_F.h"

extern float sin_tab[];

#define GRID_FREQ 50
#define ISR_FREQUENCY 25000
#define PI 3.14159265359
#define Interrupt_FREQ 1000
SPLL_1ph_SOGI_F spll1;

//
// Globals
//
Uint32 EPwm7TimerIntCount;
Uint32 EPwm8TimerIntCount;
Uint32 EPwm6TimerIntCount;
Uint16 EPwm7_DB_Direction;
Uint16 EPwm8_DB_Direction;
Uint16 EPwm6_DB_Direction;

float GrisMeas = 0;
float index = 0, m = 0.8;

int role = 1, hnb = 0;


//
// Function Prototypes
//
void Gpio_setup1(void);
void InitEPwm2Example(void);
void configureDAC(void);

__interrupt void cpu_timer0_isr(void);
__interrupt void epwm2_isr(void);

void main(void)
{
    InitSysCtrl();
    //
    // enable PWM7, PWM8 and PWM6
    //
    CpuSysRegs.PCLKCR2.bit.EPWM7=1;
    CpuSysRegs.PCLKCR2.bit.EPWM8=1;
    CpuSysRegs.PCLKCR2.bit.EPWM6=1;
    //
    // Step 2. Initialize GPIO:
    // This example function is found in the F2837xS_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    InitGpio();
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(65, GPIO_OUTPUT, GPIO_PUSHPULL);



    //
    // For this case just init GPIO pins for ePWM7, ePWM8, ePWM9
    // These functions are in the F2837xS_EPwm.c file
    //
    InitEPwm7Gpio();
    InitEPwm8Gpio();
    InitEPwm6Gpio();

    configureDAC();
    DINT;

    InitPieCtrl();
    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();
    Gpio_setup1();


    EALLOW;  // This is needed to write to EALLOW protected registers

    PieVectTable.EPWM7_INT = &epwm2_isr;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    EDIS;

    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200, 20);


    // Step 4. Initialize the Device Peripherals:
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =0;
    EDIS;

    InitEPwm2Example();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC =1;
    EDIS;


    EPwm7TimerIntCount = 0;
    EPwm8TimerIntCount = 0;
    EPwm6TimerIntCount = 0;


    CpuTimer0Regs.TCR.all = 0x4000;

    IER |= M_INT1;
    IER |= M_INT3;

    //
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    //
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;


//     Enable EPWM INTn in the PIE: Group 3 interrupt 1-3

    PieCtrlRegs.PIEIER3.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx8 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx6 = 1;

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    SPLL_1ph_SOGI_F_init(GRID_FREQ,((float)(1.0/ISR_FREQUENCY)),&spll1);
    SPLL_1ph_SOGI_F_coeff_update(((float)(1.0/ISR_FREQUENCY)),(float)(2*PI*GRID_FREQ),&spll1);

    while(1)
    {

    }
}

void Gpio_setup1(void)
{
   //
   // These can be combined into single statements for improved
   // code efficiency.
   //

   //
   // Enable PWM1-3 on GPIO0-GPIO5
   //
   EALLOW;
   //
   // Enable an GPIO output on GPIO2, set it low
   //
   GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;   // Enable pullup on GPIO2
   GpioDataRegs.GPASET.bit.GPIO2 = 1;   // Load output latch
   GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;  // GPIO2 = GPIO2
   GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;   // GPIO2 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pullup on GPIO20
    GpioDataRegs.GPASET.bit.GPIO20 = 1;   // Load output latch
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;  // GPIO20 = GPIO20
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;   // GPIO20 = output
   EDIS;
}

void InitEPwm2Example()
{
    EPwm7Regs.TBPRD = 2000;                       // Set timer period
    EPwm7Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm7Regs.TBCTR = 0x0000;                     // Clear counter

    EPwm8Regs.TBPRD = 2000;                       // Set timer period
    EPwm8Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm8Regs.TBCTR = 0x0000;                     // Clear counter

    EPwm6Regs.TBPRD = 2000;                       // Set timer period
    EPwm6Regs.TBPHS.bit.TBPHS = 0x0000;           // Phase is 0
    EPwm6Regs.TBCTR = 0x0000;                     // Clear counter

    //
    // Setup TBCLK
    //
    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Master module
//    EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;     // Sync down-stream module
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm7Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Slow just to observe on
                                                   // the scope

    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // slave module
//    EPwm8Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;     // Sync down-stream module
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm8Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Slow just to observe on
                                                   // the scope

    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // slave module
//    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;     // Sync down-stream module
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // Slow just to observe on
                                                   // the scope

    //
    // Setup compare
    //
    EPwm7Regs.CMPA.bit.CMPA = 1000;
    EPwm8Regs.CMPA.bit.CMPA = 1000;
    EPwm6Regs.CMPA.bit.CMPA = 1000;

    //
    // Set actions
    //
    EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM2A on Zero
    EPwm7Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM2A on Zero
    EPwm8Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;            // Set PWM2A on Zero
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;

    //
    // Active Low complementary PWMs - setup the deadband
    //
    EPwm7Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm7Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm7Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm7Regs.DBRED.bit.DBRED = RED;
    EPwm7Regs.DBFED.bit.DBFED = FED;

    EPwm8Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm8Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm8Regs.DBRED.bit.DBRED = RED;
    EPwm8Regs.DBFED.bit.DBFED = FED;

    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;
    EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm6Regs.DBRED.bit.DBRED = RED;
    EPwm6Regs.DBFED.bit.DBFED = FED;

    //
    // Interrupt where we will modify the deadband
    //
    EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm7Regs.ETSEL.bit.INTEN = 1;                // Enable INT
    EPwm7Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event
}

__interrupt void epwm2_isr(void)
{

    if(role) GpioDataRegs.GPASET.bit.GPIO20 = 1;   // Load output latch
    else GpioDataRegs.GPACLEAR.bit.GPIO20=1;   // Load output latch

    GpioDataRegs.GPASET.bit.GPIO2 = 1;   // Load output latch

    index = index + 4.096;
    if(index >= 2048) index -= 2048;
    GrisMeas = m*sin_tab[(int)index];
    spll1.u[0] = GrisMeas;

    if(GrisMeas > 0)
    {
        EPwm6Regs.CMPA.bit.CMPA = 0;
    }
    else
    {
        EPwm6Regs.CMPA.bit.CMPA = 2000;
    }
    if(GrisMeas <= 0.0000) GrisMeas += 1.00000;
    EPwm7Regs.CMPA.bit.CMPA = (int)2000*(2.0*GrisMeas-1.00000);    // Set compare A value
    EPwm8Regs.CMPA.bit.CMPA = (int)2000*(2.0*GrisMeas);     // Set compare A value

    SPLL_1ph_SOGI_F_FUNC(&spll1);

    DacaRegs.DACVALS.all = spll1.theta[0] * 651;
    DacbRegs.DACVALS.all = GrisMeas * 1000 + 1500;



//    float hnb = index/17.0;

    GpioDataRegs.GPACLEAR.bit.GPIO2=1;   // Load output latch


    EPwm7TimerIntCount++;

    //
    // Clear INT flag for this timer
    //
    EPwm7Regs.ETCLR.bit.INT = 1;

    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void cpu_timer0_isr(void)
{



    CpuTimer0.InterruptCount++;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


void configureDAC(void)
{
    EALLOW;
    DacbRegs.DACCTL.bit.DACREFSEL = 1;          // Use ADC references
    DacbRegs.DACCTL.bit.LOADMODE = 0;           // Load on next SYSCLK
                    // Set mid-range
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;         // Enable DAC

    DacaRegs.DACCTL.bit.DACREFSEL = 1;          // Use ADC references
    DacaRegs.DACCTL.bit.LOADMODE = 0;           // Load on next SYSCLK
                    // Set mid-range
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;         // Enable DAC

    DaccRegs.DACCTL.bit.DACREFSEL = 1;          // Use ADC references
    DaccRegs.DACCTL.bit.LOADMODE = 0;           // Load on next SYSCLK
                        // Set mid-range
    DaccRegs.DACOUTEN.bit.DACOUTEN = 1;         // Enable DAC
    EDIS;
}


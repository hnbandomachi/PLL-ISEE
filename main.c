// voltage-sensor  --> ADCA1
// xuat theta      --> ADAA0

// Signal          Acquisition (ns)        Cycles          ACQPS       ADC PIN     CHSEL
// Vg              200                     200/5=40        40-1=39     ADCINA0     0
// Ig              200                     200/5=40        40-1=39     ADCINA1     1
// Vpv             200                     200/5=40        40-1=39     ADCINB0     0
// Ipv             200                     200/5=40        40-1=39     ADCINB1     1

#include "F28x_Project.h"
#include "Solar_F.h"

//
// Defines
//
#define RED             10U
#define FED             10U
#define GRID_FREQ       50
#define ISR_FREQUENCY   25000
#define PI              3.14159265359

#define I0              15
#define phi             0

#define PWM_PRD         2000    // 25 kHz

extern float sin_tab[];

//
// Globals
//
Uint32 EPwm7TimerIntCount;
Uint32 EPwm8TimerIntCount;
Uint32 EPwm6TimerIntCount;
Uint16 EPwm7_DB_Direction;
Uint16 EPwm8_DB_Direction;
Uint16 EPwm6_DB_Direction;

SPLL_1ph_SOGI_F spll1;

Uint16 VgSample;
Uint16 IgSample;
Uint16 VpvSample;
Uint16 IpvSample;

float32 Vg;
float32 Ig;
float32 Vpv;
float32 Ipv;

float32 outt = 0;
float32 index = 0, m = 0.8;
int role = 1;

//
// Function Prototypes
//
void Gpio_setup1(void);
void InitEPwm2Example(void);
void ConfigureADC(void);
void configureDAC(void);
void SetupADCEpwm(void);

void CurrrentControlNoPV(void);
void SingleStagePV(void);

__interrupt void cpu_timer0_isr(void);
__interrupt void epwm2_isr(void);

void main(void)
{
    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xS_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Step 2. Initialize GPIO:
    // This example function is found in the F2837xS_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    InitGpio();
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(65, GPIO_OUTPUT, GPIO_PUSHPULL);

    //
    // enable PWM7, PWM8 and PWM6
    //
    CpuSysRegs.PCLKCR2.bit.EPWM7 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM8 = 1;
    CpuSysRegs.PCLKCR2.bit.EPWM6 = 1;

    //
    // For this case just init GPIO pins for ePWM7, ePWM8, ePWM9
    // These functions are in the F2837xS_EPwm.c file
    //
    InitEPwm7Gpio();
    InitEPwm8Gpio();
    InitEPwm6Gpio();

    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xS_PieCtrl.c file.
    //
    InitPieCtrl();
    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //

    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xS_DefaultIsr.c.
    // This function is found in F2837xS_PieVect.c.
    //
    InitPieVectTable();

    // Setup fror role and timing pin
    Gpio_setup1();

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM7_INT = &epwm2_isr;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    EDIS;

    //
    // Configure the ADC and power it up
    //
    ConfigureADC();

    //
    // Setup the ADC for ePWM triggered conversions on sensor
    //
    SetupADCEpwm();

    //
    // Step 3. Initialize the Device Peripheral. This function can be
    //         found in F2837xS_CpuTimers.c
    //
    InitCpuTimers(); // For this example, only initialize the Cpu Timers
    //
    // Configure CPU-Timer 0 to interrupt every 20us
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    //
    ConfigCpuTimer(&CpuTimer0, 200, 20);

    //
    // To ensure precise timing, use write-only instructions to write to the
    // entire register. Therefore, if any of the configuration bits are changed in
    // ConfigCpuTimer and InitCpuTimers (in F2837xS_cputimervars.h), the below
    // settings must also be updated.
    //
    CpuTimer0Regs.TCR.all = 0x4000;

    // Step 4. Initialize the Device Peripherals:
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    InitEPwm2Example();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    //
    // Step 5. User specific code, enable interrupts:
    // Initialize counters:
    //
    EPwm7TimerIntCount = 0;
    EPwm8TimerIntCount = 0;
    EPwm6TimerIntCount = 0;

    //
    // Enable CPU INT3 which is connected to EPWM1-3 INT:
    // To know which INT? Search in "TMS320F2837xS Microcontrollers - Technical Reference Manual - Table 3-2. PIE Channel Mapping"
    IER |= M_INT1;
    IER |= M_INT3;

    //
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    //
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    // Enable EPWM INTn in the PIE: Group 3 interrupt 1-3

    PieCtrlRegs.PIEIER3.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx8 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx6 = 1;

    EINT; // Enable Global interrupt INTM
    ERTM; // Enable Global realtime interrupt DBGM

    // Put the applicated source here
    SPLL_1ph_SOGI_F_init(GRID_FREQ, ((float)(1.0 / ISR_FREQUENCY)), &spll1);
    SPLL_1ph_SOGI_F_coeff_update(((float)(1.0 / ISR_FREQUENCY)), (float)(2 * PI * GRID_FREQ), &spll1);

    do
    {
        //
        //convert, wait for completion, and store results
        //start conversions immediately via software, ADCA
        //
        AdcaRegs.ADCSOCFRC1.all = 0x0003; //SOC0 and SOC1

        //
        //start conversions immediately via software, ADCB
        //
        AdcbRegs.ADCSOCFRC1.all = 0x0003; //SOC0 and SOC1

        //
        //wait for ADCA to complete, then acknowledge flag
        //
        while (AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0)
            ;
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

        //
        //wait for ADCB to complete, then acknowledge flag
        //
        while (AdcbRegs.ADCINTFLG.bit.ADCINT1 == 0)
            ;
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

        //
        //store results
        //
        VgSample = AdcaResultRegs.ADCRESULT0;
        IgSample = AdcaResultRegs.ADCRESULT1;
        VpvSample = AdcbResultRegs.ADCRESULT0;
        IpvSample = AdcbResultRegs.ADCRESULT1;

        //
        //at this point, conversion results are stored in
        //VgSample, IgSample, VpvSample, and IpvSample
        //

        //
        //software breakpoint, hit run again to get updated conversions
        //

    } while (1);
}

//
// ConfigureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC B
//
void ConfigureADC(void)
{
    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

//
// SetupADCEpwm - Configure ADC EPWM acquisition window and trigger
//
void SetupADCEpwm(void)
{
    Uint16 sensor_acqps = 39;

    //
    //Select the channels to convert and end of conversion flag
    //

    // "TRIGSEL" search google
    EALLOW;
    // ADCA: Vg, Ig
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;            //SOC0 will convert internal //connection A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = sensor_acqps; //sample window is 39 ~ 200 (ns) SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;            //SOC1 will convert internal //connection A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = sensor_acqps; //sample window is 39 ~ 200 (ns) SYSCLK cycles

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    // ADCB: Vpv, Ipv
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;            //SOC0 will convert internal //connection B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = sensor_acqps; //sample window is 39 ~ 200 (ns) SYSCLK cycles
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1;            //SOC1 will convert internal //connection B1
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = sensor_acqps; //sample window is 39 ~ 200 (ns) SYSCLK cycles

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;
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
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;  // Enable pullup on GPIO2
    GpioDataRegs.GPASET.bit.GPIO2 = 1;  // Load output latch
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0; // GPIO2 = GPIO2
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;  // GPIO2 = output

    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;  // Enable pullup on GPIO20
    GpioDataRegs.GPASET.bit.GPIO20 = 1;  // Load output latch
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0; // GPIO20 = GPIO20
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;  // GPIO20 = output
    EDIS;
}

void InitEPwm2Example()
{
    EPwm7Regs.TBPRD = PWM_PRD;             // Set timer period
    EPwm7Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm7Regs.TBCTR = 0x0000;           // Clear counter

    EPwm8Regs.TBPRD = PWM_PRD;             // Set timer period
    EPwm8Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm8Regs.TBCTR = 0x0000;           // Clear counter

    EPwm6Regs.TBPRD = PWM_PRD;             // Set timer period
    EPwm6Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm6Regs.TBCTR = 0x0000;           // Clear counter

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
    EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // slave module
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
    EPwm7Regs.CMPA.bit.CMPA = PWM_PRD/2;
    EPwm8Regs.CMPA.bit.CMPA = PWM_PRD/2;
    EPwm6Regs.CMPA.bit.CMPA = PWM_PRD/2;

    //
    // Set actions
    //
    EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Set PWM2A on Zero
    EPwm7Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Set PWM2A on Zero
    EPwm8Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Set PWM2A on Zero
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
    EPwm7Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
    EPwm7Regs.ETSEL.bit.INTEN = 1;            // Enable INT
    EPwm7Regs.ETPS.bit.INTPRD = ET_1ST;       // Generate INT on 1st event
}

__interrupt void epwm2_isr(void)
{
    // Calculate the real value

    Vg = (float32)(VgSample - 2512.0)*0.2344322344;
    Ig = (float32)(IgSample - 2512.0)*0.01932098706;
    Vpv = (float32)(VpvSample / 4096.0);
    Ipv = (float32)(IpvSample / 4096.0);
    
    CurrrentControlNoPV();
    // SingleStagePV();

    if (role)
        GpioDataRegs.GPASET.bit.GPIO20 = 1; // Load output latch
    else
        GpioDataRegs.GPACLEAR.bit.GPIO20 = 1; // Load output latch

    GpioDataRegs.GPASET.bit.GPIO2 = 1; // Load output latch

    index = index + 4.096;
    if (index >= 2048)
        index -= 2048;
    outt = m * sin_tab[(int)index];
    spll1.u[0] = outt;

    if (outt > 0.0000)
    {
        EPwm6Regs.CMPA.bit.CMPA = 0;
    }
    else
    {
        EPwm6Regs.CMPA.bit.CMPA = PWM_PRD;
    }
    if (outt <= 0.0000)
        outt += 1.00000;
    EPwm7Regs.CMPA.bit.CMPA = (int)PWM_PRD * (2.0 * outt - 1.00000); // Set compare A value
    EPwm8Regs.CMPA.bit.CMPA = (int)PWM_PRD * (2.0 * outt);           // Set compare A value

    SPLL_1ph_SOGI_F_FUNC(&spll1);

    // DacaRegs.DACVALS.all = spll1.theta[0] * 651;
    // DacbRegs.DACVALS.all = outt * 1000 + 1500;

    GpioDataRegs.GPACLEAR.bit.GPIO2 = 1; // Load output latch

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

void CurrrentControlNoPV()
{

}

void SingleStagePV()
{

}

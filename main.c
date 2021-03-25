// voltage-sensor  --> ADCA1
// xuat theta      --> ADAA0

#include "F28x_Project.h"
#include "Solar_F.h"

extern float sin_tab[];

#define GRID_FREQ 50
#define ISR_FREQUENCY 20000
#define PI 3.14159265359
#define Interrupt_FREQ 1000
SPLL_1ph_SOGI_F spll1;

float GrisMeas = 0;
float index = 0;

void Gpio_setup1(void);

__interrupt void cpu_timer0_isr(void);

void main(void)
{
    InitSysCtrl();
    ConfigureDAC();
    InitGpio();
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(65, GPIO_OUTPUT, GPIO_PUSHPULL);

    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();
    Gpio_setup1();


    EALLOW;  // This is needed to write to EALLOW protected registers

    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    EDIS;

    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 200, 50);

    CpuTimer0Regs.TCR.all = 0x4000;

    IER |= M_INT1;
    IER |= M_INT13;
    IER |= M_INT14;

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

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
   GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;   // Enable pullup on GPIO71
   GpioDataRegs.GPASET.bit.GPIO2 = 1;   // Load output latch
   GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;  // GPIO71 = GPIO71
   GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;   // GPIO71 = output
   EDIS;
}

__interrupt void cpu_timer0_isr(void)
{

    GpioDataRegs.GPASET.bit.GPIO2 = 1;   // Load output latch

    index = index + 5.12;
    if(index >= 2048) index = 0;
    GrisMeas = sin_tab[(int)index];
    spll1.u[0] = GrisMeas;
    SPLL_1ph_SOGI_F_FUNC(&spll1);

    DacaRegs.DACVALS.all = spll1.theta[0] * 651;
    DacbRegs.DACVALS.all = GrisMeas * 1000 + 1500;
//    float hnb = index/17.0;

    GpioDataRegs.GPACLEAR.bit.GPIO2=1;   // Load output latch


    CpuTimer0.InterruptCount++;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


void ConfigureDAC(void)
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



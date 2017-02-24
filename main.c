#include "F2806x_Device.h"
#include "F2806x_PieVect.h"
#include "math.h"

//Declaring function
interrupt void YourISR(void);
interrupt void ADC_ISR(void);

// Declare global variables for subroutine YourISR here.
int32 fclk = 90000000;			//system clock frequency
int32 ftmr = 1000;				//LED frequency
int32 fpwm = 30000;				//PWM freq
float32 duty = 0;  			//initialize duty cycle
float32 ial = 0;         			//declare motor current variable
Uint16 v;						//voltage from ADC as an unsigned int
float32 v_converted = 0;			//voltage converted to volts
float32 ii[1000];				//current array
float32 tt[1000];				//rotary position array
float32 vv[1000];
Uint32 t = 0;
Uint32 f = 0;
Uint32 l = 0;						//index for the current array
Uint32 counter = 0;				//Position counter from QEP
int32 signed_counter = 0;			//Conversion to signed of the above


// controller design parameters
float32 Vdc = 24;                           // power supply voltage
float32 T = 0.001;                          // controller period
float32 alpha=127.0865;						// alpha
float32 beta=751.8797;						// beta
float32 lambda_r = 58;                      // regulator bandwidth parameter
float32 lambda_e ;              // estimator bandwidth parameter
float32 Umax = 23;
// controller feedback gain values
float32 LL1;
float32 LL2;
float32 KK11;
float32 KK12;
float32 KK2;
// reference input
float32 rr[2] = {5*6.2831853, 0};
float32 r = 5*6.2831853;
// initial conditions
float32 xhat1 = 0;                       	// estimator states
float32 xhattemp = 0;
float32 xhat2 = 0;
float32 sigma = 0;                          // regulator state
float32 ustar = 0;
float32 u = 0;                              // first actuator input
float32 y = 0;                              // first sensor output


void main(void)
{
	lambda_e = 4*lambda_r;              // estimator bandwidth parameter
	LL1 = 2*lambda_e-alpha;
	LL2 = lambda_e*lambda_e-2*alpha*lambda_e+alpha*alpha;
	KK11 = (3*lambda_r*lambda_r)/beta;
	KK12 = (3*lambda_r-alpha)/beta;
	KK2 = lambda_r*lambda_r*lambda_r/beta;

	// Enable write register
	EALLOW;
	SysCtrlRegs.WDCR= 0x68; //watchdog disable
	if(SysCtrlRegs.PLLSTS.bit.DIVSEL==2 || SysCtrlRegs.PLLSTS.bit.DIVSEL==3) //SysClkInit
	{
		SysCtrlRegs.PLLSTS.bit.DIVSEL=0;
	}
	SysCtrlRegs.PLLSTS.bit.MCLKOFF=1;
	SysCtrlRegs.PLLCR.bit.DIV=9;
	while(!(SysCtrlRegs.PLLSTS.bit.PLLLOCKS)){}
	SysCtrlRegs.PLLSTS.bit.MCLKOFF=0;
	SysCtrlRegs.PLLSTS.bit.DIVSEL=3;

	//timer initialization
	CpuTimer0Regs.TCR.bit.TSS=1;
	CpuTimer0Regs.PRD.all=(fclk/ftmr)-1; //TimerInit
	CpuTimer0Regs.TPR.bit.TDDR=0;
	CpuTimer0Regs.TPRH.bit.TDDRH=0;
	CpuTimer0Regs.TCR.bit.TRB=1;
	CpuTimer0Regs.TCR.bit.TIE=1;
	CpuTimer0Regs.TCR.bit.TSS=0;

	//adc CLOCK
	SysCtrlRegs.PCLKCR0.bit.ADCENCLK=1;
	asm(" NOP");
	asm(" NOP");
	AdcRegs.ADCCTL2.bit.CLKDIV2EN=1;
	AdcRegs.ADCCTL2.bit.CLKDIV4EN=0;
	AdcRegs.ADCCTL2.bit.ADCNONOVERLAP=1;

	AdcRegs.ADCCTL1.bit.ADCPWDN=1;
	AdcRegs.ADCCTL1.bit.ADCBGPWD=1;
	AdcRegs.ADCCTL1.bit.ADCREFPWD=1;
	AdcRegs.ADCCTL1.bit.ADCENABLE=1;

	// wait for the initilizaiton
	Uint32 i;
	for (i=0; i<(0.0015*fclk); i++) { }

	//Conv config
	AdcRegs.ADCSOC0CTL.bit.CHSEL=2;
	AdcRegs.ADCSOC0CTL.bit.ACQPS=6;
	AdcRegs.ADCSOC0CTL.bit.TRIGSEL=7;


	//ADC interrupt initialization (pie vector)
	AdcRegs.INTSEL1N2.bit.INT1SEL=0;		//selecting SOC0
	AdcRegs.INTSEL1N2.bit.INT1E = 1;		//enabling soc0 (1)
	AdcRegs.ADCCTL1.bit.INTPULSEPOS=1;

	PieVectTable.ADCINT1= &ADC_ISR;
	PieCtrlRegs.PIEIER1.bit.INTx1=1;

	//pwm INIT
	GpioCtrlRegs.GPAMUX1.bit.GPIO0=1;//init pwm_A as ePWM1
	GpioCtrlRegs.GPAMUX1.bit.GPIO2=1;//init pwm_B
	//pwm Clock enable
	SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK=1;
	asm(" NOP");
	asm(" NOP");
	SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK=1;
	asm(" NOP");
	asm(" NOP");
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=0;
	asm(" NOP");
	asm(" NOP");

	//counter config PWM1
	EPwm1Regs.TBCTL.bit.CTRMODE=2;
	EPwm1Regs.TBPRD=fclk/fpwm/2;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV=0;
	EPwm1Regs.TBCTL.bit.CLKDIV=0;
	//counter config PWM2
	EPwm2Regs.TBCTL.bit.CTRMODE=2;
	EPwm2Regs.TBPRD=fclk/fpwm/2;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV=0;
	EPwm2Regs.TBCTL.bit.CLKDIV=0;

	//output actions
	EPwm1Regs.AQCTLA.bit.CAU = 1;
	EPwm1Regs.AQCTLA.bit.CAD = 2;
	EPwm2Regs.AQCTLA.bit.CAU = 2;
	EPwm2Regs.AQCTLA.bit.CAD = 1;

	//event trigger
	EPwm2Regs.ETSEL.bit.SOCASEL=2;
	EPwm2Regs.ETSEL.bit.SOCAEN=1;
	EPwm2Regs.ETPS.bit.SOCAPRD=1;

	//time base clock
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=1;
	asm(" NOP");
	asm(" NOP");

	//QEP configuration
	GpioCtrlRegs.GPAMUX2.bit.GPIO20=1; //set pin as eQEP1 input A
	GpioCtrlRegs.GPAMUX2.bit.GPIO21=1; //set pin as eQEP1 input B
	//GpioCtrlRegs.GPAMUX2.bit.GPIO23=1; //set pin as eQEP1 index

	SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK = 1;	//Enable clock module
	EQep1Regs.QPOSMAX = 0xFFFFFFFF;			//set max counter
	EQep1Regs.QPOSINIT = 0;					//initializing position
	EQep1Regs.QEPCTL.bit.QPEN = 1;			//enable module counter
	EQep1Regs.QEPCTL.bit.SWI = 1;

	//interrupt initialization (pie vector)
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
	PieVectTable.TINT0 = &YourISR;
	PieCtrlRegs.PIEIER1.bit.INTx7=1;
	
	//CPU interrupt
	IER=1;
	EINT;

	// Configure GPIO Port A (pin 1 and 3) to be output
	GpioCtrlRegs.GPADIR.bit.GPIO1=1;
	GpioCtrlRegs.GPADIR.bit.GPIO3=1;
	// Setting Port A Pin 1 and 3 to be high (connects to motor controller)
	GpioDataRegs.GPASET.bit.GPIO1=1;
	GpioDataRegs.GPASET.bit.GPIO3=1;

	SysCtrlRegs.WDCR= 0x28; //watchdog enable
	EDIS;

	while(1)
	{
		//servicing watch dog timer
		EALLOW;
		SysCtrlRegs.WDKEY = 0x55;
		SysCtrlRegs.WDKEY = 0xAA;
		EDIS;
	}
}


interrupt void ADC_ISR(void)
{	//ADC ISR to find the motor current
	v = AdcResult.ADCRESULT0;					//fetch voltage from ADC
	v_converted = 3.3 / 4096 * v;				//Convert to volts
	ial = 8.6806-5.2794 * v_converted;			//Use formula to find motor current

	if ((ial > 3.75) || (ial <-3.75)) {				//disables the motor if the current is too high
		GpioDataRegs.GPACLEAR.bit.GPIO1=1;
		GpioDataRegs.GPACLEAR.bit.GPIO3=1;
	}
	AdcRegs.ADCINTFLGCLR.bit.ADCINT1=1;
	PieCtrlRegs.PIEACK.all= PIEACK_GROUP1;		// acknowledge ISR
}

interrupt void YourISR(void)
{	//timer ISR to update duty cycle, QEP angular position, and the data that will be exported later

	EPwm1Regs.CMPA.half.CMPA = duty * 1500;		//set duty cycle of the pwm to desired value
	EPwm1Regs.CMPB = duty * 1500;
	EPwm2Regs.CMPA.half.CMPA = duty * 1500;
	EPwm2Regs.CMPB = duty * 1500;

	// estimator
	counter = EQep1Regs.QPOSCNT; 				//update the QEP counter variable
	signed_counter = (int32)counter;
	y = 2 * 3.1415 / 1000 * signed_counter; 	//convert to an angular position in radians

	// Euler's method
	// input calculation
	ustar = -KK11 * xhat1 - KK12 * xhat2 - KK2 * sigma;
	if(ustar > Umax) u = Umax;
	else if(ustar < (-1 * Umax)) u = -Umax;
	else u = ustar;
	// Integrator calculation
	if(ustar > Umax || ustar < (-1 * Umax) ) { }
	else sigma = sigma + T * (y-r);
	// sigma=sigma+T*(y-r);
	// updating xhat
	xhattemp = xhat1;
	xhat1 = xhat1 + T * xhat2 - T * LL1 * (xhat1 - y);
	xhat2 = xhat2 - T * alpha *xhat2 + T * beta * u - T * LL2 * (xhattemp - y);

	// actuator duty cycle update
	// voltage changes from -24 to +24, hence, divided by 48
	duty = (u + 24) / 48;

	// create a 1000 pt array 
	if(!(t % 1000)) {
		t = 0;
		r = rr[f];
		f++;
		f = f % 2;
	}
	//create a 1000 pt array for initial current and another for initial positions
	if(l<1000 && !(t % 2)) {}
		ii[l] = ial;
		tt[l] = y;
		vv[l] = u;
		l++;
	}

	t++;		// increment timer (keep track of array)
	AdcRegs.ADCSOCFRC1.bit.SOC0 = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;		// acnkowledging ISR
}

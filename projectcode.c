//#include "board.h"
//#include "fsl_debug_console.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
//#include "pin_mux.h"
#include <math.h>
#include "MKL46Z4.h"                    // NXP::Device:Startup
#define I2C_RELEASE_SDA_PORT PORTE
#define I2C_RELEASE_SCL_PORT PORTE
#define I2C_RELEASE_SDA_GPIO GPIOE
#define I2C_RELEASE_SDA_PIN 25U
#define I2C_RELEASE_SCL_GPIO GPIOE
#define I2C_RELEASE_SCL_PIN 24U
#define I2C_RELEASE_BUS_COUNT 100U
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ACCEL_I2C_CLK_SRC I2C0_CLK_SRC
#define ACCEL_I2C_CLK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)
typedef struct transfer {
			//uint32_t flags;            /*!< A transfer flag which controls the transfer. */  // dat 0 neu muon tao s
			uint8_t slaveAddress;      /*!< 7-bit slave address. */
			uint8_t direction; /*!< A transfer direction, read or write. */
			uint32_t subaddress;       /*!< A sub address. Transferred MSB first. */
			uint8_t subaddressSize;    /*!< A size of the command buffer. */
			uint8_t *volatile data;    /*!< A transfer buffer. */
			volatile size_t dataSize; 
} i2c_transfer; 
typedef enum slcd_phase_type{
    kSLCD_PhaseAActivate = 0x01U,  /*!< LCD waveform phase A activates. */
    kSLCD_PhaseBActivate = 0x02U,  /*!< LCD waveform phase B activates. */
    kSLCD_PhaseCActivate = 0x04U,  /*!< LCD waveform phase C activates. */
    kSLCD_PhaseDActivate = 0x08U,  /*!< LCD waveform phase D activates. */
    kSLCD_PhaseEActivate = 0x10U,  /*!< LCD waveform phase E activates. */
    kSLCD_PhaseFActivate = 0x20U,  /*!< LCD waveform phase F activates. */
    kSLCD_PhaseGActivate = 0x40U,  /*!< LCD waveform phase G activates. */
    kSLCD_PhaseHActivate = 0x80U,   /*!< LCD waveform phase H activates. */
	  LCD_Clear = 0x00U
} slcd_phase_t;
#define TransferDefaultFlag  (uint32_t)0x0,       /*!< A transfer starts with a start signal, stops with a stop signal. */
#define TransferNoStartFlag  (uint32_t)0x1,       /*!< A transfer starts without a start signal, only support write only or write+read with no start flag, do not support read only with no start flag. */
#define TransferRepeatedStartFlag (uint32_t)0x2, /*!< A transfer starts with a repeated start signal. */
#define TransferNoStopFlag  (uint32_t)0x4,        /*!< A transfer ends without a stop signal. */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void SetSimConfig(){
	  SIM->CLKDIV1 = 0x10010000U;
	// outdiv1: 1000: 2
	//outdiv4: 001: 2
	  SIM->SOPT2 = ((SIM->SOPT2 & ~SIM_SOPT2_PLLFLLSEL_MASK) | SIM_SOPT2_PLLFLLSEL(1U));  // lua chon clock la MCGPLLCLK 
	  SIM->SOPT1 = ((SIM->SOPT1 & ~SIM_SOPT1_OSC32KSEL_MASK) | SIM_SOPT1_OSC32KSEL(3U));  // lua chon clock cho LCD la LPO 1khz
}
void SetSimSafeDivs(void){
      SIM->CLKDIV1 = 0x10030000U;
	// set gia tri 1000 cho truong OUTDIV1: chia cho 2
	// set gia tri 011 cho truong OUTDIV4: chia cho 4
}
void SetInternalRefClkConfig(){
    MCG->C2 = (MCG->C2 & ~MCG_C2_IRCS_MASK) | (MCG_C2_IRCS(0));  // lua chon slow internal clock
    MCG->C1 = (MCG->C1 & ~(MCG_C1_IRCLKEN_MASK | MCG_C1_IREFSTEN_MASK)) | (uint8_t)MCG_C1_IRCLKEN_MASK; //MCGIRCLK active
    //Doi cho co MCGIRCLK duoc set tren thanh ghi status  
        while (((MCG->S & MCG_S_IRCST_MASK) >> MCG_S_IRCST_SHIFT) != 0){}
}
void PLL_Init(){  
    MCG->C2 &= ~MCG_C2_LP_MASK; // Disable lowpower. 
    MCG->C1 = ((MCG->C1 & ~(MCG_C1_CLKS_MASK | MCG_C1_IREFS_MASK)) | MCG_C1_CLKS(2U));
		// dat 2 bit 7-6 cua C1 ve khong( lua chon PLL hoac FLL tuy thuoc vao bit PLLS), sau do set lai bang 10 => lua chon external clock 
    // Doi cho den khi thanh ghi status mcg cat nhap gia tri theo gia tri cua thanh ghi c1 da setting o phia trens
    while ((MCG->S & (MCG_S_IREFST_MASK | MCG_S_CLKST_MASK)) !=(MCG_S_IREFST(0U) | MCG_S_CLKST(2U))){}
    MCG->C6 &= ~MCG_C6_PLLS_MASK; // tam thoi lua chon FLL 
			while (MCG->S & MCG_S_PLLST_MASK){}  // doi cho den khi FLL duoc chon 
    {
    uint8_t mcg_c5 = 0U;
    mcg_c5 |= MCG_C5_PRDIV0(0x1U); 
		// divide factor cho PLL tu nguon external clock = 2, clock cua PLL = external clock/2;
    MCG->C5 = mcg_c5; 
			MCG->C6 = (MCG->C6 & ~MCG_C6_VDIV0_MASK) | MCG_C6_VDIV0(0x0U); // Multiply factory cho VCO output of PLL is 24, VCO = PLL *24;
    MCG->C5 |= ((uint32_t)MCG_C5_PLLCLKEN0_MASK | (uint32_t)0U);  // MCGPLLCLK is active, enable module PLL doc lap voi PLLS
    while (!(MCG->S & MCG_S_LOCK0_MASK)){}  // doi cho den khi PLL duoc enable
    }
    MCG->C6 |= MCG_C6_PLLS_MASK;  // chon PLL cho dau ra cua MCGOUTCLOCK
    while (!(MCG->S & MCG_S_PLLST_MASK)){}  // doi co den khi PLL duoc chon
			MCG->C1 = (MCG->C1 & ~MCG_C1_CLKS_MASK) | MCG_C1_CLKS(0); // quay lai setting thanh ghi C1: CLKS = 00 => lua chon nguon tu PLL
	  while (((MCG->S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 3){}  // doi cho den khi output la PLL
}
void InitOsc0(){  // Ham nay enable OSCERCLK, dat dai tan so ve che do hoat dong cua giao dong ky, va lua chon nguon clock tham chieu ben ngoai cua module MCG tu giao dong ky
	  OSC0->CR = 0x80;  // 
	  /* 
	bit 7: 1 External reference clock is enabled
	bit 5: 0 External reference clock is disabled in Stop mode
	bit 3:Oscillator 2 pF Capacitor Load Configure: 0 Disable the selection.
	bit 2:Oscillator 4 pF Capacitor Load Configure: 0 Disable the selection.
	bit 1:Oscillator 8 pF Capacitor Load Configure: 0 Disable the selection
	bit 0:Oscillator 16 pF Capacitor Load Configure: 0 Disable the selection.
	  */ 
		MCG->C2 = 0x94;
		/*  
		Cac bit cua thanh ghi C2:
		Bit 7:  1 Generate a reset request on a loss of OSC0 external reference clock 
		Bit 5-4: 01 Encoding 1 — High frequency range selected for the crystal oscillator: 3-32 Mhz
		Bit 3: 0 Configure crystal oscillator for low-power operation.
		Bit 2: External Reference Select: 1 Oscillator requested.
		Bit 1: Low Power Select: 0 FLL or PLL is not disabled in bypass modes
		Bit 0: Internal Reference Clock Select: 0 Slow internal reference clock selected.
		*/
    if ((OSC0->CR & OSC_CR_ERCLKEN_MASK))
    {  // Kiem tra bit 1 thanh ghi OSC0->S xem qua trinh thiet lap OSC da hoan thanh chua ?, bit nay duoc dat gia tri 1 khi qua trinh thiet lap hoan thanh
        while (!(MCG->S & MCG_S_OSCINIT0_MASK)){}
    }
}
void SetFllExtRefDiv(void){// ham any lua chon nguon clock cho MCGOUTCLK la FLL hoac PLL, su dung nguon clock cho FLL la slow internal: 32khz, MCGIRCLK inactive tam thoi disable
	  MCG->C1 = 0x4;
	  /*
	Bit 7-6: Selects the clock source for MCGOUTCLK: Encoding 0 — Output of FLL or PLL is selected (depends on PLLS control bit)
	Bit 5-3: FLL External Reference Divider 000, ta khong su dung nguon external
  Bit 2: Internal Reference Select: 1 The slow internal reference clock is selected
	Bit 1: Internal Reference Clock Enable: 0 MCGIRCLK inactive
	Bit 0: Controls whether or not the internal reference clock remains enabled when the MCG enters Stop mode: 0 disable
	*/
}
static void i2c_release_bus_delay(void){
    uint32_t i = 0;
    for (i = 0; i < 100U; i++)
    {
        __NOP();
    }
}
void I2C_ReleaseBus(void){
  	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	  PORTE->PCR[24] = (uint32_t)(1<<0|1<<1|1<<8);
		PORTE->PCR[25] = (uint32_t)(1<<0|1<<1|1<<8);
	  GPIOE->PDDR |= (uint32_t)(1<<24|1<<25);
	  GPIOE->PDOR |= (uint32_t)(1<<24|1<<25);
     // gui tin hieu start
	  GPIOE->PCOR |= (uint32_t)(1<<25);
    i2c_release_bus_delay();
    // Gui 9 xung tren SCL va giu SDA high 
    for (int i = 0; i < 9; i++)
    {
			  GPIOE->PCOR |= (uint32_t)(1<<24);
        i2c_release_bus_delay(); 

			  GPIOE->PSOR |= (uint32_t)(1<<25);
        i2c_release_bus_delay();

			  GPIOE->PSOR |= (uint32_t)(1<<24);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    // Send stop 
		GPIOE->PCOR |= (uint32_t)(1<<24);
    i2c_release_bus_delay();

		GPIOE->PCOR |= (uint32_t)(1<<25);
    i2c_release_bus_delay();

		GPIOE->PSOR |= (uint32_t)(1<<24);
    i2c_release_bus_delay();

	  GPIOE->PSOR |= (uint32_t)(1<<25);
    i2c_release_bus_delay();
}
int volatile state=0;
void MY_SLCD_StartDisplay(){
    LCD->GCR |= LCD_GCR_LCDEN_MASK;
}
void MY_LCD_StopDisplay(){
    LCD->GCR &= ~LCD_GCR_LCDEN_MASK;
}
void MY_SLCD_SetBackPlanePhase(LCD_Type* base,uint32_t pinIndx, slcd_phase_t phase);
void MY_SLCD_SetFrontPlaneSegments(LCD_Type *base, uint32_t pinIndx, uint8_t operation);
void LCD_clear();
void LCD_Init(LCD_Type *base);
void display_number(int x,int digit);
void display_outofrange();
void display_decimal(int x);
uint32_t MY_I2C_MasterGetStatusFlags(I2C_Type *base);
int MY_I2C_MasterWriteBlocking(I2C_Type *base, const uint8_t *txBuff, size_t txSize);
void I2C_ReleaseBus(void);
void I2C_WriteAccelReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);
void I2C_ReadAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);
int MY_I2C_MasterTransferBlocking(I2C_Type* base, i2c_transfer * xfer);
int MY_I2C_MasterRepeatedStart(I2C_Type *base, uint8_t address, uint8_t direction);
void I2C_MasterStopSignal(I2C_Type* base);
void MY_I2C_MasterReadBlocking(I2C_Type *base, uint8_t *rxBuff, size_t rxSize);
void I2C_MasterStartSignal(I2C_Type* base, uint8_t address, uint8_t direction);
void I2C_ConfigurePins(void){ // Cap clock va configure cac chan PTE24 va PTE25
	SIM->SCGC5 = SIM->SCGC5 | SIM_SCGC5_PORTE_MASK; //clock to PTE24 and PTE25 for I2C0
  PORTE->PCR[24] = (((1u<<10) & ~(1u<<9)) | (1u<<8)|(1u<<0)|(1u<<2)|(1u<<1))&(~(1u<<4))&(~(1u<<6));
	PORTE->PCR[25] = (((1u<<10) & ~(1u<<9)) | (1u<<8)|(1u<<0)|(1u<<2)|(1u<<1))&(~(1u<<4))&(~(1u<<6));
}
void I2C_WriteAccelReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value){
    i2c_transfer masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = 0;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &value;
    masterXfer.dataSize = 1;
    MY_I2C_MasterTransferBlocking(I2C0,&masterXfer);
	   //PRINTF("Dang gui du lieu den cam bien \r\n");
     while(!((I2C0->S)&I2C_S_TCF_MASK)){
    } 	
		 //PRINTF("Da nhan duoc du lieu tu cam bien \r\n");
}

void I2C_ReadAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize){
    i2c_transfer masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = 1;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = rxBuff;
    masterXfer.dataSize = rxSize;
	  MY_I2C_MasterTransferBlocking(I2C0,&masterXfer);
     while(!((I2C0->S)&I2C_S_TCF_MASK)){
    } 
    //PRINTF("DA out ra khoi vong while \r\n");
}
void Clear_flags(){ 
     uint32_t t =(I2C_S_ARBL_MASK|I2C_S_IICIF_MASK)|(I2C_FLT_STOPF_MASK << 8);  // lay gia tri co co phat hien tin hieu stop o thanh ghi FLT
		if(t&(I2C_FLT_STOPF_MASK << 8)){  // neu detect duoc tin hieu stop
		     // xoa co stop neu co o thanh ghi FLT
        I2C0->FLT |= (uint8_t)(t >> 8U);
		}
		// xoa tat ca cac bit thanh ghi status, clear bit ARBL va set bit interrupt detect
	  I2C0->S = (uint8_t)t;
}
void Init_Master(){ 
			SIM->SCGC4 = SIM->SCGC4 | SIM_SCGC4_I2C0_MASK;
			/* Reset the module. */
			I2C0->A1 = 0;
			I2C0->F = 0;
			I2C0->C1 = 0;
			I2C0->S = 0xFFU;
			I2C0->C2 = 0;
			I2C0->FLT = 0x50U;  
			I2C0->RA = 0;
			// Disable module I2C
			I2C0->C1 &= ~(I2C_C1_IICEN_MASK);
	    // Clear Flags
			Clear_flags();
	    I2C0->F = I2C_F_MULT(0U) | I2C_F_ICR(31U);  // Set baurd rate for i2c module
			/* Read out the FLT register. */
			uint8_t fltReg =  0;
			//fltReg = I2C0->FLT;
			//fltReg &= ~(I2C_FLT_SHEN_MASK); // disable stop hold
			//fltReg &= ~(I2C_FLT_FLT_MASK);
      //fltReg |= I2C_FLT_FLT(0U);  // glitchFilterWidth = 0;
      I2C0->FLT = fltReg;
      I2C0->C1 = I2C_C1_IICEN(1); // Enable module I2C
} 
int32_t volatile mstick = 0;
int32_t volatile blink_mstick=0;
void init_systick(){
	   NVIC_SetPriority(SysTick_IRQn,(1<<7)-1);
     SysTick->LOAD = SystemCoreClock/1000; 
	   SysTick->CTRL = 1<<2 |  1<<1 | 1<<0;
}
int number_of_step = 0;
void toggle_green_led(){
	 PTD->PDOR^= (uint8_t)(1<<5);
}
void SysTick_Handler (void){
	 if(blink_mstick>=1000){
	 blink_mstick = 0;
   toggle_green_led();
	 }
   mstick++;
	 blink_mstick++;
}
void delay(uint32_t tick){
   while(mstick < tick);
	 mstick = 0;
}
void green_led_init(){
	 SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	 PORTD->PCR[5] =  1<<8; // dat chan 5 cong D o che do IO
	 PTD->PDDR =  1<<5; // dat chan 5 o che do doc va ghi 
	 PTD->PDOR &= ~(1<<5); // set led chan 5 sang
}
void red_led_init(){
   SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
   PORTE->PCR[29] |= 1<<8;
	 PTE->PDDR = 1<<29;
	 PTE->PDOR &= ~(1<<29);
	 SysTick->CTRL &= (int32_t)~(1<<0);
}
void toggle_red_led(){
   PTE->PDOR^= (uint32_t)(1<<29);
}
void portc_portd_nvic_init(){
	 NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
	 SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	 SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
   PORTC->PCR[3]   |=  ((uint8_t)(1<<0)|(uint8_t)(1<<1)|PORT_PCR_IRQC(0xA)|PORT_PCR_MUX(1));
	 PORTC->PCR[12]  |=  ((uint8_t)(1<<0)|(uint8_t)(1<<1)|PORT_PCR_IRQC(0xA)|PORT_PCR_MUX(1));
   NVIC_SetPriority(PORTC_PORTD_IRQn,(1<<7));
	 NVIC_EnableIRQ(PORTC_PORTD_IRQn);
}
void PORTC_PORTD_IRQHandler(){
	 if(PORTC->PCR[3]&(uint32_t)(1<<24)){
	 SysTick->CTRL ^= (int32_t)(1<<0);
	 if(state==0){
	  state = 1;
		MY_SLCD_StartDisplay();
		display_decimal(0);
	 }
	 else if(state==1){
		 number_of_step = 0;
	   state=0;
		 MY_LCD_StopDisplay();
	 }
	 PORTC->PCR[3] |= PORT_PCR_ISF_MASK; 
	 toggle_red_led();
 }
   else if(PORTC->PCR[12]&(uint32_t)(1<<24)){
	 number_of_step = 0;
	 display_decimal(0);
	 PORTC->PCR[12] |= PORT_PCR_ISF_MASK;
	 }
}
int MY_I2C_MasterTransferBlocking(I2C_Type* base, i2c_transfer * xfer);
int main(void){   
	  int number_of_detect = 0;
	  int is_detecting_step=0;
	  int number_of_peak_detect = 0;
	  int head =0;
    int tail =1;
	  float average_accel = 0;
		float current_sample =0;
		float pre_sample=0;
		float accel=0;	
  	char cmd[10];
	  float number_of_sample = 1;
	  float peak_value=0;
    //BOARD_InitPins();
	  //BOARD_InitDebugConsole();
    //start BOARD_BootClockRUN();
	  SetSimSafeDivs();
    InitOsc0();
    SetFllExtRefDiv();
		PLL_Init();
		SetInternalRefClkConfig();
	  SetSimConfig();
    SystemCoreClock = 48000000U;
		// end BOARD_BootClockRUN();
		
	  I2C_ReleaseBus();
    I2C_ConfigurePins();
	  uint32_t time_out = 0;
		
		//PRINTF("GIA TRI DANG CAN TEST: %d\r\n",kOSC_ErClkEnable);
	  LCD_Init(LCD);
	  green_led_init();
	  init_systick();
	  red_led_init();
	  portc_portd_nvic_init();
    uint8_t rx_buffer[] = {0,0,0,0,0,0};
		Init_Master();
		MY_SLCD_SetBackPlanePhase(LCD, 40, kSLCD_PhaseAActivate); /* SLCD COM0 --- LCD_P40. */
    MY_SLCD_SetBackPlanePhase(LCD, 52, kSLCD_PhaseBActivate); /* SLCD COM1 --- LCD_P52. */
    MY_SLCD_SetBackPlanePhase(LCD, 19, kSLCD_PhaseCActivate); /* SLCD COM2 --- LCD_P19. */
    MY_SLCD_SetBackPlanePhase(LCD, 18, kSLCD_PhaseDActivate); /* SLCD COM3 --- LCD_P18. */
	  I2C_ReadAccelRegs(I2C0,0x1D,0x0D,rx_buffer,1);
		uint8_t databyte =0;
		I2C_WriteAccelReg(I2C0,0x1D,0x2AU,databyte);  // disable accelerometer, ghi vao thang ghi control 1
		databyte = (uint8_t)(1<<0)+ (uint8_t)(1<<4);
		I2C_WriteAccelReg(I2C0,0x1D,0x0EU,databyte);  // ghi bit 1 vao HPF_Out va FS0 vao thanh ghi XYZ_DATA_CFG Register
		databyte = (uint8_t)(1<<0)+(uint8_t)(1<<4);  
		I2C_WriteAccelReg(I2C0,0x1D,0x0F,databyte);  // write to HP_FILTER_CUTOFF register
		databyte = 0x0D;
		I2C_WriteAccelReg(I2C0,0x1D,0x2AU,databyte);   // ghi 00001101 vao thanh ghi control 1
	  int16_t x,y,z =0;
		uint32_t timeout1, timeout2 = 0;
		int trigger_timeout1,trigger_timeout2=0;
	  float substract= 0;
		// read first sample data 
    I2C_ReadAccelRegs(I2C0,0x1D,0x01,rx_buffer,6);
    x = (int16_t)(256U*rx_buffer[0]|rx_buffer[1])/4U; 
	  y = (int16_t)(256u*rx_buffer[2]|rx_buffer[3])/4U;
		z = (int16_t)(256U*rx_buffer[4]|rx_buffer[5])/4U;
		x*=2; y*=2;
		accel = (float)x*x+(float)y*y+(float)z*z;
	  current_sample = sqrt(accel);
		peak_value = current_sample;
		//queue_data[0]  = current_sample;  // first sample data to queue
		average_accel  = current_sample; // the first value of average value
		//average_of_peak_value = current_sample;
    while (1)
    {
			I2C_ReadAccelRegs(I2C0,0x1D,0x01,rx_buffer,6);
			x = (int16_t)(256U*rx_buffer[0]|rx_buffer[1])/4U; 
			y = (int16_t)(256u*rx_buffer[2]|rx_buffer[3])/4U;
			z = (int16_t)(256U*rx_buffer[4]|rx_buffer[5])/4U;
			x*=2; y*=2;
			accel = (float)x*x+(float)y*y;
			current_sample = sqrt(accel);
		  if(trigger_timeout1 == 1&&is_detecting_step == 0){
			   timeout1++;
				 if(timeout1 > 150){number_of_peak_detect =0;
					 timeout1 = 0;
					 trigger_timeout1 = 0;
				 }
			}
		  if(is_detecting_step ==0&&number_of_peak_detect <10){
			   if(current_sample>1000){
					  if(trigger_timeout1 == 0)trigger_timeout1 =1;
				    number_of_peak_detect++;
				 }
			}
			else if(is_detecting_step ==0&&number_of_peak_detect>=10){is_detecting_step = 1;
				  timeout1 = 0;
				  trigger_timeout1 = 0;
				  number_of_peak_detect = 0;
			}
			else if(is_detecting_step==1){
			   timeout2++;
				 if(timeout2>300){
						   timeout2 = 0;
							 is_detecting_step = 0;
					     number_of_detect = 0;
					     number_of_peak_detect = 0;
				 }
			   if(current_sample <= 150&&number_of_detect<10){
				      number_of_detect++;
				 }
				 else if(number_of_detect>=10){
					     if(timeout2 >=100){
				       LCD_clear();
					     number_of_step++;
					     display_decimal(number_of_step);
							 }
							 timeout2 = 0;
							 is_detecting_step = 0;
					     number_of_detect = 0;
					     number_of_peak_detect = 0;
				 }
			}
	    
			sprintf(cmd,"%.2f",current_sample);
			//PRINTF("%s\r\n",cmd);
			delay(1);
    }
}
uint32_t MY_I2C_MasterGetStatusFlags(I2C_Type *base){
		// kiem tra thanh ghi status{
    uint32_t statusFlags = base->S;
    /* if (base->FLT & I2C_FLT_STOPF_MASK)  // kiem tra xem co phat hien tin hieu Stop tren Bus I2C khong
    {
        statusFlags |= (I2C_FLT_STOPF_MASK<<8);  // danh dau bit phat hien tin hieu stop o bit so 14 (bat dau tu bit so 0)
    } */
    return statusFlags;
}
void I2C_MasterStartSignal(I2C_Type* base, uint8_t address, uint8_t direction){  // Generate tin hieu start va gui dia chi cua  ngoai vi + mode:(write=0/read=1)
    uint8_t result =0;
    uint32_t statusFlags = MY_I2C_MasterGetStatusFlags(base);  //  lay gia tri thanh ghi status kem theo 2 bit detect start signal va stop signal dat o bit 12 va 14
	   if (statusFlags & I2C_S_BUSY_MASK) // kiem tra bus co ban khong 
    {
        result = 1;
				//PRINTF("Bus Hien tai dang busy \r\n");
    }
		  else
    {
        /* Send the START signal. */
        base->C1 |= I2C_C1_MST_MASK | I2C_C1_TX_MASK;  // chuyen sang che do master va transmit mode
			  base->D = (((uint32_t)address) << 1U |direction);  // day du lieu gom dia chi ngoai vi + mode hoat dong
		}
}
void I2C_MasterStopSignal(I2C_Type* base){
    base->C1 &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK | I2C_C1_TXAK_MASK); // chuyen sang mode slave,  Receiver, va co gui tin hieu ack khi nhan data
    while (base->S & I2C_S_BUSY_MASK) // Doi den khi bus duoc giai phong 
    {
    }
} 
void MY_I2C_MasterReadBlocking(I2C_Type *base, uint8_t *rxBuff, size_t rxSize)  // Ham nay thuc hien viec doc nhieu byte du lieu vao rxBuff,
{																																																//neu bo dem da het,no se gui nak va co the kem theo stop signal tuy theo co flags
    volatile uint8_t dummy = 0;
    while (!(base->S & I2C_S_TCF_MASK))     // kiem tra neu qua trinh nhan lan truoc da hoan thanh chua ?
    {
    }
    base->S = I2C_S_IICIF_MASK;  // xoa co ngat
    base->C1 &= ~(I2C_C1_TX_MASK | I2C_C1_TXAK_MASK); // dat master o trang thai nhan va co gui ack
    /* If rxSize equals 1, configure to send NAK. */
    if (rxSize == 1)    // neu buffer bo nhan chi con du de doc 1 byte data nua
    {
        /* Issue NACK on read. */
        base->C1 |= I2C_C1_TXAK_MASK;  // dat master o che do gui nak de ket thuc phien doc du lieu, nak se duoc gui sau khi nhan duoc data byte tai lan gui tiep theo
    }
    dummy = base->D;  // gia doc du lieu tu thanh ghi data cua i2c
    while ((rxSize--))
    {
        while (!(base->S & I2C_S_IICIF_MASK)){}
        base->S = I2C_S_IICIF_MASK;
        /* Single byte use case. */
        if (rxSize == 0)  // da truyen di nak roi
        { 
            I2C_MasterStopSignal(base);  // tao tin hieu stop de ket thuc phien truyen  
        }
        if (rxSize == 1)  // neu buffer ben nhan chi con du de doc 1 byte nua
        {
           // set thanh ghi C1 de truyen nak sau khi nhan duoc byte du lieu tiep theo
            base->C1 |= I2C_C1_TXAK_MASK;  // dat master o che do gui nak de ket thuc phien doc sau khi doc byte data cuoi cung 
        }
        //Read from the data register. 
        *rxBuff++ = base->D;  // doc du lieu tu thanh ghi data cua i2c module
    }
}
int MY_I2C_MasterWriteBlocking(I2C_Type *base, const uint8_t *txBuff, size_t txSize){  
	  uint8_t statusFlags = 0;
    uint8_t result  =0;
    while (!(base->S & I2C_S_TCF_MASK)) // Cho cho den khi Transfet complete, bit 7 thanh ghi status
    {
    }
    base->S = I2C_S_IICIF_MASK;  // Xoa co ngat, bit 1 thanh ghi status
    base->C1 |= I2C_C1_TX_MASK;  // chuyen sang mode Trasmit
		 while (txSize--)
    {
        base->D = *txBuff++;  // Dat byte gui vao thanh ghi data cua i2c module
        while (!(base->S & I2C_S_IICIF_MASK))  // Doi cho den khi co ngat, bit 1 thanh ghi status
        {
        }
				statusFlags = base->S;  // Lay trang thai sau khi gui thanh ghi status
        base->S = I2C_S_IICIF_MASK;  // Xoa co ngat, bit 1 thanh ghi status
        if (statusFlags & I2C_S_ARBL_MASK)  // kiem  tra thanh ghi status xem co bi xung dot khong 
        {   //PRINTF("Co loi xung dot tren bus i2c khi truyen databyte \r\n");
            base->S = I2C_S_ARBL_MASK;  // neu co xung dot thi clear bit ARBL thanh ghi Status
					  result = 1;          // Danh dau ket qua truyen da bi xung dot
        }
        if ((statusFlags & I2C_S_RXAK_MASK) && txSize)  // neu nhan duoc tin hieu nak, kiem tra xem con du lieu can phai gui khong,
        {																								// neu nhu co thi tuc la ben nhan khong nhan duoc du lieu
							
																												// neu nhu khong thi tuc la ben nhan muon ket thuc viec doc du lieu, dieu nay chi xay ra khi thiet bi goi ham nay dang o slave mode
            base->S = I2C_S_RXAK_MASK;   
					  //PRINTF("Nhan duoc NAK khi truyen databyte \r\n");
            result =2;                        					// danh dau da nhan duoc nak
        }

        if (result != 0)break;  // neu co xung dot hoac ben nhan khong nhan duoc data, dung viec gui thoat ra khoi vong while
		}
		if (result == 0||result == 2) // neu nhan duoc nak do ben nhan khong nhan duoc du lieu hoac neu qua trinh truyen tat ca data trong buff thanh cong
    {
        base->S = I2C_S_IICIF_MASK;  // Xoa co ngat
        // Send stop. 
        I2C_MasterStopSignal(base); // gui tin hieu stop, dung qua trinh gui du lieu
    }
	  return result;
}
int MY_I2C_MasterRepeatedStart(I2C_Type *base, uint8_t address, uint8_t direction){ // direction: read=1, write =0
    uint8_t savedMult;
    uint32_t statusFlags = MY_I2C_MasterGetStatusFlags(base);
    uint8_t time_delay = 6;
    if ((statusFlags & I2C_S_BUSY_MASK) && ((base->C1 & I2C_C1_MST_MASK) == 0)) // kiem tra neu bus dang ban va dang o slave mode
    {
         return 1;
    }
    else
    {
        savedMult = base->F;  // luu lai trang thai thanh ghi F
        //base->F = savedMult & (~I2C_F_MULT_MASK); // dat gia tri cua mult = 00 vao thanh ghi F
        // gui lai tin hieu start
        base->C1 |= I2C_C1_RSTA_MASK | I2C_C1_TX_MASK;  // tao tin hieu start va chuyens sang che do transmit
        // Dat lai trang thai cua thanh ghi F
        base->F = savedMult;  // gan lai gia tri mult cu
        // tao thoi gian delay de doi tin hieu Restart
         while (time_delay--)
        {
            __NOP();
        } 
        base->D = (((uint32_t)address) << 1U | direction);  //  gui dia chi slave cung voi mode Read
			}
}
int MY_I2C_MasterTransferBlocking(I2C_Type* base, i2c_transfer * xfer){
	   uint8_t direction = xfer->direction; 
	   Clear_flags();
    // cho den khi thanh ghi data san sang
    while (!(base->S & I2C_S_TCF_MASK)){};  
    if ((xfer->subaddressSize > 0) && (xfer->direction == 1))  // kiem tra xem neu can gui dia chi thanh ghi va mode hoat dong cua master la read
    {
        direction = 0;  // set mode hoat dong tam thoi la viet, can phai gui dia chi cua thanh ghi truoc khi muon doc data tu thanh ghi
    }
        I2C_MasterStartSignal(base, xfer->slaveAddress, direction);  // ham nay vua tao tin hieu start, vua gui dia chi cua ngoai vi + mode hoat dong    
        while (!(base->S & I2C_S_IICIF_MASK))  // cho cho den khi co ngat xay ra sau khi da gui dia chi cua ngoai vi 
        {
        } 
						if (base->S & I2C_S_ARBL_MASK){  // neu phat hien co xung dot thi ngung viec doc hay viet
						/* Clear arbitration lost flag. */
					  //PRINTF("PHAT HIEN XUNG DOT KHI TRUYEN DIA CHI CUA SLAVE \r\n");
						base->S = I2C_S_ARBL_MASK;   
						return 1;
						}
						/* Check NAK */
						else if (base->S&I2C_S_RXAK_MASK){  // neu nhan duoc ack thi gui tin hieu stop, ngung viec doc/viet
						//PRINTF("NHAN DUOC NAK KHI TRUYEN DIA CHI CUA SLAVE \r\n");
						I2C_MasterStopSignal(base);
						return 1;
						}	
 
    /* Send subaddress. */  //Gui dia chi cua thanh ghi muon doc/viet cua ngoai vi
    if (xfer->subaddressSize)  // kiem tra xem neu van can phai gui dia chi cua thanh ghi cua slave
    {
        do
        {
            /* Clear interrupt pending flag. */
            base->S = I2C_S_IICIF_MASK;  // xoa co ngat sau khi nhan duoc ack tu lan gui dia chi lan truoc
            xfer->subaddressSize--;  
            base->D = ((xfer->subaddress) >> (8 * xfer->subaddressSize));  // neu kich co cua dia chi la 1 byte, thi tuc la truyen di xfer->subaddress
            /* Wait until data transfer complete. */
           while (!(base->S & I2C_S_IICIF_MASK)){} // doi cho den khi nhan duoc ack cua byte dia chi thanh ghi da gui
            /* Check if there's transfer error. */
						if (base->S & I2C_S_ARBL_MASK){
					  //PRINTF("PHAT HIEN XUNG DOT KHI TRUYEN DIA CHI CUA THANH GHI \r\n");
						/* Clear arbitration lost flag. */
						base->S = I2C_S_ARBL_MASK;    // kiem tra co bi xung dot hay khong 
						return 1;
						}
						/* Check NAK */
						else if (base->S&I2C_S_RXAK_MASK){
				    //PRINTF("NHAN DUOC NAK SAU KHI TRUYEN DIA CHI CUA THANH GHI \r\n");
						I2C_MasterStopSignal(base);  // kiem tra neu nhan duoc nak
						return 1;
						}	
        } while (xfer->subaddressSize > 0); // thuc hien xong vong while nay se gui tat ca dia chi subaddress

        if (xfer->direction == 1)   // sau khi da gui tat ca dia chi subadress, kiem tra xem neu mode truyen vao ham TransferBlocking la doc
        {
            /* Clear pending flag. */
            base->S = I2C_S_IICIF_MASK;  // xoa co ngat
            /* Send repeated start and slave address. */
            MY_I2C_MasterRepeatedStart(base, xfer->slaveAddress,1);  // gui tin hieu repeatedn start va dia chi cua ngoai vi + mode doc
            /* Wait until data transfer complete. */ 
            while (!(base->S & I2C_S_IICIF_MASK)){}  // cho den khi nhan duoc ack sau khi gui lai tin hieu start va dia chi slave
            /* Check if there's transfer error. */
            if (base->S & I2C_S_ARBL_MASK){
						//PRINTF("PHAT HIEN XUNG DOT KHI TRUYEN TIN HIEU RESTART + DIA CHI SLAVE \r\n");
						base->S = I2C_S_ARBL_MASK;      // xoa co arbitration lost
						return 1;
						}
						/* Check NAK */
						else if (base->S&I2C_S_RXAK_MASK){
						//PRINTF("NHAN DUOC NAK KHI TRUYEN TIN HIEU RESTART + DIA CHI SLAVE \r\n");
						I2C_MasterStopSignal(base); // neu nhan nak gui stop signal	
						return 1;
						}	
        }
    }  // sau khi ket thuc khoi lenh nay, tat cac dia chi trong subadress neu co thi deu duoc gui di het, neu mode cua Ham TransferBlocking la doc thi se gui them ca repeated start signal + dia chi slave
    /* Transmit data. */
		// sau khi ket thuc qua trinh gui dia chi thanh ghi o mode viet, hoac qua trinh gui dia chi thanh ghi + repeated start signal va dia chi slave o mode doc
    if ((xfer->direction == 0) && (xfer->dataSize > 0)) // neu dang o mode viet va neu co du lieu trong tx_buffer 
    {
        /* Send Data. */
         MY_I2C_MasterWriteBlocking(base, xfer->data, xfer->dataSize);
    }
    /* Receive Data. */
    if ((xfer->direction == 1) && (xfer->dataSize > 0)) // kiem tra xem neu o mode doc va rx_buffer con trong 
    {
         MY_I2C_MasterReadBlocking(base, xfer->data, xfer->dataSize);
    }
    return 0;
}
void MY_SLCD_SetBackPlanePhase(LCD_Type* base,uint32_t pinIndx, slcd_phase_t phase){
		base->WF8B[pinIndx]= phase; 
}
void MY_SLCD_SetFrontPlaneSegments(LCD_Type *base, uint32_t pinIndx, uint8_t operation){
			base->WF8B[pinIndx] = operation;  
}
void LCD_clear(){
      MY_SLCD_SetBackPlanePhase(LCD,11,LCD_Clear);
			MY_SLCD_SetBackPlanePhase(LCD,10,LCD_Clear);
			MY_SLCD_SetBackPlanePhase(LCD,38,LCD_Clear);
			MY_SLCD_SetBackPlanePhase(LCD,53,LCD_Clear);
			MY_SLCD_SetBackPlanePhase(LCD,8,LCD_Clear);
			MY_SLCD_SetBackPlanePhase(LCD,7,LCD_Clear);
      MY_SLCD_SetBackPlanePhase(LCD,17,LCD_Clear);
			MY_SLCD_SetBackPlanePhase(LCD,37,LCD_Clear);
}
void LCD_Init(LCD_Type *base){
	  SIM->SCGC5 |= SIM_SCGC5_SLCD_MASK;  // cap clock cho module LCD
		base->GCR = 0x8B0004B;
		// **Note: parameter for setting:
		// Duty: su dung 4 dang song => 011
		// LCD Clock Prescaler: LCD frame frequency = LCD_clock/(((DUTY=3)+1) * 8 * (4 + LCLK=1)) * Y => 001
	  // Source: Select altenate source => 1: MCGIRCLK
		// LCDEN: LCD Driver disable
		// LCDSTP: 0: LCD driver, charge pump, resistor bias network, and voltage regulator continue while in Stop mode
		// LCDDOZE: 0
		// FFR: 0 standard frame rate
		// ALTSOURCE: 0 Select Alternate Clock Source 1
		// 13–12 ALTDIV: 0 Divide factor = 1 (No divide)
		// FDCIEN:  0 No interrupt request is generated when fault detection is completed
		// PADSAFE: 0 LCD frontplane and backplane functions enabled
		// VSUPPLY: 0 Drive VLL3 internally from VDD
		// CPSEL: 1 LCD charge pump is selected. Resistor network disabled. (The internal 1/3-bias is forced.)
		// LADJ: 11 Slowest clock source for charge pump (LCD glass capacitance 1000 pF or 500pF or lower if FFR is set )
		// RVTRIM: 1000
		// RVEN: Regulated voltage disabled.
		base->AR = (uint32_t)0;
		// BLINK: 0 disable blinking
		// ALT: 0 normal display mode
		// BLANK:0 normal or alternate display mode
		// BMODE: 0 Display blank during the blink period
		// BRATE: Blink rate = LCD clock/2^(12+0)
    base->BPEN[0] = 0x000c0000U;  // set lcd  19,18 pin as backplane pin
    base->BPEN[1] = 0x00100100U;  // set lcd 40,52 pin as backplane pin
    base->PEN[0] = 0x000e0d80U;
    base->PEN[1] = 0x00300160U;
		// enable all pin of lcd module 
    base->FDCR = 0;  // disable fault detection
    for (uint32_t regNum = 0; regNum < 16; regNum++)base->WF[regNum] = 0;
}
void display_number(int x,int digit){
   if(digit==4){
		   if(x==0){ MY_SLCD_SetFrontPlaneSegments(LCD,11,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate|kSLCD_PhaseDActivate);
								 MY_SLCD_SetFrontPlaneSegments(LCD,10,kSLCD_PhaseBActivate|kSLCD_PhaseDActivate|kSLCD_PhaseAActivate);
			 }
			 if(x==1)MY_SLCD_SetFrontPlaneSegments(LCD,11,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 if(x==2){MY_SLCD_SetFrontPlaneSegments(LCD,11,kSLCD_PhaseDActivate|kSLCD_PhaseCActivate);
				 MY_SLCD_SetFrontPlaneSegments(LCD,10,kSLCD_PhaseBActivate|kSLCD_PhaseAActivate|kSLCD_PhaseCActivate);
			 }
			 if(x==3){
			 MY_SLCD_SetFrontPlaneSegments(LCD,11,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate|kSLCD_PhaseDActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,10,kSLCD_PhaseAActivate|kSLCD_PhaseCActivate);
			 }
			 if(x==4){
			 MY_SLCD_SetFrontPlaneSegments(LCD,11,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,10,kSLCD_PhaseCActivate|kSLCD_PhaseDActivate);
			 }
			 if(x==5){
			 MY_SLCD_SetFrontPlaneSegments(LCD,10,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,11,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate);
			 }
			 if(x==6){
			 MY_SLCD_SetFrontPlaneSegments(LCD,10,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate|kSLCD_PhaseBActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,11,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate);
			 }
			 if(x==7){
			 MY_SLCD_SetFrontPlaneSegments(LCD,11,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 }
			 if(x==8){
			  MY_SLCD_SetFrontPlaneSegments(LCD,11,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
				MY_SLCD_SetFrontPlaneSegments(LCD,10,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate|kSLCD_PhaseBActivate);
			 }
			 if(x==9){
			  MY_SLCD_SetFrontPlaneSegments(LCD,10,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate);
				MY_SLCD_SetFrontPlaneSegments(LCD,11,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 }
		}
		else if(digit==3){
		 if(x==0){MY_SLCD_SetFrontPlaneSegments(LCD,38,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate|kSLCD_PhaseDActivate);
							MY_SLCD_SetFrontPlaneSegments(LCD,53,kSLCD_PhaseBActivate|kSLCD_PhaseDActivate|kSLCD_PhaseAActivate);
		 }
			 if(x==1)MY_SLCD_SetFrontPlaneSegments(LCD,38,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 if(x==2){MY_SLCD_SetFrontPlaneSegments(LCD,38,kSLCD_PhaseDActivate|kSLCD_PhaseCActivate);
				 MY_SLCD_SetFrontPlaneSegments(LCD,53,kSLCD_PhaseBActivate|kSLCD_PhaseAActivate|kSLCD_PhaseCActivate);
			 }
			 if(x==3){
			 MY_SLCD_SetFrontPlaneSegments(LCD,38,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate|kSLCD_PhaseDActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,53,kSLCD_PhaseAActivate|kSLCD_PhaseCActivate);
			 }
			 if(x==4){
			 MY_SLCD_SetFrontPlaneSegments(LCD,38,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,53,kSLCD_PhaseCActivate|kSLCD_PhaseDActivate);
			 }
			 if(x==5){
			 MY_SLCD_SetFrontPlaneSegments(LCD,38,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,53,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate);
			 }
			 if(x==6){
			 MY_SLCD_SetFrontPlaneSegments(LCD,53,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate|kSLCD_PhaseBActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,38,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate);
			 }
			 if(x==7){
			 MY_SLCD_SetFrontPlaneSegments(LCD,38,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 }
			 if(x==8){
			  MY_SLCD_SetFrontPlaneSegments(LCD,38,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
				MY_SLCD_SetFrontPlaneSegments(LCD,53,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate|kSLCD_PhaseBActivate);
			 }
			 if(x==9){
			  MY_SLCD_SetFrontPlaneSegments(LCD,53,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate);
				MY_SLCD_SetFrontPlaneSegments(LCD,38,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 }

		}
		else if(digit==2){
		if(x==0){MY_SLCD_SetFrontPlaneSegments(LCD,8,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate|kSLCD_PhaseDActivate);
						 MY_SLCD_SetFrontPlaneSegments(LCD,7,kSLCD_PhaseBActivate|kSLCD_PhaseDActivate|kSLCD_PhaseAActivate);
		}
			 if(x==1)MY_SLCD_SetFrontPlaneSegments(LCD,8,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 if(x==2){MY_SLCD_SetFrontPlaneSegments(LCD,8,kSLCD_PhaseDActivate|kSLCD_PhaseCActivate);
				 MY_SLCD_SetFrontPlaneSegments(LCD,7,kSLCD_PhaseBActivate|kSLCD_PhaseAActivate|kSLCD_PhaseCActivate);
			 }
			 if(x==3){
			 MY_SLCD_SetFrontPlaneSegments(LCD,8,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate|kSLCD_PhaseDActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,7,kSLCD_PhaseAActivate|kSLCD_PhaseCActivate);
			 }
			 if(x==4){
			 MY_SLCD_SetFrontPlaneSegments(LCD,8,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,7,kSLCD_PhaseCActivate|kSLCD_PhaseDActivate);
			 }
			 if(x==5){
				MY_SLCD_SetFrontPlaneSegments(LCD,8,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,7,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate);
			 }
			 if(x==6){
			 MY_SLCD_SetFrontPlaneSegments(LCD,7,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate|kSLCD_PhaseBActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,8,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate);
			 }
			 if(x==7){
			 MY_SLCD_SetFrontPlaneSegments(LCD,8,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 }
			 if(x==8){
			  MY_SLCD_SetFrontPlaneSegments(LCD,8,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
				MY_SLCD_SetFrontPlaneSegments(LCD,7,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate|kSLCD_PhaseBActivate);
			 }
			 if(x==9){
			  MY_SLCD_SetFrontPlaneSegments(LCD,7,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate);
				MY_SLCD_SetFrontPlaneSegments(LCD,8,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 }

		}
	  else if(digit==1){
		if(x==0){MY_SLCD_SetFrontPlaneSegments(LCD,17,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate|kSLCD_PhaseDActivate);
			       MY_SLCD_SetFrontPlaneSegments(LCD,37,kSLCD_PhaseBActivate|kSLCD_PhaseDActivate|kSLCD_PhaseAActivate);
		}
			 if(x==1)MY_SLCD_SetFrontPlaneSegments(LCD,17,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 if(x==2){MY_SLCD_SetFrontPlaneSegments(LCD,17,kSLCD_PhaseDActivate|kSLCD_PhaseCActivate);
				 MY_SLCD_SetFrontPlaneSegments(LCD,37,kSLCD_PhaseBActivate|kSLCD_PhaseAActivate|kSLCD_PhaseCActivate);
			 }
			 if(x==3){
			 MY_SLCD_SetFrontPlaneSegments(LCD,17,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate|kSLCD_PhaseDActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,37,kSLCD_PhaseAActivate|kSLCD_PhaseCActivate);
			 }
			 if(x==4){
			 MY_SLCD_SetFrontPlaneSegments(LCD,17,kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,37,kSLCD_PhaseCActivate|kSLCD_PhaseDActivate);
			 }
			 if(x==5){
				 MY_SLCD_SetFrontPlaneSegments(LCD,17,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,37,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate);
			 }
			 if(x==6){
			 MY_SLCD_SetFrontPlaneSegments(LCD,37,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate|kSLCD_PhaseBActivate);
			 MY_SLCD_SetFrontPlaneSegments(LCD,17,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate);
			 }
			 if(x==7){
			 MY_SLCD_SetFrontPlaneSegments(LCD,17,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 }
			 if(x==8){
			  MY_SLCD_SetFrontPlaneSegments(LCD,17,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
				MY_SLCD_SetFrontPlaneSegments(LCD,37,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate|kSLCD_PhaseBActivate);
			 }
			 if(x==9){
			  MY_SLCD_SetFrontPlaneSegments(LCD,37,kSLCD_PhaseAActivate|kSLCD_PhaseDActivate|kSLCD_PhaseCActivate);
				MY_SLCD_SetFrontPlaneSegments(LCD,17,kSLCD_PhaseDActivate|kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
			 }
		}
}
void display_outofrange(){
      uint32_t test = kSLCD_PhaseBActivate|kSLCD_PhaseCActivate|kSLCD_PhaseDActivate|kSLCD_PhaseAActivate;
      MY_SLCD_SetFrontPlaneSegments(LCD,10,test);
	    MY_SLCD_SetFrontPlaneSegments(LCD,38,(uint32_t)kSLCD_PhaseBActivate|kSLCD_PhaseCActivate);
	    MY_SLCD_SetFrontPlaneSegments(LCD,53,(uint32_t)kSLCD_PhaseBActivate|kSLCD_PhaseAActivate|kSLCD_PhaseDActivate);
	    MY_SLCD_SetFrontPlaneSegments(LCD,8,(uint32_t)kSLCD_PhaseBActivate|kSLCD_PhaseCActivate|kSLCD_PhaseDActivate);
	    MY_SLCD_SetFrontPlaneSegments(LCD,7,(uint32_t)kSLCD_PhaseBActivate|kSLCD_PhaseAActivate|kSLCD_PhaseDActivate);

}
void display_decimal(int x){
      int arr[4];
	    for(int i=0;i<4;i++){
			    arr[i] = x%10;
			    x=(int)x/10;
			}
			if(x>=1)display_outofrange();
			else{
			display_number(arr[0],4);
		  display_number(arr[1],3);
			display_number(arr[2],2);
			display_number(arr[3],1);
			}
}
#include "sys.h" 

// Set the Vector Table Offset Address
// NVIC_VectTab: base address
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset)
{
	SCB->VTOR = NVIC_VectTab | (Offset & (u32)0x1FFFFF80); // Set NVIC vector table offset register
	                                                       // Determines whether the vector table is located in CODE or RAM
}

// Configure NVIC interrupt priority group
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)
{
	u32 temp, temp1;
	temp1 = (~NVIC_Group) & 0x07; // Extract the last three bits
	temp1 <<= 8;
	temp = SCB->AIRCR;            // Read previous configuration
	temp &= 0X0000F8FF;           // Clear previous group setting
	temp |= 0X05FA0000;           // Write key
	temp |= temp1;
	SCB->AIRCR = temp;            // Set group
}

// Initialize NVIC
// NVIC_PreemptionPriority: preemption priority
// NVIC_SubPriority: response sub-priority
// NVIC_Channel: interrupt number
// NVIC_Group: priority group (0~4)
// Priority grouping:
// Group 0: 0 bits for preemption, 4 bits for sub-priority
// Group 1: 1 bit for preemption, 3 bits for sub-priority
// Group 2: 2 bits for preemption, 2 bits for sub-priority
// Group 3: 3 bits for preemption, 1 bit for sub-priority
// Group 4: 4 bits for preemption, 0 bits for sub-priority
// Lower number = higher priority
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority, u8 NVIC_Channel, u8 NVIC_Group)
{
	u32 temp;
	MY_NVIC_PriorityGroupConfig(NVIC_Group); // Set priority group
	temp = NVIC_PreemptionPriority << (4 - NVIC_Group);
	temp |= NVIC_SubPriority & (0x0F >> NVIC_Group);
	temp &= 0xF; // Only use lower 4 bits
	NVIC->ISER[NVIC_Channel / 32] |= (1 << NVIC_Channel % 32); // Enable interrupt
	NVIC->IP[NVIC_Channel] |= temp << 4; // Set preemption and sub-priority
}

// Configure external interrupts
// GPIOx: 0~6 for GPIOA~G
// BITx: bit position
// TRIM: trigger mode: 1 = falling edge; 2 = rising edge; 3 = both edges
void Ex_NVIC_Config(u8 GPIOx, u8 BITx, u8 TRIM)
{
	u8 EXTADDR;
	u8 EXTOFFSET;
	EXTADDR = BITx / 4; // Determine which EXTI register
	EXTOFFSET = (BITx % 4) * 4;
	RCC->APB2ENR |= 0x01; // Enable AFIO clock
	AFIO->EXTICR[EXTADDR] &= ~(0x000F << EXTOFFSET); // Clear old setting
	AFIO->EXTICR[EXTADDR] |= GPIOx << EXTOFFSET; // Map EXTI.BITx to GPIOx.BITx
	EXTI->IMR |= 1 << BITx; // Enable interrupt
 	if (TRIM & 0x01) EXTI->FTSR |= 1 << BITx; // Enable falling edge trigger
	if (TRIM & 0x02) EXTI->RTSR |= 1 << BITx; // Enable rising edge trigger
}

// Reset all RCC clock registers
void MYRCC_DeInit(void)
{
 	RCC->APB1RSTR = 0x00000000;
	RCC->APB2RSTR = 0x00000000;
	RCC->AHBENR = 0x00000014;   // Enable flash and SRAM clocks, disable others in sleep mode
	RCC->APB2ENR = 0x00000000;  // Disable all peripheral clocks
	RCC->APB1ENR = 0x00000000;
	RCC->CR |= 0x00000001;      // Enable HSI
	RCC->CFGR &= 0xF8FF0000;    // Reset SW[1:0], HPRE, PPRE1/2, ADCPRE, MCO
	RCC->CR &= 0xFEF6FFFF;      // Reset HSEON, CSSON, PLLON
	RCC->CR &= 0xFFFBFFFF;      // Reset HSEBYP
	RCC->CFGR &= 0xFF80FFFF;    // Reset PLLSRC, PLLXTPRE, PLLMUL, USBPRE
	RCC->CIR = 0x00000000;      // Disable all interrupts

	// Set vector table base address
#ifdef  VECT_TAB_RAM
	MY_NVIC_SetVectorTable(0x20000000, 0x0);
#else
	MY_NVIC_SetVectorTable(0x08000000, 0x0);
#endif
}

// Enter low power sleep mode (WFI instruction)
void WFI_SET(void)
{
    __asm__ volatile ("WFI");
}

// Disable all interrupts
void INTX_DISABLE(void)
{
    __asm__ volatile ("CPSID I");
}

// Enable all interrupts
void INTX_ENABLE(void)
{
    __asm__ volatile ("CPSIE I");
}

// Set Main Stack Pointer
void MSR_MSP(uint32_t addr)
{
    __asm__ volatile (
        "MSR MSP, %0\n"
        "BX LR\n"
        :
        : "r" (addr)
        :
    );
}

// Enter standby mode
void Sys_Standby(void)
{
	SCB->SCR |= 1 << 2;         // Set SLEEPDEEP bit
	RCC->APB1ENR |= 1 << 28;    // Enable power interface clock
	PWR->CSR |= 1 << 8;         // Enable wake-up pin
	PWR->CR |= 1 << 2;          // Clear wake-up flag
	PWR->CR |= 1 << 1;          // Set PDDS
	WFI_SET();                  // Enter standby mode
}

// Perform software system reset
void Sys_Soft_Reset(void)
{
	SCB->AIRCR = 0X05FA0000 | (u32)0x04;
}

// Set JTAG mode
// #define JTAG_SWD_DISABLE   0x02
// #define SWD_ENABLE         0x01
// #define JTAG_SWD_ENABLE    0x00
void JTAG_Set(u8 mode)
{
	u32 temp;
	temp = mode;
	temp <<= 25;
	RCC->APB2ENR |= 1 << 0;      // Enable AFIO clock
	AFIO->MAPR &= 0xF8FFFFFF;    // Clear bits [26:24]
	AFIO->MAPR |= temp;          // Apply mode
}

// Initialize system clock
// PLL: selected multiplier (2 to 16)
void Stm32_Clock_Init(u8 PLL)
{
	unsigned char temp = 0;
	MYRCC_DeInit();		         // Reset and configure vector table
 	RCC->CR |= 0x00010000;     // Enable external high-speed oscillator (HSEON)
	while (!(RCC->CR >> 17));  // Wait until HSE is ready
	RCC->CFGR = 0X00000400;    // APB1=DIV2, APB2=DIV1, AHB=DIV1
	PLL -= 2;
	RCC->CFGR |= PLL << 18;    // Set PLL multiplier (2–16)
	RCC->CFGR |= 1 << 16;	     // PLL source: HSE
	FLASH->ACR |= 0x32;	       // FLASH latency: 2 wait states

	RCC->CR |= 0x01000000;     // Enable PLL
	while (!(RCC->CR >> 25));  // Wait until PLL is locked
	RCC->CFGR |= 0x00000002;   // Set PLL as system clock
	while (temp != 0x02)       // Wait until PLL is used as system clock
	{
		temp = RCC->CFGR >> 2;
		temp &= 0x03;
	}
}

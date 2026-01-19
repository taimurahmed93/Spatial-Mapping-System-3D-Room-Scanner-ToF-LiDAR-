// COMP 2DX3
// Taimur Ahmed
// Bus speed 22 MHZ
// 

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"


#define I2C_MCS_ACK             0x00000008  
#define I2C_MCS_DATACK          0x00000008  
#define I2C_MCS_ADRACK          0x00000004  
#define I2C_MCS_STOP            0x00000004  // STOP

#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  
#define I2C_MCR_MFE             0x00000010  
#define I2C_MCS_START           0x00000002  
#define I2C_MCS_ERROR           0x00000002  


#define MAXRETRIES              5         
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;      //I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;   
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};														

    GPIO_PORTB_AFSEL_R |= 0x0C;           													
    GPIO_PORTB_ODR_R |= 0x08;             													

    GPIO_PORTB_DEN_R |= 0x0C;             													
          												
                                                                       

  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    
    I2C0_MCR_R = I2C_MCR_MFE;                      									
    I2C0_MTPR_R = 0b0000000000000101000000000111011;               
                                        
        
}



//The VL53L1X reset
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    
    GPIO_PORTG_DIR_R &= 0x00;                                   
  GPIO_PORTG_AFSEL_R &= ~0x01;                                
  GPIO_PORTG_DEN_R |= 0x01;                                  
                                 
  GPIO_PORTG_AMSEL_R &= ~0x01;                    

    return;
}

//stepper motor 
void PortH_Init (void){
  
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;// Activate the clock
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){}; 
  GPIO_PORTH_DIR_R = 0b00001111;
  GPIO_PORTH_DEN_R = 0b00001111;
  // Return from the function
  return;
}

void PortF_Init(void){
  
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // Activate the clock
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};
  GPIO_PORTF_DIR_R=0b00010011;// 
  GPIO_PORTF_DEN_R=0b00010011; // 
  // Return from the function
  return;
}

volatile unsigned long Falling_Edges = 0;


void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};
  GPIO_PORTJ_DIR_R &= ~0x03;    										
  GPIO_PORTJ_DEN_R |= 0x03;     							
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 					
	GPIO_PORTJ_AMSEL_R &= ~0x03;										
	GPIO_PORTJ_PUR_R |= 0x03;									
}


void VL53L1X_XSHUT(void){
  
    GPIO_PORTG_DIR_R |= 0x01;
    GPIO_PORTG_DATA_R &= 0b11111110;
    FlashAllLEDs();
    SysTick_Wait10ms(10); 
    GPIO_PORTG_DIR_R &= ~0x01;
}


//clock wise 
void forward(void) {
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10ms(1); 	
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000110; 
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10ms(1);
 
}

// motor control 
void CCW_spin (void) {
	for (int i = 0; i < 512; i++){ 
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10ms(1);
	}
}


//MAIN Function	

int pos  = 0;

uint16_t	dev = 0x29;		
int status=0;
int fl = 0; 

int main(void) {
	
	uint8_t byteDate = 0x00;
	uint8_t id, name = 0x00;
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize fucntions 
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	PortH_Init ();
	I2C_Init();
	UART_Init();
	PortJ_Init();	
	PortF_Init();

	
	GPIO_PORTF_DATA_R = 0b00000000;

	
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	UART_printf("ToF sensor booted.\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); // 
	

  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

  status = VL53L1X_StartRanging(dev);   

	int delay = 1; 
	int pos  = 0;
	int step = 32; 
	double angle = 0;
	
	
	while(1) {  
		
		if((GPIO_PORTJ_DATA_R&0x1)==0){ 
			SysTick_Wait10ms(30);
			fl ^= 1; 
		}
		
		
	  while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
      
          VL53L1_WaitMs(dev, 5);
	  }
		dataReady = 0; 
	  
		if (fl == 1){ 
			
			forward(); 
			
			//ToF transmits data
			if (pos %step == 0 && pos !=0){ 
				status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
				status = VL53L1X_GetDistance(dev, &Distance);					
				status = VL53L1X_GetSignalRate(dev, &SignalRate);
				status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
				status = VL53L1X_GetSpadNb(dev, &SpadNum);
					
				//When angle is 11.25 - send data and flash led 
			
				// Flash LED D1
				FlashLED3(delay);
				FlashLED4(delay);
				//clear interrupt
				status = VL53L1X_ClearInterrupt(dev); 
				
				angle+= 11.25; 
				
				
				sprintf(printf_buffer,"%u\r\n", Distance);
        UART_printf(printf_buffer);
				SysTick_Wait10ms(50);
			}
			
			if (pos  == 512){ 
				fl ^=1; 
				pos  = 0;  
				angle=0;  
				
				CCW_spin (); 
				FlashAdditional();
			}
			pos ++; 
  		}
  
	}

}


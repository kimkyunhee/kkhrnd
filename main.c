/**
  ******************************************************************************
	PROJECT			     : TCM SERIER(TOTAL COUNTER)
	PROGRAMED BY	   : Kyun-Hee, Kim
	DATE			       : 2022. 08. 31.
	MCU				       : STM8L152R8T6
	MODULE  		     : main.c
	COMPILER		     : COSMIC CXSTM8
  DEVELOPMENT TOOL : ST Visual Develop, ST Visual Programmer

	CSS	             : Enabled
	VD	             : 3.3V
	PKG1	    			 : LQFP64
	RSTC	    			 : Disabled
	OSC TYPE  			 : Resonate
	OSC RANGE 			 : HSE 16MHz
	PLL	      			 : Disabled
	
	LCD DRIVER       : PT16527-PTC(LQFP64-1010)
	EEPROM           : 24LC32A-I/SN
	SCHMITT TRIGGER  : SN74AUP1G14DBV(SOT-23(5))DBV5
  ******************************************************************************

/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"
#include "stdio.h"

/* Private typedef -----------------------------------------------------------*/

#define TACT_KEY  GPIOB
#define DIP_KEY  GPIOF
#define DIP2_KEY  GPIOG
#define FND_POINT	4

#define PUTCHAR_PROTOTYPE char putchar (char c)
#define GETCHAR_PROTOTYPE char getchar (void)

#define S_DO			1<<4
#define S_CS			1<<5
#define S_SCK			1<<6
#define S_DI			1<<7

#define CP1_INPUT_ON		GPIOB->IDR & 1<<1
#define CP1_INPUT_OFF		(GPIOB->IDR & 1<<1)==0

#define CP2_INPUT_ON		GPIOB->IDR & 1<<2
#define CP2_INPUT_OFF		(GPIOB->IDR & 1<<2)==0
#define RST_INPUT_ON		GPIOB->IDR & 1<<3
#define RST_INPUT_OFF		(GPIOB->IDR & 1<<3)==0

#define NPN_INPUT 1
#define PNP_INPUT 0

#define BACK_LIGHT_ON		GPIOG->ODR |= 1<<1

#define ADDRESS  0xA0
#define SPEED    100000

bool cnt_start_flag;
bool seg_stop_flag;
unsigned int signal_cnt, signal, exti_cnt, minus, key_check, up_mode_cnt, down_mode_cnt, new_sw;
unsigned char buf_small[4];
uint8_t rx_buf[128];
uint8_t buf[256];
unsigned int buf_big[4];

unsigned int M_down_data1_4h = 9999;
unsigned long M_down_data1_6h = 999999;

unsigned int data1, data2, data3;

unsigned int data1_4h;
unsigned long data1_6;
unsigned int eeprom_data1_4h;
unsigned long eeprom_data1_6;

unsigned char data_temp_6[6];
unsigned char data_temp_4h[4];
unsigned int LCD_data[51];

unsigned int exti_signal = 0;
unsigned int cnt;
unsigned int old_sw, new_sw = 0;

bool COUNTER_MODE;

/* 입력 변수 -----------------------------------------------------------*/
unsigned int old_data1;
unsigned int CP2_key_check;//지령입력
unsigned int but_check;//개별입력
unsigned int first_cp1_in, second_cp1_in, third_cp1_in;//위상차 
unsigned int first_cp2_in, second_cp2_in, third_cp2_in, CW_CCW, rotate_cp1, rotate_cp2, data_9999;//위상차 
unsigned int low_in, high_in, CCW, CW, plus, rotate, minus_data;//위상차 
unsigned int but_check, minus_high_check, minus_low_check;//개별입력
unsigned int minus_check;//지령입력 
unsigned int CP1_key_check, CP2_key_check;//UPUP
unsigned int low_in, high_in, CCW, CW, plus, rotate, minus_data;//위상차 
/* 입력 변수 -----------------------------------------------------------*/

unsigned int timer_cnt, key_cnt, key_flag, timer_flag;
bool _99_59, _999_9;
bool _99_59_99, _999_59_9, _9999_59, _99999_9;
unsigned int time_plus;
signed int timer4_4digit=9999, down_data1=99, down_data2=99;
unsigned long timer6_2digit1, timer6_2digit2, timer6_2digit3, timer6_3digit1, timer6_3digit2, timer6_4digit1;
signed long down_timer6_99_1=99, down_timer6_59_2=59, down_timer6_99_3=99, down_timer6_999_1=999, down_timer6_599_2=599, down_timer6_9999_1=9999, down_timer6_5999_1=5999;
signed long down_timer6_59_1=59;
unsigned long down_DATA, timer_cps=92;
signed int down_99 = 99, down_59 = 59;

unsigned int _1CPS, _30CPS, _2KCPS, _5KCPS, RESET_ON, EEPROM_ON, DIS_POINT;
unsigned int DP000_0, DP00_00, DP0_000;
unsigned int DP00000_0, DP0000_00, DP000_000, DP00_0000, DP0_00000;
unsigned int rst_input_flag, rst_input_cnt, rst_key_check;

/* timer mode -----------------------------------------------------------*/
void M_timer_up_99_99s(void);
void M_timer_up_999_9s(void);

/* DISPLAY-FND -----------------------------------------------------------*/
void DIS_Dec_6(unsigned long data);
void ALPHA_DIS_6(unsigned char *display_data);

void DIS_Dec_4h(unsigned int data);
void ALPHA_DIS_4h(unsigned char *display_data);

/* USART-PRINTF -----------------------------------------------------------*/
static void USART_Config(void);
void TX_CH(char ch);
void TX_STR(char* str);
char putchar(char ch);

/* I2C-EEPROM -----------------------------------------------------------*/
void I2C_Config(void);
void EEPROM_write(uint16_t addr, uint8_t data);
uint8_t EEPROM_read(uint16_t addr);
void EEPROM_read_block(uint16_t addr, uint8_t *buf, uint16_t size);

static void CLK_Config(void)
{
	CLK_DeInit();	
	
	CLK_HSEConfig(CLK_HSE_ON); // HSE Enable
	
  // High speed internal clock prescaler:
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);// System Clock Divider: 1

  // Select HSE as system clock source
  CLK_SYSCLKSourceSwitchCmd(ENABLE);
	CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSE);//System Clock Source HSE

  while (CLK_GetSYSCLKSource() != CLK_SYSCLKSource_HSE)
  {}

  // Enable TIM3 CLK
	//CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, ENABLE); // Peripheral Clock Enable
	CLK_PeripheralClockConfig(CLK_Peripheral_I2C1, ENABLE);
}

void GPIO_Config(void)
{
	GPIO_DeInit(GPIOB);
	GPIO_DeInit(GPIOC);
	GPIO_DeInit(GPIOD);
	GPIO_DeInit(GPIOE);
	GPIO_DeInit(GPIOG);
		
	GPIO_Init(GPIOB, GPIO_Pin_1, GPIO_Mode_In_FL_No_IT); //CP1 INPUT
	GPIO_Init(GPIOB, GPIO_Pin_2, GPIO_Mode_In_FL_No_IT); //CP2 INPUT
	GPIO_Init(GPIOB, GPIO_Pin_3, GPIO_Mode_In_FL_No_IT); //RST INPUT
	GPIO_Init(GPIOB, (GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_4), GPIO_Mode_Out_PP_High_Fast); //SPI
	
	GPIO_Init(GPIOG, GPIO_Pin_1, GPIO_Mode_Out_PP_High_Fast); //BL
	GPIO_Init(GPIOG, GPIO_Pin_2, GPIO_Mode_In_PU_No_IT); //DISPLAY TACT RST KEY
	//i2c
	GPIOC->CR1 |= 0x80;  // Push-pull
  GPIOC->CR2 |= 0x80;  // Output speed up to 10 MHz


	//DIP SW
	GPIO_Init(GPIOD, (GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3), GPIO_Mode_In_PU_No_IT); //TIM
	GPIO_Init(GPIOD, GPIO_Pin_0, GPIO_Mode_In_PU_No_IT); //UP DOWN
	GPIO_Init(GPIOE, (GPIO_Pin_2 | GPIO_Pin_5), GPIO_Mode_In_PU_No_IT); //CPS
	GPIO_Init(GPIOE, GPIO_Pin_1, GPIO_Mode_In_PU_No_IT); //RST ON OFF
	GPIO_Init(GPIOE, GPIO_Pin_0, GPIO_Mode_In_PU_No_IT); //EEPROM ON OFF
	GPIO_Init(GPIOG, GPIO_Pin_3, GPIO_Mode_In_PU_No_IT); //CNTIM

	GPIO_Init(GPIOG, GPIO_Pin_0, GPIO_Mode_In_PU_No_IT); //NPNPNP
	
	GPIO_Init(GPIOB, GPIO_Pin_0, GPIO_Mode_In_PU_No_IT); //12V CHECK / EEPROM HIGH
}

void delay_ns(unsigned int  ns)
{
	nop();
	//nop();
	//ms *=1;
	//while(ms--)nop();
}
void delay_1ms(unsigned int  ms)
{
	uint8_t i;
	ms *= 9;
	while(ms--)
	{
		for(i=0; i<250; i++) 
		{
			nop();
		}
	}
}

void EXIT_Rising_Config(void)//npn
{
	EXTI_DeInit();
	EXTI_SetPinSensitivity(EXTI_Pin_1, EXTI_Trigger_Rising);
	GPIO_Init(GPIOB, GPIO_Pin_1, GPIO_Mode_In_FL_IT);
}

void EXIT_Falling_Config(void)//pnp
{
	EXTI_DeInit();
	EXTI_SetPinSensitivity(EXTI_Pin_1, EXTI_Trigger_Falling);
	GPIO_Init(GPIOB, GPIO_Pin_1, GPIO_Mode_In_FL_IT);
}

void TIMER3_Config(void)
{
	CLK->PCKENR1 |= 0x02;    								//카운터 3번에 clock
	
	TIM3->PSCR = TIM3_Prescaler_16;        	// 8 / 16000000 = 0.000001
																			    // 0.0000005 * 1000(0x03e8) == 1ms
	TIM3->ARRH = 0x03;
	TIM3->ARRL = 0xe8;
	
	
	TIM3->SR1 = 0xFE;												//인터럽트 플레이그 클리어
	TIM3->IER |= 0x01;    									//인터럽트 업데이트
	TIM3->CR1 |= 0x01;    									//카운터 이네이블
	
//enableInterrupts();
}

unsigned char LCD_SEG_NUM5[]={
0x09,0x00,0X0A,0x02,0x03,0x03,0x0b,0x01,0x0b,0x03,//0123456789
0x00,//null
0x02,//마이너스 표시
0x0b,0x0b,0x09,0x0a,0x0f,0x0b,0x0b,0x0b,0x09,0x08,//A.B.C.D.E.F.G.H.I.J.
0x0a,0x09,0x09,0x09,0x0a,0x0b,0x09,0x0b,0x03,0x0b,//K.L.M.N.O.P.Q.R.S.T.
0x09,0x0d,0x0d,0x04,0x03,0x04//U.V.W.X.Y.Z.
};
unsigned char LCD_SEG_NUM6[]={
0x08,0x00,0X0A,0x0a,0x02,0x0a,0x0a,0x00,0x0a,0x0a,//0123456789
0x00,//null
0x02,//마이너스 표시
0x02,0x0a,0x08,0x0a,0x0f,0x02,0x0a,0x02,0x00,0x08,//A.B.C.D.E.F.G.H.I.J.
0x04,0x08,0x01,0x05,0x0a,0x02,0x0c,0x06,0x0a,0x0a,//K.L.M.N.O.P.Q.R.S.T.
0x08,0x00,0x04,0x05,0x0a,0x08//U.V.W.X.Y.Z.
};
unsigned char LCD_SEG_NUM7[]={
0x06,0x06,0X02,0x06,0x06,0x04,0x04,0x06,0x06,0x06,//0123456789
0x00,//null
0x00,//마이너스 표시
0x06,0x04,0x00,0x06,0x01,0x00,0x04,0x06,0x00,0x06,//A.B.C.D.E.F.G.H.I.J.
0x01,0x00,0x07,0x06,0x04,0x02,0x06,0x02,0x04,0x00,//K.L.M.N.O.P.Q.R.S.T.
0x06,0x01,0x06,0x01,0x06,0x01//U.V.W.X.Y.Z.
};


void PT16527(unsigned int *data)
{
   unsigned int i=0;
   unsigned char addr_id = 0x41, functions_data=0x10;
   unsigned char push_count=0;
   
   GPIOB->ODR &= ~(1<<4); // CE핀.로우;
   
   GPIOB->ODR &= ~(1<<5); // SCK포트->ODR = SCK핀.로우;
   delay_ns(1);
	 
   //    1/4듀티 ID
   for(i=0; i < 8; i++)
   {
      if(0x01 & (addr_id>>i))GPIOB->ODR |= 1<<6;//SI포트->ODR = SI핀.하이; 
      else
      {
				GPIOB->ODR &= ~(1<<6);// SI포트->ODR = SI핀.로우; 
      }
      GPIOB->ODR |= 1<<5; // SCK포트->ODR = SCK핀.하이;
      delay_ns(1);
      GPIOB->ODR &= ~(1<<5); // SCK포트->ODR = SCK핀.로우;
      delay_ns(1);
   }
	 GPIOB->ODR |= 1<<4; //CE포트->ODR = CE핀.하이;
   delay_ns(1);

   //  204개 데이터
   for(i=0; i <204; i++)
   {
      if(0x01 & (data[i/FND_POINT]>>push_count))GPIOB->ODR |= 1<<6;//SI포트->ODR = SI핀.하이;
      else
      {
				GPIOB->ODR &= ~(1<<6);//SI포트->ODR = SI핀.로우;
      }
      GPIOB->ODR |= 1<<5; // SCK포트->ODR = SCK핀.하이;
      delay_ns(1);
      GPIOB->ODR &= ~(1<<5); // SCK포트->ODR = SCK핀.로우;
      delay_ns(1);
      
      push_count++;
      if(push_count>=FND_POINT)push_count=0;
   }
	 
	 // 픽스 데이타 
   for(i=0; i < 2; i++)
   {
      if(0x01 & (0x00>>i))GPIOB->ODR |= 1<<6;//SI포트->ODR = SI핀.하이; 
      else
      {
				GPIOB->ODR &= ~(1<<6);//SI포트->ODR = SI핀.로우; 
      }
      GPIOB->ODR |= 1<<5; ////SCK포트->ODR = SCK핀.하이;
      delay_ns(1);
      GPIOB->ODR &= ~(1<<5);//SCK포트->ODR = SCK핀.로우;
      delay_ns(1);
		}
	  // DT / CU 데이타 
		for(i=0; i < 2; i++)
    {
      if(0x01 & (0x02>>i))GPIOB->ODR |= 1<<6;//SI포트->ODR = SI핀.하이; 
      else
      {
				GPIOB->ODR &= ~(1<<6);//SI포트->ODR = SI핀.로우; 
      }
      GPIOB->ODR |= 1<<5; ////SCK포트->ODR = SCK핀.하이;
      delay_ns(1);
      GPIOB->ODR &= ~(1<<5);//SCK포트->ODR = SCK핀.로우;
      delay_ns(1);
		}
   //    펑션데이터
   for(i=0; i <8; i++)
   {
      if(0x01 & (functions_data>>i))GPIOB->ODR |= 1<<6;//SI포트->ODR = SI핀.하이; 
      else
      {
				GPIOB->ODR &= ~(1<<6);//SI포트->ODR = SI핀.로우; 
      }
      GPIOB->ODR |= 1<<5; ////SCK포트->ODR = SCK핀.하이;
      delay_ns(1);
      GPIOB->ODR &= ~(1<<5);//SCK포트->ODR = SCK핀.로우;
      delay_ns(1);
		}
}

void DIS_Dec_4h(unsigned int data)
{
	unsigned int temp=0;

	if(data < 1000)
	{
		if(DP0_000)
		{
			if(minus && data > 0)data_temp_4h[3] = 10;
			else data_temp_4h[3] = 0;
		}
		else data_temp_4h[3] = 10;
	}
	else
		data_temp_4h[3] = (unsigned char)(data /1000);
	
	if(data<100)
	{
		if(DP00_00 | DP0_000)data_temp_4h[2] = 0;
		else data_temp_4h[2] = 10;
	}
	else
	{
		temp = data % 1000;
		data_temp_4h[2] = (unsigned char)(temp / 100);
	}
	if(data<10)
	{
		if(DP000_0 | DP00_00 | DP0_000)data_temp_4h[1] = 0;
		else data_temp_4h[1] = 10;
	}
	else
	{
		temp = data % 100;
		data_temp_4h[1] = (unsigned char)(temp / 10);
	}
	data_temp_4h[0]= (unsigned char)(data % 10);
	
	if(COUNTER_MODE)
	{
		if(minus)//==============================================================================================================================================
		{
			if(data > 0)
			{
				if(DP0_000)// -0.01
				{
					LCD_data[30] = LCD_SEG_NUM6[11];// 100의 자리가 0보다 크면 1000의 자리 마이너스 표시 
					LCD_data[31] = LCD_SEG_NUM5[11];// 100의 자리가 0보다 크면 1000의 자리 마이너스 표시 
				}
				else if(DP00_00)//-0.01
				{
					LCD_data[30] = LCD_SEG_NUM6[11];// 100의 자리가 0보다 크면 1000의 자리 마이너스 표시 
					LCD_data[31] = LCD_SEG_NUM5[11];// 100의 자리가 0보다 크면 1000의 자리 마이너스 표시 
				}
				else //DP000_0 //-0.0
				{
					if(data > 99)
					{
						LCD_data[30] = LCD_SEG_NUM6[11];// 100의 자리가 0보다 크면 1000의 자리 마이너스 표시 
						LCD_data[31] = LCD_SEG_NUM5[11];// 100의 자리가 0보다 크면 1000의 자리 마이너스 표시 
					}
					else
					{
						LCD_data[27] = LCD_SEG_NUM6[11];// 10의 자리가 10보다 작으면 100의 자리 마이너스 표시
						LCD_data[28] = LCD_SEG_NUM5[11];// 10의 자리가 10보다 작으면 100의 자리 마이너스 표시 
					}
				}
			}
			else
			{
				LCD_data[24] = LCD_SEG_NUM6[data_temp_4h[1]];// 1의 자리가 0보다 작으면 10의 자리 null 표시 
				LCD_data[25] = LCD_SEG_NUM5[data_temp_4h[1]];// 1의 자리가 0보다 작으면 10의 자리 null 표시 
			}
		}
	}
	else
	{
		if(minus)
		{
			if(data_temp_4h[2] < 10)
			{
				LCD_data[30] = LCD_SEG_NUM6[11];// 100의 자리가 10보다 작으면 1000의 자리 마이너스 표시 
				LCD_data[31] = LCD_SEG_NUM5[11];// 100의 자리가 10보다 작으면 1000의 자리 마이너스 표시 
			}
			else if(data_temp_4h[1] < 10)
			{
				LCD_data[27] = LCD_SEG_NUM6[11];// 10의 자리가 10보다 작으면 100의 자리 마이너스 표시
				LCD_data[28] = LCD_SEG_NUM5[11];// 10의 자리가 10보다 작으면 100의 자리 마이너스 표시 
			}
			else if(data_temp_4h[0] < 10)
			{
				if(data_temp_4h[0] == 0)
				{
					LCD_data[24] = LCD_SEG_NUM6[data_temp_4h[1]];
					LCD_data[25] = LCD_SEG_NUM5[data_temp_4h[1]];
				}
				else 
				{
					LCD_data[24] = LCD_SEG_NUM6[11];// 1의 자리가 10보다 작으면 10의 자리 마이너스 표시 
					LCD_data[25] = LCD_SEG_NUM5[11];
				}
			}
			else LCD_data[0] = 0xff; // 1000의 자리 0 표시
		}
	}

	PT16527(LCD_data);
	/////1
	LCD_data[29] = LCD_SEG_NUM7[data_temp_4h[3]];
	LCD_data[30] = LCD_SEG_NUM6[data_temp_4h[3]];
	LCD_data[31] = LCD_SEG_NUM5[data_temp_4h[3]];
	/////2
	LCD_data[26] = LCD_SEG_NUM7[data_temp_4h[2]];
	LCD_data[27] = LCD_SEG_NUM6[data_temp_4h[2]];
	LCD_data[28] = LCD_SEG_NUM5[data_temp_4h[2]];
	/////3
	LCD_data[23] = LCD_SEG_NUM7[data_temp_4h[1]];
	LCD_data[24] = LCD_SEG_NUM6[data_temp_4h[1]];
	LCD_data[25] = LCD_SEG_NUM5[data_temp_4h[1]];
	///////4
	LCD_data[20] = LCD_SEG_NUM7[data_temp_4h[0]];
	LCD_data[21] = LCD_SEG_NUM6[data_temp_4h[0]];
	LCD_data[22] = LCD_SEG_NUM5[data_temp_4h[0]];


	/////1
	if(data_temp_4h[3] == 1 || data_temp_4h[3] == 4 || data_temp_4h[3] == 10)LCD_data[17] &= ~(0X02);
	else LCD_data[17] |= 0X02;//1A
	/////2
	if(data_temp_4h[2] == 1 || data_temp_4h[2] == 4 || data_temp_4h[2] == 10)LCD_data[17] &= ~(0X01);
	else LCD_data[17] |= 0X01;//2A
	/////3
	if(data_temp_4h[1] == 1 || data_temp_4h[1] == 4 || data_temp_4h[1] == 10)LCD_data[18] &= ~(0X01);
	else LCD_data[18] |= 0X01;//3A
	///////4
	if(data_temp_4h[0] == 1 || data_temp_4h[0] == 4 || data_temp_4h[0] == 10)LCD_data[18] &= ~(0X02);
	else LCD_data[18] |= 0X02;//4A

	
}

//bool _99_99, _999_9,_9999,_99_59,_999_9;
//bool _99999_9, _999999, _99_59_99, _999_59_9, _9999_59;

void DIS_Dec_4h_2d(unsigned int data1, unsigned int data2)
{
	unsigned int temp=0;
	//int i, j = 0;
	if(_99_59)
	{
		if(data1<10)data_temp_4h[3] = 10;//null
		else
		{
			temp = data1 % 100;
			data_temp_4h[3] = (unsigned char)(temp / 10);
		}
		
		if(DIS_POINT)
		{
			if(data1 <= 0)data_temp_4h[2] = 0;//null
			else data_temp_4h[2]= (unsigned char)(data1 % 10);
		}
		else
		{
			if(data1 <= 0)data_temp_4h[2] = 10;//null
			else data_temp_4h[2]= (unsigned char)(data1 % 10);
		}
		
		if(data2<10)
		{
			if(DIS_POINT)data_temp_4h[1] = 0;//null
			else
			{
				if(data1 <= 0)data_temp_4h[1] = 10;
				else data_temp_4h[1] = 0;
			}
		}
		else
		{
			temp = data2 % 100;
			data_temp_4h[1] = (unsigned char)(temp / 10);
		}
		data_temp_4h[0]= (unsigned char)(data2 % 10);
	}
	else if(_999_9)
	{
		if(data1 < 100)data_temp_4h[3] = 10;
		else
			data_temp_4h[3] = (unsigned char)(data1 /100);
		
		if(data1<10)data_temp_4h[2] = 10;
		else
		{
			temp = data1 % 100;
			data_temp_4h[2] = (unsigned char)(temp / 10);
		}
		if(DIS_POINT)data_temp_4h[1]= (unsigned char)(data1 % 10);
		else
		{
			if(data1 <= 0)data_temp_4h[1] = 10;
			else data_temp_4h[1]= (unsigned char)(data1 % 10);
			
		}
		
		data_temp_4h[0] = (unsigned char)(data2 % 10);
	}
	else 
	{
		if(data2 < 1000)data_temp_4h[3] = 10;
		else
			data_temp_4h[3] = (unsigned char)(data2 /1000);
		
		if(data2<100)data_temp_4h[2] = 10;
		else
		{
			temp = data2 % 1000;
			data_temp_4h[2] = (unsigned char)(temp / 100);
		}
		if(data2<10)data_temp_4h[1] = 10;
		else
		{
			temp = data2 % 100;
			data_temp_4h[1] = (unsigned char)(temp / 10);
		}
		data_temp_4h[0]= (unsigned char)(data2 % 10);
	}
	
	PT16527(LCD_data);
	/////1
	LCD_data[29] = LCD_SEG_NUM7[data_temp_4h[3]];
	LCD_data[30] = LCD_SEG_NUM6[data_temp_4h[3]];
	LCD_data[31] = LCD_SEG_NUM5[data_temp_4h[3]];
	/////2
	LCD_data[26] = LCD_SEG_NUM7[data_temp_4h[2]];
	LCD_data[27] = LCD_SEG_NUM6[data_temp_4h[2]];
	LCD_data[28] = LCD_SEG_NUM5[data_temp_4h[2]];
	/////3
	LCD_data[23] = LCD_SEG_NUM7[data_temp_4h[1]];
	LCD_data[24] = LCD_SEG_NUM6[data_temp_4h[1]];
	LCD_data[25] = LCD_SEG_NUM5[data_temp_4h[1]];
	/////4
	LCD_data[20] = LCD_SEG_NUM7[data_temp_4h[0]];
	LCD_data[21] = LCD_SEG_NUM6[data_temp_4h[0]];
	LCD_data[22] = LCD_SEG_NUM5[data_temp_4h[0]];
	/////1
	if(data_temp_4h[3] == 1 || data_temp_4h[3] == 4 || data_temp_4h[3] == 10)LCD_data[17] &= ~(0X02);
	else LCD_data[17] |= 0X02;//1A
	/////2
	if(data_temp_4h[2] == 1 || data_temp_4h[2] == 4 || data_temp_4h[2] == 10)LCD_data[17] &= ~(0X01);
	else LCD_data[17] |= 0X01;//2A
	/////3
	if(data_temp_4h[1] == 1 || data_temp_4h[1] == 4 || data_temp_4h[1] == 10)LCD_data[18] &= ~(0X01);
	else LCD_data[18] |= 0X01;//3A
	///////4
	if(data_temp_4h[0] == 1 || data_temp_4h[0] == 4 || data_temp_4h[0] == 10)LCD_data[18] &= ~(0X02);
	else LCD_data[18] |= 0X02;//4A

}
//bool _99_59_99, _999_59_9, _9999_59;
void DIS_Dec_6h_2d(unsigned int data1, unsigned int data2)
{
	unsigned long temp=0;
	//int i, j = 0;

	if(_999_59_9)//999/599
	{
		if(data1 < 100)data_temp_6[5] = 10;
		else
			data_temp_6[5] = (unsigned char)(data1 /100);	
		if(data1<10)data_temp_6[4] = 10;
		else
		{
			temp = data1 % 100;
			data_temp_6[4] = (unsigned char)(temp / 10);
		}
		data_temp_6[3]= (unsigned char)(data1 % 10);
//////////////////////
		if(data2 < 100)data_temp_6[2] = 0;
		else
			data_temp_6[2] = (unsigned char)(data2 /100);	
		if(data2<10)data_temp_6[1] = 0;
		else
		{
			temp = data2 % 100;
			data_temp_6[1] = (unsigned char)(temp / 10);
		}
		data_temp_6[0]= (unsigned char)(data2 % 10);
	}
	else if(_99_59_99)//99/5999
	{
		if(data1<10)data_temp_6[5] = 10;
		else
		{
			temp = data1 % 100;
			data_temp_6[5] = (unsigned char)(temp / 10);
		}
		data_temp_6[4]= (unsigned char)(data1 % 10);
		//
		if(data2 < 1000)data_temp_6[3] = 0;
		else
			data_temp_6[3] = (unsigned char)(data2 /1000);
		if(data2<100)data_temp_6[2] = 0;
		else
		{
			temp = data2 % 1000;
			data_temp_6[2] = (unsigned char)(temp / 100);
		}
		if(data2<10)data_temp_6[1] = 0;
		else
		{
			temp = data2 % 100;
			data_temp_6[1] = (unsigned char)(temp / 10);
		}
		data_temp_6[0]= (unsigned char)(data2 % 10);
		
	}
	else//_9999_59
	{
		if(data1 < 1000)data_temp_6[5] = 10;
		else
			data_temp_6[5] = (unsigned char)(data1 /1000);
		if(data1<100)data_temp_6[4] = 10;
		else
		{
			temp = data1 % 1000;
			data_temp_6[4] = (unsigned char)(temp / 100);
		}
		if(data1<10)data_temp_6[3] = 10;
		else
		{
			temp = data1 % 100;
			data_temp_6[3] = (unsigned char)(temp / 10);
		}
		data_temp_6[2]= (unsigned char)(data1 % 10);
		//
		if(data2<10)data_temp_6[1] = 0;
		else
		{
			temp = data2 % 100;
			data_temp_6[1] = (unsigned char)(temp / 10);
		}
		data_temp_6[0]= (unsigned char)(data2 % 10);
		
	}

	PT16527(LCD_data);
	/////1
	LCD_data[29] = LCD_SEG_NUM7[data_temp_6[5]];
	LCD_data[30] = LCD_SEG_NUM6[data_temp_6[5]];
	LCD_data[31] = LCD_SEG_NUM5[data_temp_6[5]];
	/////2
	LCD_data[26] = LCD_SEG_NUM7[data_temp_6[4]];
	LCD_data[27] = LCD_SEG_NUM6[data_temp_6[4]];
	LCD_data[28] = LCD_SEG_NUM5[data_temp_6[4]];
	/////3
	LCD_data[23] = LCD_SEG_NUM7[data_temp_6[3]];
	LCD_data[24] = LCD_SEG_NUM6[data_temp_6[3]];
	LCD_data[25] = LCD_SEG_NUM5[data_temp_6[3]];
	///////4
	LCD_data[20] = LCD_SEG_NUM7[data_temp_6[2]];
	LCD_data[21] = LCD_SEG_NUM6[data_temp_6[2]];
	LCD_data[22] = LCD_SEG_NUM5[data_temp_6[2]];
	/////5
	LCD_data[17] = LCD_SEG_NUM7[data_temp_6[1]];
	LCD_data[18] = LCD_SEG_NUM6[data_temp_6[1]];
	LCD_data[19] = LCD_SEG_NUM5[data_temp_6[1]];
	/////6
	LCD_data[14] = LCD_SEG_NUM7[data_temp_6[0]];
	LCD_data[15] = LCD_SEG_NUM6[data_temp_6[0]];
	LCD_data[16] = LCD_SEG_NUM5[data_temp_6[0]];

	/////1
	if(data_temp_6[5] == 1 || data_temp_6[5] == 4 || data_temp_6[5] == 10)LCD_data[11] &= ~(0X01);
	else LCD_data[11] |= 0X01;//1A
	/////2
	if(data_temp_6[4] == 1 || data_temp_6[4] == 4 || data_temp_6[4] == 10)LCD_data[11] &= ~(0X02);
	else LCD_data[11] |= 0X02;//2A
	/////3
	if(data_temp_6[3] == 1 || data_temp_6[3] == 4 || data_temp_6[3] == 10)LCD_data[12] &= ~(0X01);
	else LCD_data[12] |= 0X01;//3A
	///////4
	if(data_temp_6[2] == 1 || data_temp_6[2] == 4 || data_temp_6[2] == 10)LCD_data[12] &= ~(0X02);
	else LCD_data[12] |= 0X02;//4A
	/////5
	if(data_temp_6[1] == 1 || data_temp_6[1] == 4 || data_temp_6[1] == 10)LCD_data[13] &= ~(0X01);
	else LCD_data[13] |= 0X01;//5A
	/////6
	if(data_temp_6[0] == 1 || data_temp_6[0] == 4 || data_temp_6[0] == 10)LCD_data[13] &= ~(0X02);
	else LCD_data[13] |= 0X02;//6A
	
}
void DIS_Dec_6h_3d(unsigned int data1, unsigned int data2, unsigned int data3)
{
	unsigned long temp=0;
	//int i, j = 0;
	if(data1<10)data_temp_6[5] = 10;
	else
	{
		temp = data1 % 100;
		data_temp_6[5] = (unsigned char)(temp / 10);
	}
	data_temp_6[4]= (unsigned char)(data1 % 10);
	
	if(data2<10)data_temp_6[3] = 0;
	else
	{
		temp = data2 % 100;
		data_temp_6[3] = (unsigned char)(temp / 10);
	}
	data_temp_6[2]= (unsigned char)(data2 % 10);
	
	if(data3<10)data_temp_6[1] = 0;
	else
	{
		temp = data3 % 100;
		data_temp_6[1] = (unsigned char)(temp / 10);
	}
	data_temp_6[0]= (unsigned char)(data3 % 10);
	
	PT16527(LCD_data);
	/////1
	LCD_data[29] = LCD_SEG_NUM7[data_temp_6[5]];
	LCD_data[30] = LCD_SEG_NUM6[data_temp_6[5]];
	LCD_data[31] = LCD_SEG_NUM5[data_temp_6[5]];
	/////2
	LCD_data[26] = LCD_SEG_NUM7[data_temp_6[4]];
	LCD_data[27] = LCD_SEG_NUM6[data_temp_6[4]];
	LCD_data[28] = LCD_SEG_NUM5[data_temp_6[4]];
	/////3
	LCD_data[23] = LCD_SEG_NUM7[data_temp_6[3]];
	LCD_data[24] = LCD_SEG_NUM6[data_temp_6[3]];
	LCD_data[25] = LCD_SEG_NUM5[data_temp_6[3]];
	///////4
	LCD_data[20] = LCD_SEG_NUM7[data_temp_6[2]];
	LCD_data[21] = LCD_SEG_NUM6[data_temp_6[2]];
	LCD_data[22] = LCD_SEG_NUM5[data_temp_6[2]];
	/////5
	LCD_data[17] = LCD_SEG_NUM7[data_temp_6[1]];
	LCD_data[18] = LCD_SEG_NUM6[data_temp_6[1]];
	LCD_data[19] = LCD_SEG_NUM5[data_temp_6[1]];
	/////6
	LCD_data[14] = LCD_SEG_NUM7[data_temp_6[0]];
	LCD_data[15] = LCD_SEG_NUM6[data_temp_6[0]];
	LCD_data[16] = LCD_SEG_NUM5[data_temp_6[0]];

	/////1
	if(data_temp_6[5] == 1 || data_temp_6[5] == 4 || data_temp_6[5] == 10)LCD_data[11] &= ~(0X01);
	else LCD_data[11] |= 0X01;//1A
	/////2
	if(data_temp_6[4] == 1 || data_temp_6[4] == 4 || data_temp_6[4] == 10)LCD_data[11] &= ~(0X02);
	else LCD_data[11] |= 0X02;//2A
	/////3
	if(data_temp_6[3] == 1 || data_temp_6[3] == 4 || data_temp_6[3] == 10)LCD_data[12] &= ~(0X01);
	else LCD_data[12] |= 0X01;//3A
	///////4
	if(data_temp_6[2] == 1 || data_temp_6[2] == 4 || data_temp_6[2] == 10)LCD_data[12] &= ~(0X02);
	else LCD_data[12] |= 0X02;//4A
	/////5
	if(data_temp_6[1] == 1 || data_temp_6[1] == 4 || data_temp_6[1] == 10)LCD_data[13] &= ~(0X01);
	else LCD_data[13] |= 0X01;//5A
	/////6
	if(data_temp_6[0] == 1 || data_temp_6[0] == 4 || data_temp_6[0] == 10)LCD_data[13] &= ~(0X02);
	else LCD_data[13] |= 0X02;//6A
	
}

void DIS_Dec_6(unsigned long data)
{
	unsigned long temp=0;
	//int i, j = 0;
	if(_99999_9)
	{
		if(data < 100000)data_temp_6[5] = 10;
		else
			data_temp_6[5] = (unsigned char)(data /100000);
			
		if(data<10000)data_temp_6[4] = 10;
		else
		{
			temp = data % 100000;
			data_temp_6[4] = (unsigned char)(temp / 10000);
		}
		
		if(data<1000)data_temp_6[3] = 10;
		else
		{
			temp = data % 10000;
			data_temp_6[3] = (unsigned char)(temp / 1000);
		}
		
		if(data<100)data_temp_6[2] = 10;
		else
		{
			temp = data % 1000;
			data_temp_6[2] = (unsigned char)(temp / 100);
		}
		
		if(data<10)data_temp_6[1] = 0;
		else
		{
			temp = data % 100;
			data_temp_6[1] = (unsigned char)(temp / 10);
		}
		data_temp_6[0]= (unsigned char)(data % 10);
	}
	else 
	{
		if(data < 100000)data_temp_6[5] = 10;
		else
			data_temp_6[5] = (unsigned char)(data /100000);
			
		if(data<10000)data_temp_6[4] = 10;
		else
		{
			temp = data % 100000;
			data_temp_6[4] = (unsigned char)(temp / 10000);
		}
		
		if(data<1000)data_temp_6[3] = 10;
		else
		{
			temp = data % 10000;
			data_temp_6[3] = (unsigned char)(temp / 1000);
		}
		
		if(data<100)data_temp_6[2] = 10;
		else
		{
			temp = data % 1000;
			data_temp_6[2] = (unsigned char)(temp / 100);
		}
		
		if(data<10)data_temp_6[1] = 10;
		else
		{
			temp = data % 100;
			data_temp_6[1] = (unsigned char)(temp / 10);
		}
		data_temp_6[0]= (unsigned char)(data % 10);
	}

	if(minus)
	{
		if(data_temp_6[4] < 10)
		{
			LCD_data[30] = LCD_SEG_NUM6[11];// 100의 자리가 10보다 작으면 1000의 자리 마이너스 표시 
			LCD_data[31] = LCD_SEG_NUM5[11];// 100의 자리가 10보다 작으면 1000의 자리 마이너스 표시 
		}
		else if(data_temp_6[3] < 10)
		{
			LCD_data[27] = LCD_SEG_NUM6[11];// 10의 자리가 10보다 작으면 100의 자리 마이너스 표시
			LCD_data[28] = LCD_SEG_NUM5[11];// 10의 자리가 10보다 작으면 100의 자리 마이너스 표시 
		}
		else if(data_temp_6[2] < 10)
		{
			LCD_data[24] = LCD_SEG_NUM6[11];// 100의 자리가 10보다 작으면 1000의 자리 마이너스 표시 
			LCD_data[25] = LCD_SEG_NUM5[11];// 100의 자리가 10보다 작으면 1000의 자리 마이너스 표시 
		}
		else if(data_temp_6[1] < 10)
		{
			LCD_data[21] = LCD_SEG_NUM6[11];// 10의 자리가 10보다 작으면 100의 자리 마이너스 표시
			LCD_data[22] = LCD_SEG_NUM5[11];// 10의 자리가 10보다 작으면 100의 자리 마이너스 표시 
		}
		else if(data_temp_6[0] < 10)
		{
			if(data_temp_6[0] == 0)
			{
				LCD_data[18] = LCD_SEG_NUM6[data_temp_6[1]];
				LCD_data[19] = LCD_SEG_NUM5[data_temp_6[1]];
			}
			else 
			{
				LCD_data[18] = LCD_SEG_NUM6[11];// 1의 자리가 10보다 작으면 10의 자리 마이너스 표시 
				LCD_data[19] = LCD_SEG_NUM5[11];
			}
		}
		else LCD_data[0] = 0xff; // 1000의 자리 0 표시
	}
	
	PT16527(LCD_data);
	/////1
	LCD_data[29] = LCD_SEG_NUM7[data_temp_6[5]];
	LCD_data[30] = LCD_SEG_NUM6[data_temp_6[5]];
	LCD_data[31] = LCD_SEG_NUM5[data_temp_6[5]];
	/////2
	LCD_data[26] = LCD_SEG_NUM7[data_temp_6[4]];
	LCD_data[27] = LCD_SEG_NUM6[data_temp_6[4]];
	LCD_data[28] = LCD_SEG_NUM5[data_temp_6[4]];
	/////3
	LCD_data[23] = LCD_SEG_NUM7[data_temp_6[3]];
	LCD_data[24] = LCD_SEG_NUM6[data_temp_6[3]];
	LCD_data[25] = LCD_SEG_NUM5[data_temp_6[3]];
	///////4
	LCD_data[20] = LCD_SEG_NUM7[data_temp_6[2]];
	LCD_data[21] = LCD_SEG_NUM6[data_temp_6[2]];
	LCD_data[22] = LCD_SEG_NUM5[data_temp_6[2]];
	/////5
	LCD_data[17] = LCD_SEG_NUM7[data_temp_6[1]];
	LCD_data[18] = LCD_SEG_NUM6[data_temp_6[1]];
	LCD_data[19] = LCD_SEG_NUM5[data_temp_6[1]];
	/////6
	LCD_data[14] = LCD_SEG_NUM7[data_temp_6[0]];
	LCD_data[15] = LCD_SEG_NUM6[data_temp_6[0]];
	LCD_data[16] = LCD_SEG_NUM5[data_temp_6[0]];

	/////1
	if(data_temp_6[5] == 1 || data_temp_6[5] == 4 || data_temp_6[5] == 10)LCD_data[11] &= ~(0X01);
	else LCD_data[11] |= 0X01;//1A
	/////2
	if(data_temp_6[4] == 1 || data_temp_6[4] == 4 || data_temp_6[4] == 10)LCD_data[11] &= ~(0X02);
	else LCD_data[11] |= 0X02;//2A
	/////3
	if(data_temp_6[3] == 1 || data_temp_6[3] == 4 || data_temp_6[3] == 10)LCD_data[12] &= ~(0X01);
	else LCD_data[12] |= 0X01;//3A
	///////4
	if(data_temp_6[2] == 1 || data_temp_6[2] == 4 || data_temp_6[2] == 10)LCD_data[12] &= ~(0X02);
	else LCD_data[12] |= 0X02;//4A
	/////5
	if(data_temp_6[1] == 1 || data_temp_6[1] == 4 || data_temp_6[1] == 10)LCD_data[13] &= ~(0X01);
	else LCD_data[13] |= 0X01;//5A
	/////6
	if(data_temp_6[0] == 1 || data_temp_6[0] == 4 || data_temp_6[0] == 10)LCD_data[13] &= ~(0X02);
	else LCD_data[13] |= 0X02;//6A
	
}
void ALPHA_DIS_4h(unsigned char *display_data)
{
	unsigned char i=0, j=4;
	unsigned char c=0;

	for(i=0; i<4; i++)
	{	
		j--;
		if(display_data[i] >= 'a')buf_small[j] = (display_data[i] - 'a') + 12;
		else if(display_data[i] >= 'A')buf_small[j] = (display_data[i] - 'A') + 12;
		else if(display_data[i] == ' ')buf_small[j] = 10;
		else
			buf_small[j] = display_data[i] - '0';
	}
	PT16527(LCD_data);
	/////3
	LCD_data[23] = LCD_SEG_NUM7[data_temp_6[3]];
	LCD_data[24] = LCD_SEG_NUM6[data_temp_6[3]];
	LCD_data[25] = LCD_SEG_NUM5[data_temp_6[3]];
	///////4
	LCD_data[20] = LCD_SEG_NUM7[data_temp_6[2]];
	LCD_data[21] = LCD_SEG_NUM6[data_temp_6[2]];
	LCD_data[22] = LCD_SEG_NUM5[data_temp_6[2]];
	/////5
	LCD_data[17] = LCD_SEG_NUM7[data_temp_6[1]];
	LCD_data[18] = LCD_SEG_NUM6[data_temp_6[1]];
	LCD_data[19] = LCD_SEG_NUM5[data_temp_6[1]];
	/////6
	LCD_data[14] = LCD_SEG_NUM7[data_temp_6[0]];
	LCD_data[15] = LCD_SEG_NUM6[data_temp_6[0]];
	LCD_data[16] = LCD_SEG_NUM5[data_temp_6[0]];
}

void ALPHA_DIS_6(unsigned char *display_data)
{
	unsigned char i=0, j=4;
	unsigned char c=0;

	for(i=0; i<4; i++)
	{	
		j--;
		if(display_data[i] >= 'a')buf_small[j] = (display_data[i] - 'a') + 12;
		else if(display_data[i] >= 'A')buf_small[j] = (display_data[i] - 'A') + 12;
		else if(display_data[i] == ' ')buf_small[j] = 10;
		else
			buf_small[j] = display_data[i] - '0';
	}
	PT16527(LCD_data);
	/////1
	LCD_data[29] = LCD_SEG_NUM7[data_temp_6[5]];
	LCD_data[30] = LCD_SEG_NUM6[data_temp_6[5]];
	LCD_data[31] = LCD_SEG_NUM5[data_temp_6[5]];
	/////2
	LCD_data[26] = LCD_SEG_NUM7[data_temp_6[4]];
	LCD_data[27] = LCD_SEG_NUM6[data_temp_6[4]];
	LCD_data[28] = LCD_SEG_NUM5[data_temp_6[4]];
	/////3
	LCD_data[23] = LCD_SEG_NUM7[data_temp_6[3]];
	LCD_data[24] = LCD_SEG_NUM6[data_temp_6[3]];
	LCD_data[25] = LCD_SEG_NUM5[data_temp_6[3]];
	///////4
	LCD_data[20] = LCD_SEG_NUM7[data_temp_6[2]];
	LCD_data[21] = LCD_SEG_NUM6[data_temp_6[2]];
	LCD_data[22] = LCD_SEG_NUM5[data_temp_6[2]];
	/////5
	LCD_data[17] = LCD_SEG_NUM7[data_temp_6[1]];
	LCD_data[18] = LCD_SEG_NUM6[data_temp_6[1]];
	LCD_data[19] = LCD_SEG_NUM5[data_temp_6[1]];
	/////6
	LCD_data[14] = LCD_SEG_NUM7[data_temp_6[0]];
	LCD_data[15] = LCD_SEG_NUM6[data_temp_6[0]];
	LCD_data[16] = LCD_SEG_NUM5[data_temp_6[0]];
}

/* I2C-EEPROM -----------------------------------------------------------*/
void I2C_Config(void) {
	CLK->PCKENR1 |= 0x08;  // SYSCLK to peripheral enabled I2C1
  I2C1->FREQR   = 0x10;  // 16 MHz
  I2C1->CCRH    = 0x00;
  I2C1->CCRL    = 0x50;  // CCR=100KHz/2/16MHz=10000ns/2/62.5ns=0x50
  I2C1->OARH    = 0x40;  // Address mode configuration
  I2C1->TRISER  = 0x11;  // Maximum rise time in Fast/Standard mode
  I2C1->CR1    |= 0x01;  // Peripheral enable
	//I2C1->CR2  = 0X05;
}
void EEPROM_write(uint16_t addr, uint8_t data) {
  //I2C_write_ADDRESS
	I2C1->CR2 |= 0x04;              // Acknowledge returned after a byte is received
  while((I2C1->SR3&0x02)==0x02);  // Bus busy
  I2C1->CR2 |= 0x01;
  while((I2C1->SR1&0x01)!=0x01);  // EV5 Start bit (Master mode)
	I2C1->DR = ADDRESS&0xFE;
  while((I2C1->SR1&0x82)!=0x82);  // EV6 Master mode, EV8_1 Data register empty
  (void) I2C1->SR3;               // clear EV6  I2C1->SR1:0x82->0x80
  //I2C_write_addr
  I2C1->DR = (addr>>8)&0xFF;
  while((I2C1->SR1&0x04)!=0x04);  // EV8 Data byte transfer succeeded
  I2C1->DR = addr&0xFF;
  while((I2C1->SR1&0x04)!=0x04);  // EV8 Data byte transfer succeeded
  //I2C_write_data
  I2C1->DR = (unsigned char)(data & 0XFF);
  while((I2C1->SR1&0x84)!=0x84);  // EV8_2 Data byte transfer succeeded
	I2C1->CR2 |= 0x02;
  while((I2C1->SR3&0x01)!=0x01);
}

uint8_t EEPROM_read(uint16_t addr) {
  uint8_t data;
  //I2C_start
  I2C1->CR2  = 0x04;              // Acknowledge returned after a byte is received
  while((I2C1->SR3&0x02)==0x02);  // Bus busy
	I2C1->CR2 |= 0x01;
  while((I2C1->SR1&0x01)!=0x01);  // EV5 Start bit (Master mode)
  //I2C_write_ADDRESS
  I2C1->DR = ADDRESS&0xFE;
  while((I2C1->SR1&0x02)!=0x02);  // EV6 Master mode
  (void) I2C1->SR3;               // clear EV6
  //I2C_write_addr
  I2C1->DR = (addr>>8)&0xFF;
  while((I2C1->SR1&0x04)!=0x04);  // EV8 Data byte transfer succeeded
  I2C1->DR = addr&0xFF;
  while((I2C1->SR1&0x04)!=0x04);  // EV8 Data byte transfer succeeded
  //I2C_start
  I2C1->CR2 |= 0x01;
  while((I2C1->SR1&0x01)!=0x01);  // EV5 Start bit (Master mode)
  //I2C_write_ADDRESS
  I2C1->DR = ADDRESS|0x01;
  while((I2C1->SR1&0x02)!=0x02);  // EV6 Master mode
  (void) I2C1->SR3;               // clear EV6
  //I2C_read
	I2C1->CR2 &= ~0x04;             // No acknowledge returned
  while((I2C1->SR1&0x40)!=0x40);  // EV7_1 Data register not empty
  data = I2C1->DR;
  //I2C_stop
  I2C1->CR2 |= 0x02;
  while((I2C1->SR3&0x01)!=0x01);
  return data;
}

void eeprom_save_4h(void)
{
	EEPROM_write(1, data_temp_4h[3]);
	delay_1ms(10);
	EEPROM_write(2, data_temp_4h[2]);
	delay_1ms(10);
	EEPROM_write(3, data_temp_4h[1]);
	delay_1ms(10);
	EEPROM_write(4, data_temp_4h[0]);
	delay_1ms(10);
}
void eeprom_save_6(void)
{
	EEPROM_write(1, data_temp_6[5]);
	delay_1ms(10);
	EEPROM_write(2, data_temp_6[4]);
	delay_1ms(10);
	EEPROM_write(1, data_temp_6[3]);
	delay_1ms(10);
	EEPROM_write(2, data_temp_6[2]);
	delay_1ms(10);
	EEPROM_write(3, data_temp_6[1]);
	delay_1ms(10);
	EEPROM_write(4, data_temp_6[0]);
	delay_1ms(10);
}
void eeprom_road_4h(void)
{
	data_temp_4h[3] = EEPROM_read(1);delay_1ms(5);
	data_temp_4h[2] = EEPROM_read(2);delay_1ms(5);
	data_temp_4h[1] = EEPROM_read(3);delay_1ms(5);
	data_temp_4h[0] = EEPROM_read(4);delay_1ms(5);
	eeprom_data1_4h = (unsigned int)(data_temp_4h[3]*1000) + (unsigned int)(data_temp_4h[2]*100) + (unsigned int)(data_temp_4h[1]*10) + (unsigned int)(data_temp_4h[0]*1);
}
void eeprom_road_6(void)
{
	data_temp_6[5] = EEPROM_read(1);delay_1ms(5);
	data_temp_6[4] = EEPROM_read(2);delay_1ms(5);
	data_temp_6[3] = EEPROM_read(3);delay_1ms(5);
	data_temp_6[2] = EEPROM_read(4);delay_1ms(5);
	data_temp_6[1] = EEPROM_read(5);delay_1ms(5);
	data_temp_6[0] = EEPROM_read(6);delay_1ms(5);
	eeprom_data1_6 = (unsigned int)(data_temp_6[5]*100000) + (unsigned int)(data_temp_6[4]*10000) + (unsigned int)(data_temp_6[3]*1000) + (unsigned int)(data_temp_6[2]*100) + (unsigned int)(data_temp_6[1]*10) + (unsigned int)(data_temp_6[0]*1);
}
/* USART-PRINTF -----------------------------------------------------------*/
void USART_Config(void) {
  CLK->PCKENR1 |= 0x20; // SYSCLK to peripheral enabled USART
  GPIOC->DDR |= 0x08;   // USART RX:PC2, TX:PC3
  GPIOC->CR1 |= 0x08;
  USART1->BRR2 = 0x0B;	// 16000000/9600=1667=0x0683, 16000000/115200=139=0x008B
	USART1->BRR1 = 0x08;  
  USART1->CR2  = 0x08;  // Transmitter enable
  USART1->CR2 |= 0x20;  // Receiver interrupt enable
}
void TX_CH(char ch) {
  while(!(USART1->SR & 0x80));  // Transmit data register empty
  USART1->DR = ch;
}
void TX_STR(char* str) {
  while(*str) TX_CH(*str++);
}
char putchar(char ch) {
  TX_CH(ch);
  return ch;
}


/* INPUT_MODE -----------------------------------------------------------*/

//==============================================================
//==============================================================
//
//
//
//					SEGMENT 4H INPUT MODE START
//
//
//
//==============================================================
//==============================================================


//==============================================================
//					SEGMENT 4H INPUT MODE UP A================================================================================================================
//==============================================================

void M_up_down_a_4h(void)//지령입력
{
	DP000_0 = 0;
	DP00_00 = 0;
	DP0_000 = 1;
	RESET_ON = 1;
	
	if(DP0_000)
	{
		if(minus && data1_4h > 0)LCD_data[26] |= 0x08;//
		else LCD_data[29] |= 0x08;//
	}
	else if(DP00_00)
	{
		LCD_data[26] |= 0x08;//
	}
	else if(DP000_0)
	{
		LCD_data[23] |= 0x08;//
	}
	else
	{
		LCD_data[29] &= ~(0x08);//
		LCD_data[26] &= ~(0x08);//
		LCD_data[23] &= ~(0x08);//
	}
	
	if(RST_INPUT_ON)//GPIOB->IDR & 1<<3
	{
		rst_input_flag=1;
		if((rst_input_cnt >= 50) && rst_key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1_4h = 0; 
			DIS_Dec_4h(data1_4h);
			rst_key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		rst_input_flag=1;
		if((rst_input_cnt >= 50) && rst_key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1_4h = 0; 
			DIS_Dec_4h(data1_4h);
			rst_key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		rst_key_check = 0;
		rst_input_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		
		if(exti_signal)
		{			
			if(GPIOB->IDR & 1<<1)
			{
				if((exti_cnt >= 10) && key_check == 0)
				{
					exti_cnt = 10;
					if(GPIOB->IDR & 1<<2)
					{
						if((up_mode_cnt >= 10) && CP2_key_check == 0)
						{
							up_mode_cnt = 10;
							if(minus)data1_4h++;
							else data1_4h--;
							CP2_key_check = 1;
						}
						DIS_Dec_4h(data1_4h);
					}
					else
					{					
						if(minus)data1_4h--;
						else data1_4h++;
					}
					key_check = 1;
				}
				DIS_Dec_4h(data1_4h);
			}
			else exti_signal = 0;
		}
		else
		{
			CP2_key_check = 0;
			key_check = 0;
			DIS_Dec_4h(data1_4h);
		}
		
		if(minus)
		{
			if(data1_4h > 999) data1_4h = 0;
		}
		else
		{
			if(data1_4h > 9999) data1_4h = 0;
		}
		
		if(data1_4h == 0)
		{
			if(GPIOB->IDR & 1<<2)minus = 1;
			else minus = 0;
		}
	}
DIS_Dec_4h(data1_4h);

}
//==============================================================
//					SEGMENT 4H INPUT MODE UP B
//==============================================================

void M_up_down_b_4h(void)//개별입력 
{
	if((GPIOB->IDR & 1<<1)&&old_sw == 0)
	{signal = 1;
		if((signal_cnt >= 20) && key_check == 0)
		{
			signal_cnt = 20;
			
			if(minus)data1_4h--;
			else data1_4h++;
			//printf("1111111\r\n");
			key_check = 1;
		}
		if(TACT_KEY->IDR & 1<<2)
		{
			if((up_mode_cnt >= 20) && (new_sw == 0))
			{
				up_mode_cnt = 20;
				
				if(minus)data1_4h++;
				else data1_4h--;
				//printf("3333333\r\n");
				new_sw = 1;
			}
			DIS_Dec_4h(data1_4h);
			//printf("zzzzzzzzzzzzzzzzzzz\r\n");
		}
		else new_sw = 0;
		
		DIS_Dec_4h(data1_4h);
		//printf("2222222\r\n");
	}
	else if((TACT_KEY->IDR & 1<<2) && key_check == 0)// 1 0
	{
		if((up_mode_cnt >= 20) && (old_sw == 0))
		{
			up_mode_cnt = 20;
			
			if(minus)data1_4h++;
			else data1_4h--;
			//printf("3333333\r\n");
			old_sw = 1;
		}
		if(GPIOB->IDR & 1<<1)
		{
			if((down_mode_cnt >= 20) && but_check == 0)
			{
				down_mode_cnt = 20;
				
				if(minus)data1_4h--;
				else data1_4h++;
				//printf("1111111\r\n");
				but_check = 1;
			}
			DIS_Dec_4h(data1_4h);
			//printf("zzzzzzzzzzzzzzzzzzz\r\n");
			
		}
		else but_check = 0;
			
		DIS_Dec_4h(data1_4h);
		//printf("444444444\r\n");
	}
	else
	{
		but_check = 0;
		new_sw = 0;
		old_sw = 0;
		signal = 0;
		key_check = 0;
		DIS_Dec_4h(data1_4h);
	}
	
	if(minus)
	{
		if(data1_4h > 999) data1_4h = 0;
	}
	else
	{
		if(data1_4h > 9999) data1_4h = 0;
	}
	if(data1_4h == 0)
	{
		if((TACT_KEY->IDR & 1<<2) && but_check == 0)
		{
			if(TACT_KEY->IDR & 1<<1)
			{
				minus = 0;
				printf("111111\r\n");
			}
			else 
			{
				minus = 1;
				printf("222222222222\r\n");
			}
		}
		
		if((TACT_KEY->IDR & 1<<1) && old_sw == 0)
		{
			if(TACT_KEY->IDR & 1<<2)
			{
				minus = 1;
				printf("333333\r\n");
			}
			else
			{
				minus = 0;
				printf("44444444\r\n");
			}
		}
	}
	
	if(RST_INPUT_ON)
	{
		printf("main_end!! \r\n");
		data1_4h = 0; 
		DIS_Dec_4h(data1_4h);
	}
	
	if(RESET_ON)
	{
		if(GPIOG->IDR & 1<<2)
		{
			data1_4h = 0; 
			DIS_Dec_4h(data1_4h);
		}
	}	
	
}
//==============================================================
//					SEGMENT 4H INPUT MODE UP C
//==============================================================

void minus_data1_4h(void)
{
	if(data1_4h == 0)
	{
		if(minus_data == 0) minus = 0;
		else minus = 1;
	}
}
void M_up_down_c_4h(void)//위상차 입력 
{
	if((exti_signal) && CCW == 0)
	{
		if((GPIOB->IDR & 1<<1) && CCW == 0)//high1
		{
			if(exti_cnt >= 50)
			{minus_data = 1;
				exti_cnt = 50;
				printf("CCW\r\n");
				//delay_1ms(1000);
				CW = 1;
				minus_data1_4h();
				if(seg_stop_flag == 0)
				{
					if(minus)
					{printf("in\r\n");
						data1_4h++;
						plus = 1;
					}
					else
					{
						data1_4h--;
						plus = 0;
					}
					rotate = 1;
					DIS_Dec_4h(data1_4h);
					printf("DIS_Dec %d\r\n", data1_4h);
					seg_stop_flag = 1;
					
				}
				if(TACT_KEY->IDR & 1<<2)//high2
				{
					signal = 1;
					if(signal_cnt >= 50)
					{
						signal_cnt = 50;
						high_in = 1;
						
						printf("high_in\r\n");
						//delay_1ms(1000);
					}
				}
				else
				{
					DIS_Dec_4h(data1_4h);
					high_in = 0; 
					signal = 0;
				}
			}
		}
		else exti_signal = 0;
	}
	else if((TACT_KEY->IDR & 1<<2) && CW == 0)//high2
	{
		signal = 1;
		if(signal_cnt >= 50)//high1
		{minus_data = 0;
			signal_cnt = 50;
			printf("CW\r\n");
			//delay_1ms(1000);
			CCW = 1;
			
			if((exti_signal)&& CCW)
			{
				if(GPIOB->IDR & 1<<1)//high
				{printf("low_in\r\n");
					if(exti_cnt >= 50)
					{
						exti_cnt = 50;
						low_in = 1;
						
						printf("low_in\r\n");
						//delay_1ms(1000);
					}
				}
				else 
				{
					low_in = 0; 
					exti_signal = 0;
				}
			}
		}
	}
	else if((GPIOB->IDR & 1<<1) == 0 && high_in == 1)
	{
		printf("else if1\r\n");
		//delay_1ms(1000);
		if((TACT_KEY->IDR & 1<<2) == 0)
		{
			rotate = 0;
			printf("out1\r\n");
			//delay_1ms(1000);
			high_in = 0;
			CW = 0;
		}
	}
	else if((TACT_KEY->IDR & 1<<2) == 0 && low_in == 1)
	{
		printf("else if2\r\n");
		//delay_1ms(1000);
		if((GPIOB->IDR & 1<<1) == 0)
		{
			minus_data1_4h();
			if(seg_stop_flag == 0)
			{
				if(minus)
				{
					data1_4h--;
					//plus = 0;
				}
				else
				{
					data1_4h++;
					//plus = 1;
				}
				DIS_Dec_4h(data1_4h);
				printf("DIS_Dec %d\r\n", data1_4h);
				seg_stop_flag = 1;
			}
			
			printf("out2\r\n");
			//delay_1ms(1000);
			low_in = 0;
			CCW = 0;
		}
	}
	else
	{
		if(rotate)
		{
			if(plus){
				data1_4h--;
				DIS_Dec_4h(data1_4h);
				rotate = 0;
				printf("if2\r\n");
			}
			else{
				data1_4h++;
				DIS_Dec_4h(data1_4h);
				rotate = 0;
				printf("if2\r\n");
			}
		}
		else
		{
			//exti_signal = 0;
			seg_stop_flag = 0;
			plus = 0;
			printf("else1\r\n");
			CCW = 0;
			CW = 0;
			signal = 0;
			DIS_Dec_4h(data1_4h);
			//delay_1ms(1000);
		}
	}
	
	if(minus)
	{
		if(data1_4h > 999) data1_4h = 0;
	}
	else
	{
		if(data1_4h > 9999) data1_4h = 0;
	}

	if(RST_INPUT_ON)
	{
		printf("main_end!! \r\n");
		data1_4h = 0; 
		DIS_Dec_4h(data1_4h);
	}
	
	if(RESET_ON)
	{
		if(GPIOG->IDR & 1<<2)
		{
			data1_4h = 0; 
			DIS_Dec_4h(data1_4h);
		}
	}	
}
//==============================================================
//					SEGMENT 4H INPUT MODE UP UP
//==============================================================

void M_up_4h(void)
{
	if(exti_signal)
	{
		if(GPIOB->IDR & 1<<1)
		{
			if((exti_cnt >= 50) && CP1_key_check == 0)
			{
				if(TACT_KEY->IDR & 1<<2)
				{
					signal = 1;
					if((signal_cnt >= 20) && (CP2_key_check == 0))
					{
						CP2_key_check = 1;
						key_check = 1;
					}
					printf("TACT_KEY\r\n");
				}
				else
				{
					CP1_key_check = 0;
					CP2_key_check = 0;
					
					if(key_check){
						data1_4h++;
						key_check = 0;
					}
					
					printf("else\r\n");
					//DIS_Dec_4h(data1_4h);
				}
			}
			//printf("exti_signal %d\r\n", data1_4h);
			DIS_Dec_4h(data1_4h);
		}
		else exti_signal = 0;
	}
	else
	{
		key_check = 1;
		signal = 0;
		CP1_key_check = 0;
		CP2_key_check = 0;
		DIS_Dec_4h(data1_4h);
	}
	
	if(data1_4h > 9999) data1_4h = 0; //DIS_Dec(data1);


	if(RST_INPUT_ON)
	{
		printf("main_end!! \r\n");
		data1_4h = 0; 
		DIS_Dec_4h(data1_4h);
	}
	
	if(RESET_ON)
	{
		if(GPIOG->IDR & 1<<2)
		{
			data1_4h = 0; 
			DIS_Dec_4h(data1_4h);
		}
	}	
}
//==============================================================
//					SEGMENT 4H INPUT MODE DOWN D
//==============================================================

void M_up_down_d_4h(void)//지령 입력
{
	if(exti_signal)
	{			
		if(GPIOB->IDR & 1<<1)
		{
			if((exti_cnt >= 10) && key_check == 0)
			{
				exti_cnt = 10;
				if(GPIOB->IDR & 1<<2)
				{
					if((up_mode_cnt >= 10) && CP2_key_check == 0)
					{
						up_mode_cnt = 10;
						if(minus)M_down_data1_4h--;
						else M_down_data1_4h++;
						CP2_key_check = 1;
					}
				  DIS_Dec_4h(M_down_data1_4h);
				}
				else
				{					
					if(minus)M_down_data1_4h++;
						else M_down_data1_4h--;
				}
				key_check = 1;
			}
			DIS_Dec_4h(M_down_data1_4h);
		}
		else exti_signal = 0;
	}
	else
	{
		CP2_key_check = 0;
		key_check = 0;
		DIS_Dec_4h(M_down_data1_4h);

	}

	if(minus)
	{
		if(M_down_data1_4h > 999) M_down_data1_4h = 0;
	}
	else
	{
		if(M_down_data1_4h > 9999) M_down_data1_4h = 0;
	}

	if(M_down_data1_4h == 0)
	{
		if((GPIOB->IDR & 1<<1)&&(GPIOB->IDR & 1<<2)) minus = 0;
		if((GPIOB->IDR & 1<<1)&&(GPIOB->IDR & 1<<2)==0) minus = 1;
	}
	
	if(RST_INPUT_ON)
	{
		printf("main_end!! \r\n");
		M_down_data1_4h = 9999; 
		DIS_Dec_4h(M_down_data1_4h);
	}
	
	if(RESET_ON)
	{
		if(GPIOG->IDR & 1<<2)
		{
			M_down_data1_4h = 9999; 
			DIS_Dec_4h(M_down_data1_4h);
		}
	}	
}
//==============================================================
//					SEGMENT 4H INPUT MODE DOWN E
//==============================================================

void M_up_down_e_4h(void)//개별 입력 
{
	if((GPIOB->IDR & 1<<1)&&old_sw == 0)
	{signal = 1;
		if((signal_cnt >= 20) && key_check == 0)
		{
			signal_cnt = 20;
			
			if(minus)M_down_data1_4h++;
			else M_down_data1_4h--;
			//printf("1111111\r\n");
			key_check = 1;
		}
		if(TACT_KEY->IDR & 1<<2)
		{
			if((up_mode_cnt >= 20) && (new_sw == 0))
			{
				up_mode_cnt = 20;
				
				if(minus)M_down_data1_4h--;
				else M_down_data1_4h++;
				//printf("3333333\r\n");
				new_sw = 1;
			}
			DIS_Dec_4h(M_down_data1_4h);
			//printf("zzzzzzzzzzzzzzzzzzz\r\n");
		}
		else new_sw = 0;
		
		DIS_Dec_4h(M_down_data1_4h);
		//printf("2222222\r\n");
	}
	else if((TACT_KEY->IDR & 1<<2) && key_check == 0)// 1 0
	{
		if((up_mode_cnt >= 20) && (old_sw == 0))
		{
			up_mode_cnt = 20;
			
			if(minus)M_down_data1_4h--;
			else M_down_data1_4h++;
			//printf("3333333\r\n");
			old_sw = 1;
		}
		if(GPIOB->IDR & 1<<1)
		{
			if((down_mode_cnt >= 20) && but_check == 0)
			{
				down_mode_cnt = 20;
				
				if(minus)M_down_data1_4h++;
				else M_down_data1_4h--;
				//printf("1111111\r\n");
				but_check = 1;
			}
			DIS_Dec_4h(M_down_data1_4h);
			//printf("zzzzzzzzzzzzzzzzzzz\r\n");
			
		}
		else but_check = 0;
			
		DIS_Dec_4h(M_down_data1_4h);
		//printf("444444444\r\n");
	}
	else
	{
		but_check = 0;
		new_sw = 0;
		old_sw = 0;
		signal = 0;
		key_check = 0;
		DIS_Dec_4h(M_down_data1_4h);
	}
	
	if(minus)
	{
		if(M_down_data1_4h > 999) M_down_data1_4h = 0;
	}
	else
	{
		if(M_down_data1_4h > 9999) M_down_data1_4h = 0;
	}

	//minus 표준 
	if(M_down_data1_4h == 0)
	{
		if((GPIOB->IDR & 1<<1) && minus_low_check == 0)
		{minus_high_check = 1;
					if(GPIOB->IDR & 1<<2) minus = 0;
					else minus = 1;
		}
		else if((GPIOB->IDR & 1<<2) && minus_high_check == 0)
		{minus_low_check = 1;
					if(GPIOB->IDR & 1<<1) minus = 1;
					else minus = 0;
		}
		else
		{
			minus_high_check = 0;
			minus_low_check = 0;
		}
	}
	
	if(RST_INPUT_ON)
	{
		printf("RST\r\n");
		minus = 0;
		M_down_data1_4h = 9999; 
		DIS_Dec_4h(M_down_data1_4h);
	}
	
	if(RESET_ON)
	{
		if(GPIOG->IDR & 1<<2)
		{
			M_down_data1_4h = 9999; 
			DIS_Dec_4h(M_down_data1_4h);
		}
	}	
		
}

//==============================================================
//					SEGMENT 4H INPUT MODE DOWN F
//==============================================================

void M_up_down_f_4h(void)//위상차 입력 
{
	if((GPIOB->IDR & 1<<1) && (GPIOB->IDR & 1<<2)==0 && second_cp2_in == 0)//11111zzzzzzzzzzzzzzzzzzzzzzz
	{
		if((down_mode_cnt >= 50) && CW == 0)
		{
			if(minus)
			{
				M_down_data1_4h--;
			}
			else
			{
				M_down_data1_4h++;
			}
			
			//DIS_Dec_4h(M_down_data1_4h);
			
			first_cp1_in = 1;
			third_cp1_in = 0;
			second_cp1_in = 0;
			
			rotate_cp1 = 1;
			
			CW = 1;
		}
		
		if(M_down_data1_4h == 0)
		{
			if(GPIOB->IDR & 1<<2)
			{
				minus = 1;
				printf("minus %d \r\n", minus);
			}
			else minus = 0;
		}
		
		DIS_Dec_4h(M_down_data1_4h);
		printf("first_cp1_in \r\n");
	}
	else if((GPIOB->IDR & 1<<1) && (GPIOB->IDR & 1<<2) && first_cp1_in == 1)//2222222
	{
		if((up_mode_cnt >= 50) && CCW == 0)
		{
			second_cp1_in = 1;
			third_cp1_in = 0;
			
			rotate_cp1 = 1;
			
			CCW = 1;
		}
		
		printf("second_cp1_in \r\n");
	}
	else if((GPIOB->IDR & 1<<1)== 0 && (GPIOB->IDR & 1<<2) && second_cp1_in == 1)//3333333
	{
		if((up_mode_cnt > 50) && CW_CCW == 0)
		{
			third_cp1_in = 1;
			
			rotate_cp1 = 0;
			
			CW_CCW = 1;
		}
		
		printf("third_cp1_in \r\n");
	}
	else if((GPIOB->IDR & 1<<1)== 0 && (GPIOB->IDR & 1<<2) && second_cp1_in == 0)//33333333
	{
		if((up_mode_cnt >= 50) && CCW == 0)
		{
			first_cp2_in = 1;
			third_cp2_in = 0;
			second_cp2_in = 0;
			
			rotate_cp2 = 1;
			
			CCW = 1;
		}
		
		if(M_down_data1_4h == 0)
		{
			if(GPIOB->IDR & 1<<2)
			{
				minus = 1;
				printf("minus %d \r\n", minus);
			}
			else minus = 0;
		}
		
		printf("first_cp2_in \r\n");
	}
	else if((GPIOB->IDR & 1<<1) && (GPIOB->IDR & 1<<2)&& first_cp2_in == 1)//222222
	{
		if((down_mode_cnt >= 50) && CW == 0)
		{
			second_cp2_in = 1;
			third_cp2_in = 0;
			
			rotate_cp2 = 1;
			
			CW = 1;
		}
		
		printf("second_cp2_in %d\r\n", minus);
	}
	else if((GPIOB->IDR & 1<<1) && (GPIOB->IDR & 1<<2)==0 && second_cp2_in == 1)//1111111
	{
		if((down_mode_cnt >= 50) && CW_CCW == 0)
		{
			
			third_cp2_in = 1;
			rotate_cp2 = 0;
			
			CW_CCW = 1;
		}
		
		printf("third_cp2_in \r\n");
	}
	else
	{
		if(rotate_cp1 && third_cp1_in == 0){
			//--
			if(minus)M_down_data1_4h++;
			else
			{
				if(data_9999){
					minus = 1;
					M_down_data1_4h++;
				}
				else M_down_data1_4h--;
			}
			
			rotate_cp1 = 0;
			printf("111111\r\n");
		}
		if(rotate_cp1 == 0 && third_cp1_in){
			//
			third_cp1_in = 0;
			printf("222222\r\n");
		}
		if(rotate_cp2 && third_cp2_in == 0){
			//++
			rotate_cp2 = 0;
			printf("333333\r\n");
		}
		if(rotate_cp2 == 0 && third_cp2_in){
			//				
			if(minus){M_down_data1_4h++;
			printf("+++++++++++++++\r\n");}
			else{ M_down_data1_4h--;
			printf("--------------- %d\r\n",minus );}
			third_cp2_in = 0;
			printf("4444444\r\n");
		}
		else 
		{
			first_cp1_in=0;
			second_cp1_in=0;
			third_cp1_in=0;
			
			first_cp2_in=0;
			second_cp2_in=0;
			third_cp2_in=0;
			
			CCW = 0;
			CW = 0;
			CW_CCW = 0;
			
			rotate_cp1 = 0;
			rotate_cp2 = 0;
			
			DIS_Dec_4h(M_down_data1_4h);
		}
	}

	if(minus)
	{
		if(M_down_data1_4h > 999) M_down_data1_4h = 0;
	}
	if(minus == 0)
	{
		if(M_down_data1_4h > 9999)
		{
			data_9999 = 1;
			M_down_data1_4h = 0;
			
		}
	}
	else data_9999 = 0;
	
	if(RST_INPUT_ON)
	{
		printf("RST\r\n");
		minus = 0;
		M_down_data1_4h = 9999; 
		DIS_Dec_4h(M_down_data1_4h);
	}
	
	if(RESET_ON)
	{
		if(GPIOG->IDR & 1<<2)
		{
			M_down_data1_4h = 9999; 
			DIS_Dec_4h(M_down_data1_4h);
		}
	}	
	
}
//==============================================================
//					SEGMENT 4H INPUT MODE DOWN DOWN
//==============================================================


void M_down_4h(void)//감산 입력 
{
	if(exti_signal)
	{
		if(GPIOB->IDR & 1<<1)
		{
			if((exti_cnt >= 50) && CP1_key_check == 0)
			{
				if(TACT_KEY->IDR & 1<<2)
				{
					signal = 1;
					if((signal_cnt >= 20) && (CP2_key_check == 0))
					{
						CP2_key_check = 1;
						key_check = 1;
					}
					printf("TACT_KEY\r\n");
				}
				else
				{
					CP1_key_check = 0;
					CP2_key_check = 0;
					
					if(key_check){
						M_down_data1_4h--;
						key_check = 0;
					}
					
					printf("else\r\n");
					//DIS_Dec_4h(data1_4h);
				}
			}
			//printf("exti_signal %d\r\n", data1_4h);
			DIS_Dec_4h(M_down_data1_4h);
		}
		else exti_signal = 0;
	}
	else
	{
		key_check = 1;
		signal = 0;
		CP1_key_check = 0;
		CP2_key_check = 0;
		DIS_Dec_4h(M_down_data1_4h);
	}
	
	if(M_down_data1_4h > 9999) M_down_data1_4h = 0; //DIS_Dec(data1);

	if(RST_INPUT_ON)
	{
		printf("RST\r\n");
		minus = 0;
		M_down_data1_4h = 9999; 
		DIS_Dec_4h(M_down_data1_4h);
	}
	if(RESET_ON)
	{
		if(GPIOG->IDR & 1<<2)
		{
			M_down_data1_4h = 9999; 
			DIS_Dec_4h(M_down_data1_4h);
		}
	}	
}

/* INPUT_MODE -----------------------------------------------------------*/

//==============================================================
//==============================================================
//
//
//
//					SEGMENT 4H TIMER MODE START
//
//
//
//==============================================================
//==============================================================


//==============================================================
//					SEGMENT 4H TIMER UP MODE 99.99s
//==============================================================
void M_timer_up_99_99s(void)
{
	LCD_data[19] |= 0x08;//TIM
	_99_59 = 1;
	DIS_POINT = 1;
	RESET_ON = 1;
	if(DIS_POINT)	LCD_data[26] |= 0x08;
	
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1 = 0; 
			data2 = 0; 
			DIS_Dec_4h_2d(data1, data2);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1 = 0; 
			data2 = 0; 
			DIS_Dec_4h_2d(data1, data2);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h_2d(data1, data2);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
		
		if(timer_cnt >= 90)//1ms*9
		{
			data2++;
			DIS_Dec_4h_2d(data1, data2);
			timer_cnt = 0;
			
			if(data2 > 99)
			{
				data1++;
				data2 = 0;
			}
			if(data1 > 99)
			{
				data1 = 0;
			}
		}
		
	}
		
		DIS_Dec_4h_2d(data1, data2);
	
	
}
//==============================================================
//					SEGMENT 4H TIMER UP MODE 999.9s
//==============================================================
void M_timer_up_999_9s(void)
{
	LCD_data[19] |= 0x08;//TIM
	DIS_POINT = 1;
	RESET_ON = 1;
	_999_9 = 1;
	if(DIS_POINT)	LCD_data[23] |= 0x08;
	
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1 = 0; 
			data2 = 0; 
			DIS_Dec_4h_2d(data1, data2);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1 = 0; 
			data2 = 0; 
			DIS_Dec_4h_2d(data1, data2);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h_2d(data1, data2);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
		
		if(timer_cnt >= 90)//1ms*90
		{
			data2++;
			DIS_Dec_4h_2d(data1, data2);
			timer_cnt = 0;
			
			if(data2 > 9)
			{
				data1++;
				data2 = 0;
			}
			if(data1 > 999)data1 = 0;
		}
	}
	DIS_Dec_4h_2d(data1, data2);
}
//==============================================================
//					SEGMENT 4H TIMER UP MODE 9999s
//==============================================================
void M_timer_up_9999s(void)
{
	LCD_data[19] |= 0x08;//TIM
	DIS_POINT = 1;
	RESET_ON = 1;
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1_4h = 0; 
			DIS_Dec_4h(data1_4h);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1_4h = 0; 
			DIS_Dec_4h(data1_4h);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h(data1_4h);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
		
		if(timer_cnt >= 999)//1ms*999
		{
			data1_4h++;
			DIS_Dec_4h(data1_4h);
			timer_cnt = 0;
			
			if(data1_4h > 9999)data1_4h = 0;
		}
	}
	DIS_Dec_4h(data1_4h);
}
//==============================================================
//					SEGMENT 4H TIMER UP MODE 99m59s
//==============================================================
void M_timer_up_99m59s(void)
{
	TIM3->ARRH = 0x00;
	TIM3->ARRL = 0x64;
	_99_59 = 1;
	LCD_data[19] |= 0x08;//TIM
	DIS_POINT = 1;
	RESET_ON = 1;
	
	if(DIS_POINT)	LCD_data[26] |= 0x08;
	
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1 = 0; 
			data2 = 0; 
			DIS_Dec_4h_2d(data1, data2);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1 = 0; 
			data2 = 0; 
			DIS_Dec_4h_2d(data1, data2);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h_2d(data1, data2);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
		
		if(timer_cnt >= 9886)//0.1ms*9985 //9885빠름 9887느림  
		{
			data2++;
			DIS_Dec_4h_2d(data1, data2);
			timer_cnt = 0;
			
			if(data2 > 59)
			{
				data1++;
				data2 = 0;
			}
			if(data1 > 99)
			{
				data1 = 0;
			}
		}
	}
	
	DIS_Dec_4h_2d(data1, data2);
}
//==============================================================
//					SEGMENT 4H TIMER UP MODE 999.9m
//==============================================================
void M_timer_up_999_9m(void)
{
	_999_9 = 1;
	DIS_POINT = 1;
	RESET_ON = 1;
	LCD_data[19] |= 0x08;//TIM
	if(DIS_POINT)	LCD_data[23] |= 0x08;
	
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1 = 0; 
			data2 = 0; 
			DIS_Dec_4h_2d(data1, data2);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1 = 0; 
			data2 = 0; 
			DIS_Dec_4h_2d(data1, data2);
			key_check = 1;
			timer_flag = 0;
			
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h_2d(data1, data2);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
				
		if(timer_cnt >= 6000)//1ms*6000
		{
			data2++;
			DIS_Dec_4h_2d(data1, data2);
			timer_cnt = 0;
			
			if(data2 > 9)
			{
				data1++;
				data2 = 0;
			}
			if(data1 > 999)
			{
				data1 = 0;
			}
		}
	}
	DIS_Dec_4h_2d(data1, data2);

}
//==============================================================
//					SEGMENT 4H TIMER UP MODE 99h59m
//==============================================================
void M_timer_up_99h59m(void)
{
	_99_59 = 1;
	
	LCD_data[19] |= 0x08;//TIM
	DIS_POINT = 1;
	RESET_ON = 1;
	
	if(DIS_POINT)	LCD_data[26] |= 0x08;
	
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1 = 0; 
			data2 = 0; 
			DIS_Dec_4h_2d(data1, data2);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1 = 0; 
			data2 = 0; 
			DIS_Dec_4h_2d(data1, data2);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h_2d(data1, data2);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
		
		if(timer_cnt >= 60000)//1ms*10000
		{
			data2++;
			DIS_Dec_4h_2d(data1, data2);
			timer_cnt = 0;
			
			if(data2 > 59)
			{
				data1++;
				data2 = 0;
			}
			if(data1 > 99)
			{
				data1 = 0;
			}
		}
	}
		DIS_Dec_4h_2d(data1, data2);

}
//==============================================================
//					SEGMENT 4H TIMER UP MODE 999.9h
//==============================================================
void M_timer_up_999_9h(void)
{
	TIM3->PSCR = TIM3_Prescaler_32;
	TIM3->ARRH = 0xc3;
	TIM3->ARRL = 0x50;//100ms
	
	_999_9 = 1;
	DIS_POINT = 0;
	RESET_ON = 1;
	LCD_data[19] |= 0x08;//TIM
	if(DIS_POINT)	LCD_data[23] |= 0x08;
	
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1 = 0; 
			data2 = 0; 
			DIS_Dec_4h_2d(data1, data2);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1 = 0; 
			data2 = 0; 
			DIS_Dec_4h_2d(data1, data2);
			key_check = 1;
			timer_flag = 0;
			
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h_2d(data1, data2);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
		
		if(timer_cnt >= 36000-1)
		{
			data2++;
			DIS_Dec_4h_2d(data1, data2);
			timer_cnt = 0;
			
			if(data2 > 9)
			{
				data1++;
				data2 = 0;
			}
			if(data1 > 999)
			{
				data1 = 0;
			}
		}
	}
	DIS_Dec_4h_2d(data1, data2);
}
//==============================================================
//					SEGMENT 4H TIMER UP MODE 9999h
//==============================================================
void M_timer_up_9999h(void)
{
	TIM3->PSCR = TIM3_Prescaler_32;
	TIM3->ARRH = 0xc3;
	TIM3->ARRL = 0x50;
	
	LCD_data[19] |= 0x08;//TIM
	//DIS_POINT = 1;
	RESET_ON = 1;
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1_4h = 0; 
			DIS_Dec_4h(data1_4h);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			data1_4h = 0; 
			DIS_Dec_4h(data1_4h);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h(data1_4h);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
		
		if(timer_cnt >= 36000-1)//100ms*36000
		{
			data1_4h++;
			DIS_Dec_4h(data1_4h);
			timer_cnt = 0;
			
			if(data1_4h > 9999)data1_4h = 0;
		}
	}
	DIS_Dec_4h(data1_4h);
	
}
//==============================================================
//					SEGMENT 4H TIMER DOWN MODE 99.99s
//==============================================================
void M_timer_down_99_99s(void)//99.99 -> 0.0 -> -0.00 -> -9.99
{
	TIM3->PSCR = TIM3_Prescaler_4;
	TIM3->ARRH = 0x01;
	TIM3->ARRL = 0x90;
	_99_59 = 1;
	
	LCD_data[19] |= 0x08;//TIM
	DIS_POINT = 1;
	RESET_ON = 1;
	if(DIS_POINT)	LCD_data[26] |= 0x08;
	
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			down_data1 = 99; 
			down_data2 = 99; 
			DIS_Dec_4h_2d(down_data1, down_data2);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			down_data1 = 99; 
			down_data2 = 99; 
			DIS_Dec_4h_2d(down_data1, down_data2);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h_2d(down_data1, down_data2);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
		
		if(down_DATA >= 9999)
		{
			LCD_data[30] = LCD_SEG_NUM6[11];
			LCD_data[31] = LCD_SEG_NUM5[11];
			printf("down flag %d \r\n",down_DATA);
			if(timer_cnt >= timer_cps)//0.01ms*91//867느리고 865 빠르고
			{
				down_data2++;
				timer_cnt = 0;
				
				if(down_data2 > 99)
				{
					down_data1++;
					down_data2 = 0;
				}
				if(down_data1 > 9)down_data1 = 0;
			}
		}
		else
		{
			if(timer_cnt >= timer_cps)//0.1ms*91
			{
				down_data2--;//99 -> 0
				timer_cnt = 0;
				down_DATA++;
				//printf("down count %d \r\n",down_DATA);//0
				if(down_data2 < 0)
				{
					down_data1--;
					down_data2 = 99;
				}
			}
		}
	}
	DIS_Dec_4h_2d(down_data1, down_data2);
}
//==============================================================
//					SEGMENT 4H TIMER DOWN MODE 999.9s
//==============================================================
void M_timer_down_999_9s(void)
{
	TIM3->PSCR = TIM3_Prescaler_4;
	TIM3->ARRH = 0x01;
	TIM3->ARRL = 0x90;
	timer_cps = 978;//0.1ms
	
	DIS_POINT = 1;
	RESET_ON = 1;
	LCD_data[19] |= 0x08;//TIM
	if(DIS_POINT)	LCD_data[23] |= 0x08;
	
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			timer4_4digit = 9999;
			DIS_Dec_4h(timer4_4digit);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			timer4_4digit = 9999;
			DIS_Dec_4h(timer4_4digit);
			key_check = 1;
			timer_flag = 0;
			
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h(timer4_4digit);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
		
		if(down_DATA >= 9999)
		{
			if(timer_cnt >= timer_cps)//1ms*90
			{
				timer4_4digit++;
				DIS_Dec_4h(timer4_4digit);
				timer_cnt = 0;
				
				if(timer4_4digit > 999)
				{
					timer4_4digit = 0;
				}
			}
			
			if(timer4_4digit > 99)
			{
				LCD_data[30] = LCD_SEG_NUM6[11];
				LCD_data[31] = LCD_SEG_NUM5[11];
			}
			else
			{
				LCD_data[27] = LCD_SEG_NUM6[11];
				LCD_data[28] = LCD_SEG_NUM5[11];
			}
		}
		else
		{
			if(timer_cnt >= timer_cps)//1ms*90
			{
				timer4_4digit--;
				DIS_Dec_4h(timer4_4digit);
				down_DATA++;
				timer_cnt = 0;
				
				if(timer4_4digit < 0)
				{
					timer4_4digit = 9999;
				}
			}
		}
	}
	DIS_Dec_4h(timer4_4digit);
}
//==============================================================
//					SEGMENT 4H TIMER DOWN MODE 9999s
//==============================================================
void M_timer_down_9999s(void)
{
	timer_cps = 998;//0.1ms
	
	RESET_ON = 1;
	LCD_data[19] |= 0x08;//TIM
	
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			timer4_4digit = 9999;
			DIS_Dec_4h(timer4_4digit);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			timer4_4digit = 9999;
			DIS_Dec_4h(timer4_4digit);
			key_check = 1;
			timer_flag = 0;
			
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h(timer4_4digit);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
		
		if(down_DATA >= 9999)
		{
			if(timer_cnt >= timer_cps)//1ms*90
			{
				timer4_4digit++;
				DIS_Dec_4h(timer4_4digit);
				timer_cnt = 0;
				
				if(timer4_4digit > 999)
				{
					timer4_4digit = 0;
				}
			}
			
			if(timer4_4digit > 99)
			{
				LCD_data[30] = LCD_SEG_NUM6[11];
				LCD_data[31] = LCD_SEG_NUM5[11];
			}
			else if(timer4_4digit > 9)
			{
				LCD_data[27] = LCD_SEG_NUM6[11];
				LCD_data[28] = LCD_SEG_NUM5[11];
			}
			else
			{
				LCD_data[24] = LCD_SEG_NUM6[11];
				LCD_data[25] = LCD_SEG_NUM5[11];
			}
		}
		else
		{
			if(timer_cnt >= timer_cps)//1ms*90
			{
				timer4_4digit--;
				DIS_Dec_4h(timer4_4digit);
				down_DATA++;
				timer_cnt = 0;
				
				if(timer4_4digit < 0)
				{
					timer4_4digit = 9999;
				}
			}
		}
	}
	DIS_Dec_4h(timer4_4digit);
}
//==============================================================
//					SEGMENT 4H TIMER DOWN MODE 99m59s
//==============================================================
void M_timer_down_99m59s(void)
{
	_99_59 = 1;
	LCD_data[19] |= 0x08;//TIM
	timer_cps = 998;
	DIS_POINT = 1;
	RESET_ON = 1;
	
	if(DIS_POINT)	LCD_data[26] |= 0x08;
	
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			down_99 = 99; 
			down_59 = 59; 
			DIS_Dec_4h_2d(down_99, down_59);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			down_99 = 99; 
			down_59 = 59; 
			DIS_Dec_4h_2d(down_99, down_59);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h_2d(down_99, down_59);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
		
		if(down_DATA >= 9959)
		{
			if(timer_cnt >= timer_cps)//1ms*90
			{
				down_59++;
				DIS_Dec_4h_2d(down_99, down_59);
				timer_cnt = 0;
				
				if(down_59 > 59)
				{
					down_99--;
					down_59 = 0;
				}
				if(down_99 > 9)down_99 = 0;
			}
			
			if(down_59 > 59)
			{
				LCD_data[30] = LCD_SEG_NUM6[11];
				LCD_data[31] = LCD_SEG_NUM5[11];
			}
			else if(down_59 > 9)
			{
				LCD_data[27] = LCD_SEG_NUM6[11];
				LCD_data[28] = LCD_SEG_NUM5[11];
			}
			else
			{
				LCD_data[24] = LCD_SEG_NUM6[11];
				LCD_data[25] = LCD_SEG_NUM5[11];
			}
		}
		else
		{
			if(timer_cnt >= timer_cps)//1ms*90
			{
				down_59--;
				DIS_Dec_4h_2d(down_99, down_59);
				down_DATA++;
				timer_cnt = 0;
				
				if(down_59 < 0)
				{
					down_99--;
					down_59 = 59;
				}
			}
		}
	}
	DIS_Dec_4h_2d(down_99, down_59);
}
//==============================================================
//					SEGMENT 4H TIMER DOWN MODE 999.9m
//==============================================================
void M_timer_down_999_9m(void)
{
	timer_cps = 60000;//1ms*60000
	
	DIS_POINT = 1;
	RESET_ON = 1;
	LCD_data[19] |= 0x08;//TIM
	if(DIS_POINT)	LCD_data[23] |= 0x08;
	
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			timer4_4digit = 9999; 
			DIS_Dec_4h(timer4_4digit);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			timer4_4digit = 9999; 
			DIS_Dec_4h(timer4_4digit);
			key_check = 1;
			timer_flag = 0;
			
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h(timer4_4digit);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
		
		
		if(down_DATA >= 9999)
		{
			if(timer_cnt >= timer_cps)//1ms*90
			{
				timer4_4digit++;
				DIS_Dec_4h(timer4_4digit);
				timer_cnt = 0;
				
				if(timer4_4digit > 999)
				{
					timer4_4digit = 0;
				}
			}
			
			if(timer4_4digit > 99)
			{
				LCD_data[30] = LCD_SEG_NUM6[11];
				LCD_data[31] = LCD_SEG_NUM5[11];
			}
			else
			{
				LCD_data[27] = LCD_SEG_NUM6[11];
				LCD_data[28] = LCD_SEG_NUM5[11];
			}
		}
		else
		{
			if(timer_cnt >= timer_cps)//1ms*90
			{
				timer4_4digit--;
				DIS_Dec_4h(timer4_4digit);
				down_DATA++;
				timer_cnt = 0;
				
				if(timer4_4digit < 0)
				{
					timer4_4digit = 9999;
				}
			}
		}
	}
	DIS_Dec_4h(timer4_4digit);
}
//==============================================================
//					SEGMENT 4H TIMER DOWN MODE 99h59m
//==============================================================
void M_timer_down_99h59m(void)
{
	_99_59 = 1;
	
	LCD_data[19] |= 0x08;//TIM
	timer_cps = 60000;
	DIS_POINT = 1;
	RESET_ON = 1;
	
	if(DIS_POINT)	LCD_data[26] |= 0x08;
	
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			down_99 = 99; 
			down_59 = 59; 
			DIS_Dec_4h_2d(down_99, down_59);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			down_99 = 99; 
			down_59 = 59; 
			DIS_Dec_4h_2d(down_99, down_59);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h_2d(down_99, down_59);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
		
		if(down_DATA >= 9959)
		{
			if(timer_cnt >= timer_cps)//1ms*90
			{
				down_59++;
				DIS_Dec_4h_2d(down_99, down_59);
				timer_cnt = 0;
				
				if(down_59 > 59)
				{
					down_99--;
					down_59 = 0;
				}
				if(down_99 > 9)down_99 = 0;
			}
			
			if(down_59 > 59)
			{
				LCD_data[30] = LCD_SEG_NUM6[11];
				LCD_data[31] = LCD_SEG_NUM5[11];
			}
			else if(down_59 > 9)
			{
				LCD_data[27] = LCD_SEG_NUM6[11];
				LCD_data[28] = LCD_SEG_NUM5[11];
			}
			else
			{
				LCD_data[24] = LCD_SEG_NUM6[11];
				LCD_data[25] = LCD_SEG_NUM5[11];
			}
		}
		else
		{
			if(timer_cnt >= timer_cps)//1ms*90
			{
				down_59--;
				DIS_Dec_4h_2d(down_99, down_59);
				down_DATA++;
				timer_cnt = 0;
				
				if(down_59 < 0)
				{
					down_99--;
					down_59 = 59;
				}
			}
		}
	}
	DIS_Dec_4h_2d(down_99, down_59);
}
//==============================================================
//					SEGMENT 4H TIMER DOWN MODE 999.9h
//==============================================================
void M_timer_down_999_9h(void)
{
	TIM3->PSCR = TIM3_Prescaler_32;
	TIM3->ARRH = 0xc3;
	TIM3->ARRL = 0x50;//100ms
	timer_cps = 36000-1;//1h
	
	DIS_POINT = 1;
	RESET_ON = 1;
	LCD_data[19] |= 0x08;//TIM
	if(DIS_POINT)	LCD_data[23] |= 0x08;
	
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			timer4_4digit = 9999;
			DIS_Dec_4h(timer4_4digit);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			timer4_4digit = 9999;
			DIS_Dec_4h(timer4_4digit);
			key_check = 1;
			timer_flag = 0;
			
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h(timer4_4digit);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
		
		if(down_DATA >= 9999)
		{
			if(timer_cnt >= timer_cps)//1ms*90
			{
				timer4_4digit++;
				DIS_Dec_4h(timer4_4digit);
				timer_cnt = 0;
				
				if(timer4_4digit > 999)
				{
					timer4_4digit = 0;
				}
			}
			
			if(timer4_4digit > 99)
			{
				LCD_data[30] = LCD_SEG_NUM6[11];
				LCD_data[31] = LCD_SEG_NUM5[11];
			}
			else
			{
				LCD_data[27] = LCD_SEG_NUM6[11];
				LCD_data[28] = LCD_SEG_NUM5[11];
			}
		}
		else
		{
			if(timer_cnt >= timer_cps)//1ms*90
			{
				timer4_4digit--;
				DIS_Dec_4h(timer4_4digit);
				down_DATA++;
				timer_cnt = 0;
				
				if(timer4_4digit < 0)
				{
					timer4_4digit = 9999;
				}
			}
		}
	}
	DIS_Dec_4h(timer4_4digit);
}
//==============================================================
//					SEGMENT 4H TIMER DOWN MODE 9999h
//==============================================================
void M_timer_down_9999h(void)
{
	TIM3->PSCR = TIM3_Prescaler_32;
	TIM3->ARRH = 0xc3;
	TIM3->ARRL = 0x50;//100ms
	timer_cps = 36000-1;//1h
	
	RESET_ON = 1;
	LCD_data[19] |= 0x08;//TIM
	
	if(RST_INPUT_ON)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			timer4_4digit = 9999;
			DIS_Dec_4h(timer4_4digit);
			key_check = 1;
			timer_flag = 0;
		}
	}
	else if(RESET_ON == 1 && (GPIOG->IDR & 1<<2)==0)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			LCD_data[19] |= 0x01;//RST
			timer4_4digit = 9999;
			DIS_Dec_4h(timer4_4digit);
			key_check = 1;
			timer_flag = 0;
			
		}
	}
	else if(GPIOB->IDR & 1<<2)
	{
		key_flag = 1;
		if((key_cnt >= 50) && key_check == 0)
		{
			DIS_Dec_4h(timer4_4digit);
			LCD_data[18] |= 0x08;//INH
			key_check = 1;
			timer_flag = 0;
		}
	}
	else
	{
		key_check = 0;
		key_flag = 0;
		timer_flag = 1;
		LCD_data[19] &= ~(0x01);//RST
		LCD_data[18] &= ~(0x08);//INH
		
		if(down_DATA >= 9999)
		{
			if(timer_cnt >= timer_cps)//1ms*90
			{
				timer4_4digit++;
				DIS_Dec_4h(timer4_4digit);
				timer_cnt = 0;
				
				if(timer4_4digit > 999)
				{
					timer4_4digit = 0;
				}
			}
			
			if(timer4_4digit > 99)
			{
				LCD_data[30] = LCD_SEG_NUM6[11];
				LCD_data[31] = LCD_SEG_NUM5[11];
			}
			else if(timer4_4digit > 9)
			{
				LCD_data[27] = LCD_SEG_NUM6[11];
				LCD_data[28] = LCD_SEG_NUM5[11];
			}
			else
			{
				LCD_data[24] = LCD_SEG_NUM6[11];
				LCD_data[25] = LCD_SEG_NUM5[11];
			}
		}
		else
		{
			if(timer_cnt >= timer_cps)//1ms*90
			{
				timer4_4digit--;
				DIS_Dec_4h(timer4_4digit);
				down_DATA++;
				timer_cnt = 0;
				
				if(timer4_4digit < 0)
				{
					timer4_4digit = 9999;
				}
			}
		}
	}
	DIS_Dec_4h(timer4_4digit);
}
//==============================================================
//==============================================================
//
//
//
//					SEGMENT 4H TIMER MODE END
//
//
//
//==============================================================
//==============================================================


/* 6H_TIMER_MODE -----------------------------------------------------------*/
//==============================================================
//==============================================================
//
//
//
//					SEGMENT 4H TIMER MODE END
//
//
//
//==============================================================
//==============================================================


//==============================================================
//					SEGMENT 6H TIMER UP MODE 99999.9s
//==============================================================
//bool _99_59_99, _999_59_9, _9999_59;
//signed long timer6_2digit1, timer6_2digit2, timer6_2digit3, timer6_3digit1, timer6_3digit2, timer6_4digit1;
void M_timer_up_99999_9s(void)//DIS_Dec_6(data1_6);//bool _99_59_99, _999_59_9, _9999_59;
{
	_99999_9 = 1;
	LCD_data[17] |= 0x08;
	LCD_data[13] |= 0x08;//TIM
	
	if(timer_cnt >= 9)//1ms*9
	{
		data1_6++;
		DIS_Dec_6(data1_6);
		timer_cnt = 0;
		
		if(data1_6 > 999999)
		{
			data1_6++;
			data1_6 = 0;
		}
	}
	
	DIS_Dec_6(data1_6);
}
//==============================================================
//					SEGMENT 6H TIMER UP MODE 999999s
//==============================================================
void M_timer_up_999999s(void)//DIS_Dec_6(data1_6);//bool _99_59_99, _999_59_9, _9999_59;
{
	LCD_data[13] |= 0x08;//TIM
	if(timer_cnt >= 999)//1ms*999
	{
		data1_6++;
		DIS_Dec_6(data1_6);
		timer_cnt = 0;
		
		if(data1_6 > 999999)data1_6 = 0;
	}
	DIS_Dec_6(data1_6);
}
//==============================================================
//					SEGMENT 6H TIMER UP MODE 99m59.99s
//==============================================================
void M_timer_up_99m59_99s(void)//DIS_Dec_6h_2d(timer6_2digit1, timer6_4digit1);//bool _99_59_99, _999_59_9, _9999_59, _99_5999;
{
	_99_59_99 = 1;
	LCD_data[20] |= 0x08;
	LCD_data[26] |= 0x08;
	LCD_data[13] |= 0x08;//TIM
	
	if(timer_cnt >= 9)//1ms*9
	{
		timer6_4digit1++;
		DIS_Dec_6h_2d(timer6_2digit1, timer6_4digit1);
		timer_cnt = 0;
		
		if(timer6_4digit1 > 5999)
		{
			timer6_2digit1++;
			timer6_4digit1 = 0;
		}
		if(timer6_2digit1 > 99)
		{
			timer6_2digit1 = 0;
			time_plus++;
		}
	}

	DIS_Dec_6h_2d(timer6_2digit1, timer6_4digit1);
}
//==============================================================
//					SEGMENT 6H TIMER UP MODE 999m599s
//==============================================================
void M_timer_up_999m59_9s(void)//DIS_Dec_6h_2d(timer6_3digit1, timer6_3digit2);//bool _99_59_99, _999_59_9, _9999_59;
{
	_999_59_9 = 1;
	LCD_data[23] |= 0x08;
	LCD_data[17] |= 0x08;
	LCD_data[13] |= 0x08;//TIM
	
	if(timer_cnt >= 90)//1ms*90
	{
		timer6_3digit2++;
		DIS_Dec_6h_2d(timer6_3digit1, timer6_3digit2);
		timer_cnt = 0;
		
		if(timer6_3digit2 > 599)
		{
			timer6_3digit1++;
			timer6_3digit2 = 0;
		}
		if(timer6_3digit1 > 999)timer6_3digit1 = 0;
	}
	DIS_Dec_6h_2d(timer6_3digit1, timer6_3digit2);
}
//==============================================================
//					SEGMENT 6H TIMER UP MODE 99999.9m
//==============================================================
void M_timer_up_99999_9m(void)//DIS_Dec_6(data1_6);//bool _99_59_99, _999_59_9, _9999_59;
{
	_99999_9 = 1;
	LCD_data[17] |= 0x08;
	LCD_data[13] |= 0x08;//TIM
	
	if(timer_cnt >= 6000)//1ms*6000
	{
		data1_6++;
		DIS_Dec_6(data1_6);
		timer_cnt = 0;
		
		if(data1_6 > 999999)
		{
			data1_6 = 0;
		}

	}
	DIS_Dec_6(data1_6);

}
//==============================================================
//					SEGMENT 6H TIMER UP MODE 99h59m59s
//==============================================================
void M_timer_up_99h59m59s(void)//DIS_Dec_6h_3d(timer6_2digit1, timer6_2digit2, timer6_2digit3);//bool _99_59_99, _999_59_9, _9999_59;
{
	TIM3->ARRH = 0x00;
	TIM3->ARRL = 0x64;
	LCD_data[20] |= 0x08;
	LCD_data[26] |= 0x08;
	LCD_data[13] |= 0x08;//TIM
	
	_99_59 = 1;
	if(timer_cnt >= 9886)//0.1ms*9985 //9885빠름 9887느림  
	{
		timer6_2digit3++;
		DIS_Dec_6h_3d(timer6_2digit1, timer6_2digit2, timer6_2digit3);
		timer_cnt = 0;
		
		if(timer6_2digit3 > 59)
		{
			timer6_2digit2++;
			timer6_2digit3 = 0;
		}
		if(timer6_2digit2 > 59)
		{
			timer6_2digit1++;
			timer6_2digit3 = 0;
		}
		if(timer6_2digit1 > 99)
		{
			timer6_2digit1 = 0;
		}
	}
	DIS_Dec_6h_3d(timer6_2digit1, timer6_2digit2, timer6_2digit3);
}
//==============================================================
//					SEGMENT 6H TIMER UP MODE 9999h59m
//==============================================================
void M_timer_up_9999h59m(void)//DIS_Dec_6h_2d(timer6_4digit1, timer6_2digit1);//bool _99_59_99, _999_59_9, _9999_59;
{
	LCD_data[20] |= 0x08;
	LCD_data[13] |= 0x08;//TIM
	
	if(timer_cnt >= 60000)//1ms*10000
	{
		timer6_2digit1++;
		DIS_Dec_6h_2d(timer6_4digit1, timer6_2digit1);
		timer_cnt = 0;
		
		if(timer6_2digit1 > 59)
		{
			timer6_4digit1++;
			timer6_2digit1 = 0;
		}
		if(timer6_4digit1 > 9999)
		{
			timer6_4digit1 = 0;
		}
	}
		DIS_Dec_6h_2d(timer6_4digit1, timer6_2digit1);

}
//==============================================================
//					SEGMENT 6H TIMER UP MODE 99999.9h
//==============================================================
void M_timer_up_99999_9h(void)//DIS_Dec_6(data1_6);//bool _99_59_99, _999_59_9, _9999_59;
{
	_99999_9 = 1;
	LCD_data[17] |= 0x08;
	LCD_data[13] |= 0x08;//TIM
	
	if(timer_cnt >= 60000)//10ms*60000
	{
		data1_6++;
		DIS_Dec_6(data1_6);
		timer_cnt = 0;
		
		if(data1_6 > 999999)
		{
			data1_6 = 0;
		}
	}
	DIS_Dec_6(data1_6);
}

//bool _99_59_99, _999_59_9, _9999_59;
//signed long down_timer6_99_1=99, down_timer6_59_2=59, down_timer6_99_3=99, down_timer6_999_1=999, down_timer6_599_2=599, down_timer6_9999_1=9999, down_timer6_5999_1=5999;
//signed long down_timer6_59_1=59;
//==============================================================
//					SEGMENT 6H TIMER DOWN MODE 99999.9s
//==============================================================
void M_timer_down_99999_9s(void)
{
	
	TIM3->PSCR = TIM3_Prescaler_4;
	TIM3->ARRH = 0x01;
	TIM3->ARRL = 0x90;
	timer_cps = 978;//0.1ms
	_99999_9 = 1;
	LCD_data[17] |= 0x08;
	LCD_data[13] |= 0x08;//TIM
	
	if(down_DATA >= 999999)
	{
		if(timer_cnt >= timer_cps)//1ms*90
		{
			M_down_data1_6h++;
			DIS_Dec_6(M_down_data1_6h);
			timer_cnt = 0;
			
			if(M_down_data1_6h > 99999)
			{
				M_down_data1_6h = 0;
			}
		}
		
		if(M_down_data1_6h > 9999)
		{
			LCD_data[27] = LCD_SEG_NUM6[11];
			LCD_data[28] = LCD_SEG_NUM5[11];
		}
		else if(M_down_data1_6h > 999)
		{
			LCD_data[24] = LCD_SEG_NUM6[11];
			LCD_data[25] = LCD_SEG_NUM5[11];
		}
		else
		{
			LCD_data[21] = LCD_SEG_NUM6[11];
			LCD_data[22] = LCD_SEG_NUM5[11];
		}
	}
	else
	{
		if(timer_cnt >= timer_cps)//1ms*90
		{
			M_down_data1_6h--;
			DIS_Dec_6(M_down_data1_6h);
			down_DATA++;
			timer_cnt = 0;
		}
	}
	DIS_Dec_6(M_down_data1_6h);
}
//==============================================================
//					SEGMENT 6H TIMER DOWN MODE 999999s
//==============================================================
void M_timer_down_999999s(void)//DIS_Dec_6(M_down_data1_6h);
{
	//TIM3->PSCR = TIM3_Prescaler_4;
	//TIM3->ARRH = 0x01;
	//TIM3->ARRL = 0x90;
	//timer_cps = 978;//0.1ms
	timer_cps = 998;//0.1ms
	_99999_9 = 1;
	//LCD_data[17] |= 0x08;
	LCD_data[13] |= 0x08;//TIM
	
	if(down_DATA >= 999999)
	{
		if(timer_cnt >= timer_cps)//1ms*90
		{
			M_down_data1_6h++;
			DIS_Dec_6(M_down_data1_6h);
			timer_cnt = 0;
			
			if(M_down_data1_6h > 99999)
			{
				M_down_data1_6h = 0;
			}
		}
		
		if(M_down_data1_6h > 9999)
		{
			LCD_data[27] = LCD_SEG_NUM6[11];
			LCD_data[28] = LCD_SEG_NUM5[11];
		}
		else if(M_down_data1_6h > 999)
		{
			LCD_data[24] = LCD_SEG_NUM6[11];
			LCD_data[25] = LCD_SEG_NUM5[11];
		}
		else if(M_down_data1_6h > 99)
		{
			LCD_data[21] = LCD_SEG_NUM6[11];
			LCD_data[22] = LCD_SEG_NUM5[11];
		}
		else 
		{
			LCD_data[18] = LCD_SEG_NUM6[11];
			LCD_data[19] = LCD_SEG_NUM5[11];
		}
	}
	else
	{
		if(timer_cnt >= timer_cps)//1ms*90
		{
			M_down_data1_6h--;
			DIS_Dec_6(M_down_data1_6h);
			down_DATA++;
			timer_cnt = 0;
		}
	}
	DIS_Dec_6(M_down_data1_6h);
}
//==============================================================
//					SEGMENT 6H TIMER DOWN MODE 99m59.99s
//==============================================================
void M_timer_down_99m59_99s(void)//DIS_Dec_6h_2d(down_timer6_99_1, down_timer6_5999_1)
{
	_99_59_99 = 1;
	LCD_data[20] |= 0x08;
	LCD_data[26] |= 0x08;
	LCD_data[13] |= 0x08;//TIM
	
	if(down_DATA >= 995999)
	{
		if(timer_cnt >= 9)//1ms*90
		{
			down_timer6_5999_1++;
			DIS_Dec_6h_2d(down_timer6_99_1, down_timer6_5999_1);
			timer_cnt = 0;
			
			if(down_timer6_5999_1 > 5999)
			{
				down_timer6_99_1++;
				down_timer6_5999_1 = 0;
			}
			if(down_timer6_99_1 > 9)
			{
				down_timer6_99_1 = 0;
			}
		}
		
		if(down_timer6_5999_1 > 0)
		{
			LCD_data[30] = LCD_SEG_NUM6[11];
			LCD_data[31] = LCD_SEG_NUM5[11];
		}
	}
	else
	{
		if(timer_cnt >= 9)//1ms*9
		{
			down_timer6_5999_1--;
			DIS_Dec_6h_2d(down_timer6_99_1, down_timer6_5999_1);
			down_DATA++;
			timer_cnt = 0;
			
			if(down_timer6_5999_1 < 0)
			{
				down_timer6_99_1--;
				down_timer6_5999_1 = 9999;
			}
		}
	}

	DIS_Dec_6h_2d(down_timer6_99_1, down_timer6_5999_1);
	
}
//==============================================================
//					SEGMENT 6H TIMER DOWN MODE 999m59.9s
//==============================================================
void M_timer_down_999m59_9s(void)//DIS_Dec_6h_2d(down_timer6_999_1, down_timer6_599_2)
{
	_999_59_9 = 1;
	LCD_data[23] |= 0x08;
	LCD_data[17] |= 0x08;
	LCD_data[13] |= 0x08;//TIM
	
	if(down_DATA >= 999599)
	{
		if(timer_cnt >= 90)//1ms*90
		{
			down_timer6_599_2++;
			DIS_Dec_6h_2d(down_timer6_999_1, down_timer6_599_2);
			timer_cnt = 0;
			
			if(down_timer6_599_2 > 599)
			{
				down_timer6_999_1++;
				down_timer6_599_2 = 0;
			}
			if(down_timer6_999_1 > 99)
			{
				down_timer6_999_1 = 0;
			}
		}
		
		if(down_timer6_999_1 > 9)
		{
			LCD_data[30] = LCD_SEG_NUM6[11];
			LCD_data[31] = LCD_SEG_NUM5[11];
		}
		else
		{
			LCD_data[27] = LCD_SEG_NUM6[11];
			LCD_data[28] = LCD_SEG_NUM5[11];
		}
		
	}
	else
	{
		if(timer_cnt >= 90)//1ms*90
		{
			down_timer6_599_2--;
			down_DATA++;
			DIS_Dec_6h_2d(down_timer6_999_1, down_timer6_599_2);
			timer_cnt = 0;
			
			if(down_timer6_599_2 < 0)
			{
				down_timer6_999_1--;
				down_timer6_599_2 = 599;
			}
			if(down_timer6_999_1 < 0)down_timer6_999_1 = 999;
		}
	}
	DIS_Dec_6h_2d(down_timer6_999_1, down_timer6_599_2);
}
//==============================================================
//					SEGMENT 6H TIMER DOWN MODE 99999.9m
//==============================================================
void M_timer_down_99999_9m(void)//DIS_Dec_6(M_down_data1_6h);
{
	_99999_9 = 1;
	LCD_data[17] |= 0x08;
	LCD_data[13] |= 0x08;//TIM
	
	if(down_DATA >= 999999)
	{
		if(timer_cnt >= 6000)//1ms*90
		{
			M_down_data1_6h--;
			DIS_Dec_6(M_down_data1_6h);
			timer_cnt = 0;
			
			if(M_down_data1_6h > 99999)M_down_data1_6h = 0;
		}
		
		if(M_down_data1_6h > 9999)//10000
		{
			LCD_data[30] = LCD_SEG_NUM6[11];
			LCD_data[31] = LCD_SEG_NUM5[11];
		}
		else if(M_down_data1_6h > 999)//1000
		{
			LCD_data[27] = LCD_SEG_NUM6[11];
			LCD_data[28] = LCD_SEG_NUM5[11];
		}
		else if(M_down_data1_6h > 99)//100
		{
			LCD_data[24] = LCD_SEG_NUM6[11];
			LCD_data[25] = LCD_SEG_NUM5[11];
		}
		else //10
		{
			LCD_data[21] = LCD_SEG_NUM6[11];
			LCD_data[22] = LCD_SEG_NUM5[11];
		}
	}
	else
	{
		if(timer_cnt >= 6000)//1ms*6000
		{
			M_down_data1_6h--;
			down_DATA++;
			DIS_Dec_6(M_down_data1_6h);
			timer_cnt = 0;
			
			if(M_down_data1_6h < 0)
			{
				M_down_data1_6h = 999999;
			}
		}
	}
	DIS_Dec_6(M_down_data1_6h);

}
//==============================================================
//					SEGMENT 6H TIMER DOWN MODE 99h59m59s
//==============================================================
void M_timer_down_99h59m59s(void)//DIS_Dec_6h_3d(down_timer6_99_1, down_timer6_59_1, down_timer6_59_2)
{
	TIM3->ARRH = 0x00;
	TIM3->ARRL = 0x64;
	LCD_data[20] |= 0x08;
	LCD_data[26] |= 0x08;
	LCD_data[13] |= 0x08;//TIM
	
	if(down_DATA >= 995959)
	{
		if(timer_cnt >= 9886)//1ms*90
		{
			down_timer6_59_2++;
			DIS_Dec_6h_3d(down_timer6_99_1, down_timer6_59_1, down_timer6_59_2);
			timer_cnt = 0;
			
			if(down_timer6_59_2 > 59)
			{
				down_timer6_59_1++;
				down_timer6_59_2 = 0;
			}
			if(down_timer6_59_1 >59)
			{
				down_timer6_99_1++;
				down_timer6_59_1 = 0;
			}
			if(down_timer6_99_1 > 99)
			{
				down_timer6_99_1 = 0;
			}
			
			if(down_timer6_99_1 > 0)//10000
			{
				LCD_data[30] = LCD_SEG_NUM6[11];
				LCD_data[31] = LCD_SEG_NUM5[11];
			}
		}
	}
	else
	{
		if(timer_cnt >= 9886)//0.1ms*9985 //9885빠름 9887느림  
		{
			down_timer6_59_2--;
			down_DATA++;
			DIS_Dec_6h_3d(down_timer6_99_1, down_timer6_59_1, down_timer6_59_2);
			timer_cnt = 0;
			
			if(down_timer6_59_2 < 0)
			{
				down_timer6_59_1--;
				down_timer6_59_2 = 59;
			}
			if(down_timer6_59_1 < 0)
			{
				down_timer6_99_1--;
				down_timer6_59_1 = 59;
			}
			if(down_timer6_99_1 < 0)
			{
				down_timer6_99_1 = 99;
			}
		}
	}
	DIS_Dec_6h_3d(down_timer6_99_1, down_timer6_59_1, down_timer6_59_2);
}
//==============================================================
//					SEGMENT 6H TIMER DOWN MODE 9999h59m
//==============================================================
void M_timer_down_9999h59m(void)//DIS_Dec_6h_2d(down_timer6_9999_1, down_timer6_59_2)
{
	LCD_data[20] |= 0x08;
	LCD_data[13] |= 0x08;//TIM
	
	if(down_DATA >= 999959)
	{
		if(timer_cnt >= 60000)//1ms*90
		{
			down_timer6_59_2++;
			DIS_Dec_6h_2d(down_timer6_9999_1, down_timer6_59_2);
			timer_cnt = 0;
			
			if(down_timer6_59_2 > 59)
			{
				down_timer6_9999_1++;
				down_timer6_59_2 = 0;
			}
			if(down_timer6_9999_1 > 9999)
			{
				down_timer6_9999_1 = 0;
			}
			
			if(down_timer6_9999_1 > 9)//10
			{
				LCD_data[30] = LCD_SEG_NUM6[11];
				LCD_data[31] = LCD_SEG_NUM5[11];
			}
			else //1
			{
				LCD_data[27] = LCD_SEG_NUM6[11];
				LCD_data[28] = LCD_SEG_NUM5[11];
			}
		}
	}
	else
	{
		
		if(timer_cnt >= 60000)//1ms*10000
		{
			down_timer6_59_2--;
			down_DATA++;
			DIS_Dec_6h_2d(down_timer6_9999_1, down_timer6_59_2);
			timer_cnt = 0;
			
			if(down_timer6_59_2 < 0)
			{
				down_timer6_9999_1--;
				down_timer6_59_2 = 59;
			}
		}
	}
	DIS_Dec_6h_2d(down_timer6_9999_1, down_timer6_59_2);

}
//==============================================================
//					SEGMENT 6H TIMER DOWN MODE 99999.9h
//==============================================================
void M_timer_down_99999_9h(void)//DIS_Dec_6(M_down_data1_6h);
{
	_99999_9 = 1;
	LCD_data[17] |= 0x08;
	LCD_data[13] |= 0x08;//TIM
	
	if(down_DATA >= 999999)
	{
		if(timer_cnt >= 60000)//1ms*90
		{
			M_down_data1_6h--;
			DIS_Dec_6(M_down_data1_6h);
			timer_cnt = 0;
			
			if(M_down_data1_6h > 99999)M_down_data1_6h = 0;
		}
		
		if(M_down_data1_6h > 9999)//10000
		{
			LCD_data[30] = LCD_SEG_NUM6[11];
			LCD_data[31] = LCD_SEG_NUM5[11];
		}
		else if(M_down_data1_6h > 999)//1000
		{
			LCD_data[27] = LCD_SEG_NUM6[11];
			LCD_data[28] = LCD_SEG_NUM5[11];
		}
		else if(M_down_data1_6h > 99)//100
		{
			LCD_data[24] = LCD_SEG_NUM6[11];
			LCD_data[25] = LCD_SEG_NUM5[11];
		}
		else //10
		{
			LCD_data[21] = LCD_SEG_NUM6[11];
			LCD_data[22] = LCD_SEG_NUM5[11];
		}
	}
	else
	{
		if(timer_cnt >= 60000)//1ms*6000
		{
			M_down_data1_6h--;
			down_DATA++;
			DIS_Dec_6(M_down_data1_6h);
			timer_cnt = 0;
			
			if(M_down_data1_6h < 0)
			{
				M_down_data1_6h = 999999;
			}
		}
	}
	DIS_Dec_6(M_down_data1_6h);

}


//==============================================================
//==============================================================
//
//
//
//					SEGMENT 6H TIMER MODE END
//
//
//
//==============================================================
//==============================================================


//==============================================================
//
//
//
//
//
//					SEGMENT 6H INPUT MODE START
//
//
//
//
//
//==============================================================

//==============================================================
//					SEGMENT 6H INPUT MODE UP A
//					SEGMENT 6H INPUT MODE UP A
//==============================================================
void M_up_down_a_6h(void)//지령입력 _6h
{
	if(exti_signal)
	{			
		if(GPIOB->IDR & 1<<1)
		{
			if((exti_cnt >= 10) && key_check == 0)
			{
				exti_cnt = 10;
				if(GPIOB->IDR & 1<<2)
				{
					if((up_mode_cnt >= 10) && CP2_key_check == 0)
					{
						up_mode_cnt = 10;
						if(minus)data1_6++;
						else data1_6--;
						CP2_key_check = 1;
					}
				  DIS_Dec_6(data1_6);
				}
				else
				{					
					if(minus)data1_6--;
					else data1_6++;
				}
				key_check = 1;
			}
			DIS_Dec_6(data1_6);
		}
		else exti_signal = 0;
	}
	else
	{
		CP2_key_check = 0;
		key_check = 0;
		DIS_Dec_6(data1_6);
	}

	if(minus)
	{
		if(data1_6 > 99999) data1_6 = 0;
	}
	else
	{
		if(data1_6 > 999999) data1_6 = 0;
	}

	if(data1_6 == 0)
	{
		if(GPIOB->IDR & 1<<2)minus = 1;
		else minus = 0;
	}

	if(RST_INPUT_ON)
	{
		printf("main_end!! \r\n");
		data1_6 = 0; 
		DIS_Dec_6(data1_6);
	}
}
//==============================================================
//					SEGMENT 6H INPUT MODE UP B
//					SEGMENT 6H INPUT MODE UP B
//==============================================================
void M_up_down_b_6h(void)//개별입력 _6h
{
	if((GPIOB->IDR & 1<<1)&&old_sw == 0)
	{signal = 1;
		if((signal_cnt >= 20) && key_check == 0)
		{
			signal_cnt = 20;
			
			if(minus)data1_6--;
			else data1_6++;
			//printf("1111111\r\n");
			key_check = 1;
		}
		if(TACT_KEY->IDR & 1<<2)
		{
			if((up_mode_cnt >= 20) && (new_sw == 0))
			{
				up_mode_cnt = 20;
				
				if(minus)data1_6++;
				else data1_6--;
				//printf("3333333\r\n");
				new_sw = 1;
			}
			DIS_Dec_6(data1_6);
			//printf("zzzzzzzzzzzzzzzzzzz\r\n");
		}
		else new_sw = 0;
		
		DIS_Dec_6(data1_6);
		//printf("2222222\r\n");
	}
	else if((TACT_KEY->IDR & 1<<2) && key_check == 0)// 1 0
	{
		if((up_mode_cnt >= 20) && (old_sw == 0))
		{
			up_mode_cnt = 20;
			
			if(minus)data1_6++;
			else data1_6--;
			//printf("3333333\r\n");
			old_sw = 1;
		}
		if(GPIOB->IDR & 1<<1)
		{
			if((down_mode_cnt >= 20) && but_check == 0)
			{
				down_mode_cnt = 20;
				
				if(minus)data1_6--;
				else data1_6++;
				//printf("1111111\r\n");
				but_check = 1;
			}
			DIS_Dec_6(data1_6);
			//printf("zzzzzzzzzzzzzzzzzzz\r\n");
			
		}
		else but_check = 0;
			
		DIS_Dec_6(data1_6);
		//printf("444444444\r\n");
	}
	else
	{
		but_check = 0;
		new_sw = 0;
		old_sw = 0;
		signal = 0;
		key_check = 0;
		DIS_Dec_6(data1_6);
	}
	
	if(minus)
	{
		if(data1_6 > 99999) data1_6 = 0;
	}
	else
	{
		if(data1_6 > 999999) data1_6 = 0;
	}
	if(data1_6 == 0)
	{
		if((TACT_KEY->IDR & 1<<2) && but_check == 0)
		{
			if(TACT_KEY->IDR & 1<<1)
			{
				minus = 0;
				printf("111111\r\n");
			}
			else 
			{
				minus = 1;
				printf("222222222222\r\n");
			}
		}
		
		if((TACT_KEY->IDR & 1<<1) && old_sw == 0)
		{
			if(TACT_KEY->IDR & 1<<2)
			{
				minus = 1;
				printf("333333\r\n");
			}
			else
			{
				minus = 0;
				printf("44444444\r\n");
			}
		}
	}
	
	if(RST_INPUT_ON)
	{
		printf("main_end!! \r\n");
		data1_6 = 0; 
		DIS_Dec_6(data1_6);
	}
}
//==============================================================
//					SEGMENT 6H INPUT MODE UP C
//					SEGMENT 6H INPUT MODE UP C
//==============================================================

void minus_data1_6h(void)
{
	if(data1_6 == 0)
	{
		if(minus_data == 0) minus = 0;
		else minus = 1;
	}
}
void M_up_down_c_6h(void)//위상차 입력 _6h
{
	if((exti_signal) && CCW == 0)
	{
		if((GPIOB->IDR & 1<<1) && CCW == 0)//high1
		{
			if(exti_cnt >= 50)
			{minus_data = 1;
				exti_cnt = 50;
				printf("CCW\r\n");
				//delay_1ms(1000);
				CW = 1;
				minus_data1_6h();
				if(seg_stop_flag == 0)
				{
					if(minus)
					{printf("in\r\n");
						data1_6++;
						plus = 1;
					}
					else
					{
						data1_6--;
						plus = 0;
					}
					rotate = 1;
					DIS_Dec_6(data1_6);
					printf("DIS_Dec %d\r\n", data1_6);
					seg_stop_flag = 1;
					
				}
				if(TACT_KEY->IDR & 1<<2)//high2
				{
					signal = 1;
					if(signal_cnt >= 50)
					{
						signal_cnt = 50;
						high_in = 1;
						
						printf("high_in\r\n");
						//delay_1ms(1000);
					}
				}
				else
				{
					DIS_Dec_6(data1_6);
					high_in = 0; 
					signal = 0;
				}
			}
		}
		else exti_signal = 0;
	}
	else if((TACT_KEY->IDR & 1<<2) && CW == 0)//high2
	{
		signal = 1;
		if(signal_cnt >= 50)//high1
		{minus_data = 0;
			signal_cnt = 50;
			printf("CW\r\n");
			//delay_1ms(1000);
			CCW = 1;
			
			if((exti_signal)&& CCW)
			{
				if(GPIOB->IDR & 1<<1)//high
				{printf("low_in\r\n");
					if(exti_cnt >= 50)
					{
						exti_cnt = 50;
						low_in = 1;
						
						printf("low_in\r\n");
						//delay_1ms(1000);
					}
				}
				else 
				{
					low_in = 0; 
					exti_signal = 0;
				}
			}
		}
	}
	else if((GPIOB->IDR & 1<<1) == 0 && high_in == 1)
	{
		printf("else if1\r\n");
		//delay_1ms(1000);
		if((TACT_KEY->IDR & 1<<2) == 0)
		{
			rotate = 0;
			printf("out1\r\n");
			//delay_1ms(1000);
			high_in = 0;
			CW = 0;
		}
	}
	else if((TACT_KEY->IDR & 1<<2) == 0 && low_in == 1)
	{
		printf("else if2\r\n");
		//delay_1ms(1000);
		if((GPIOB->IDR & 1<<1) == 0)
		{
			minus_data1_6h();
			if(seg_stop_flag == 0)
			{
				if(minus)
				{
					data1_6--;
					//plus = 0;
				}
				else
				{
					data1_6++;
					//plus = 1;
				}
				DIS_Dec_6(data1_6);
				printf("DIS_Dec %d\r\n", data1_6);
				seg_stop_flag = 1;
			}
			
			printf("out2\r\n");
			//delay_1ms(1000);
			low_in = 0;
			CCW = 0;
		}
	}
	else
	{
		if(rotate)
		{
			if(plus){
				data1_6--;
				DIS_Dec_6(data1_6);
				rotate = 0;
				printf("if2\r\n");
			}
			else{
				data1_6++;
				DIS_Dec_6(data1_6);
				rotate = 0;
				printf("if2\r\n");
			}
		}
		else
		{
			//exti_signal = 0;
			seg_stop_flag = 0;
			plus = 0;
			printf("else1\r\n");
			CCW = 0;
			CW = 0;
			signal = 0;
			DIS_Dec_6(data1_6);
			//delay_1ms(1000);
		}
	}
	
	if(minus)
	{
		if(data1_6 > 99999) data1_6 = 0;
	}
	else
	{
		if(data1_6 > 999999) data1_6 = 0;
	}

	if(RST_INPUT_ON)
	{
		printf("main_end!! \r\n");
		data1_6 = 0; 
		DIS_Dec_6(data1_6);
	}
}

//==============================================================
//					SEGMENT 6H INPUT MODE UP UP
//					SEGMENT 6H INPUT MODE UP UP
//==============================================================
void M_up_6h(void)
{
	if(exti_signal)
	{
		if(GPIOB->IDR & 1<<1)
		{
			if((exti_cnt >= 50) && CP1_key_check == 0)
			{
				if(TACT_KEY->IDR & 1<<2)
				{
					signal = 1;
					if((signal_cnt >= 20) && (CP2_key_check == 0))
					{
						CP2_key_check = 1;
						key_check = 1;
					}
					printf("TACT_KEY\r\n");
				}
				else
				{
					CP1_key_check = 0;
					CP2_key_check = 0;
					
					if(key_check){
						data1_6++;
						key_check = 0;
					}
					
					printf("else\r\n");
					//DIS_Dec_4h(data1_4h);
				}
			}
			//printf("exti_signal %d\r\n", data1_4h);
			DIS_Dec_6(data1_6);
		}
		else exti_signal = 0;
	}
	else
	{
		key_check = 1;
		signal = 0;
		CP1_key_check = 0;
		CP2_key_check = 0;
		DIS_Dec_6(data1_6);
	}
	
	if(data1_6 > 999999) data1_6 = 0; //DIS_Dec(data1);


	if(RST_INPUT_ON)
	{
		printf("main_end!! \r\n");
		data1_6 = 0; 
		DIS_Dec_6(data1_6);
	}
}
//==============================================================
//					SEGMENT 6H INPUT MODE DOWN D
//					SEGMENT 6H INPUT MODE DOWN D
//==============================================================

void M_up_down_d_6h(void)//지령 입력_6h
{

	if(exti_signal)
	{			
		if(GPIOB->IDR & 1<<1)
		{
			if((exti_cnt >= 10) && key_check == 0)
			{
				exti_cnt = 10;
				if(GPIOB->IDR & 1<<2)
				{
					if((up_mode_cnt >= 10) && CP2_key_check == 0)
					{
						up_mode_cnt = 10;
						if(minus)M_down_data1_6h--;
						else M_down_data1_6h++;
						CP2_key_check = 1;
					}
				  DIS_Dec_6(M_down_data1_6h);
				}
				else
				{					
					if(minus)M_down_data1_6h++;
						else M_down_data1_6h--;
				}
				key_check = 1;
			}
			DIS_Dec_6(M_down_data1_6h);
		}
		else exti_signal = 0;
	}
	else
	{
		CP2_key_check = 0;
		key_check = 0;
		DIS_Dec_6(M_down_data1_6h);

	}

	if(minus)
	{
		if(M_down_data1_6h > 99999) M_down_data1_6h = 0;
	}
	else
	{
		if(M_down_data1_6h > 999999) M_down_data1_6h = 0;
	}

	if(M_down_data1_6h == 0)
	{
		if((GPIOB->IDR & 1<<1)&&(GPIOB->IDR & 1<<2)) minus = 0;
		if((GPIOB->IDR & 1<<1)&&(GPIOB->IDR & 1<<2)==0) minus = 1;
	}
	
	if(RST_INPUT_ON)
	{
		printf("main_end!! \r\n");
		M_down_data1_6h = 0;
		DIS_Dec_6(M_down_data1_6h);
	}
}
//==============================================================
//					SEGMENT 6H INPUT MODE DOWN E
//					SEGMENT 6H INPUT MODE DOWN E
//==============================================================
void M_up_down_e_6h(void)//개별 입력 _6h
{
	if((GPIOB->IDR & 1<<1)&&old_sw == 0)
	{signal = 1;
		if((signal_cnt >= 20) && key_check == 0)
		{
			signal_cnt = 20;
			
			if(minus)M_down_data1_6h++;
			else M_down_data1_6h--;
			//printf("1111111\r\n");
			key_check = 1;
		}
		if(TACT_KEY->IDR & 1<<2)
		{
			if((up_mode_cnt >= 20) && (new_sw == 0))
			{
				up_mode_cnt = 20;
				
				if(minus)M_down_data1_6h--;
				else M_down_data1_6h++;
				//printf("3333333\r\n");
				new_sw = 1;
			}
			DIS_Dec_6(M_down_data1_6h);
			//printf("zzzzzzzzzzzzzzzzzzz\r\n");
		}
		else new_sw = 0;
		
		DIS_Dec_6(M_down_data1_6h);
		//printf("2222222\r\n");
	}
	else if((TACT_KEY->IDR & 1<<2) && key_check == 0)// 1 0
	{
		if((up_mode_cnt >= 20) && (old_sw == 0))
		{
			up_mode_cnt = 20;
			
			if(minus)M_down_data1_6h--;
			else M_down_data1_6h++;
			//printf("3333333\r\n");
			old_sw = 1;
		}
		if(GPIOB->IDR & 1<<1)
		{
			if((down_mode_cnt >= 20) && but_check == 0)
			{
				down_mode_cnt = 20;
				
				if(minus)M_down_data1_6h++;
				else M_down_data1_6h--;
				//printf("1111111\r\n");
				but_check = 1;
			}
			DIS_Dec_6(M_down_data1_6h);
			//printf("zzzzzzzzzzzzzzzzzzz\r\n");
			
		}
		else but_check = 0;
			
		DIS_Dec_6(M_down_data1_6h);
		//printf("444444444\r\n");
	}
	else
	{
		but_check = 0;
		new_sw = 0;
		old_sw = 0;
		signal = 0;
		key_check = 0;
		DIS_Dec_6(M_down_data1_6h);
	}
	
	if(minus)
	{
		if(M_down_data1_6h > 99999) M_down_data1_6h = 0;
	}
	else
	{
		if(M_down_data1_6h > 999999) M_down_data1_6h = 0;
	}

	//minus 표준 
	if(M_down_data1_6h == 0)
	{
		if((GPIOB->IDR & 1<<1) && minus_low_check == 0)
		{minus_high_check = 1;
					if(GPIOB->IDR & 1<<2) minus = 0;
					else minus = 1;
		}
		else if((GPIOB->IDR & 1<<2) && minus_high_check == 0)
		{minus_low_check = 1;
					if(GPIOB->IDR & 1<<1) minus = 1;
					else minus = 0;
		}
		else
		{
			minus_high_check = 0;
			minus_low_check = 0;
		}
	}
	
	if(RST_INPUT_ON)
	{
		printf("RST\r\n");
		minus = 0;
		M_down_data1_6h = 999999; 
		DIS_Dec_6(M_down_data1_6h);
	}
		
}

//==============================================================
//					SEGMENT 6H INPUT MODE DOWN F
//					SEGMENT 6H INPUT MODE DOWN F
//==============================================================
void M_up_down_f_6h(void)//위상차 입력 _6h
{
	if((GPIOB->IDR & 1<<1) && (GPIOB->IDR & 1<<2)==0 && second_cp2_in == 0)//11111zzzzzzzzzzzzzzzzzzzzzzz
	{
		if((down_mode_cnt >= 50) && CW == 0)
		{
			if(minus)
			{
				M_down_data1_6h--;
			}
			else
			{
				M_down_data1_6h++;
			}
			
			//DIS_Dec_4h(M_down_data1_4h);
			
			first_cp1_in = 1;
			third_cp1_in = 0;
			second_cp1_in = 0;
			
			rotate_cp1 = 1;
			
			CW = 1;
		}
		
		if(M_down_data1_6h == 0)
		{
			if(GPIOB->IDR & 1<<2)
			{
				minus = 1;
				printf("minus %d \r\n", minus);
			}
			else minus = 0;
		}
		
		DIS_Dec_6(M_down_data1_6h);
		printf("first_cp1_in \r\n");
	}
	else if((GPIOB->IDR & 1<<1) && (GPIOB->IDR & 1<<2) && first_cp1_in == 1)//2222222
	{
		if((up_mode_cnt >= 50) && CCW == 0)
		{
			second_cp1_in = 1;
			third_cp1_in = 0;
			
			rotate_cp1 = 1;
			
			CCW = 1;
		}
		
		printf("second_cp1_in \r\n");
	}
	else if((GPIOB->IDR & 1<<1)== 0 && (GPIOB->IDR & 1<<2) && second_cp1_in == 1)//3333333
	{
		if((up_mode_cnt > 50) && CW_CCW == 0)
		{
			third_cp1_in = 1;
			
			rotate_cp1 = 0;
			
			CW_CCW = 1;
		}
		
		printf("third_cp1_in \r\n");
	}
	else if((GPIOB->IDR & 1<<1)== 0 && (GPIOB->IDR & 1<<2) && second_cp1_in == 0)//33333333
	{
		if((up_mode_cnt >= 50) && CCW == 0)
		{
			first_cp2_in = 1;
			third_cp2_in = 0;
			second_cp2_in = 0;
			
			rotate_cp2 = 1;
			
			CCW = 1;
		}
		
		if(M_down_data1_6h == 0)
		{
			if(GPIOB->IDR & 1<<2)
			{
				minus = 1;
				printf("minus %d \r\n", minus);
			}
			else minus = 0;
		}
		
		printf("first_cp2_in \r\n");
	}
	else if((GPIOB->IDR & 1<<1) && (GPIOB->IDR & 1<<2)&& first_cp2_in == 1)//222222
	{
		if((down_mode_cnt >= 50) && CW == 0)
		{
			second_cp2_in = 1;
			third_cp2_in = 0;
			
			rotate_cp2 = 1;
			
			CW = 1;
		}
		
		printf("second_cp2_in %d\r\n", minus);
	}
	else if((GPIOB->IDR & 1<<1) && (GPIOB->IDR & 1<<2)==0 && second_cp2_in == 1)//1111111
	{
		if((down_mode_cnt >= 50) && CW_CCW == 0)
		{
			
			third_cp2_in = 1;
			rotate_cp2 = 0;
			
			CW_CCW = 1;
		}
		
		printf("third_cp2_in \r\n");
	}
	else
	{
		if(rotate_cp1 && third_cp1_in == 0){
			//--
			if(minus)M_down_data1_6h++;
			else
			{
				if(data_9999){
					minus = 1;
					M_down_data1_6h++;
				}
				else M_down_data1_6h--;
			}
			
			rotate_cp1 = 0;
			printf("111111\r\n");
		}
		if(rotate_cp1 == 0 && third_cp1_in){
			//
			third_cp1_in = 0;
			printf("222222\r\n");
		}
		if(rotate_cp2 && third_cp2_in == 0){
			//++
			rotate_cp2 = 0;
			printf("333333\r\n");
		}
		if(rotate_cp2 == 0 && third_cp2_in){
			//				
			if(minus){M_down_data1_6h++;
			printf("+++++++++++++++\r\n");}
			else{ M_down_data1_6h--;
			printf("--------------- %d\r\n",minus );}
			third_cp2_in = 0;
			printf("4444444\r\n");
		}
		else 
		{
			first_cp1_in=0;
			second_cp1_in=0;
			third_cp1_in=0;
			
			first_cp2_in=0;
			second_cp2_in=0;
			third_cp2_in=0;
			
			CCW = 0;
			CW = 0;
			CW_CCW = 0;
			
			rotate_cp1 = 0;
			rotate_cp2 = 0;
			
			DIS_Dec_6(M_down_data1_6h);
		}
	}

	if(minus)
	{
		if(M_down_data1_6h > 99999) M_down_data1_6h = 0;
	}
	
	if(minus == 0)
	{
		if(M_down_data1_6h > 999999)
		{
			data_9999 = 1;
			M_down_data1_6h = 0;
			
		}
	}
	else data_9999 = 0;
	
	if(RST_INPUT_ON)
	{
		printf("RST\r\n");
		minus = 0;
		M_down_data1_6h = 999999; 
		DIS_Dec_6(M_down_data1_6h);
	}
	
}

//==============================================================
//					SEGMENT 6H INPUT MODE DOWN DOWN
//					SEGMENT 6H INPUT MODE DOWN DOWN
//==============================================================
void M_down_6h(void)//감산 입력 _6h
{
	if(exti_signal)
	{
		if(GPIOB->IDR & 1<<1)
		{
			if((exti_cnt >= 50) && CP1_key_check == 0)
			{
				if(TACT_KEY->IDR & 1<<2)
				{
					signal = 1;
					if((signal_cnt >= 20) && (CP2_key_check == 0))
					{
						CP2_key_check = 1;
						key_check = 1;
					}
					printf("TACT_KEY\r\n");
				}
				else
				{
					CP1_key_check = 0;
					CP2_key_check = 0;
					
					if(key_check){
						M_down_data1_6h--;
						key_check = 0;
					}
					
					printf("else\r\n");
					//DIS_Dec_4h(data1_4h);
				}
			}
			//printf("exti_signal %d\r\n", data1_4h);
			DIS_Dec_6(M_down_data1_6h);
		}
		else exti_signal = 0;
	}
	else
	{
		key_check = 1;
		signal = 0;
		CP1_key_check = 0;
		CP2_key_check = 0;
		DIS_Dec_6(M_down_data1_6h);
	}
	
	if(M_down_data1_6h > 999999) M_down_data1_6h = 0; //DIS_Dec(data1);

	if(RST_INPUT_ON)
	{
		printf("RST\r\n");
		minus = 0;
		M_down_data1_6h = 999999; 
		DIS_Dec_6(M_down_data1_6h);
	}
	
}
//==============================================================
//					SEGMENT 6 INPUT MODE END
//==============================================================

/* CPS_MODE -----------------------------------------------------------*/
void cps_mode(void)
{
	if((GPIOE->ODR |= 1<<5)&&(GPIOE->ODR |= 1<<2)==0)_1CPS;
	else if((GPIOE->ODR |= 1<<5)==0&&(GPIOE->ODR |= 1<<2)==0)_30CPS;
	else if((GPIOE->ODR |= 1<<5)==0&&(GPIOE->ODR |= 1<<2))_2KCPS;
	else _5KCPS;
}

/* RESET_MODE -----------------------------------------------------------*/
void reset_mode(void)
{
	if(GPIOE->ODR |= 1<<1)RESET_ON = 1;
	else RESET_ON = 0;
}

/* E2P_MODE -----------------------------------------------------------*/
void eeprom_mode(void)
{
	if(GPIOE->ODR |= 1<<0)EEPROM_ON = 0;
	else EEPROM_ON = 1;
}

/* NPNP_MODE -----------------------------------------------------------*/
void npnpn_mode(void)
{
	if(GPIOG->ODR |= 1<<0)EXIT_Rising_Config();//npn
	else EXIT_Falling_Config();//pnp
}

/* CNT_TIM_MODE -----------------------------------------------------------*/
void mode_select(void)
{
	if(GPIOG->ODR |= 1<<3)COUNTER_MODE = 1;
	else COUNTER_MODE = 0;
	
	if(COUNTER_MODE)
	{
		if((GPIOD->ODR |= 1<<3)==0&&(GPIOD->ODR |= 1<<1)==0&&(GPIOD->ODR |= 1<<0)==0)M_up_down_a_4h(); //가산 지령입력
		else if((GPIOD->ODR |= 1<<3)&&(GPIOD->ODR |= 1<<1)==0&&(GPIOD->ODR |= 1<<0)==0)M_up_down_b_4h(); //가산 개별입력 
		else if((GPIOD->ODR |= 1<<3)==0&&(GPIOD->ODR |= 1<<1)&&(GPIOD->ODR |= 1<<0)==0)M_up_down_c_4h(); //가산 위상차입력 
		else if((GPIOD->ODR |= 1<<3)&&(GPIOD->ODR |= 1<<1)&&(GPIOD->ODR |= 1<<0)==0)M_up_4h(); //가산 입력 
		else if((GPIOD->ODR |= 1<<3)==0&&(GPIOD->ODR |= 1<<1)==0&&(GPIOD->ODR |= 1<<0))M_up_down_d_4h(); //감산 기령입력 
		else if((GPIOD->ODR |= 1<<3)&&(GPIOD->ODR |= 1<<1)==0&&(GPIOD->ODR |= 1<<0))M_up_down_e_4h(); //감산 개별입력 
		else if((GPIOD->ODR |= 1<<3)==0&&(GPIOD->ODR |= 1<<1)&&(GPIOD->ODR |= 1<<0))M_up_down_f_4h(); //감산 위상차 입력 
		else if((GPIOD->ODR |= 1<<3)&&(GPIOD->ODR |= 1<<1)&&(GPIOD->ODR |= 1<<0))M_down_4h(); // 감산 입력 
		else {}//err
	}
	else //timer mode
	{
		if(GPIOD->ODR |= 1<<3)//down
		{
			if((GPIOD->ODR |= 1<<2)==0&&(GPIOD->ODR |= 1<<3)==0&&(GPIOD->ODR |= 1<<1)==0)M_timer_down_99_99s();
			else if((GPIOD->ODR |= 1<<2)&&(GPIOD->ODR |= 1<<3)==0&&(GPIOD->ODR |= 1<<1)==0)M_timer_down_999_9s();
			else if((GPIOD->ODR |= 1<<2)==0&&(GPIOD->ODR |= 1<<3)&&(GPIOD->ODR |= 1<<1)==0)M_timer_down_9999s();
			else if((GPIOD->ODR |= 1<<2)&&(GPIOD->ODR |= 1<<3)&&(GPIOD->ODR |= 1<<1)==0)M_timer_down_99m59s();
			else if((GPIOD->ODR |= 1<<2)==0&&(GPIOD->ODR |= 1<<3)==0&&(GPIOD->ODR |= 1<<1))M_timer_down_999_9m(); //미검증 
			else if((GPIOD->ODR |= 1<<2)&&(GPIOD->ODR |= 1<<3)==0&&(GPIOD->ODR |= 1<<1))M_timer_down_99h59m(); //미검증 
			else if((GPIOD->ODR |= 1<<2)==0&&(GPIOD->ODR |= 1<<3)&&(GPIOD->ODR |= 1<<1))M_timer_down_999_9h(); //미검증 
			else if((GPIOD->ODR |= 1<<2)&&(GPIOD->ODR |= 1<<3)&&(GPIOD->ODR |= 1<<1))M_timer_down_9999h(); //미검증
			else {}//err
			
		}
		else//up
		{
			if((GPIOD->ODR |= 1<<2)==0&&(GPIOD->ODR |= 1<<3)==0&&(GPIOD->ODR |= 1<<1)==0)M_timer_up_99_99s();// check
			else if((GPIOD->ODR |= 1<<2)&&(GPIOD->ODR |= 1<<3)==0&&(GPIOD->ODR |= 1<<1)==0)M_timer_up_999_9s();// check
			else if((GPIOD->ODR |= 1<<2)==0&&(GPIOD->ODR |= 1<<3)&&(GPIOD->ODR |= 1<<1)==0)M_timer_up_9999s();// check
			else if((GPIOD->ODR |= 1<<2)&&(GPIOD->ODR |= 1<<3)&&(GPIOD->ODR |= 1<<1)==0)M_timer_up_99m59s();// check
			else if((GPIOD->ODR |= 1<<2)==0&&(GPIOD->ODR |= 1<<3)==0&&(GPIOD->ODR |= 1<<1))M_timer_up_999_9m();//
			else if((GPIOD->ODR |= 1<<2)&&(GPIOD->ODR |= 1<<3)==0&&(GPIOD->ODR |= 1<<1))M_timer_up_99h59m();//
			else if((GPIOD->ODR |= 1<<2)==0&&(GPIOD->ODR |= 1<<3)&&(GPIOD->ODR |= 1<<1))M_timer_up_999_9h();//
			else if((GPIOD->ODR |= 1<<2)&&(GPIOD->ODR |= 1<<3)&&(GPIOD->ODR |= 1<<1))M_timer_up_9999h();//
			else {}//err
		}
	}
}



int seg,a,i,lcd_on = 0;
char ans;

int count=0;

unsigned char *rrxx_buf;
void main(void)
{
	//uint8_t vvvrrr=0x31;
	//printf("main_start!! \r\n");
	CLK_Config();
	GPIO_Config();
	USART_Config();
  TIMER3_Config();
	EXIT_Rising_Config();
	//EXIT_Falling_Config();
	//DIS_Dec_4h(0);
	//I2C_Config();
	
	enableInterrupts();
	BACK_LIGHT_ON;
	
		
	while (1)
  {
		//reset_mode();
		
		//mode_select();
		//npnpn_mode();
		//eeprom_mode();
		//cps_mode();
		
	
		/*
		if(timer_cnt >= 1)
		{
			old_sw = !old_sw;
			
			if(old_sw)GPIOG->ODR |= 1<<1;
			else GPIOG->ODR &= ~(1<<1);
			timer_cnt = 0;
		}
		*/
		
		COUNTER_MODE = 1;
		/*----------------------4H counter------------------------------------*/
		M_up_down_a_4h(); //가산 지령입력 check ok!!
		//M_up_down_b_4h(); //가산 개별입력 check ok!!
		//M_up_down_c_4h(); //가산 위상차입력 check ok!!
		//M_up_4h(); //가산 입력 check ok!!
		//M_up_down_d_4h(); //감산 기령입력 check ok!!
		//M_up_down_e_4h(); //감산 개별입력 check ok!!
		
		//M_up_down_f_4h(); //감산 위상차 입력 check ok!!
		
		//M_down_4h(); // 감산 입력 check ok!!
		

		
		/*----------------------6H counter------------------------------------*/
		
		//M_up_down_a_6h(); //가산 지령입력  check ok!!
		//M_up_down_b_6h(); //가산 개별입력  check ok!!
		//M_up_down_c_6h(); //가산 위상차입력  check ok!!
		//M_up_6h(); //가산 입력  check ok!!
		//M_up_down_d_6h(); //감산 기령입력  check ok!!
		//M_up_down_e_6h(); //감산 개별입력  check ok!!
		//M_up_down_f_6h(); //감산 위상차 입력  check ok!!
		
		//M_down_6h(); // 감산 입력  check ok!!
		
		/*----------------------4H timer------------------------------------*/
		
		//M_timer_up_99_99s();// check rst dp
		//M_timer_up_999_9s();// check rst dp
		//M_timer_up_9999s();// check rst dp
		//M_timer_up_99m59s();// check rst dp
		//M_timer_up_999_9m();// check rst dp
		//M_timer_up_99h59m();// check rst dp
		//M_timer_up_999_9h();// check rst dp
		//M_timer_up_9999h();// check rst dp

		//M_timer_down_99_99s();// check rst dp
		//M_timer_down_999_9s();// check rst dp
		//M_timer_down_9999s();// check rst dp
		//M_timer_down_99m59s();// check rst dp
		//M_timer_down_999_9m(); //미검증 // check rst dp
		//M_timer_down_99h59m(); //미검증 // check rst dp
		//M_timer_down_999_9h(); //미검증 // check rst dp
		//M_timer_down_9999h(); //미검증 // check rst dp
		
		/*----------------------6H timer------------------------------------*/
		//M_timer_up_99999_9s();
		//M_timer_up_999999s();
		//M_timer_up_99m59_99s();
		//M_timer_up_999m59_9s();
		//M_timer_up_99999_9m();
		//M_timer_up_99h59m59s();
		//M_timer_up_9999h59m();
		//M_timer_up_99999_9h();
		
		//M_timer_down_99999_9s();
		//M_timer_down_999999s();
		//M_timer_down_99m59_99s();
		//M_timer_down_999m59_9s();
		//M_timer_down_99999_9m();
		//M_timer_down_99h59m59s();
		//M_timer_down_9999h59m();
		//M_timer_down_99999_9h();
	}
}

@far @interrupt void TIM3_UPD_OVF_TRG_BRK_USART3_TX_IRQHandler(void) //1ms
{
	if(COUNTER_MODE)
	{
		if(exti_signal)exti_cnt++;
		else exti_cnt = 0;
	
		if(signal)signal_cnt++;
		else signal_cnt = 0;
	
		if(GPIOB->IDR & 1<<2)up_mode_cnt++;
		else up_mode_cnt = 0;
	
		if(GPIOB->IDR & 1<<1)down_mode_cnt++;
		else down_mode_cnt = 0;
		
		if(rst_input_flag)rst_input_cnt++;
		else rst_input_cnt = 0;
		
	}	
	else
	{	
		if(timer_flag)timer_cnt++;
		else timer_cnt = 0;
	
		if(GPIOG->IDR & 1<<2)key_cnt++;
		else key_cnt = 0;
		//if(GPIOG->IDR & 1<<2)key_cnt++;
		//else key_flag = 0;
	}

	TIM3->SR1 = 0xFE;//인터럽트 플레이그 클리어
}


@far @interrupt void EXTI1_IRQHandler(void)
{	
	exti_signal = 1;
	
  EXTI->SR1  |= EXTI_IT_Pin1;  // EXTI_ClearITPendingBit(EXTI_IT_Pin1);
}
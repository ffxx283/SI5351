#ifndef _si5351a_h
#define _si5351a_h

#include "stm32f10x.h"

typedef unsigned  short int   uint;
typedef enum {FALSE = 0, TRUE = !FALSE} bool;

//IIC×ÜÏßÒý½ÅÅäÖÃ
#define SCL_H         GPIOA->BSRR = GPIO_Pin_3
#define SCL_L         GPIOA->BRR  = GPIO_Pin_3 
   
#define SDA_H         GPIOA->BSRR = GPIO_Pin_5
#define SDA_L         GPIOA->BRR  = GPIO_Pin_5

#define SCL_read      GPIOA->IDR  & GPIO_Pin_3
#define SDA_read      GPIOA->IDR  & GPIO_Pin_5


#define SI_CLK0_CONTROL	16			// Register definitions
#define SI_CLK1_CONTROL	17
#define SI_CLK2_CONTROL	18
#define SI_SYNTH_PLL_A	26
#define SI_SYNTH_PLL_B	34
#define SI_SYNTH_MS_0		42
#define SI_SYNTH_MS_1		50
#define SI_SYNTH_MS_2		58
#define SI_PLL_RESET		177

#define SI_R_DIV_1		0x00			// R-division ratio definitions
#define SI_R_DIV_2		0b00010000
#define SI_R_DIV_4		0b00100000
#define SI_R_DIV_8		0b00110000
#define SI_R_DIV_16		0b01000000
#define SI_R_DIV_32		0b01010000
#define SI_R_DIV_64		0b01100000
#define SI_R_DIV_128		0b01110000

#define SI_CLK_SRC_PLL_A	0x00
#define SI_CLK_SRC_PLL_B	0b00100000
#define XTAL_FREQ	25000000			// Crystal frequency



static void I2C_GPIO_Config(void);
void I2C_SendByte(u8 dat);
uint8_t i2cSendRegister(uint8_t reg, uint8_t data);
void IIC_SI5351A_GPIO_Init(void);


void setupPLL(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom);
void setupMultisynth(uint8_t synth,uint32_t divider,uint8_t rDiv);

void si5351aSetFrequency(uint32_t frequency , u8 Chanal );






#endif


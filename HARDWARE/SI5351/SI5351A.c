#include "si5351a.h"

static void I2C_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd  (RCC_APB2Periph_GPIOA, ENABLE );  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_3;  /* PA3-I2C_SCL、PA
	5-I2C_SDA*/
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

///////////IIC初始化//////////////
void IIC_SI5351A_GPIO_Init(void)
{	   
  I2C_GPIO_Config();
}  
////////////粗略延时函数//////////
void Delay_1us(u16 n)//约1us,1100k
{
  unsigned int x=5,i=0;
  for(i=0;i<n;i++)
  {while(x--);x=5;}
}
void Delay_ms(u8 n)//约0.5ms，2k
{
  unsigned int x=5000,i=0;
  for(i=0;i<n;i++)
  {while(x--);x=5000;}
}
////////IIC启动函数//////////
void I2C_Start(void)
{
  SDA_H; 	
  SCL_H;
  Delay_1us(1);
  if(!SDA_read) return;//SDA线为低电平则总线忙,退出
  SDA_L;
  Delay_1us(1);
  if(SDA_read) return;//SDA线为高电平则总线出错,退出
  SDA_L;
  Delay_1us(1);
  SCL_L;
}
//**************************************
//IIC停止信号
//**************************************
void I2C_Stop(void)
{
  SDA_L;
  SCL_L;
  Delay_1us(1);
  SCL_H;
  SDA_H;
  Delay_1us(1);                 //延时
} 
//**************************************
//IIC发送应答信号
//入口参数:ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(u8 i)
{
  if(1==i)SDA_H;                  //写应答信号
  else SDA_L;
  SCL_H;                    //拉高时钟线
  Delay_1us(1);                 //延时
  SCL_L ;                  //拉低时钟线
  Delay_1us(1);    
} 
//**************************************
//IIC等待应答
//返回值：ack (1:ACK 0:NAK)
//**************************************
 bool I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
  unsigned int i;
  SDA_H;			
  Delay_1us(1);
  SCL_H;
  Delay_1us(1);
  while(SDA_read){i++;if(i==5000)break;}
  if(SDA_read)
  {SCL_L;
  Delay_1us(1);
  return FALSE;}
  SCL_L;
  Delay_1us(1);
  return TRUE;
}

//**************************************
//向IIC总线发送一个字节数据
//**************************************
void I2C_SendByte(u8 dat)
{
  unsigned int i;
  SCL_L;
  for (i=0; i<8; i++)         //8位计数器
  {
    if(dat&0x80){SDA_H;}   //送数据口
    else SDA_L;
    SCL_H;                //拉高时钟线
    Delay_1us(1);             //延时
    SCL_L;                //拉低时钟线
    Delay_1us(1); 		  //延时
    dat <<= 1;          //移出数据的最高位  
  }					 
}

//**************************************
//从IIC总线接收一个字节数据
//**************************************
u8 I2C_RecvByte()
{
  u8 i;
  u8 dat = 0;
  SDA_H;                    //使能内部上拉,准备读取数据,
  for (i=0; i<8; i++)         //8位计数器
  {
    dat <<= 1;
    SCL_H;                //拉高时钟线
    Delay_1us(1);            //延时
    if(SDA_read) //读数据    
    {
      dat |=0x01;
    }                           
    SCL_L;                //拉低时钟线
    Delay_1us(1);
  } 	
  return dat;
} 
//**************************************
//向IIC设备写入一个字节数据
//**************************************
bool Single_WriteI2C(u8 Slave_Address,u8 REG_Address,u8 REG_data)
{
  I2C_Start();              //起始信号
  I2C_SendByte(Slave_Address);   //发送设备地址+写信号
  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
  I2C_SendByte(REG_Address);    //内部寄存器地址，
  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
  I2C_SendByte(REG_data);       //内部寄存器数据，
  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
  I2C_Stop();                   //发送停止信号
	return TRUE;
}


u8 Single_ReadI2C(u8 Slave_Address,u8 REG_Address)
{
  u8 REG_data;
  I2C_Start();                   //起始信号
  I2C_SendByte(Slave_Address);    //发送设备地址+写信号
  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;} 
  I2C_SendByte(REG_Address);     //发送存储单元地址，从0开始
  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;} 
  I2C_Start();                   //起始信号
  I2C_SendByte(Slave_Address+1);  //发送设备地址+读信号
  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
  REG_data=I2C_RecvByte();       //读出寄存器数据
  I2C_SendACK(1);                //发送停止传输信号
  I2C_Stop();                    //停止信号
  return REG_data;
}


uint8_t i2cSendRegister(uint8_t reg, uint8_t data)
{
  I2C_Start();              //起始信号
  I2C_SendByte(0xC0);   //发送设备地址+写信号
  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
  I2C_SendByte(reg);    //内部寄存器地址，
  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
  I2C_SendByte(data);       //内部寄存器数据，
  if(!I2C_WaitAck()){I2C_Stop(); return FALSE;}
  I2C_Stop(); 
  return 0;
}


void setupPLL(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom)
{
  uint32_t P1;					// PLL config register P1
  uint32_t P2;					// PLL config register P2
  uint32_t P3;					// PLL config register P3

  P1 = (uint32_t)(128 * ((float)num / (float)denom));
  P1 = (uint32_t)(128 * (uint32_t)(mult) + P1 - 512);
  P2 = (uint32_t)(128 * ((float)num / (float)denom));
  P2 = (uint32_t)(128 * num - denom * P2);
  P3 = denom;

  i2cSendRegister(pll + 0, (P3 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 1, (P3 & 0x000000FF));
  i2cSendRegister(pll + 2, (P1 & 0x00030000) >> 16);
  i2cSendRegister(pll + 3, (P1 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 4, (P1 & 0x000000FF));
  i2cSendRegister(pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
  i2cSendRegister(pll + 6, (P2 & 0x0000FF00) >> 8);
  i2cSendRegister(pll + 7, (P2 & 0x000000FF));
}


void setupMultisynth(uint8_t synth,uint32_t divider,uint8_t rDiv)
{
  uint32_t P1;					// Synth config register P1
  uint32_t P2;					// Synth config register P2
  uint32_t P3;					// Synth config register P3

  P1 = 128 * divider - 512;
  P2 = 0;							// P2 = 0, P3 = 1 forces an integer value for the divider
  P3 = 1;

  i2cSendRegister(synth + 0,   (P3 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 1,   (P3 & 0x000000FF));
  i2cSendRegister(synth + 2,   ((P1 & 0x00030000) >> 16) | rDiv);
  i2cSendRegister(synth + 3,   (P1 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 4,   (P1 & 0x000000FF));
  i2cSendRegister(synth + 5,   ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
  i2cSendRegister(synth + 6,   (P2 & 0x0000FF00) >> 8);
  i2cSendRegister(synth + 7,   (P2 & 0x000000FF));
}


void si5351aSetFrequency(uint32_t frequency , u8 Chanal )
{
  uint32_t pllFreq;
  uint32_t xtalFreq = XTAL_FREQ;// Crystal frequency
  uint32_t l;
  float f;
  uint8_t mult;
  uint32_t num;
  uint32_t denom;
  uint32_t divider;

  divider = 900000000 / frequency;// Calculate the division ratio. 900,000,000 is the maximum internal 
                                                                  // PLL frequency: 900MHz
  if (divider % 2) divider--;		// Ensure an even integer division ratio

  pllFreq = divider * frequency;	// Calculate the pllFrequency: the divider * desired output frequency

  mult = pllFreq / xtalFreq;		// Determine the multiplier to get to the required pllFrequency
  l = pllFreq % xtalFreq;			// It has three parts:
  f = l;							// mult is an integer that must be in the range 15..90
  f *= 1048575;					// num and denom are the fractional parts, the numerator and denominator
  f /= xtalFreq;					// each is 20 bits (range 0..1048575)
  num = f;						// the actual multiplier is  mult + num / denom
  denom = 1048575;				// For simplicity we set the denominator to the maximum 1048575
  // Set up PLL A with the calculated multiplication ratio
  setupPLL(SI_SYNTH_PLL_A, mult, num, denom);
                                                                  // Set up MultiSynth divider 0, with the calculated divider. 
                                                                  // The final R division stage can divide by a power of two, from 1..128. 
                                                                  // reprented by constants SI_R_DIV1 to SI_R_DIV128 (see si5351a.h header file)
                                                                  // If you want to output frequencies below 1MHz, you have to use the 
                                                                  // final R division stage
  if( Chanal == 0 ){
		setupMultisynth(SI_SYNTH_MS_0,divider,SI_R_DIV_1);
                                                                  // Reset the PLL. This causes a glitch in the output. For small changes to 
                                                                  // the parameters, you don't need to reset the PLL, and there is no glitch
		i2cSendRegister(SI_PLL_RESET,0xA0);	
                                                                  // Finally switch on the CLK0 output (0x4F)
                                                                  // and set the MultiSynth0 input to be PLL A
		i2cSendRegister(SI_CLK0_CONTROL, 0x4F|SI_CLK_SRC_PLL_A);
	}
	else if ( Chanal == 1 ){
		setupMultisynth(SI_SYNTH_MS_1,divider,SI_R_DIV_1);
		i2cSendRegister(SI_PLL_RESET,0xA0);	
		i2cSendRegister(SI_CLK1_CONTROL, 0x4F|SI_CLK_SRC_PLL_A);
	}
		else if ( Chanal == 2 ){
		setupMultisynth(SI_SYNTH_MS_2,divider,SI_R_DIV_1);
		i2cSendRegister(SI_PLL_RESET,0xA0);	
		i2cSendRegister(SI_CLK2_CONTROL, 0x4F|SI_CLK_SRC_PLL_A);
		}
}






/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

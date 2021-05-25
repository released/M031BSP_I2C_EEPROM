/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M031 MCU.
 *
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#include "i2c_master.h"

typedef enum{
	flag_Erase = 0 ,
	flag_Dump ,
	flag_Read ,
	flag_WriteAddr ,
	flag_WriteData ,

	flag_WriteData1 ,
	flag_WriteData2 ,

	/*---------------*/
	flag_RegCtrl , 
	
	flag_DEFAULT	
}flag_Index;

#define HIBYTE(v1)              					((uint8_t)((v1)>>8))                      //v1 is UINT16
#define LOBYTE(v1)              					((uint8_t)((v1)&0xFF))

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

uint32_t conter_tick = 0;


// I2C
#define EEPROM_SLAVE_ADDR    					(0xA0)


volatile uint8_t g_u8DeviceAddr_m;
volatile uint8_t g_u8DataLen_m;
volatile uint8_t rawlenth;
volatile uint16_t g_au16Reg;
volatile uint8_t g_u8EndFlag = 0;
uint8_t *g_au8Buffer;

typedef void (*I2C_FUNC)(uint32_t u32Status);

I2C_FUNC __IO I2Cx_Master_HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/

void convertDecToBin(int n)
{
    int k = 0;
    unsigned char *p = (unsigned char*)&n;
    int val2 = 0;
    int i = 0;
    for(k = 0; k <= 1; k++)
    {
        val2 = *(p+k);
        for (i = 7; i >= 0; i--)
        {
            if(val2 & (1 << i))
                printf("1");
            else
                printf("0");
        }
        printf(" ");
    }
}

void Delay(uint16_t delay)
{
	while(delay--);
}

void tick_counter(void)
{
	conter_tick++;
}

uint32_t get_tick(void)
{
	return (conter_tick);
}

void I2Cx_Master_LOG(uint32_t u32Status)
{
	#if defined (DEBUG_LOG_MASTER_LV1)
    printf("%s  : 0x%2x \r\n", __FUNCTION__ , u32Status);
	#endif
}


void I2Cx_Master_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(MASTER_I2C);

    if (I2C_GET_TIMEOUT_FLAG(MASTER_I2C))
    {
        /* Clear I2C Timeout Flag */
        I2C_ClearTimeoutFlag(MASTER_I2C);                   
    }    
    else
    {
        if (I2Cx_Master_HandlerFn != NULL)
            I2Cx_Master_HandlerFn(u32Status);
    }
}

void I2Cx_MasterRx_multi(uint32_t u32Status)
{
    if(u32Status == MASTER_START_TRANSMIT) //0x08                       	/* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(MASTER_I2C, ((g_u8DeviceAddr_m << 1) | I2C_WR));    				/* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);

		I2Cx_Master_LOG(u32Status);
    }
    else if(u32Status == MASTER_TRANSMIT_ADDRESS_ACK) //0x18        			/* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(MASTER_I2C, HIBYTE(g_au16Reg));
        I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI );

		set_flag(flag_RegCtrl , ENABLE);
		
		I2Cx_Master_LOG(u32Status);
    }
    else if(u32Status == MASTER_TRANSMIT_ADDRESS_NACK) //0x20            	/* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI | I2C_CTL_STA | I2C_CTL_STO);
		
		I2Cx_Master_LOG(u32Status);
    }
    else if(u32Status == MASTER_TRANSMIT_DATA_ACK) //0x28                  	/* DATA has been transmitted and ACK has been received */
    {
        if (rawlenth > 0)
        {
			if (is_flag_set(flag_RegCtrl))
			{
				set_flag(flag_RegCtrl , DISABLE);

				I2C_SET_DATA(MASTER_I2C, LOBYTE(g_au16Reg));
	        	I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI );
			}
			else
			{		
				I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI | I2C_CTL_STA);				//repeat start
			}
        }
		else
		{
			I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI | I2C_CTL_STO);
			g_u8EndFlag = 1;
		}
		
		I2Cx_Master_LOG(u32Status);
    }
    else if(u32Status == MASTER_REPEAT_START) //0x10                  		/* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(MASTER_I2C, ((g_u8DeviceAddr_m << 1) | I2C_RD));   		/* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI );
		
		I2Cx_Master_LOG(u32Status);
    }
    else if(u32Status == MASTER_RECEIVE_ADDRESS_ACK) //0x40                	/* SLA+R has been transmitted and ACK has been received */
    {
		if (rawlenth > 1)
			I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);
		else
			I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);

		I2Cx_Master_LOG(u32Status);
    }
	else if(u32Status == MASTER_RECEIVE_DATA_ACK) //0x50                 	/* DATA has been received and ACK has been returned */
    {
        g_au8Buffer[g_u8DataLen_m++] = (unsigned char) I2C_GetData(MASTER_I2C);
        if (g_u8DataLen_m < (rawlenth-1))
		{
			I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);
		}
		else
		{
			I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);
		}
		
		I2Cx_Master_LOG(u32Status);
    }
    else if(u32Status == MASTER_RECEIVE_DATA_NACK) //0x58                  	/* DATA has been received and NACK has been returned */
    {
        g_au8Buffer[g_u8DataLen_m++] = (unsigned char) I2C_GetData(MASTER_I2C);
        I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI | I2C_CTL_STO);
        g_u8EndFlag = 1;

		
		I2Cx_Master_LOG(u32Status);
    }
    else
    {
		#if defined (DEBUG_LOG_MASTER_LV1)
        /* TO DO */
        printf("I2Cx_MasterRx_multi Status 0x%x is NOT processed\n", u32Status);
		#endif
    }
}

void I2Cx_MasterTx_multi(uint32_t u32Status)
{	
    if(u32Status == MASTER_START_TRANSMIT)  //0x08                     	/* START has been transmitted */
    {
        I2C_SET_DATA(MASTER_I2C, ((g_u8DeviceAddr_m << 1) | I2C_WR));    			/* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);

		I2Cx_Master_LOG(u32Status);
		
    }
    else if(u32Status == MASTER_TRANSMIT_ADDRESS_ACK)  //0x18           	/* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(MASTER_I2C, HIBYTE(g_au16Reg));
        I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI );

		set_flag(flag_RegCtrl , ENABLE);
		
		I2Cx_Master_LOG(u32Status);	
    }
    else if(u32Status == MASTER_TRANSMIT_ADDRESS_NACK) //0x20           /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI | I2C_CTL_STA | I2C_CTL_STO);

		I2Cx_Master_LOG(u32Status);	
    }
    else if(u32Status == MASTER_TRANSMIT_DATA_ACK) //0x28              	/* DATA has been transmitted and ACK has been received */
    {
        if(g_u8DataLen_m < rawlenth)
        {
			if (is_flag_set(flag_RegCtrl))
			{
				set_flag(flag_RegCtrl , DISABLE);

				I2C_SET_DATA(MASTER_I2C, LOBYTE(g_au16Reg));
	        	I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);
			}
			else
			{
	            I2C_SET_DATA(MASTER_I2C, g_au8Buffer[g_u8DataLen_m++]);
	            I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI);
			}
        }
        else
        {
            I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI | I2C_CTL_STO);
            g_u8EndFlag = 1;
        }

		I2Cx_Master_LOG(u32Status);		
    }
    else if(u32Status == MASTER_ARBITRATION_LOST) //0x38
    {
		I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_STA_SI_AA);

		I2Cx_Master_LOG(u32Status);		
    }
    else if(u32Status == BUS_ERROR) //0x00
    {
		I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_STO_SI_AA);
		I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_SI_AA);
		
		I2Cx_Master_LOG(u32Status);		
    }		
    else
    {
		#if defined (DEBUG_LOG_MASTER_LV1)
        /* TO DO */
        printf("I2Cx_MasterTx_multi Status 0x%x is NOT processed\n", u32Status);
		#endif
    }
}

void I2Cx_WriteMultiToSlaveIRQ(uint8_t address,uint16_t reg,uint8_t *data,uint16_t len)
{		
	g_u8DeviceAddr_m = address;
	rawlenth = len;
	g_au16Reg = reg;
	g_au8Buffer = data;

	g_u8DataLen_m = 0;
	g_u8EndFlag = 0;

	/* I2C function to write data to slave */
	I2Cx_Master_HandlerFn = (I2C_FUNC)I2Cx_MasterTx_multi;

//	printf("I2Cx_MasterTx_multi finish\r\n");

	/* I2C as master sends START signal */
	I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_STA);

	/* Wait I2C Tx Finish */
	while(g_u8EndFlag == 0);
	g_u8EndFlag = 0;
}

void I2Cx_ReadMultiFromSlaveIRQ(uint8_t address,uint16_t reg,uint8_t *data,uint16_t len)
{ 
	g_u8DeviceAddr_m = address;
	rawlenth = len;
	g_au16Reg = reg ;
	g_au8Buffer = data;

	g_u8EndFlag = 0;
	g_u8DataLen_m = 0;

	/* I2C function to read data from slave */
	I2Cx_Master_HandlerFn = (I2C_FUNC)I2Cx_MasterRx_multi;

//	printf("I2Cx_MasterRx_multi finish\r\n");
	
	I2C_SET_CONTROL_REG(MASTER_I2C, I2C_CTL_STA);

	/* Wait I2C Rx Finish */
	while(g_u8EndFlag == 0);
	
}



void I2Cx_Init(void)	//PB1 : SCL , PB0 : SDA
{
	if (MASTER_I2C == I2C0)
	{
    	SYS_ResetModule(I2C0_RST);
	}
	else
	{
    	SYS_ResetModule(I2C1_RST);
	}


    /* Open I2C module and set bus clock */
    I2C_Open(MASTER_I2C, 100000);

    I2C_SetSlaveAddr(MASTER_I2C, 0, EEPROM_SLAVE_ADDR, 0);   /* Slave Address : 1101011b */

    /* Get I2C1 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(MASTER_I2C));

    I2C_EnableInt(MASTER_I2C);
    NVIC_EnableIRQ(MASTER_I2C_IRQn);
	
}

void EEPROM_TEST(void)
{
	uint8_t value = 0;
	uint16_t reg = 0;	
	uint8_t array[2] = {0};
	
	uint8_t u8SlaveAddr = EEPROM_SLAVE_ADDR >>1;

	#if 1	//clear EEPROM
	printf("clear EEPROM\r\n");	
	value = 0xFF;
	for (reg = 0 ; reg < 0x100 ; reg++ )
	{
		I2Cx_WriteMultiToSlaveIRQ(u8SlaveAddr , reg , &value , 1);		
		CLK_SysTickDelay(3500);
	}

	#endif

	value = 0xF4;
	reg = 0x00;
	I2Cx_WriteMultiToSlaveIRQ(u8SlaveAddr , reg , &value , 1);
	CLK_SysTickDelay(3500);
	printf("WR : 0x%2X : 0x%2X \r\n" ,reg ,value);

	value = 0;
	I2Cx_ReadMultiFromSlaveIRQ(u8SlaveAddr , reg , &value , 1);
	printf("RD : 0x%2X : 0x%2X \r\n" ,reg ,value);
	
	value = 0x12;
	reg = 0x01;	
	I2Cx_WriteMultiToSlaveIRQ(u8SlaveAddr , reg , &value , 1);
	CLK_SysTickDelay(3500);
	printf("WR : 0x%2X : 0x%2X \r\n" ,reg ,value);

	value = 0;
	I2Cx_ReadMultiFromSlaveIRQ(u8SlaveAddr , reg , &value , 1);
	printf("RD : 0x%2X : 0x%2X \r\n" ,reg ,value);
	
	value = 0x34;
	reg = 0x02;		
	I2Cx_WriteMultiToSlaveIRQ(u8SlaveAddr , reg , &value , 1);
	CLK_SysTickDelay(3500);
	printf("WR : 0x%2X : 0x%2X \r\n" ,reg ,value);

	value = 0;
	I2Cx_ReadMultiFromSlaveIRQ(u8SlaveAddr , reg , &value , 1);
	printf("RD : 0x%2X : 0x%2X \r\n" ,reg ,value);
	
	value = 0x56;
	reg = 0x03;	
	I2Cx_WriteMultiToSlaveIRQ(u8SlaveAddr , reg , &value , 1);
	CLK_SysTickDelay(3500);
	printf("WR : 0x%2X : 0x%2X \r\n" ,reg ,value);

	value = 0;
	I2Cx_ReadMultiFromSlaveIRQ(u8SlaveAddr , reg , &value , 1);
	printf("RD : 0x%2X : 0x%2X \r\n" ,reg ,value);	

	array[1] = 0x12;
	array[0] = 0x46;	
	reg = 0x1320;	
	I2Cx_WriteMultiToSlaveIRQ(u8SlaveAddr , reg , array , 2);
	CLK_SysTickDelay(3500);
	printf("WR : 0x%2X : 0x%2X , 0x%2X \r\n" ,reg ,array[0],array[1]);

	value = 0;
	I2Cx_ReadMultiFromSlaveIRQ(u8SlaveAddr , reg , &value , 1);
	printf("RD : 0x%2X : 0x%2X \r\n" ,reg , value);
	value = 0;	
	I2Cx_ReadMultiFromSlaveIRQ(u8SlaveAddr , reg+1 , &value , 1);
	printf("RD : 0x%2X : 0x%2X \r\n" ,reg+1 ,value);

	#if 1	//dump EEPROM
	printf("dump EEPROM\r\n");	
	for (reg = 0 ; reg < 0x100 ; reg++ )
	{
		I2Cx_ReadMultiFromSlaveIRQ(u8SlaveAddr , reg , &value , 1);
		printf("0x%2X," ,value);

		if ((reg+1)%8 ==0)
        {
            printf("\r\n");
        }
	}

	#endif

	
}

void EEPROM_Process(void)
{
	uint8_t u8SlaveAddr = EEPROM_SLAVE_ADDR >>1;
	uint16_t i = 0;
	uint8_t value = 0;
	static uint8_t addr = 0;
	static uint8_t temp = 0;
	const uint8_t data1[16] = 
	{
		0x23 , 0x16 , 0x80 , 0x49 , 0x56 , 0x30 , 0x17 , 0x22 ,
		0x33 , 0x46 , 0x55 , 0x27 , 0x39 , 0x48 , 0x57 , 0x60			
	};

	if (is_flag_set(flag_Dump))
	{
		set_flag(flag_Dump , DISABLE);
		
		printf("dump EEPROM\r\n");
		for (i = 0 ; i < 0x100 ; i++ )
		{
			I2Cx_ReadMultiFromSlaveIRQ(u8SlaveAddr , i , &value , 1);
			printf("0x%2X," ,value);

			if ((i+1)%8 ==0)
	        {
	            printf("\r\n");
	        }
		}
	}

	if (is_flag_set(flag_WriteAddr))		// fix vaule , to incr address
	{
		set_flag(flag_WriteAddr , DISABLE);
	
		value = 0x01;
		I2Cx_WriteMultiToSlaveIRQ(u8SlaveAddr , addr , &value , 1);
		printf("WR : 0x%2X : 0x%2X \r\n" , addr++ , value);
	}

	if (is_flag_set(flag_WriteData))		// incr vaule , to fix address
	{
		set_flag(flag_WriteData , DISABLE);
	
		value = temp++;
		addr = 0x10;
		I2Cx_WriteMultiToSlaveIRQ(u8SlaveAddr , addr , &value , 1);
		printf("WR : 0x%2X : 0x%2X \r\n" , addr++ , value);	
	}

	if (is_flag_set(flag_WriteData1))
	{
		set_flag(flag_WriteData1 , DISABLE);

		addr = 0x40;
		for ( i = 0 ; i < 16; i++)
		{
			I2Cx_WriteMultiToSlaveIRQ(u8SlaveAddr , addr++ , (uint8_t *) &data1[i] , 1);
//			CLK_SysTickDelay(1000);
			printf("WR : 0x%2X : 0x%2X \r\n" , addr , data1[i]);	
			
		}		
	}	

	if (is_flag_set(flag_Erase))
	{
		set_flag(flag_Erase , DISABLE);
	
		printf("clear EEPROM\r\n");
		value = 0xFF;
		for (i = 0 ; i < 0x100 ; i++ )
		{
			I2Cx_WriteMultiToSlaveIRQ(u8SlaveAddr , i , &value , 1);		
		}
	}
	
}


void TMR3_IRQHandler(void)
{
//	static uint32_t LOG = 0;
	static uint16_t CNT = 0;
	
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        TIMER_ClearIntFlag(TIMER3);

		tick_counter();

		if (CNT++ >= 1000)
		{		
			CNT = 0;
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
			PB14 ^= 1;

		}

    }
}


void TIMER3_Init(void)
{
    TIMER_Open(TIMER3, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER3);
    NVIC_EnableIRQ(TMR3_IRQn);	
    TIMER_Start(TIMER3);
}


void UARTx_Process(void)
{
	uint8_t res = 0;
	
	res = UART_READ(UART0);

	printf("UARTx_Process = %c\r\n" ,res);

	if (res == 'x' || res == 'X')
	{
		NVIC_SystemReset();
	}

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case '1' :
				set_flag(flag_Dump , ENABLE);

				break;

			case '2': 
				set_flag(flag_WriteAddr , ENABLE);
			
				break;

			case '3': 
				set_flag(flag_WriteData , ENABLE);			
		
				break;			

			case '4': 
				set_flag(flag_WriteData1 , ENABLE);			
		
				break;	

			case '5': 
				set_flag(flag_WriteData2 , ENABLE);			
		
				break;	


			case '0' : 
				set_flag(flag_Erase , ENABLE);

				break;
		
			case 'Z':
			case 'z':				
				NVIC_SystemReset();
				break;				
		}
	}
}

void UART02_IRQHandler(void)
{	

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART02_IRQn);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
	
    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Waiting for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);


    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
	
    CLK_EnableModuleClock(TMR3_MODULE);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_PCLK1, 0);

	#if 1	// I2C0
    CLK_EnableModuleClock(I2C0_MODULE);
	#else	// I2C1
    CLK_EnableModuleClock(I2C1_MODULE);
	#endif

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set I2C1 multi-function pins */
	#if 1	// I2C0
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk )) |
                    (SYS_GPC_MFPL_PC0MFP_I2C0_SDA | SYS_GPC_MFPL_PC1MFP_I2C0_SCL);
	#else	// I2C1
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk )) |
                    (SYS_GPB_MFPL_PB0MFP_I2C1_SDA | SYS_GPB_MFPL_PB1MFP_I2C1_SCL);
	#endif
    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{

    SYS_Init();

    UART0_Init();

    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);

	I2Cx_Init();

	TIMER3_Init();

	EEPROM_TEST();

    /* Got no where to go, just loop forever */
    while(1)
    {
		EEPROM_Process();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

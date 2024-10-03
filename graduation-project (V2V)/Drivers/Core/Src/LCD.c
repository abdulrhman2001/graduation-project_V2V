/*
 * LCD.c
 *
 * Created: 2/23/2018 4:38:26 PM
 *  Author: Mohamed Zaghlol
 */ 
#include "LCD.h"
#include "main.h"
#include "LCD_config.h"
void LCD_vInit(void)
{
#if defined eight_bits_mode
	HAL_Delay(200);
	



	#elif defined four_bits_mode
	HAL_Delay(200);
	LCD_vSend_cmd(RETURN_HOME); //return home
	HAL_Delay(10);
	LCD_vSend_cmd(FOUR_BITS); //4bit mode
	HAL_Delay(1);
	LCD_vSend_cmd(CURSOR_ON_DISPLAN_ON);//display on cursor on
	HAL_Delay(1);
	LCD_vSend_cmd(CLR_SCREEN);//clear the screen
	HAL_Delay(10);
	LCD_vSend_cmd(ENTRY_MODE); //entry mode
	HAL_Delay(1);
	#endif
}


static void send_falling_edge(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // SET THE THE ENABLE PIN HIGH FOR 2MS
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // SET THE THE ENABLE PIN LOW FOR 2MS
	HAL_Delay(5);
}
void LCD_vSend_cmd(char cmd)
{
	#if defined eight_bits_mode

	DIO_write_port('A',cmd);
	HAL_GPIO_
	DIO_write('B',RS,0);
	send_falling_edge();
	
	#elif defined four_bits_mode
	write_high_nibble(cmd>>4);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	send_falling_edge();
	write_high_nibble(cmd);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	send_falling_edge();
	#endif
	HAL_Delay(1);
}

void LCD_vSend_char(char data)
{
#if defined eight_bits_mode

	DIO_write_port('A',data);
	HAL_GPIO_
	DIO_write('B',RS,0);
	send_falling_edge();
	
	#elif defined four_bits_mode
	    write_high_nibble(data>>4);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
		send_falling_edge();
		write_high_nibble(data);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
		send_falling_edge();
	#endif
	HAL_Delay(1);
}


void LCD_vSend_string(char *data)
{
	while((*data)!='\0')
	{
		LCD_vSend_char(*data);
		data++;
	}
}
void LCD_clearscreen()
{
	LCD_vSend_cmd(CLR_SCREEN);
	HAL_Delay(10);
}
void LCD_movecursor(char row,char coloumn)
{
	char data ;
	if(row>2||row<1||coloumn>16||coloumn<1)
	{
		data=0x80;
	}
	else if(row==1)
	{
		data=0x80+coloumn-1 ;
	}
	else if (row==2)
	{
		data=0xc0+coloumn-1;
	}
	LCD_vSend_cmd(data);
	HAL_Delay(1);
}
void write_high_nibble(unsigned char value)
{
	unsigned char *ptr=0;
	value<<=4;
	ptr = &(GPIOB->ODR);
	*ptr&=0x0f;
	*ptr|=value;
}

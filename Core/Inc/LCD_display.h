/*
 * LCD_display.h
 *
 *  Created on: Aug 23, 2023
 *      Author: Admin
 */

#ifndef INC_LCD_DISPLAY_H_
#define INC_LCD_DISPLAY_H_

#include "LCD_lib.h"
#include "main.h"
#include "stm32f4xx_hal_spi.h"

#define MAX(a, b)  (((a) > (b)) ? (a) : (b))
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))

//color define
#define LCD_WIDTH    	240
#define LCD_HEIGHT   	400

#define FONT_1206    	12
#define FONT_1608    	16
#define FONT_GB2312  	16

#define WHITE          0xFFFF
#define BLACK          0x0000
#define BLUE           0x001F
#define BRED           0XF81F
#define GRED 		   0XFFE0
#define GBLUE		   0X07FF
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0
#define BROWN 		   0XBC40
#define BRRED 		   0XFC07
#define GRAY  		   0X8430

#define LCD_DC_H()		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, 1)
#define LCD_DC_L()		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8, 0)

#define LCD_CS_H()		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, 1)
#define LCD_CS_L()		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, 0)

#define LCD_BKL_H()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1)
#define LCD_BKL_L()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0)

#define LCD_RST_H()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1)
#define LCD_RST_L()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0)

#define LCD_CMD                0
#define LCD_DATA               1

#define ST7789_DEVICE
//#define HX8347D_DEVICE

extern SPI_HandleTypeDef hspi1;

void lcd_init();

#endif /* INC_LCD_DISPLAY_H_ */

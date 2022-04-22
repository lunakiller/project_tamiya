#ifndef __SSD1306_CONF_H
#define __SSD1306_CONF_H

#define STM32F3
#define SSD1306_USE_I2C

// I2C Configuration
#define SSD1306_I2C_PORT        hi2c2
#define SSD1306_I2C_ADDR        (0x3C << 1)

// Include only needed fonts
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18
// #define SSD1306_INCLUDE_FONT_16x26

#define SSD1306_HEIGHT          32

#endif /* __SSD1306_CONF_H */

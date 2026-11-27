#include "oled.h"
#include "i2c.h"
#include "string.h"
#include "stdio.h"

u8g2_t u8g2;
uint16_t ss=0;
static void delay_us(uint32_t time)
{
	uint32_t i  =10 * time;
	while(i--);
}
/**
 * @brief OLED屏驱动接口
 * @details 实现将显示内容写入到OLED屏中
 * @param u8x8 屏幕实例对象
 * @param msg 要进行得操作
 * @param arg_int 操作参数
 * @param arg_ptr 操作参数指针
 */
	
static uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
	static uint8_t buffer[32];
	static uint8_t buf_idx;
    uint8_t *data;

    switch (msg) {
        case U8X8_MSG_BYTE_INIT:
            /* add your custom code to init i2c subsystem */
            /* MX_I2C2_Init();	I2C已经被初始化过,不需要再初始化*/
            break;

        case U8X8_MSG_BYTE_START_TRANSFER:
            buf_idx = 0;
            break;
		
        case U8X8_MSG_BYTE_SEND:
            data = (uint8_t *)arg_ptr;
            while (arg_int > 0) 
			{
                buffer[buf_idx++] = *data;
                data++;
                arg_int--;
            }
            break;

        case U8X8_MSG_BYTE_END_TRANSFER:
            if (HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, buffer, buf_idx, 0xFFFF) != HAL_OK)
			{
				ss = 1;
                return 0;
            }
            break;
			
        case U8X8_MSG_BYTE_SET_DC:
            break;
        default:
            return 0;
    }
    return 1;
}

/**
* @breif 为u8g2 库提供的延时及gpio操作接口
* @param u8x8 屏幕对象实例指针
* @param msg 要进行的操作
* @param arg_int 操作参数
* @param arg_ptr 操作参数指针
*/
static uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{

    switch (msg)
    {
    case U8X8_MSG_DELAY_100NANO: // delay arg_int * 100 nano seconds
        __NOP();
        break;
    case U8X8_MSG_DELAY_10MICRO: // delay arg_int * 10 micro seconds
        for (uint16_t n = 0; n < 320; n++)
        {
            __NOP();
        }
        break;
    case U8X8_MSG_DELAY_MILLI: // delay arg_int * 1 milli second
        HAL_Delay(1);
        break;
    case U8X8_MSG_DELAY_I2C: // arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
        delay_us(5);
        break;                    // arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
    case U8X8_MSG_GPIO_I2C_CLOCK: // arg_int=0: Output low at I2C clock pin
        break;                    // arg_int=1: Input dir with pullup high for I2C clock pin
    case U8X8_MSG_GPIO_I2C_DATA:  // arg_int=0: Output low at I2C data pin
        break;                    // arg_int=1: Input dir with pullup high for I2C data pin
    case U8X8_MSG_GPIO_MENU_SELECT:
        u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
        break;
    case U8X8_MSG_GPIO_MENU_NEXT:
        u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
        break;
    case U8X8_MSG_GPIO_MENU_PREV:
        u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
        break;
    case U8X8_MSG_GPIO_MENU_HOME:
        u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
        break;
    default:
        u8x8_SetGPIOResult(u8x8, 1); // default return value
        break;
    }
    return 1;
}

/**
  * @brief u8g2 初始化
  * @detials 完成u8g2对象初始化，相关驱动接口注册， 屏幕初始化
  * @retval None.
*/
void u8g2_init()
{
//    //U8G2_R0：默认使用U8G2_R0即可（用于配置屏幕是否要旋转）
    u8g2_Setup_ssd1306_i2c_128x32_univision_f(&u8g2, U8G2_R0, u8x8_byte_hw_i2c, u8x8_gpio_and_delay); // 初始化u8g2 结构体 ，使用硬件IIC
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_ClearBuffer(&u8g2);
}

void draw_str(uint16_t x, uint16_t y, const char *str)
{
	/* 第一步：清除缓冲区数据 */
	u8g2_ClearBuffer(&u8g2);	
	
	/* 字体模式选择 */
	u8g2_SetFontMode(&u8g2, 1); 
	
	/* 字体方向选择 */
	u8g2_SetFontDirection(&u8g2, 0); 
	
	/* 字库选择 */
	u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr); 
	
	/* 将 Hiwonder 写入缓冲区 */
	u8g2_DrawStr(&u8g2, (u8g2_uint_t)x, (u8g2_uint_t)y, str); 
	
	/* 第三步：发送数据刷新屏幕 将缓冲区数据发送oled刷新数据，刷新屏幕 */
	u8g2_SendBuffer(&u8g2); 
	
	HAL_Delay(200);
}

void u8g2_display_var(float num)
{
    char buffer[32];
    sprintf(buffer, "Num:%.2f", num);  // 格式化整数到字符串

    // 开始绘制
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);  // 设置字体

    // 显示变量的值
    u8g2_DrawStr(&u8g2, 0, 12, buffer);  // 输出字符串

    // 刷新显示
    u8g2_SendBuffer(&u8g2);
	HAL_Delay(200);
}

	

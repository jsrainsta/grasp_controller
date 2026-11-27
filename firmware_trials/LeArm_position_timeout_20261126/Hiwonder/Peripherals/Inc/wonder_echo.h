#ifndef __WONDER_ECHO_H_
#define __WONDER_ECHO_H_

#include "stdint.h"
#include "stdbool.h"

#define WONDER_ECHO_ADDRESS		0x34
#define ECHO_RESULT_REG			0x64
#define ECHO_SPEAK_REG			0x6E
#define ECHO_CMD_REG			0x00
#define ECHO_ANNOUNCER_REG		0xFF

typedef struct WonderEchoHandle WonderEchoHandleTypeDef;
struct WonderEchoHandle
{
	uint16_t dev_addr;
	uint8_t result;
	
	uint8_t transmit_status;
	uint8_t receive_status;

	uint8_t (*write_data)(WonderEchoHandleTypeDef* self, uint8_t* pdata, uint16_t size);
	uint8_t (*read_data)(WonderEchoHandleTypeDef* self, uint8_t* pdata, uint16_t size);
};

/**
 * @brief wonder echo初始化
 * 
 */
void wonder_echo_init(void);

/**
 * @brief wonder echo 识别到的结果
 * 
 * @return int 识别结果
 */
int echo_recognition(void);

/**
 * @brief wonder echo语音播报
 * 
 * @param  reg-功能寄存器寄存器
 * @param  speak_id-播报的id号
 * @return bool 
 */
bool echo_speak(uint8_t reg, uint8_t speak_id);

#endif

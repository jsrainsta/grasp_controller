#ifndef __WONDER_MV_H_
#define __WONDER_MV_H_

#include "stdint.h"
#include "stdbool.h"

#define WONDERMV_ADDR  0x32
#define COLOR_REG  	   0x00
//#define COLOR_REG  	   0x01
#define FACE_REG  	   0x10
#define TAG_REG  	   0x20
#define OBJECT_REG     0x30

typedef struct
{
	uint16_t w;
	uint16_t h;
	uint16_t x;
	uint16_t y;
}PositionObjectTypeDef;

typedef struct
{
	uint8_t id;
	PositionObjectTypeDef position;
}RecognitionHanleTypeDef;

typedef struct WonderMVHandle WonderMVHandleTypeDef;
struct WonderMVHandle
{
	uint16_t dev_addr;
	
	uint8_t transmit_status;
	uint8_t receive_status;
	
	uint8_t results[9];
	
	uint8_t (*write_data)(WonderMVHandleTypeDef* self, uint8_t* pdata, uint16_t size);
	uint8_t (*read_data)(WonderMVHandleTypeDef* self, uint8_t* pdata, uint16_t size);
	
};

/**
 * @brief wonder mv接口初始化
 */
void wonder_mv_init(void);

/**
 * @brief 颜色识别
 * 
 * @param  color 指向RecognitionHanleTypeDef类型的指针
 * @return true 
 * @return false 
 */
bool wonder_mv_color_recognition(RecognitionHanleTypeDef* color);

/**
 * @brief 人脸识别
 * 
 * @param  face
 * @return true 
 * @return false 
 */
bool wonder_mv_face_detection(RecognitionHanleTypeDef* face);

/**
 * @brief 标签识别
 * 
 * @param  tag
 * @return true 
 * @return false 
 */
bool wonder_mv_tag_detection(RecognitionHanleTypeDef* tag);

/**
 * @brief 物体识别
 * 
 * @param  obj
 * @return true 
 * @return false 
 */
bool wonder_mv_object_detection(RecognitionHanleTypeDef* obj);
#endif

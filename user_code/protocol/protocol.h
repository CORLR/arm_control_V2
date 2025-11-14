#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "main.h" //

void Protocol_Parse_Chunk(uint8_t* chunk, uint16_t len);
void process_encoder_data(uint8_t* pData, uint32_t len);

#endif 
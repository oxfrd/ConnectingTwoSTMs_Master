/*
 * RingBuffer.h
 *	Simple ring buffer with one empty space before tail
 *	Remember about safe multiple access in RTOS!
 *  Created on: 18 sie 2022
 *      Author: oxford
 */

#ifndef INC_RINGBUFFER_H_
#define INC_RINGBUFFER_H_

#include <stdint.h>

#define RINGBUFF_SIZE 17

typedef struct {
	volatile uint8_t buff[RINGBUFF_SIZE];
	uint8_t	head;
	uint8_t tail;
}RingBuffer_t;

uint8_t RingBuffWrite(RingBuffer_t *buffer, uint8_t *data, uint8_t size);
uint8_t RingBuffReadSome(RingBuffer_t *buffer, uint8_t *data, uint8_t size);
uint8_t RingBuffReadWhole(RingBuffer_t *buffer, uint8_t *data, uint8_t size);
void FlushBuff(RingBuffer_t *buffer);

#endif /* INC_RINGBUFFER_H_ */

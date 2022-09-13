/*
 * RingBuffer.c
 *
 *  Created on: 18 sie 2022
 *      Author: oxford
 */
#include "RingBuffer.h"

uint8_t RingBuffWrite(RingBuffer_t *buffer, uint8_t *data, uint8_t size)
{
	for(int i=0 ; i < size ; i++ )
	{
		uint8_t next = ((buffer->head)+1) % RINGBUFF_SIZE;

		if(next == buffer->tail)
		{
			return 1;
		} else
		{
			buffer->buff[buffer->head] = data[i];
			buffer->head = next;
		}
	}
	return 0;
}

uint8_t RingBuffReadSome(RingBuffer_t *buffer, uint8_t *data, uint8_t size)
{
	for(int i=0 ; i < size ; i++ )
	{
		uint8_t next = ((buffer->tail)+1) % RINGBUFF_SIZE;

		data[i] = buffer->buff[buffer->tail];
		buffer->tail = next;

		if(next == buffer->head)
		{
		 	return 0;
		}
	}
	return 0;
}
/*
 * RingBuffReadWhole returns:
 * 0 - works good!
 * 2 - undefined behaviour
 * 3 - too small array passed
 * 10 - empty ring buffer
 */
uint8_t RingBuffReadWhole(RingBuffer_t *buffer, uint8_t *data, uint8_t size)
{
	uint8_t dif;
	if((buffer->head) == (buffer->tail))
	{
		return 10;	//nothing to read
	}
	else if((buffer->head) > (buffer->tail))
	{
		dif = (buffer->head) - (buffer->tail);
	}
	else if((buffer->head) < (buffer->tail))
	{
		dif = RINGBUFF_SIZE + (buffer->tail) - (buffer->head);
	}else
	{
		return 2;	//undefined behaviour
	}

	if(dif>size)
		return 3; //too small array passed

	for(int i=0 ; i < dif ; i++ )
	{
		uint8_t next = ((buffer->tail)+1) % RINGBUFF_SIZE;

		data[i] = buffer->buff[buffer->tail];
		buffer->tail = next;

		if(next == buffer->head)
		{
		 	return 0;
		}
	}
	return 0;
}


void FlushBuff(RingBuffer_t *buffer)
{
	buffer->head = 0;
	buffer->tail = 0;
}

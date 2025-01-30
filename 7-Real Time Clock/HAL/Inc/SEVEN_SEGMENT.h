/*
 * SEVEN_SEGMENT.h
 *
 * Created: 6/30/2024 10:02:10 PM
 *  Author: Ahmed Ragab
 */ 


#ifndef SEVEN_SEGMENT_H_
#define SEVEN_SEGMENT_H_
#include "DIO.h"
#include "macro_function.h"

#define CC
#define seven_seg_port 'B'

void seven_seg_init_port(void);

void seven_seg_write_port(unsigned char number);

void seven_seg_init_low_nibble(void);

void seven_seg_init_high_nibble(void);

void seven_seg_write_low_nibble(unsigned char number);

void seven_seg_write_high_nibble(unsigned char number);

void seven_seg_init_nibble(unsigned char pin_start);

void seven_seg_write_nibble(unsigned char pin_start,unsigned char number);



#endif /* SEVEN_SEGMENT_H_ */
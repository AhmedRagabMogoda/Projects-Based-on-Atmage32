/*
 * I2C.h
 *
 * Created: 9/1/2024 3:01:01 PM
 *  Author: Ahmed Ragab
 */ 


#ifndef I2C_H_
#define I2C_H_

void I2C_master_init(unsigned long scl_clock);

void I2C_set_address(unsigned char address);

void I2C_start(void);

void I2C_repeated_start(void);

void I2C_stop(void);

void I2C_master_send_address_transmitter_mode(unsigned char address);

void I2C_master_send_address_receiver_mode(unsigned char address);

void I2C_master_transmit_data(char data);

char I2C_master_receive_data(void);

char I2C_master_receive_data_with_not_ACK(void);

void I2C_slave_transmit_data(char data);

char I2C_slave_receive_data(void);

#endif /* I2C_H_ */
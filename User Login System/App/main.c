/*
 * user_login_system.c
 *
 * Created: 9/2/2024 2:20:56 AM
 * Author : Ahmed Ragab
 */ 

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include "LCD.h"
#include "I2C.h"
#include "UART.h"

#define EEPROM_default_value 0xff
#define EEPROM_status_address 0x00
#define EEPROM_first_address_of_password 0x21
#define EEPROM_scond_address_of_password 0x22
#define EEPROM_thrid_address_of_password 0x23
#define EEPROM_fourth_address_of_password 0x24
#define EEPROM_first_address_of_name 0x25
#define EEPROM_scond_address_of_name 0x26
#define EEPROM_thrid_address_of_name 0x27
#define EEPROM_fourth_address_of_name 0x28
#define EEPROM_five_address_of_name 0x29

int main(void)
{
	unsigned char count;
	char value, val_1;
	char name[5];
	char check_name[5];
	char pass[4];
	char check_pass[4];

	LCD_init();
	UART_init(9600);
	I2C_master_init(50000);
	_delay_ms(300);

	// Check if EEPROM is in default state
	I2C_start();
	I2C_master_send_address_transmitter_mode(0b1010000); // EEPROM Write Mode
	I2C_master_transmit_data(EEPROM_status_address); // Send status address
	I2C_repeated_start(); // Repeated start
	I2C_master_send_address_receiver_mode(0b1010000); // EEPROM Read Mode
	value = I2C_master_receive_data_with_not_ACK(); // Read value from EEPROM
	I2C_stop();

	if (EEPROM_default_value == value)
	{
		LCD_send_string("   Create name");
		_delay_ms(1000);
		LCD_clr_screen();
		LCD_send_string("user name:");

		for (count = 0; count < 5; count++)
		{
			LCD_move_cursor(1, 11 + count);
			val_1 = UART_receive_data();
			LCD_send_data(val_1);
			name[count] = val_1;
			
			// Write the name to EEPROM
			I2C_start();
			I2C_master_send_address_transmitter_mode(0b1010000);
			I2C_master_transmit_data(EEPROM_first_address_of_name + count);
			I2C_master_transmit_data(val_1);
			I2C_stop();
		}
		val_1 = UART_receive_data(); //next line in pc (enter)
		_delay_ms(500);
		LCD_clr_screen();
		LCD_send_string("password:");

		for (count = 0; count < 4; count++)
		{
			val_1 = UART_receive_data();
			LCD_send_data(val_1);
			_delay_ms(500);
			LCD_move_cursor(1, 10 + count);
			LCD_send_data('*');
			pass[count] = val_1;
			
			// Write the password to EEPROM
			I2C_start();
			I2C_master_send_address_transmitter_mode(0b1010000);
			I2C_master_transmit_data(EEPROM_first_address_of_password + count);
			I2C_master_transmit_data(val_1);
			I2C_stop();
		}

		_delay_ms(500);
		// Mark EEPROM as initialized
		I2C_start();
		I2C_master_send_address_transmitter_mode(0b1010000);
		I2C_master_transmit_data(EEPROM_status_address);
		I2C_master_transmit_data(0);
		I2C_stop();
	}
	val_1 = UART_receive_data(); //next line in pc (enter)
	_delay_ms(500);
	LCD_clr_screen();
	LCD_send_string("user login ");
	_delay_ms(1000);
	LCD_clr_screen();
	LCD_send_string("user name:");

	for (count = 0; count < 5; count++)
	{
		LCD_move_cursor(1, 11 + count);
		val_1 = UART_receive_data();
		LCD_send_data(val_1);
		check_name[count] = val_1;
		// Read stored name from EEPROM for validation
		I2C_start();
		I2C_master_send_address_transmitter_mode(0b1010000);
		I2C_master_transmit_data(EEPROM_first_address_of_name + count);
		I2C_repeated_start();
		I2C_master_send_address_receiver_mode(0b1010000);
		name[count] = I2C_master_receive_data_with_not_ACK(); // Read the stored name
		I2C_stop();
	}
	_delay_ms(500);
	if (name[0] != check_name[0] || name[1] != check_name[1] || name[2] != check_name[2] || name[3] != check_name[3] || name[4] != check_name[4]) // Validate entered name
	{
		// Handle name mismatch case
		LCD_clr_screen();
		LCD_send_string("user not exit!");
		while(1); // Halt in error
	}
	val_1 = UART_receive_data(); //next line in pc (enter)
	_delay_ms(500);
	LCD_clr_screen();
	LCD_send_string("password:");

	for (count = 0; count < 4; count++)
	{
		val_1 = UART_receive_data();
		LCD_send_data(val_1);
		_delay_ms(500);
		LCD_move_cursor(1, 10 + count);
		LCD_send_data('*');
		check_pass[count] = val_1;
		// Read stored password from EEPROM for validation
		I2C_start();
		I2C_master_send_address_transmitter_mode(0b1010000);
		I2C_master_transmit_data(EEPROM_first_address_of_password + count);
		I2C_repeated_start();
		I2C_master_send_address_receiver_mode(0b1010000);
		pass[count] = I2C_master_receive_data_with_not_ACK(); // Read the stored password
		I2C_stop();
	}
	_delay_ms(500);
	if (pass[0] != check_pass[0] || pass[1] != check_pass[1] || pass[2] != check_pass[2] || pass[3] != check_pass[3]) // Validate entered password
	{
		// Handle password mismatch case
		LCD_clr_screen();
		LCD_send_string("Wrong Password!");
		while(1); // Halt in error
	}
	val_1 = UART_receive_data(); //next line in pc (enter)
	// If all validations pass, proceed to next steps
	LCD_clr_screen();
	LCD_send_string("Login Successful!");
	_delay_ms(1000);
	LCD_move_cursor(2,1);
	LCD_send_string("press 1 to set");
	val_1 = UART_receive_data();
	if ('1'==val_1)
	{
		val_1 = UART_receive_data(); //next line in pc (enter)
		LCD_clr_screen();
		LCD_send_string("1-change pass");
		LCD_move_cursor(2,1);
		LCD_send_string("2-change name");
		val_1 = UART_receive_data();
		if ('1'==val_1)
		{
			val_1 = UART_receive_data(); //next line in pc (enter)
			_delay_ms(500);
			LCD_clr_screen();
			LCD_send_string("password:");

			for (count = 0; count < 4; count++)
			{
				val_1 = UART_receive_data();
				LCD_send_data(val_1);
				_delay_ms(500);
				LCD_move_cursor(1, 10 + count);
				LCD_send_data('*');
				pass[count] = val_1;
				
				// Write the password to EEPROM
				I2C_start();
				I2C_master_send_address_transmitter_mode(0b1010000);
				I2C_master_transmit_data(EEPROM_first_address_of_password + count);
				I2C_master_transmit_data(val_1);
				I2C_stop();
			}
			_delay_ms(500);
			LCD_clr_screen();
			LCD_send_string("password changed");
			_delay_ms(1000);
			LCD_clr_screen();
			LCD_send_string("    welcome");
		}
		else if ('2'==val_1)
		{
			val_1 = UART_receive_data(); //next line in pc (enter)
			_delay_ms(500);
			LCD_clr_screen();
			LCD_send_string(" to change name");
			_delay_ms(1000);
			LCD_clr_screen();
			LCD_send_string("new name:");

			for (count = 0; count < 5; count++)
			{
				LCD_move_cursor(1, 11 + count);
				val_1 = UART_receive_data();
				LCD_send_data(val_1);
				name[count] = val_1;
				
				// Write the name to EEPROM
				I2C_start();
				I2C_master_send_address_transmitter_mode(0b1010000);
				I2C_master_transmit_data(EEPROM_first_address_of_name + count);
				I2C_master_transmit_data(val_1);
				I2C_stop();
			}
			_delay_ms(500);
			LCD_clr_screen();
			LCD_send_string("name changed");
			_delay_ms(1000);
			LCD_clr_screen();
			LCD_send_string("    welcome");
		}
	}
}
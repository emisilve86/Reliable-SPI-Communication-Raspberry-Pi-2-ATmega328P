/**********************************************************************************
 * Copyright (C) 2023  Emiliano Silvestri                                         *
 *                                                                                *
 * This program is free software; you can redistribute it and/or                  *
 * modify it under the terms of the GNU General Public License                    *
 * as published by the Free Software Foundation; either version 2                 *
 * of the License, or (at your option) any later version.                         *
 *                                                                                *
 * This program is distributed in the hope that it will be useful,                *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of                 *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                  *
 * GNU General Public License for more details.                                   *
 *                                                                                *
 * You should have received a copy of the GNU General Public License              *
 * along with this program; if not, write to the Free Software                    *
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA. *
 **********************************************************************************/

#include <avr_common.h>
#include <int_handling.h>
#include <spi_transaction.h>

#define EMPTY_VALUE	0x00
#define ACK_VALUE	0xFF

#define WAIT_DATA	0x00
#define WAIT_ECHO	0x0F
#define WAIT_ACK	0xF0

static volatile char ack;
static volatile char data;
static volatile char stage;

void spi_send_data(char data)
{
	SPDR = data;

	int_trigger();
}

char spi_recv_data(void)
{
	return SPDR;
}

void spi_init(void)
{
	ack = EMPTY_VALUE;
	data = EMPTY_VALUE;
	stage = WAIT_DATA;

	SPDR = EMPTY_VALUE;

	DDRB = (1 << DDB4);
	SPCR = (1 << SPE) | (1 << SPIE);
}

ISR(SPI_STC_vect)
{
	switch (stage)
	{
		case WAIT_DATA:
			data = spi_recv_data();
			stage = WAIT_ECHO;
			spi_send_data(data);
			break;

		case WAIT_ECHO:
			stage = WAIT_ACK;
			break;

		case WAIT_ACK:
			ack = spi_recv_data();
			if (ack == ACK_VALUE) {
				// "data" is good!
				ack = EMPTY_VALUE;
			} else {
				// "data" is not good!
				data = EMPTY_VALUE;
			}
			stage = WAIT_DATA;
			break;
	}
}

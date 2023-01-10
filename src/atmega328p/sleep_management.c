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
#include <sleep_management.h>

void sleep_until_int(void)
{
	cli();
	sleep_enable();
	sei();
	sleep_cpu();
	sleep_disable();
}

void sleep_init(void)
{
	set_sleep_mode(SLEEP_MODE_IDLE);
}

/*
 * cyttsp4_input_arbiter.h
 * Cypress TrueTouch(TM) Standard Product V4 CapSense input arbiter.
 * For use with any Cypress Txx4xx parts.
 *
 * Copyright (C) 2014 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 *
 */

#include <linux/input.h>

extern int input_arbiter_init(void);
extern void input_arbiter_release(void);
extern void input_arbiter_report_key_event(struct input_dev *input, unsigned int code, int state);
extern void input_arbiter_report_touch_event(struct input_dev *input, int code);




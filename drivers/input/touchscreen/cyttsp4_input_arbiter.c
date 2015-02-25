/*
 * cyttsp4_input_arbiter.c
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include "cyttsp4_input_arbiter.h"
#include <linux/workqueue.h>
#include <linux/mutex.h>

#define UNUSED(x)       ( (void)(x) )

#define KEY_STATE_PRESSED  1
#define KEY_STATE_RELEASED 0

/* long press delay in milliseconds. we can monitor user actions this long  */
#define KEY_EVENT_HOLDOFF_TIME 700


struct key_event_t {
	struct input_dev *input;
	unsigned int code;
	int state;
	unsigned long timestamp;
	bool isKeyUpAllowed;
};

/* curently not used */
struct mt_event_t {
	unsigned long timestamp;
	int code;
};

struct arbiter_state_t {
	struct mutex lock;
	unsigned long event_holdoff_time;
	struct key_event_t last_key_event;
	struct mt_event_t last_mt_event;
	struct delayed_work key_event_work;
};

static struct arbiter_state_t* pState = NULL;

static void report_input_event_key(struct input_dev *input, unsigned int code, int state)
{
	pr_debug("REPORT INPUT KEY: code=%d state=%s\n", code, state==KEY_STATE_PRESSED
		? "PRESSED" : "RELEASED");
	input_report_key(input, code, state);
	input_sync(input);
}

static void delay_key_event_wf(struct work_struct *work)
{
	struct arbiter_state_t *pState = container_of((struct delayed_work *)work,
											struct arbiter_state_t, key_event_work);
	mutex_lock(&pState->lock);
	pr_debug("TIMER EXPIRED\n");
	pState->last_key_event.isKeyUpAllowed = true;
	report_input_event_key(pState->last_key_event.input, pState->last_key_event.code, pState->last_key_event.state);
	mutex_unlock(&pState->lock);
}


int input_arbiter_init(void)
{
	pState = kzalloc(sizeof(struct arbiter_state_t), GFP_KERNEL);
	if(pState == NULL)
		return -1;

	pState->event_holdoff_time = msecs_to_jiffies(KEY_EVENT_HOLDOFF_TIME);

	mutex_init(&pState->lock);
	INIT_DELAYED_WORK(&pState->key_event_work, delay_key_event_wf);
	pr_info("Key press detection delay: %d msec %lu jiffies\n", KEY_EVENT_HOLDOFF_TIME, pState->event_holdoff_time);
	return 0;
}

void input_arbiter_release(void)
{
	if(pState != NULL) {
		cancel_delayed_work_sync(&pState->key_event_work);
		mutex_destroy(&pState->lock);
		kfree(pState);
		pState = NULL;
	}
}



static void fill_last_key_event(struct input_dev *input, unsigned int code, int state, bool isAlowed)
{
	pState->last_key_event.code = code;
	pState->last_key_event.state = state;
	pState->last_key_event.input = input;
	pState->last_key_event.isKeyUpAllowed = isAlowed;
	pState->last_key_event.timestamp = jiffies;
}

void input_arbiter_report_key_event(struct input_dev *input, unsigned int code, int state)
{
	if(state == KEY_STATE_PRESSED)
	{
		if(pState->last_key_event.state == KEY_STATE_PRESSED)
			return;
	
		fill_last_key_event(input, code, state, false);
		schedule_delayed_work(&pState->key_event_work,pState->event_holdoff_time);
	}
	else
	{
		if(pState->last_key_event.code != code)
			return;

		/* means we got key press before but timer has not expired yet */
		if(delayed_work_pending(&pState->key_event_work))
		{
			if(!cancel_delayed_work_sync(&pState->key_event_work))
			{
				/* work has started execution. means key press has been sent already */
				mutex_lock(&pState->lock);
				/* report key UP here. KEY DOWN has been reported by work fn */
				if(pState->last_key_event.isKeyUpAllowed) {
					report_input_event_key(input, code, state);
				}
				/* update last event */
				fill_last_key_event(input, code, state, false);
				mutex_unlock(&pState->lock);
			}
			/* work has not been execured and has not been canceled. 
			  *  means we have not got touch event so treat this as short press 
			  */
			else
			{
				report_input_event_key(input, code, KEY_STATE_PRESSED);
				report_input_event_key(input, code, KEY_STATE_RELEASED);
				/* can be done without locking. no acces from different thread here */
				fill_last_key_event(input, code, state, false);
			}
		}
		/* work has expired or on execution */
		else
		{
			mutex_lock(&pState->lock);
			/* report key UP here. KEY DOWN has been reported by work fn */
			if(pState->last_key_event.isKeyUpAllowed) {
				report_input_event_key(input, code, state);
			}
			/* update last event */
			fill_last_key_event(input, code, state, false);
			mutex_unlock(&pState->lock);
		}
	}
}


void input_arbiter_report_touch_event(struct input_dev *input, int code)
{
	/* any touch event has more prioroty then key events so cancel all key events */
	/* we have pending work means we got KEY DOWN recently. try to cancel */
	pr_debug("TOUCH\n");
	if(delayed_work_pending(&pState->key_event_work))
	{
		cancel_delayed_work_sync(&pState->key_event_work);
	}
}


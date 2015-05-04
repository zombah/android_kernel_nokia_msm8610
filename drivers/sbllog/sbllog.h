 /* *  *  linux/drivers/sbl/sbllog.h *  *
  *       Copyright (C) 2013 NOKIA Inc,
  *
  *
  *
  *      This program is free software; you can redistribute it and/or modify
  *      it under the terms of the GNU General Public License version 2 as
  *      published by the Free Software Foundation.
  *
  *
  *
  *      Author: Yulin Ren (yulin.ren@nokia.com)
  *
  */

#ifndef _SBL_LOG_H_
#define _SBL_LOG_H_

#ifndef SBL_LOG_MAJOR
#define SBL_LOG_MAJOR 0
#endif




struct sbl_dev
{
 	char *data;
	unsigned long size;
};

#endif



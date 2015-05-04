/* Copyright (c) 2014, Nokia Corporation and/or its subsidiary(-ies). All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


/****************************************************************************
* INCLUDE HEADER FILES                                                      *
****************************************************************************/
#include <linux/module.h>


// APR includes
#include <mach/qdsp6v2/apr.h>

/****************************************************************************
* DEFINITIONS                                                               *
****************************************************************************/

#define VOICE_ONLY  //Define to support ADSP voice services only


/****************************************************************************
* LOCALS                                                                    *
****************************************************************************/

struct atts_t {
  void *afe_apr;
  void *vpm_apr;
  void *vsm_apr;
#ifndef VOICE_ONLY
  void *adm_apr;
  void *asm_apr;
#endif
  int device_open;
  struct mutex lock;
};

static struct atts_t atts;

static int atts_apr_register(apr_fn *callback_tbl, uint16_t portId);
static int atts_apr_deregister(void);


/****************************************************************************
* FUNCTIONS                                                                 *
****************************************************************************/
static int __init atts_apr_module_init(void)
{
  struct atts_t *atts_ptr = &atts;

  pr_debug("ATTS_APR atts_apr_module_init\n");

  atts_ptr->device_open = 0;
  atts_ptr->afe_apr = NULL;
  atts_ptr->vsm_apr = NULL;
  atts_ptr->vpm_apr = NULL;
#ifndef VOICE_ONLY
  atts_ptr->adm_apr = NULL;
  atts_ptr->asm_apr = NULL;
#endif
  mutex_init(&atts_ptr->lock);

  pr_debug("ATTS_APR atts_apr_module_init: device created successfully\n");

  return 0;
}

int atts_apr_init(apr_fn* callback_tbl, uint16_t portId)
{
  struct atts_t *atts_ptr = &atts;

  pr_debug("ATTS_APR atts_apr_init\n");

  mutex_lock(&atts_ptr->lock);
  // Open only once
  if(atts_ptr->device_open){
    mutex_unlock(&atts_ptr->lock);
    return -EBUSY;
  }

  atts_ptr->device_open++;

  atts_apr_register(callback_tbl, portId);

  mutex_unlock(&atts_ptr->lock);
  return 0;

}

int atts_apr_deinit(void)
{
  struct atts_t *atts_ptr = &atts;

  mutex_lock(&atts_ptr->lock);
  atts_ptr->device_open--;

  pr_debug("ATTS_APR atts_apr_deinit: device_open = %d\n", atts_ptr->device_open);

  if(atts_apr_deregister() < 0){
    pr_err("ATTS_APR: atts_apr_deregister error\n");
  }

  mutex_unlock(&atts_ptr->lock);
  return 0;
}

// Deregisters from APR
static int atts_apr_deregister(void)
{
  int ret = -1;
  struct atts_t *atts_ptr = &atts;

  pr_debug("ATTS_APR atts_apr_deregister\n");

  if(atts_ptr->afe_apr != NULL){
    ret = apr_deregister(atts_ptr->afe_apr);
    atts_ptr->afe_apr = NULL;
  }
  if(atts_ptr->vsm_apr != NULL){
    ret = apr_deregister(atts_ptr->vsm_apr);
    atts_ptr->vsm_apr = NULL;
  }
  if(atts_ptr->vpm_apr != NULL){
    ret = apr_deregister(atts_ptr->vpm_apr);
    atts_ptr->vpm_apr = NULL;
  }
#ifndef VOICE_ONLY
  if(atts_ptr->adm_apr != NULL){
    ret = apr_deregister(atts_ptr->adm_apr);
    atts_ptr->adm_apr = NULL:
  }
  if(atts_ptr->asm_apr != NULL){
    ret = apr_deregister(atts_ptr->asm_apr);
    atts_ptr->asm_apr = NULL;
  }
#endif
  return ret;
}

// Register to ADSP services with portId (where applicable) and the callbacks from the table (5 entries needed).
// Different callbacks used for differentiating the services.
static int atts_apr_register(apr_fn *callback_tbl, uint16_t portId)
{
  struct atts_t *atts_ptr = &atts;

  pr_debug("ATTS_APR atts_apr_register\n");
  if(atts_ptr->afe_apr == NULL){
    atts_ptr->afe_apr = apr_register("ADSP", "AFE", callback_tbl[0], portId, &atts);
  }
  if(atts_ptr->afe_apr == NULL){
    pr_err("ATTS_APR atts_apr_register: error registering to AFE apr\n");
    return -ENODEV;
  }
  if(atts_ptr->vsm_apr == NULL){
    atts_ptr->vsm_apr = apr_register("ADSP", "VSM", callback_tbl[1], 0xFFFFFFFF, &atts);
  }
  if(atts_ptr->vsm_apr == NULL){
    pr_err("ATTS_APR atts_apr_register: error registering to VSM apr\n");
    return -ENODEV;
  }
  if(atts_ptr->vpm_apr == NULL){
    atts_ptr->vpm_apr = apr_register("ADSP", "VPM", callback_tbl[2], 0xFFFFFFFF, &atts);
  }
  if(atts_ptr->vpm_apr == NULL){
    pr_err("ATTS_APR atts_apr_register: error registering to VSM apr\n");
    return -ENODEV;
  }
#ifndef VOICE_ONLY
  if(atts_ptr->adm_apr == NULL){
    atts_ptr->adm_apr = apr_register("ADSP", "ADM", callback_tbl[3], 9, &atts);
  }
  if(atts_ptr->adm_apr == NULL){
    pr_err("ATTS_APR atts_apr_register: error registering to ADM apr\n");
    return -ENODEV;
  }
  if(atts_ptr->asm_apr == NULL){
    atts_ptr->asm_apr = apr_register("ADSP", "ASM", callback_tbl[4], portId, &atts);
  }
  if(atts_ptr->asm_apr == NULL){
    pr_debug("ATTS_APR atts_apr_register: error registering to ASM apr\n");
    return -ENODEV;
  }
#endif
  return 0;
}

// Sends an APR packet
int32_t apr_async_send(void* packet)
{
  struct atts_t *atts_ptr = &atts;
  struct apr_hdr *header;
  int32_t ret = 0;

  header = (struct apr_hdr*)packet;
  pr_debug("ATTS_APR apr_async_send: packet size %d \n",header->pkt_size);

  switch(header->dest_svc){
  case APR_SVC_AFE:
    ret = apr_send_pkt(atts_ptr->afe_apr,packet);
    break;
  case APR_SVC_VSM:
    ret = apr_send_pkt(atts_ptr->vsm_apr,packet);
    break;
  case APR_SVC_VPM:
    ret = apr_send_pkt(atts_ptr->vpm_apr,packet);
    break;
#ifndef VOICE_ONLY
  case APR_SVC_ASM:
    ret = apr_send_pkt(atts_ptr->asm_apr,packet);
    break;
  case APR_SVC_ADM:
    ret = apr_send_pkt(atts_ptr->adm_apr,packet);
    break;
#endif
  default:
    pr_err("ATTS_APR apr_async_send: unknown dest_svc %d \n",header->dest_svc);
    break;
  }

  return ret;
}

late_initcall(atts_apr_module_init);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ATTS APR interface");
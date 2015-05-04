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
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <asm/uaccess.h>

// APR includes
#include <sound/apr_audio-v2.h>
#include <mach/qdsp6v2/apr.h>

// ATTS
#include <linux/atts.h>
#include "atts_private.h"

/****************************************************************************
* DEFINITIONS                                                               *
****************************************************************************/

#define TIMEOUT_MS 1500

extern int32_t apr_async_send(void* packet);
extern int atts_apr_init(apr_fn *callback_tbl, uint16_t portId);
extern int atts_apr_deinit(void);

#define DEVICE_NAME "atts"

/****************************************************************************
* LOCALS                                                                    *
****************************************************************************/

struct atts {
  int device_open;
  struct mutex lock;
  struct mutex lock_ioctl;
  wait_queue_head_t wait;
  atomic_t state;
  apr_pkt_resp *aprResponseUser;
  apr_pkt_resp *aprResponseTemp;
  atts_ioctl_req_hdr req_hdr;
};

static struct atts this_atts;

static apr_fn apr_callback_table[5] = {
  atts_afe_callback,
  atts_vsm_callback,
  atts_vpm_callback,
  NULL, //ADM not supported yet
  NULL  //ASM not supported yet
};

static const  uint8_t testclient_afe_domain = APR_DOMAIN_ADSP;
static const  uint8_t testclient_afe_svc = APR_SVC_AFE;
static const  uint8_t testclient_adm_domain = APR_DOMAIN_ADSP;
static const  uint8_t testclient_adm_svc = APR_SVC_ADM;
static const  uint8_t testclient_vpm_domain = APR_DOMAIN_ADSP;
static const  uint8_t testclient_vpm_svc = APR_SVC_VPM;
static const  uint8_t testclient_vsm_domain = APR_DOMAIN_ADSP;
static const  uint8_t testclient_vsm_svc = APR_SVC_VSM;

static const uint16_t attsPortId = 0x030D;

struct file_operations atts_fops = {
  .owner           = THIS_MODULE,
  .open            = atts_open,
  .release         = atts_release,
  .unlocked_ioctl  = atts_ioctl,
};

static struct miscdevice atts_dev = {
  MISC_DYNAMIC_MINOR,
  DEVICE_NAME,
  &atts_fops
};


/****************************************************************************
* FUNCTIONS                                                                 *
****************************************************************************/

static long atts_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
  int ret = -1;
  struct atts *atts_ptr = &this_atts;
  uint16_t errorCode  = ATTS_ERRCODE_OK;
  atts_ioctl_req_hdr* ioctlReqHdr = &atts_ptr->req_hdr;
  uint32_t moduleId = 0;
  uint32_t paramId = 0;
  uint16_t portId  = 0;
  uint8_t* payload = 0;
  uint32_t payload_length = 0;
  uint16_t tuneSize = 0;
  atts_ioctl_req* request = NULL;
  atts_ioctl_data *ioctl_req = (atts_ioctl_data*)arg;

  ret = mutex_trylock(&atts_ptr->lock_ioctl);
  if(ret == 0){
     pr_err("ATTS atts_ioctl: unable to lock lock_ioctl mutex");
     return -EBUSY;
  }

  pr_debug("ATTS atts_ioctl>\n");

  if(arg == 0){
    pr_err( "ATTS atts_ioctl: arg == NULL\n");
    errorCode = -EINVAL;
    goto exit_ioctl;
  }

  atts_ptr->aprResponseUser = NULL;

  get_user(request, &ioctl_req->request);
  get_user(atts_ptr->aprResponseUser, &ioctl_req->response);

  if(request == NULL){
    pr_err( "ATTS atts_ioctl: request == NULL\n");
    errorCode = -EINVAL;
    goto exit_ioctl;

  }
  if(atts_ptr->aprResponseUser == NULL){
    pr_err( "ATTS atts_ioctl: reponse == NULL\n");
    errorCode = -EINVAL;
    goto exit_ioctl;
  }

  ret = copy_from_user((void*)ioctlReqHdr, (void*)request, sizeof(atts_ioctl_req_hdr));

  if(ret){
    pr_err("ATTS atts_ioctl: ioctlReqHdr copy fail %d\n",ret);
    errorCode = -EINVAL;
    goto exit_ioctl;
  }

  switch(cmd){
    case IOCTL_CMD_GET_DEVICE_INFO_VERSION:
      pr_debug("ATTS atts_ioctl: >GET_DEVICE_INFO");

      moduleId = ioctlReqHdr->module_id;
      paramId = ioctlReqHdr->param_block_id;

      ret = GetAlgorithmVersion(moduleId, paramId);

      ret = wait_event_timeout(this_atts.wait,(atomic_read(&this_atts.state) == 0), msecs_to_jiffies(TIMEOUT_MS));
      if (!ret) {
        pr_err("ATTS atts_ioctl: wait_event timeout\n");
        errorCode = ATTS_ERRCODE_TIMEOUT;
        break;
      }

      atts_ptr->aprResponseTemp->hdr.aprErrCode = ParseAprResp(atts_ptr->aprResponseTemp);

      pr_debug("ATTS atts_ioctl: <GET_DEVICE_INFO");
      break;
    case IOCTL_CMD_GET_ALGORITHM_VERSION:
      pr_debug("ATTS atts_ioctl: >GET_ALGORITHM_VERSION");

      moduleId = ioctlReqHdr->module_id;
      paramId = ioctlReqHdr->param_block_id;

      ret = GetAlgorithmVersion(moduleId, paramId);

      ret = wait_event_timeout(this_atts.wait,(atomic_read(&this_atts.state) == 0), msecs_to_jiffies(TIMEOUT_MS));
      if (!ret) {
        pr_err("ATTS atts_ioctl: wait_event timeout\n");
        errorCode = ATTS_ERRCODE_TIMEOUT;
        goto exit_ioctl;
      }

      atts_ptr->aprResponseTemp->hdr.aprErrCode = ParseAprResp(atts_ptr->aprResponseTemp);

      pr_debug("ATTS atts_ioctl: <GET_ALGORITHM_VERSION");
      break;
    case IOCTL_CMD_GET_PARAMS:
      pr_debug("ATTS atts_ioctl: >GET_PARAMS");

      moduleId = ioctlReqHdr->module_id;
      paramId = ioctlReqHdr->param_block_id;
      portId = ioctlReqHdr->port_id;
      tuneSize = ioctlReqHdr->tune_size;

      switch (ioctlReqHdr->svc)
      {
        case AFE:
          {
            GetAfeParams(moduleId, portId, paramId, tuneSize);
            break;
          }
        case VPM:
          {
            GetVpmParams(moduleId, paramId, tuneSize);
            break;
          }
        case VSM:
          {
            GetVsmParams(moduleId, paramId, tuneSize);
            break;
          }
        default:
          {
            pr_err("ATTS atts_ioctl: ATTS_ERRCODE_BADPARAM");
            errorCode = ATTS_ERRCODE_BADPARAM;
            atts_ptr->aprResponseUser = NULL;
            goto exit_ioctl;
          }
      }

      ret = wait_event_timeout(this_atts.wait,(atomic_read(&this_atts.state) == 0), msecs_to_jiffies(TIMEOUT_MS));
      if (!ret) {
        pr_err( "ATTS atts_ioctl: wait_event timeout\n");
        errorCode = ATTS_ERRCODE_TIMEOUT;
        goto exit_ioctl;
      }

      atts_ptr->aprResponseTemp->hdr.aprErrCode = ParseAprResp(atts_ptr->aprResponseTemp);

      pr_debug("ATTS atts_ioctl: <GET_PARAMS");
      break;
    case IOCTL_CMD_SET_PARAMS:
      pr_debug("ATTS atts_ioctl: >SET_PARAMS");

      moduleId = ioctlReqHdr->module_id;
      portId = ioctlReqHdr->port_id;
      paramId = ioctlReqHdr->param_block_id;
      portId = ioctlReqHdr->port_id;
      payload_length = ioctlReqHdr->payload_length;
      payload = ((atts_ioctl_data*)arg)->request->payload;

      switch (ioctlReqHdr->svc)
      {
        case AFE:
          {
            SetAfeParams(moduleId, paramId, portId, payload, payload_length);
            break;
          }
        case VPM:
          {
            SetVpmParams(moduleId, paramId, payload, payload_length);
            break;
          }
        case VSM:
          {
            SetVsmParams(moduleId, paramId, payload, payload_length);
            break;
          }
        default:
          {
            pr_err( "ATTS atts_ioctl: ATTS_ERRCODE_BADPARAM");
            errorCode = ATTS_ERRCODE_BADPARAM;
            atts_ptr->aprResponseUser = NULL;
            goto exit_ioctl;
          }
      }
      ret = wait_event_timeout(this_atts.wait,(atomic_read(&this_atts.state) == 0), msecs_to_jiffies(TIMEOUT_MS));
      if (!ret) {
        pr_err( "ATTS atts_ioctl: wait_event timeout\n");
        errorCode = ATTS_ERRCODE_TIMEOUT;
        goto exit_ioctl;
      }

      atts_ptr->aprResponseTemp->hdr.aprErrCode = ParseAprResp(atts_ptr->aprResponseTemp);

      pr_debug("ATTS atts_ioctl: <SET_PARAMS");
      break;
    default:
      pr_debug("ATTS atts_ioctl: unknown command id\n");
      errorCode = -ENOTTY;
      goto exit_ioctl;

  }

  // Copy only the amount needed - i.e. header + actual payload + offset to the payload
  payload_length = sizeof(apr_pkt_resp_hdr) + atts_ptr->aprResponseTemp->hdr.data_offset
                         + atts_ptr->aprResponseTemp->hdr.payloadLength;

  if(payload_length > sizeof(apr_pkt_resp)){
    pr_err( "ATTS atts_ioctl: payload_length > sizeof(apr_pkt_resp)\n");
    payload_length = sizeof(apr_pkt_resp);
  }

  pr_debug("ATTS atts_ioctl: copy data to user - aprErrCode 0x%x\n", atts_ptr->aprResponseTemp->hdr.aprErrCode);
  if(copy_to_user(atts_ptr->aprResponseUser,(void*)atts_ptr->aprResponseTemp,payload_length)){
    pr_err( "ATTS atts_ioctl: copy_to_user failed : returned %d \n", ret);
    errorCode = ATTS_ERRCODE_NULLPTR;
  }else{
    pr_debug("ATTS atts_ioctl: copied %d bytes to user\n", payload_length);
  }

exit_ioctl:
  pr_debug("ATTS atts_ioctl<: errorCode = %d\n",errorCode);
  atts_ptr->aprResponseUser = NULL;
  mutex_unlock(&atts_ptr->lock_ioctl);
  return (long)errorCode;
}

static int atts_open(struct inode *inode, struct file *file)
{
  struct atts *atts_ptr = &this_atts;
  int ret = 0;

  pr_debug("atts_open v0.02\n");

  mutex_lock(&atts_ptr->lock);

  // Open only once
  if(atts_ptr->device_open){
    mutex_unlock(&atts_ptr->lock);
    return -EBUSY;
  }


  atts_ptr->aprResponseTemp = kzalloc(sizeof(apr_pkt_resp), GFP_KERNEL);
  if(atts_ptr->aprResponseTemp == NULL){
    pr_err( "ATTS atts_open: out of memory");
    mutex_unlock(&atts_ptr->lock);
    return -EINVAL;
  }

  atts_ptr->device_open++;

  ret = atts_apr_init(apr_callback_table, attsPortId);

  mutex_unlock(&atts_ptr->lock);
  return ret;

}

static int atts_release(struct inode *inode, struct file *file)
{
  struct atts *atts_ptr = &this_atts;

  mutex_lock(&atts_ptr->lock);
  atts_ptr->device_open--;

  pr_debug("atts_release %d\n", atts_ptr->device_open);

  if ( NULL != atts_ptr->aprResponseTemp )
  {
    kfree(atts_ptr->aprResponseTemp);
    atts_ptr->aprResponseTemp = NULL;
  }

  if(atts_apr_deinit() < 0){
    pr_err( "atts_apr_deinit failed\n");
  }

  mutex_unlock(&atts_ptr->lock);
  return 0;
}

static int32_t atts_afe_callback(struct apr_client_data *data, void *priv)
{
  return atts_common_apr_callback(data, priv, APR_SVC_AFE);
}

static int32_t atts_vsm_callback(struct apr_client_data *data, void *priv)
{
  return atts_common_apr_callback(data, priv, APR_SVC_VSM);
}

static int32_t atts_vpm_callback(struct apr_client_data *data, void *priv)
{
  return atts_common_apr_callback(data, priv, APR_SVC_VPM);
}

// Common APR callback handler
static int32_t atts_common_apr_callback(struct apr_client_data *data, void *priv, uint16_t svc)
{
  uint32_t *payload;
  struct atts *atts_ptr = &this_atts;
  int32_t payloadLength = 0;

  pr_debug("ATTS atts_common_apr_callback>\n");
  if(!data){
    pr_err( "ATTS atts_common_apr_callback: invalid data\n");
    goto fail;
  }

  payload = (uint32_t*)data->payload;
  if(!payload){
    pr_err( "ATTS atts_common_apr_callback: invalid apr payload ptr\n");
    goto fail;
  }
  pr_debug("ATTS atts_common_apr_callback: opcode=0x%x\n", data->opcode);
  pr_debug("ATTS atts_common_apr_callback: payloadsize=%d\n", data->payload_size);

  payloadLength = data->payload_size;
  payload = (uint32_t*)data->payload;

  // When the APR is coming from voice services there's actually 2 responses.
  // First one is an acknowledge message. This ack is ignored here.
  // The second message is the "normal" APR response.
  if ( (data->token >> 16) == DIAG_TOKEN &&
    payloadLength == 4 &&
    (((int32_t*)payload)[0] == VOICE_CMD_GET_PARAM_V2 ||
    ((int32_t*)payload)[0] == VOICE_CMD_SET_PARAM_V2 ) )
  {
    pr_debug("ATTS atts_common_apr_callback: IGNORING VOICE ACK CALLBACK");
    return 0;
  }
  else if ( (data->token >> 16) == DATA_TOKEN )
  {
    // TODO: implement data trace handling
    pr_debug("ATTS atts_common_apr_callback<: DATA_TOKEN\n");
    return 0;
  }

  atts_ptr->aprResponseTemp->hdr.domainId = 0;
  atts_ptr->aprResponseTemp->hdr.data = NULL;
  atts_ptr->aprResponseTemp->hdr.payloadLength = 0;

  atts_ptr->aprResponseTemp->hdr.serviceId = svc;
  atts_ptr->aprResponseTemp->hdr.token = data->token;


  pr_debug("ATTS atts_common_apr_callback: src = 0x%x, dest_svc = 0x%x\n",data->src, data->dest_svc);

  if(payloadLength <= PAYLOAD_BUF_SIZE){
    pr_debug("ATTS atts_common_apr_callback: copy payload to temp %d\n",payloadLength);
    memcpy(atts_ptr->aprResponseTemp->payload,payload,payloadLength);
    atts_ptr->aprResponseTemp->hdr.payloadLength = payloadLength;
  }else{
    pr_err( "ATTS atts_common_apr_callback: ERROR: response payload buffer < PAYLOAD_BUF_SIZE\n");
    goto fail;
  }

  atomic_set(&this_atts.state, 0);
  wake_up(&this_atts.wait);
  pr_debug("ATTS atts_common_apr_callback<\n");
  return 0;

fail:
  atomic_set(&this_atts.state, -1);
  wake_up(&this_atts.wait);

  pr_debug("ATTS atts_common_apr_callback<\n");
  return -EINVAL;

}


// Creates a new token ID. Token type, given as a parameter, is set
// to upper word of a token.
uint32_t NewToken(TOKEN_TYPE tt)
{
  const uint32_t base = tt << 16;
  static uint16_t id = 0;
  uint32_t token = base | ++id;
  pr_debug("ATTS >NewToken\n");
  pr_debug("ATTS <NewToken 0x%08X\n", token);
  return (token);
}


// Queries the algorithm version from AFE's static service.
static int32_t GetAlgorithmVersion(uint32_t moduleId, uint32_t paramId)
{
  int32_t rc = 0;
  uint32_t token = NewToken(DIAG_TOKEN);
  struct afe_port_cmd_get_param_v2 payloadData;

  pr_debug("ATTS >GetAlgorithmVersion moduleId=0x%x paramId=0x%x \n",moduleId, paramId);

  payloadData.port_id = AFE_PORT_ID_AUDIO_IF_PORT_RANGE_START;
  payloadData.payload_size = sizeof(struct afe_port_param_data_v2) + VERSION_SIZE_IN_BYTES;
  payloadData.payload_address_lsw = 0;
  payloadData.payload_address_msw = 0;
  payloadData.mem_map_handle = 0;
  payloadData.module_id = moduleId;
  payloadData.param_id = paramId;

  rc = SendAprMessage(
    APR_MSG_TYPE_SEQ_CMD, // message type
    0, // source port
    testclient_afe_domain, // dest address
    testclient_afe_svc, // dest address
    attsPortId,// dest port
    token, // transaction ID
    AFE_PORT_CMD_GET_PARAM_V2, // AFE command (v2)
    &payloadData, // payload
    sizeof(payloadData)
    );

  pr_debug("ATTS <GetAlgorithmVersion, err 0x%08X\n", rc);
  return(rc);
}

int32_t GetAfeParams(uint32_t moduleId, uint16_t portId, uint32_t paramId, uint16_t tuneSize)
{
  int32_t rc = 0;
  uint32_t token = NewToken(DIAG_TOKEN);
  struct afe_port_cmd_get_param_v2 payloadData;

  pr_debug("ATTS >GetAfeParams modId=0x%x paramId=0x%x portId=0x%xd\n", moduleId, paramId, portId);

  payloadData.payload_size = sizeof(struct afe_port_param_data_v2) + tuneSize;// - APR_HDR_SIZE;
  payloadData.payload_address_lsw = 0;
  payloadData.payload_address_msw = 0;
  payloadData.mem_map_handle = 0;
  payloadData.module_id = moduleId;
  payloadData.port_id = portId;
  payloadData.param_id = paramId;

  rc = SendAprMessage(
    APR_MSG_TYPE_SEQ_CMD,
    0, // source port
    testclient_afe_domain, // dest address
    testclient_afe_svc, // dest address
    attsPortId, // dest port
    token, // transaction ID
    AFE_PORT_CMD_GET_PARAM_V2, // AFE command
    &payloadData,// payload
    sizeof(payloadData)
    );

  pr_debug("ATTS <GetAfeParams, err 0x%08X\n", rc);
  return(rc);
}



int32_t SetAfeParams(uint32_t moduleId, uint32_t paramId, uint16_t portId, char* payload, uint32_t payload_length)
{
  int32_t rc = 0;
  uint32_t payload_size;
  void *payload_p = NULL;
  uint32_t token = NewToken(DIAG_TOKEN);
  afe_set_param_t *pp = NULL;
  uint16_t tuneSize = 4 * (((uint16_t)payload_length + 3) / 4);

  pr_debug("ATTS >SetAfeParams\n");

  payload_size = sizeof(afe_set_param_t) + tuneSize;

  pp = (afe_set_param_t *)kzalloc(payload_size, GFP_KERNEL);

  if ( pp == NULL )
  {
    pr_debug("ATTS <SetAfeParams, Memory allocation failed err 0x%08X\n", rc);
    return -ATTS_ERRCODE_NOMEMORY;
  }

  payload_p = pp;


  pp->setParamHeader.payload_size = sizeof(struct afe_port_param_data_v2) + tuneSize;
  pp->setParamHeader.payload_address_lsw = 0;
  pp->setParamHeader.payload_address_msw = 0;
  pp->setParamHeader.mem_map_handle = 0;

  pp->paramDataHeader.module_id = moduleId;
  pp->setParamHeader.port_id = portId;
  pp->paramDataHeader.param_id = paramId;
  pp->paramDataHeader.param_size = (uint16_t)payload_length;
  pp->paramDataHeader.reserved = 0;

  pr_debug("ATTS SetAfeParams: payload_length %d\n", payload_length);

  if(copy_from_user(pp + 1, payload, payload_length)){
    pr_err( "ATTS <SetAfeParams, Copy_from_user failed");
    kfree(pp);
    return -EFAULT;
  }

  pr_debug("ATTS SetAfeParams: pp->setParamHeader.port_id 0x%08X\n",pp->setParamHeader.port_id);
  pr_debug("ATTS SetAfeParams: pp->setParamHeader.port_id 0x%08X\n",pp->setParamHeader.port_id);
  pr_debug("ATTS SetAfeParams: pp->setParamHeader.payload_size %d\n",pp->setParamHeader.payload_size);
  pr_debug("ATTS SetAfeParams: pp->setParamHeader.payload_address_lsw 0x%08X\n",pp->setParamHeader.payload_address_lsw);
  pr_debug("ATTS SetAfeParams: pp->setParamHeader.payload_address_msw 0x%08X\n",pp->setParamHeader.payload_address_msw);
  pr_debug("ATTS SetAfeParams: pp->paramDataHeader.module_id 0x%08X\n",pp->paramDataHeader.module_id);
  pr_debug("ATTS SetAfeParams: pp->paramDataHeader.param_id 0x%08X\n",pp->paramDataHeader.param_id);
  pr_debug("ATTS SetAfeParams: pp->paramDataHeader.param_size %d\n",pp->paramDataHeader.param_size);
  pr_debug("ATTS SetAfeParams: pp->paramDataHeader.reserved %d\n",pp->paramDataHeader.reserved);
  pr_debug("ATTS SetAfeParams: token 0x%08X\n",token);

  rc = SendAprMessage(
    APR_MSG_TYPE_SEQ_CMD,// message type
    0, // source port
    testclient_afe_domain, // dest address
    testclient_afe_svc, // dest address
    attsPortId, // dest port
    token, // transaction ID
    AFE_PORT_CMD_SET_PARAM_V2, // AFE command
    payload_p, // payload
    payload_size
    );

  if ( NULL != payload_p )
  {
    kfree(payload_p);
  }

  pr_debug("ATTS <SetAfeParams, err 0x%08X\n", rc);
  return(rc);
}

int32_t GetVpmParams(uint32_t moduleId, uint32_t paramId, uint16_t tuneSize)
{
  int32_t rc = 0;
  uint32_t token = NewToken(DIAG_TOKEN);
  struct voice_get_param_v2_t payloadData;

  pr_debug("ATTS >GetVpmParams moduleId=0x%x paramId=0x%x \n",moduleId, paramId);

  payloadData.module_id = moduleId;
  payloadData.param_id = paramId;
  payloadData.param_max_size = sizeof(struct voice_param_data) + tuneSize;
  payloadData.payload_address_lsw = 0;
  payloadData.payload_address_msw = 0;
  payloadData.reserved = 0;
  payloadData.mem_map_handle = 0;

  rc = SendAprMessage(
    APR_MSG_TYPE_SEQ_CMD,
    attsPortId, // source port
    testclient_vpm_domain, // dest address
    testclient_vpm_svc, // dest address
    0, // source port
    token, // transaction ID
    VOICE_CMD_GET_PARAM_V2, // VOICE command
    &payloadData, // payload
    sizeof(payloadData)
    );

  pr_debug("ATTS <GetVpmParams, err 0x%08X\n", rc);
  return(rc);
}



int32_t SetVpmParams(uint32_t moduleId, uint32_t paramId, char* payload, uint32_t payload_length)
{
  int32_t rc = 0;
  uint32_t payload_size;
  void *payload_p = NULL;
  uint32_t token = NewToken(DIAG_TOKEN);
  vcmn_set_param_t *pp = NULL;

  pr_debug("ATTS >SetVpmParams\n");

  payload_size = sizeof(vcmn_set_param_t) + payload_length;
  payload_size = 4 * (((uint16_t)payload_size + 3) / 4);

  pp = (vcmn_set_param_t *) kzalloc(payload_size, GFP_KERNEL);

  if ( pp == NULL )
  {
    pr_err( "ATTS <SetVpmParams, Memory allocation failed err 0x%08X\n", rc);
    return -ATTS_ERRCODE_NOMEMORY;
  }

  payload_p = pp;

  pp->setParamHeader.payload_size = payload_size;
  pp->setParamHeader.payload_address_msw = 0;
  pp->setParamHeader.payload_address_lsw = 0;
  pp->setParamHeader.mem_map_handle = 0;

  pp->paramDataHeader.module_id = moduleId;
  pp->paramDataHeader.param_id = paramId;
  pp->paramDataHeader.param_size = (uint16_t)payload_size;

  pp->paramDataHeader.reserved = 0;
  pr_debug("ATTS SetVpmParams: payload_length %d\n", payload_size);

  if(copy_from_user(pp + 1, payload, payload_length)){
    pr_err( "ATTS <SetVpmParams, Copy_from_user failed\n");
    kfree(pp);
    return -EFAULT;
  }

  pr_debug("ATTS SetVpmParams: pp->setParamHeader.payload_size 0x%08X\n",pp->setParamHeader.payload_size);
  pr_debug("ATTS SetVpmParams: pp->setParamHeader.payload_address_msw 0x%08X\n",pp->setParamHeader.payload_address_msw);
  pr_debug("ATTS SetVpmParams: pp->setParamHeader.payload_address_lsw 0x%08X\n",pp->setParamHeader.payload_address_lsw);
  pr_debug("ATTS SetVpmParams: pp->paramDataHeader.module_id 0x%08X\n",pp->paramDataHeader.module_id);
  pr_debug("ATTS SetVpmParams: pp->paramDataHeader.param_id 0x%08X\n",pp->paramDataHeader.param_id);
  pr_debug("ATTS SetVpmParams: pp->paramDataHeader.param_size %d\n",pp->paramDataHeader.param_size);
  pr_debug("ATTS SetVpmParams: pp->paramDataHeader.reserved %d\n",pp->paramDataHeader.reserved);
  pr_debug("ATTS SetVpmParams: token 0x%08X\n",token);

  rc = SendAprMessage(
    APR_MSG_TYPE_SEQ_CMD,
    attsPortId,// source port
    testclient_vpm_domain, // dest address
    testclient_vpm_svc, // dest address
    0, // source port
    token, // transaction ID
    VOICE_CMD_SET_PARAM_V2, // VOICE command
    payload_p, // payload
    payload_size
    );

  if ( NULL != payload_p )
  {
    kfree(payload_p);
  }

  pr_debug("ATTS <SetVpmParams, err 0x%08X\n", rc);
  return(rc);
}



int32_t GetVsmParams(uint32_t moduleId, uint32_t paramId, uint16_t tuneSize)
{
  int32_t rc = 0;
  uint32_t token = NewToken(DIAG_TOKEN);
  struct voice_get_param_v2_t payloadData;

  pr_debug("ATTS >GetVsmParams moduleId=0x%x paramId=0x%x \n",moduleId, paramId);

  payloadData.module_id = moduleId;
  payloadData.param_id = paramId;
  payloadData.param_max_size = sizeof(struct voice_param_data) + tuneSize;
  payloadData.payload_address_msw = 0;
  payloadData.payload_address_lsw = 0;

  payloadData.reserved = 0;
  payloadData.mem_map_handle = 0;

  rc = SendAprMessage(
    APR_MSG_TYPE_SEQ_CMD,
    attsPortId, // source port
    testclient_vsm_domain, // dest address
    testclient_vsm_svc, // dest address
    0, // dest port
    token, // transaction ID
    VOICE_CMD_GET_PARAM_V2, // VOICE command
    &payloadData, // payload
    sizeof(payloadData)
    );

  pr_debug("ATTS <GetVsmParams, err 0x%08X\n", rc);
  return(rc);
}



int32_t SetVsmParams(uint32_t moduleId, uint32_t paramId, char* payload, uint32_t payload_length)
{
  int32_t rc = 0;
  uint32_t payload_size;
  void *payload_p = NULL;
  uint32_t token = NewToken(DIAG_TOKEN);
  vcmn_set_param_t *pp = NULL;

  pr_debug("ATTS >SetVsmParams\n");

  payload_size = sizeof(vcmn_set_param_t) + payload_length;
  payload_size = 4 * (((uint16_t)payload_size + 3) / 4);

  pp = (vcmn_set_param_t *) kzalloc(payload_size, GFP_KERNEL);

  if ( pp == NULL )
  {
    pr_err( "ATTS <SetVsmParams, Memory allocation failed err 0x%08X\n", rc);
    return ATTS_ERRCODE_NOMEMORY;
  }

  payload_p = pp;

  pp->setParamHeader.payload_size = payload_size;
  pp->setParamHeader.payload_address_msw = 0;
  pp->setParamHeader.payload_address_lsw = 0;
  pp->setParamHeader.mem_map_handle = 0;

  pp->paramDataHeader.module_id = moduleId;
  pp->paramDataHeader.param_id = paramId;
  pp->paramDataHeader.param_size = (uint16_t)payload_size;

  pp->paramDataHeader.reserved = 0;

  pr_debug("ATTS SetVsmParams: payload_length %d\n", payload_size);

  if(copy_from_user(pp + 1, payload, payload_length)){
    pr_err( "ATTS <SetVsmParams, Copy_from_user failed\n");
    kfree(pp);
    return -EFAULT;
  }

  pr_debug("ATTS SetVsmParams: pp->setParamHeader.payload_size 0x%08X\n",pp->setParamHeader.payload_size);
  pr_debug("ATTS SetVsmParams: pp->setParamHeader.payload_address_msw 0x%08X\n",pp->setParamHeader.payload_address_msw);
  pr_debug("ATTS SetVsmParams: pp->setParamHeader.payload_address_lsw 0x%08X\n",pp->setParamHeader.payload_address_lsw);
  pr_debug("ATTS SetVsmParams: pp->paramDataHeader.module_id 0x%08X\n",pp->paramDataHeader.module_id);
  pr_debug("ATTS SetVsmParams: pp->paramDataHeader.param_id 0x%08X\n",pp->paramDataHeader.param_id);
  pr_debug("ATTS SetVsmParams: pp->paramDataHeader.param_size %d\n",pp->paramDataHeader.param_size);
  pr_debug("ATTS SetVsmParams: pp->paramDataHeader.reserved %d\n",pp->paramDataHeader.reserved);
  pr_debug("ATTS SetVsmParams: token 0x%08X\n",token);

  rc = SendAprMessage(
    APR_MSG_TYPE_SEQ_CMD,
    attsPortId, // source port
    testclient_vsm_domain, // dest address
    testclient_vsm_svc, // dest address
    0, // dest port
    token, // transaction ID
    VOICE_CMD_SET_PARAM_V2, // VOICE command
    payload_p, // payload
    payload_size
    );

  if ( NULL != payload_p )
  {
    kfree(payload_p);
  }

  pr_debug("ATTS <SetVsmParams, err 0x%08X\n", rc);
  return(rc);
}



static int32_t SendAprMessage(
  uint8_t msg_type,
  uint16_t src_port,
  uint8_t dst_domain,
  uint8_t dst_svc,
  uint16_t dst_port,
  uint32_t token,
  uint32_t opcode,
  void* payload,
  uint32_t payload_size)
{
  struct apr_hdr *header;
  int32_t ret = ATTS_ERRCODE_OK;

  pr_debug("ATTS >SendAprMessage: payload_size %d \n",payload_size);

  pr_debug("ATTS >SendAprMessage: dst_port %d \n",dst_port);

  // Header
  header = (struct apr_hdr*)payload;
  header->hdr_field = APR_HDR_FIELD(msg_type,APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
  header->pkt_size = payload_size;
  header->src_port = src_port;
  header->dest_svc = dst_svc;
  header->dest_domain = dst_domain;
  header->dest_port = dst_port;
  header->token = token;
  header->opcode = opcode;

  atomic_set(&this_atts.state, 1);

  pr_debug("ATTS SendAprMessage: >> apr_send_pkt\n");

  ret = apr_async_send(payload);
  if(ret < 0){
    pr_err( "ATTS apr_send_pkt failed: %d\n",ret);
    ret = -ATTS_ERRCODE_GENERAL;
  }else{
    ret = ATTS_ERRCODE_OK;
  }
  pr_debug("ATTS SendAprMessage: << apr_send_pkt\n");

  pr_debug("ATTS <SendAprMessage err 0x%08X\n", ret);

  return ret;
}



// Parses out the payload data starting point and length of the payload data
// from a APR packet struct.
static int32_t ParseAprResp(apr_pkt_resp* resp)
{
  int32_t status = ((int32_t*)resp->payload)[0];

  pr_debug("ATTS >ParseAprResp : svc = 0x%x\n",resp->hdr.serviceId);

  switch ( resp->hdr.serviceId )
  {
  case APR_SVC_AFE:
    {
      pr_debug("ATTS ParseAprResp: APRV2_IDS_SERVICE_ID_ADSP_AFE_V\n");
      status = ((int32_t*)resp->payload)[0];
      if ( status )
      {
        if ( status == AFE_PORT_CMD_GET_PARAM_V2 || status == AFE_PORT_CMD_SET_PARAM_V2 )
        {
          pr_debug("ATTS ParseAprResp: APRV2_IBASIC_RSP_RESULT\n");
          status = ((int32_t*)resp->payload)[1];
        }
        resp->hdr.data = NULL;
      }
      else
      {
        resp->hdr.data_offset = sizeof(int32_t)+sizeof(struct afe_port_param_data_v2);
        resp->hdr.payloadLength = ((struct afe_port_param_data_v2*)(resp->payload+sizeof(int32_t)))->param_size;
      }
      break;
    }
  case APR_SVC_ADM:
    {
      pr_debug("ATTS ParseAprResp: APRV2_IDS_SERVICE_ID_ADSP_ADM_V\n");
      status = ((int32_t*)resp->payload)[0];
      if ( status )
      {
        if ( status == ADM_CMD_GET_PP_PARAMS_V5 || status == ADM_CMD_SET_PP_PARAMS_V5 )
        {
          pr_debug("ATTS ParseAprResp: APRV2_IBASIC_RSP_RESULT\n");
          status = ((int32_t*)resp->payload)[1];
        }
        resp->hdr.data = NULL;
      }
      else
      {
        resp->hdr.data_offset = sizeof(int32_t)+sizeof(struct adm_param_data_v5);
        resp->hdr.payloadLength = ((struct adm_param_data_v5*)(resp->payload+sizeof(int32_t)))->param_size;
      }
      break;
    }
  case APR_SVC_VPM:
  case APR_SVC_VSM:
    {
      pr_debug("ATTS ParseAprResp: APRV2_IDS_SERVICE_ID_ADSP_VPM_V || APRV2_IDS_SERVICE_ID_ADSP_VSM_V\n");
      status = ((int32_t*)resp->payload)[0];
      if ( status )
      {
        if ( status == VOICE_CMD_GET_PARAM_V2 || status == VOICE_CMD_SET_PARAM_V2 )
        {
          pr_debug("ATTS ParseAprResp: APRV2_IBASIC_RSP_RESULT\n");
          status = ((int32_t*)resp->payload)[1];
        }
        resp->hdr.data = NULL;
      }
      else
      {
        resp->hdr.data_offset = sizeof(int32_t)+sizeof(struct voice_param_data);
        resp->hdr.payloadLength = ((struct voice_param_data*)(resp->payload+sizeof(int32_t)))->param_size;
      }
      break;
    }
  case APR_SVC_ASM:
    {
      pr_debug("ATTS ParseAprResp: APRV2_IDS_SERVICE_ID_ADSP_ASM_V\n");

      if ( (resp->hdr.token) >> 16 == ATTS_TOKEN )
      {
        status = 0;
        resp->hdr.data_offset = sizeof(uint32_t);
        resp->hdr.payloadLength = sizeof(uint32_t) * ((uint32_t*)resp->payload)[0];
        break;
      }

      status = ((int32_t*)resp->payload)[0];
      if ( status )
      {
        if ( status == ASM_STREAM_CMD_GET_PP_PARAMS_V2 || status == ASM_STREAM_CMD_SET_PP_PARAMS_V2 )
        {
          pr_debug("ATTS ParseAprResp: APRV2_IBASIC_RSP_RESULT\n");
          status = ((int32_t*)resp->payload)[1];
        }
        resp->hdr.data = NULL;
      }
      else
      {
        resp->hdr.data_offset = sizeof(int32_t)+sizeof(struct asm_stream_param_data_v2);
        resp->hdr.payloadLength = ((struct asm_stream_param_data_v2*)(resp->payload+sizeof(int32_t)))->param_size;
      }
      break;
    }
  default:
    {
      status = ATTS_ERRCODE_UNSUPPORTED;
      resp->hdr.data = NULL;
      break;
    }
  }
  pr_debug("ATTS <ParseAprResp: err 0x%X, payloadLength %d\n", status, resp->hdr.payloadLength);
  return status;
}

static int __init atts_init(void)
{
  int ret = 0;

  struct atts *atts_ptr = &this_atts;

  pr_debug("ATTS >atts_init\n");

  ret = misc_register(&atts_dev);

  if(ret > 0) {
    pr_err( "ATTS atts_init: registering ATTS device failed with %d\n", ret);
    return ret;
  }

  atts_ptr->device_open = 0;
  atts_ptr->aprResponseUser = NULL;
  atts_ptr->aprResponseTemp = NULL;
  mutex_init(&atts_ptr->lock);
  mutex_init(&atts_ptr->lock_ioctl);
  init_waitqueue_head(&this_atts.wait);

  pr_debug("ATTS <atts_initd: device created successfully\n");

  return 0;

}

static void __exit atts_exit(void)
{
  pr_debug("ATTS atts_exit\n");

  misc_deregister(&atts_dev);
}

device_initcall(atts_init);
module_exit(atts_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ATTS device");

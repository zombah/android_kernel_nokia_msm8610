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

#ifndef ATTSPRIVATE_H
#define ATTSPRIVATE_H

struct voice_param_data
{
   uint32_t module_id;
   uint32_t param_id;
   uint16_t param_size;
   uint16_t reserved;
}voice_param_data_struct;

struct voice_set_param_v2
{
   struct apr_hdr hdr;
   uint32_t payload_address_lsw;
   uint32_t payload_address_msw;
   uint32_t payload_size;
   uint32_t mem_map_handle;
}voice_set_param_v2_struct;

struct voice_get_param_v2_t
{
   struct apr_hdr hdr;
   uint32_t payload_address_lsw;
   uint32_t payload_address_msw;
   uint32_t module_id;
   uint32_t param_id;
   uint16_t param_max_size;
   uint16_t reserved;
   uint32_t mem_map_handle;
}__packed;


typedef struct {
  struct apr_hdr hdr;
  struct afe_port_cmd_set_param_v2 setParamHeader;
  struct afe_port_param_data_v2 paramDataHeader;
} afe_set_param_t;

typedef struct {
  struct adm_cmd_set_pp_params_v5 setParamHeader;
  struct adm_param_data_v5 paramDataHeader;
} adm_set_param_t;

typedef struct {
  struct voice_set_param_v2 setParamHeader;
  struct voice_param_data paramDataHeader;
} vcmn_set_param_t;

typedef struct {
  struct asm_stream_cmd_set_pp_params_v2 setParamHeader;
  struct asm_stream_param_data_v2 paramDataHeader;
} asm_set_param_t;

// Values to be used as a upper word of a 32-bit APR message token.
// The token can be used to identify the initiator of the message.
typedef enum
{
  DIAG_TOKEN = 0xD1A6,
  ATTS_TOKEN = 0xA775,
  DATA_TOKEN = 0xDA7A
}TOKEN_TYPE;


/* Definitions */
#ifndef VOICE_CMD_GET_PARAM_V2
#define VOICE_CMD_GET_PARAM_V2 0x0001103D
#endif
#ifndef VOICE_CMD_SET_PARAM_V2
#define VOICE_CMD_SET_PARAM_V2 0x0001103C
#endif

#define ATTS_MAX_NUM_OF_ACTIVE_STREAMS  10

#define VERSION_SIZE_IN_BYTES           61


/* Functions */
static int atts_open(struct inode *, struct file *);
static int atts_release(struct inode *, struct file *);
static long atts_ioctl(struct file *f, unsigned int cmd, unsigned long arg);
static int32_t atts_common_apr_callback(struct apr_client_data *data, void *priv, uint16_t svc);
static int32_t atts_afe_callback(struct apr_client_data *data, void *priv);
static int32_t atts_vsm_callback(struct apr_client_data *data, void *priv);
static int32_t atts_vpm_callback(struct apr_client_data *data, void *priv);
static int32_t ParseAprResp(apr_pkt_resp* resp);

static int32_t GetAlgorithmVersion(uint32_t moduleId, uint32_t paramId);

static int32_t GetAfeParams(uint32_t moduleId, uint16_t portId, uint32_t paramId, uint16_t tuneSize);
static int32_t GetVpmParams(uint32_t moduleId, uint32_t paramId, uint16_t tuneSize);
static int32_t GetVsmParams(uint32_t moduleId, uint32_t paramId, uint16_t tuneSize);

static int32_t SetAfeParams(uint32_t moduleId, uint32_t paramId, uint16_t portId, char* payload, uint32_t payload_length);
static int32_t SetVpmParams(uint32_t moduleId, uint32_t paramId, char* payload, uint32_t payload_length);
static int32_t SetVsmParams(uint32_t moduleId, uint32_t paramId, char* payload, uint32_t payload_length);

static int32_t SendAprMessage(
  uint8_t msg_type,
  uint16_t src_port,
  uint8_t dst_svc,
  uint8_t dst_domain,
  uint16_t dst_port,
  uint32_t token,
  uint32_t opcode,
  void* payload,
  uint32_t payload_size);


#endif /* ATTSPRIVATE_H */


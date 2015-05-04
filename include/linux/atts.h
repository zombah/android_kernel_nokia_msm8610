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

#ifndef ATTS_H
#define ATTS_H

#include <linux/ioctl.h>


/* Error codes */
#define ATTS_ERRCODE_OK           0
#define ATTS_ERRCODE_NOTFOUND     1
#define ATTS_ERRCODE_GENERAL      2
#define ATTS_ERRCODE_UNSUPPORTED  3
#define ATTS_ERRCODE_BADPARAM     4
#define ATTS_ERRCODE_TIMEOUT      5
#define ATTS_ERRCODE_NOMEMORY     6
#define ATTS_ERRCODE_NULLPTR      7


typedef enum
{
  ADM,
  AFE,
  VSM,
  VPM,
  ASM,
  NONE
}ADSP_SERVICE;


#define PAYLOAD_BUF_SIZE 4096

typedef struct
{
  uint16_t aprErrCode;
  int16_t domainId;
  int16_t serviceId;
  uint32_t token;
  uint32_t payloadLength;
  void* data;
  uint16_t data_offset;
}apr_pkt_resp_hdr;

typedef struct
{
  apr_pkt_resp_hdr hdr;
  uint8_t payload[PAYLOAD_BUF_SIZE];
}apr_pkt_resp;


typedef struct
{
   ADSP_SERVICE svc;
   uint32_t module_id;
   uint32_t param_block_id;
   uint16_t port_id;
   uint16_t tune_size;
   uint16_t payload_length;
}atts_ioctl_req_hdr;

typedef struct
{
   atts_ioctl_req_hdr hdr;
   uint8_t payload[PAYLOAD_BUF_SIZE];
}atts_ioctl_req;


typedef struct
{
   atts_ioctl_req* request;
   apr_pkt_resp* response;
}atts_ioctl_data;

#define IOCTL_APP_TYPE 97

#define IOCTL_CMD_GET_DEVICE_INFO_VERSION  _IOR(IOCTL_APP_TYPE,0, atts_ioctl_data)
#define IOCTL_CMD_GET_ALGORITHM_VERSION    _IOR(IOCTL_APP_TYPE,1, atts_ioctl_data)
#define IOCTL_CMD_GET_PARAMS               _IOR(IOCTL_APP_TYPE,2, atts_ioctl_data)
#define IOCTL_CMD_SET_PARAMS               _IOWR(IOCTL_APP_TYPE,3,atts_ioctl_data)


#endif /* ATTS_H */

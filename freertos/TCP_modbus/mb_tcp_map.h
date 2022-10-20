/*
 * mb_tcp_map.h
 *
 *  Created on: 06-June-2020
 *      Author: urvil
 */

#ifndef MAIN_MB_TCP_MAP_H_
#define MAIN_MB_TCP_MAP_H_

#include "mb_tcp_m.h"
#include "deviceInfo.h"

#define  MODBUSMAPSIZE	MapLength
#define  MODBUSREGLIM	1000     //// TODO TODO CHECK WERE IS IT AFFECTED
//Pointer for the modbus map structure for the two ports
MBTcpMap xMBUSMap[2];

//Modbus entries input for port 1
MbTcpMVariStruct xMBEntryP1[MapLength];

//Modbus entries input for port 1
MbTcpMVariStruct xMBEntryP2[MapLength];

//Memory to save data
//Port 1
uint16_t u16P1MBReadResult[MODBUSREGLIM];
//Port 2
uint16_t u16P2MBReadResult[MODBUSREGLIM];

//Temporary buffer for the port1
uint8_t u8MB1Buf[256];

//Temporary buffer for the port2
uint8_t u8MB2Buf[256];

//Init map for the port1
void vInitMap(MBTcpMap * pxMapID,
		MbTcpMVariStruct *xMbQueryEntry,
		uart_port_t xComport, // TODO TCP SOcket
		xUARTConf *xUART,  // TODO TCP SOcket
		uint8_t *u8buf,
		uint16_t *u16ResultBuf,
		uint16_t u16SizeResultBuf,
		QueueHandle_t * xQueueData,
		QueueHandle_t * xQueue,
		QueueHandle_t * xPwerAvgQueue);


#endif /* MAIN_MB_TCP_MAP_H_ */

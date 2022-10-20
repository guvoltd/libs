///*****************************************************************************
//
//! \file mb_tcp_m.c
//! \brief Modbus TCP Slave File
//! \details This file creates a modbus TCP slave, and serves the repeatation 
//! read request which are predefined in its loop. It also serves the write 
//! requeste that are made by other functions. It also provide the callback
//! support for that function. 
//! @note This is only tested with FreeRTOS @b <>
//! 
//! \version 1.1.0
//! \date 06/06/2030
//! \par  Revision history
//! \author Dhruv Acharya
//! \copyright
//!
//! Copyright (c)  2020, Kirtan Technologies, Ahmedabad, India
//! All rights reserved.
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions
//! are met:
//!
//!		- Redistributions of source code must retain the above copyright
//!		  notice, this list of conditions and the following disclaimer.
//!		- Redistributions in binary form must reproduce the above copyright
//! 	  notice, this list of conditions and the following disclaimer in the
//!		  documentation and/or other materials provided with the distribution.
//!		- Neither the name of the <ORGANIZATION> nor the names of its
//!		  contributors may be used to endorse or promote products derived
//!		  from this software without specific prior written permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//! THE POSSIBILITY OF SUCH DAMAGE.
//
///*****************************************************************************


#include "mb_tcp_m.h"
#include "mb_tcp_map.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"  // TODO TCP SOCKET
#include "esp_err.h"

#include <time.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "GenDef.h"
#include "datamanage.h"
#include "rtcupdate.h"
#include "deviceInfo.h"

#define TCP_DEBUG LWIP_DBG_ON



//#define 	_MBDEBG_


/* ----------------------- Defines ------------------------------------------*/
#define MB_ADDRESS_BROADCAST    	( 0 )   // Modbus broadcast address.
#define MbTcpM_BUF_SIZE				( 256 )	// Maximum buffer size


//! \brief If master send a broadcast frame,the master will wait time of convert to delay,
// then master can send other frame
#define MbTcpM_DELAY_MS_CONVERT		( 200 )
//! \brief If master send a frame which is not broadcast,the master will wait sometime for slave.
// And if slave is not respond in this time,the master will process this timeout error.
// Then master can send other frame
#define MbTcpM_TIMEOUT_MS_RESPOND	( 100 )

//Various exception code for response
#define MbTcpM_Exce_Ill_Func				( 0x01 )
#define MbTcpM_Exce_Ill_Addr				( 0x02 )
#define MbTcpM_Exce_Ill_Data				( 0x03 )
#define MbTcpM_Exce_Slave_Fail				( 0x04 )
#define MbTcpM_Exce_Acknowlege				( 0x05 )
#define MbTcpM_Exce_Busy					( 0x06 )
#define MbTcpM_Exce_Mem_Parity_Err			( 0x08 )
#define MbTcpM_Exce_Gateway_Path			( 0x0A )
#define MbTcpM_Exce_Gateway_Target_Unavl	( 0x0B )

// ----------------------- Variables ----------------------------------------//
char cLoggerIPPart[16];
#define TCP_MODBUS_PORT 502

//------------------------- Function declarations ---------------------------//

/**
 * @brief create TCP socket and connect with the Device sever.
 * @details This function create TCP client Socket and connects to the server with
 * ip given in the argument.
 *
 * @param u8DevIP  Pointer to the buffer where frame data is stored
 * @param usLen     Length of the buffer.
 *
 * @return 16 bit CRC.
 */
int vCreateTCPConnection(uint8_t *u8DevIP,uint16_t u16timeout, int *iTcpSock) {
	char addr_str[128];
	int addr_family;
	int ip_protocol;
	struct sockaddr_in destAddr;
	destAddr.sin_addr.s_addr = inet_addr((const char*)u8DevIP);
	destAddr.sin_family = AF_INET;
	destAddr.sin_port = htons(TCP_MODBUS_PORT);
	addr_family = AF_INET;
	ip_protocol = IPPROTO_IP;
	inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

	int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
	//printf("%25s >> open Socket : %d\r\n",__func__,sock);
	if (sock < 0) {
		//lwip_close_r(sock);
		printf("%25s >> Unable to create socket(%s): errno %d\n", __func__, u8DevIP,sock);
		return -1;
	}

	struct timeval tv;
	tv.tv_sec = u16timeout;
	tv.tv_usec = 0;
	socklen_t len = sizeof(tv);

	//set socket timeout
	if(setsockopt(sock,SOL_SOCKET ,SO_RCVTIMEO,(void*)&tv,len)<0)
	{
		printf("%25s >> Socket unable to set Timeout\n", __func__);
	}

	int err = connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));

	if (err != 0)
	{
		printf("%25s >> Socket unable to connect: errno %d\n", __func__, errno);
		//close socket
		close(sock);
		err = -1;
	}

	*iTcpSock = sock;
	return err;
}



/**
 * @brief Calculate CRC based on MODBUS-16 logic
 * @details This function Calculate CRC based on MODBUS-16 logic.
 *
 * @param pucFrame  Pointer to the buffer where frame data is stored
 * @param usLen     Length of the buffer.
 *
 * @return 16 bit CRC.
 */
unsigned short usMbTcpMCRC16( unsigned char * pucFrame, unsigned short usLen );

/**
 * @brief	This is task for the modbus
 * @param	pV	Void pointer that contains necessary informations that need to be passed to the task
 * @return	void
 */
void vMbTcpM_Task(void *pvParameters);

/**
 * @brief	This function will execute the modbus query and will get response
 * @param	xRequest	structure containing essential information for generating modbus query
 * @param	pu8Buf		Buffer for storing data temporarily
 * @param	usT35TimeOutmSec	3.5 character delay in the ms
 * @return 	None
 */
static void vMbTcpMTCPReq(MbTcpMVariStruct *xRequest, uint8_t *pu8Buf, uint16_t u16timeout, uint8_t *);

static int TimeoutOccured(TickType_t xInitial, int timeMS)
{
	TickType_t xCurrent;

	xCurrent = xTaskGetTickCount();

	if(xCurrent >= xInitial)
	{
		if((xCurrent-xInitial) >= pdMS_TO_TICKS(timeMS))
		{
			return 1;
		}
		else
			return 0;
	}
	else
	{
		if((xCurrent + (~xInitial)) >= pdMS_TO_TICKS(timeMS))
		{
			return 1;
		}
		else
			return 0;
	}

	return 0;
}

static const unsigned char aucCRCHi[] = {
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
		0x00, 0xC1, 0x81, 0x40
};

static const unsigned char aucCRCLo[] = {
		0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
		0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
		0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
		0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
		0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
		0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
		0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
		0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
		0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
		0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
		0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
		0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
		0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
		0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
		0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
		0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
		0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
		0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
		0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
		0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
		0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
		0x41, 0x81, 0x80, 0x40
};

static void vUpdateSlaveList(uint8_t *pu8, uint16_t u16Sizepu8,MBTcpMap *pxMBMap)
{
	uint8_t u8SlaveId;

	uint8_t u8AvalFlag;

	uint16_t u16Index = 0;

	for(uint16_t u16i =0 ; u16i < pxMBMap->u16Length; u16i++)
	{
		u8SlaveId = (uint8_t)pxMBMap->pxMBQuery[u16i].ucSlaveID;

		//Reset flag
		u8AvalFlag = 0;
		//Check if slave-id is available in the array
		for(uint16_t u16j = 0 ; u16j < u16Sizepu8; u16j++)
		{
			if(pu8[u16j] == u8SlaveId)
			{
				u8AvalFlag = 1;
				break;
			}
		}

		//adding the entry
		if(u8AvalFlag == 0)
		{
			pu8[u16Index] = u8SlaveId;
			u16Index++;

			if(u16Index >= u16Sizepu8)
				break;
		}
	}

	//print the slave ids
	for(uint8_t u16i = 0; u16i < u16Index ; u16i++)
	{
		printf("%s >> [%2d] = [%2d] \n",__FUNCTION__,u16i, pu8[u16i]);
	}
}

void vMbTcpM_Task(void *pvParameters)
{
	//	xQueueModbusResultElement xQueueModbusResultElementVar;

	uint8_t u8AddToQueue = 0;
	uint8_t u8AddPoAtoQue = 0;
	uint8_t u8AddPowerAvgtoQue = 0;
	uint32_t u8Cycle = 0; // 0 = slow, 1 = Medium, 2 = fast, 255 = do nothing

	uint8_t u8TimeOutCnt = 0;
	uint8_t u8TimeOutSlaveID = 0;
	uint8_t u8TimeOutIndex = 0;

	int iTcpSock = 0;
	uint8_t u8DevIP[16] = {0};

	//flag for restart system at 4:00 am (To reset all the parameter data)
	bool bRestartFlag = false;

	uint8_t u8FirstScan = 0xFF;

	//parameter type
	int iParameterType = 0;

	//Size need to change to 20 ( Max 20 device)
	//buffer for slave id
	uint8_t u8SlaveIdBuf[MapLength];

	//reset the memory of buffer
	memset(u8SlaveIdBuf, 0, MapLength);

	TickType_t 	xLastSlowTick = 0;
	TickType_t	xLastMedTick=0;
	TickType_t	xLastFastTick=0;
	TickType_t  xPOAScanTick=0;


	printf("%s >> Starting modbus task\n", __FUNCTION__);

	//Take the argument and extract map pointers and number of the elements in the map
	MBTcpMap *pxMBMap = (MBTcpMap *)pvParameters;

	//used to store the current ticks
	TickType_t xCurrTicks;

	//calculation for the 3.5 characters delay --> Need calculations
	pxMBMap->u8char35delay = 20;

	vUpdateSlaveList(u8SlaveIdBuf, MapLength, pxMBMap);

	xEventGroupWaitBits(xEvents, RTCTIMEAQUIRED, pdFALSE, pdFALSE, portMAX_DELAY);

	printf("%s >> Going into the loop \n",__FUNCTION__);

	//	uint32_t u32TempCnt = 0;
	//Task loop
	while(1)
	{
		if(xEventGroupGetBits(xEvents) & WIFICONNECTED)
		{
			//buffer for storing the current time
			char cBuff[3] = {0};

			time_t xNow;

			struct tm xTime;

			//get the current time stamp
			time(&xNow);

			//get local time
			localtime_r(&xNow,&xTime);

			//get the current date into buffer
			strftime(cBuff, sizeof(cBuff), "%H", &xTime);
			uint8_t u8Hour = atoi(cBuff);

			//check is modbus scanning off Period is enable
			if(!(u8Hour < DataReportTimeEndsHr && u8Hour > DataReportTimeStartHr))
			{
				//set SFTP Available Bit for uploading file from flash
				xEventGroupSetBits(xEvents, SFTPAVAIL);
			}

			u8Cycle = 0x00 | u8FirstScan ;

			u8FirstScan = 0x00;

			xCurrTicks = xTaskGetTickCount();

			//Fast Scan Tick
			if( (xCurrTicks - xLastFastTick) >= (pxMBMap->xFastScanTicks))
			{
				//printf("%10d %10d  %10d  %10d \r\n",pxMBMap->xFastScanTicks, xCurrTicks, xLastFastTick, xCurrTicks-xLastFastTick);
				u8Cycle |= eScanFast;
				xLastFastTick = xCurrTicks;
			}

			//Medium Scan Tick
			else if( (xCurrTicks - xLastMedTick) >= (pxMBMap->xMedScanTicks))
			{
				u8Cycle |= eScanMed;
				xLastMedTick = xCurrTicks;
			}

			//Slow Scan Tick
			else if( (xCurrTicks - xLastSlowTick) >= (pxMBMap->xSlowScanTicks))
			{
				u8Cycle |= eScanSlow;
				xLastSlowTick = xCurrTicks;
			}
			//PoA Parameter Scan at every 1 minitue
			else if( (xCurrTicks - xPOAScanTick) >= (pdMS_TO_TICKS((1*60)*1000))-2)
			{
				printf("%s >> PoA Scan Time TickTime difference %d.....\n",__FUNCTION__, (xCurrTicks - xPOAScanTick));
				u8Cycle |= ePOAScan;
				u8Cycle |= ePowerAvgScan;
				xPOAScanTick = xCurrTicks;
			}
			else
			{
				u8Cycle |= eInvStateScan;
			}

			//Check delay get effects on multiple POA Scan Time Logs ....
			vTaskDelay(10 / portTICK_PERIOD_MS);

			//scan modbus query during scanning time only
			if((u8Cycle != 0) && (u8Hour < DataReportTimeEndsHr && u8Hour > DataReportTimeStartHr))
				//		if((u8Cycle != 0) )
			{

				//check if restart flag is set than reboot the system at 4:00 am
				if(bRestartFlag)
				{
					//check current hour
					if(u8Hour == 4)
					{
						//debug printf
						printf("%s >> Scanning started for the next day. Need to restart system for reset parameter",__FUNCTION__);

						//restart system
						esp_restart();
					}
				}


				//printf("%s >> Current Hour : %d\n",__FUNCTION__,u8Hour);
				//printf("%s >>Port : %d ----> Scaning period [%d] \r\n", __FUNCTION__,pxMBMap->xComPort, u8Cycle);

				//Modbus query scan queue
				u8AddToQueue = 0;

				//PoA  Query Scan Queue
				u8AddPoAtoQue = 0;

				//Power Average Scan Queue
				u8AddPowerAvgtoQue = 0;
				//scan slave id type
				for(uint8_t u8i = 0 ; u8SlaveIdBuf[u8i] != 0 ; u8i++)
				{
					//Modbus query scan queue
					u8AddToQueue = 0;

					//PoA  Query Scan Queue
					u8AddPoAtoQue = 0;

					//Power Average Scan Queue
					u8AddPowerAvgtoQue = 0;

					uint8_t u8TempCnt = 0;

					// 2020-02-21 used for skipping timeout inverter when constant 3 timeouts found
					u8TimeOutSlaveID = u8SlaveIdBuf[u8i];

					// Check and create tcp socket here for current slave_id
					memset((char*)u8DevIP, 0x00, 16);
					//				sprintf((char*)u8DevIP, "%s%d", cLoggerIPPart, u8SlaveIdBuf[u8i]);
					strcpy((char*)u8DevIP, cLoggerIPPart);
					sprintf((char*)&u8DevIP[strlen((char*)u8DevIP)], "%d", u8SlaveIdBuf[u8i]);

					{
						// Timeout variable init before loop starts
						u8TimeOutCnt = 0;
						u8TimeOutIndex = 0;
						//				printf("%25s >> Debug Log -----3\n", __func__);
						//Actual reading
						for(uint16_t u16Index = 0 ; u16Index < pxMBMap->u16Length; u16Index++)
						{

							//If slave id is matched
							if(pxMBMap->pxMBQuery[u16Index].ucSlaveID == u8SlaveIdBuf[u8i])
							{
								//get the parameter Id of the scan query
								iParameterType = iParameterID[pxMBMap->pxMBQuery[u16Index].u16ParameterType];

								if( (u8Cycle & ePowerAvgScan) && (iParameterType == iParameterID[OUTPUT_ACTIVE_POWER_INDEX]) )
								{
									//printf("%25s >> Starting Power Average MODBUS request loop count : %d\n", __func__, pxMBMap->u8DeviceNo);
									//scan the device for the Slave Id
									for(uint8_t u8j=0; u8j < pxMBMap->u8DeviceNo; u8j++)
									{
										//compare the slave id
										if( (pxMBMap->pxDeviceData[u8j].u8SlaveId == u8SlaveIdBuf[u8i]))
										{
											u8TempCnt++;
											//printf("%25s >> Power Average For SlaveID[%d] : %d\n",__FUNCTION__,pxMBMap->pxDeviceData[u8j].u8SlaveId,u8SlaveIdBuf[u8i]);
											//scan for the Output Active Power
											vMbTcpMTCPReq(&pxMBMap->pxMBQuery[u16Index], pxMBMap->pu8Buf, pxMBMap->u16MbTimeout, u8DevIP);

											//check response error check
											if(pxMBMap->pxMBQuery[u16Index].xResponseError == err_NoError){
												//Enable the Queue Flag
												//printf("%25s >> Power Average Request successful....\n", __func__);
												u8AddPowerAvgtoQue = 1;
											} else {
												printf("%25s >> Power Average Request Failed....\n", __func__);
												pxMBMap->pxMBQuery[u16Index].xResponseError = err_Timeout;
												u8AddPowerAvgtoQue = 1;
												// 2020-04-19 Move to Power AvG function as discussed
												//2020-03-05 Changed to less then or equal to zero consider it as failed..
												//if(strlen(pxMBMap->pxDeviceData[u8j].cInvName) == 0)
												//	vInvPowerGenerationFault(pxMBMap->pxDeviceData[u8j].cDeviceId);
												//else
												//	vInvPowerGenerationFault(pxMBMap->pxDeviceData[u8j].cInvName);
												//
												//printf("%25s >> Inverter Power generation Fault Found[%s]..\n", __func__, pxMBMap->pxDeviceData[u8j].cDeviceId);

											}

										}
									}
								}

								if( (u8Cycle & ePOAScan) && ((iParameterType >= 400) && (iParameterType <=405)))
								{
									//scan the device for the Slave Id
									for(uint8_t u8j=0; u8j < pxMBMap->u8DeviceNo; u8j++)
									{
										//printf("%s >> PoA Flag : %d,SlaveID[%d] : %d\n",__FUNCTION__,pxMBMap->pxDeviceData[u8j].bPoAFlag,pxMBMap->pxDeviceData[u8j].u8SlaveId,u8SlaveIdBuf[u8i]);
										//compare the slave id
										if( (pxMBMap->pxDeviceData[u8j].u8SlaveId == u8SlaveIdBuf[u8i]) && (pxMBMap->pxDeviceData[u8j].bPoAFlag))
										{
											//scan for the POA
											vMbTcpMTCPReq(&pxMBMap->pxMBQuery[u16Index], pxMBMap->pu8Buf, pxMBMap->u16MbTimeout, u8DevIP);

											//Enable the Queue Flag
											u8AddPoAtoQue = 1;
										}

									}
								}

								if(pxMBMap->pxMBQuery[u16Index].u8ScanCategory & u8Cycle)
								{
									u8TempCnt++;
									u8AddToQueue = 1;
									//							printf("%25s >> Timeout slavid [%d==%d], u8TimeOutIndex[%d], u16Index[%d]\n", __func__, u8TimeOutSlaveID, u8SlaveIdBuf[u8i], u8TimeOutIndex, u16Index );
									if(u8TimeOutCnt < 3)
									{
										vMbTcpMTCPReq(&pxMBMap->pxMBQuery[u16Index], pxMBMap->pu8Buf, pxMBMap->u16MbTimeout, u8DevIP);
										if(pxMBMap->pxMBQuery[u16Index].xResponseError == err_Timeout || pxMBMap->pxMBQuery[u16Index].xResponseError == err_IncharTimeout) {
											if(u8TimeOutSlaveID == u8SlaveIdBuf[u8i])
											{
												if( (u8TimeOutCnt == 0) || (u8TimeOutIndex == (u16Index-1)) ) {
													u8TimeOutCnt++;
													u8TimeOutIndex = u16Index;
												} else {  //else no constant time out found
													u8TimeOutCnt = 0;
												}

											} else {
												u8TimeOutCnt = 0;
											}
										}
									}
									else
									{
										//Upload the error status
										pxMBMap->pxMBQuery[u16Index].xResponseError = err_Timeout;
										continue;
									}



									//check the Alarm State parameter
									if(iParameterType == 20)
									{
										//get the Inverter state
										uint16_t u16ScanData = pxMBMap->pxMBQuery[u16Index].u16ReceivedData[0];

										//get the device slave id
										for(uint8_t u8Id=0; u8Id<pxMBMap->u8DeviceNo;u8Id++)
										{
											//if slave id got
											if(pxMBMap->pxDeviceData[u8Id].u8SlaveId == u8SlaveIdBuf[u8i])
											{

												//check response error check
												if(pxMBMap->pxMBQuery[u16Index].xResponseError == err_NoError)

													//if error is Ok than set the flag for the Inverter operation time count
													pxMBMap->pxDeviceData[u8Id].u8ErrorState = 1;

												//else reset the flag
												else

													//reset the error flag
													pxMBMap->pxDeviceData[u8Id].u8ErrorState = 0;

												//TO DO No Need to check scan error again
												//compare the inverter scan state value
												if( ((pxMBMap->pxDeviceData[u8Id].u16State != u16ScanData) || (u8Cycle == 0xFF)) && pxMBMap->pxMBQuery[u16Index].xResponseError == err_NoError)
												{
													//debug print
													// printf("%s >> Scan State[%d] : %d\n",__FUNCTION__,u16ScanData,pxMBMap->pxDeviceData[u8Id].u16State);
													// printf("%s >> ScanIndex : %d\n",__FUNCTION__,pxMBMap->pxDeviceData[u8Id].u8ScanIndex);
													// printf("%s >> Device %d Scan State : %d\n",__FUNCTION__,u8Id,u16ScanData);

													//store the ScanState value into another variable for the comparision
													pxMBMap->pxDeviceData[u8Id].u16State = u16ScanData;

													//update the inverter state into cyclic buffer
													pxMBMap->pxDeviceData[u8Id].u16ScanData[pxMBMap->pxDeviceData[u8Id].u8ScanIndex] = pxMBMap->pxDeviceData[u8Id].u16State;
													pxMBMap->pxDeviceData[u8Id].u64TimeStamp[pxMBMap->pxDeviceData[u8Id].u8ScanIndex] = pxMBMap->pxMBQuery[u16Index].u64TimeStampUnix;

													//increament the index
													pxMBMap->pxDeviceData[u8Id].u8ScanIndex++;

													//if buffer index is greater than 5
													if(pxMBMap->pxDeviceData[u8Id].u8ScanIndex >= 5)

														//than reset the index
														pxMBMap->pxDeviceData[u8Id].u8ScanIndex=0;
												}

												//break loop
												break;
											}

										}

									}

									//delay of 1 ms
									vTaskDelay(1 / portTICK_PERIOD_MS);

								}

								//Inverter event state Scan
								else if( u8Cycle == eInvStateScan)
								{
									//Check for Inverter State scan Parameter
									if(iParameterType == 20)
									{
										//scan the query
										vMbTcpMTCPReq(&pxMBMap->pxMBQuery[u16Index], pxMBMap->pu8Buf, pxMBMap->u16MbTimeout, u8DevIP);

										//get the inverter scan state
										uint16_t u16ScanData = pxMBMap->pxMBQuery[u16Index].u16ReceivedData[0];

										//get the device slave id
										for(uint8_t u8Id=0; u8Id<pxMBMap->u8DeviceNo;u8Id++)
										{
											//if slave id got
											if(pxMBMap->pxDeviceData[u8Id].u8SlaveId == u8SlaveIdBuf[u8i])
											{

												//check response error check
												if(pxMBMap->pxMBQuery[u16Index].xResponseError == err_NoError)

													//if error is Ok than set the flag for the Inverter operation time count
													pxMBMap->pxDeviceData[u8Id].u8ErrorState = 1;


												//else reset the flag
												else

													//reset the error flag
													pxMBMap->pxDeviceData[u8Id].u8ErrorState = 0;


												//compare the inverter scan state value with previous state
												if( (pxMBMap->pxDeviceData[u8Id].u16State != u16ScanData) || (u8Cycle == 0xFF))
												{
													//printf("%s >> Scan State...2 : %d,%d\n",__FUNCTION__,u16ScanData,pxMBMap->pxDeviceData[u8Id].u16State);
													//printf("%s >> ScanIndex...2 : %d\n",__FUNCTION__,pxMBMap->pxDeviceData[u8Id].u8ScanIndex);
													//printf("%s >> Device %d Scan State...2 : %d\n",__FUNCTION__,u8Id,u16ScanData);

													//store the current inverter state into u16State
													pxMBMap->pxDeviceData[u8Id].u16State = u16ScanData;

													//update the inverter state into cyclic buffer
													pxMBMap->pxDeviceData[u8Id].u16ScanData[pxMBMap->pxDeviceData[u8Id].u8ScanIndex] = pxMBMap->pxDeviceData[u8Id].u16State;
													pxMBMap->pxDeviceData[u8Id].u64TimeStamp[pxMBMap->pxDeviceData[u8Id].u8ScanIndex] = pxMBMap->pxMBQuery[u16Index].u64TimeStampUnix;

													//increament the buffer index
													pxMBMap->pxDeviceData[u8Id].u8ScanIndex++;

													//check the index position
													if(pxMBMap->pxDeviceData[u8Id].u8ScanIndex >= 5)
														//reset the index
														pxMBMap->pxDeviceData[u8Id].u8ScanIndex = 0;
												}

												break;
											}
										}
									}

								}
							}
						}
						//				if(u8AddToQueue == 1)
						//				{
						//					while(xQueueSend(*(pxMBMap->xQueue), &u8SlaveIdBuf[u8i], 1) != pdTRUE)
						//					{
						//						vTaskDelay(100/ portTICK_PERIOD_MS);
						//					}
						//				}
						//				vTaskDelay(100/ portTICK_PERIOD_MS);

						if(u8AddPoAtoQue == 1)
						{
							printf("%s >> Adding PoA to Queque\n",__FUNCTION__);
							while(xQueueSend(*(pxMBMap->xPoAQueue), &u8SlaveIdBuf[u8i], 1) != pdTRUE)
							{
								vTaskDelay(100/ portTICK_PERIOD_MS);
							}
							//					printf("%s >> Adding Power Average value to Queque\n",__FUNCTION__);
						}
						// Add Power Average
						if(u8AddPowerAvgtoQue == 1)
						{
							//check response error check
							//					if(pxMBMap->pxMBQuery[u16Index].xResponseError == err_NoError)
							{
								//printf("%25s >> Adding Power Average to Queque\n",__FUNCTION__);
								while(xQueueSend(*(pxMBMap->xPowerAvgQueue), &u8SlaveIdBuf[u8i], 1) != pdTRUE)
								{
									vTaskDelay(100/ portTICK_PERIOD_MS);
								}
							}
							printf("%s >> Adding Power Average value to Queque\n",__FUNCTION__);
						}
						vTaskDelay(100/ portTICK_PERIOD_MS);
					}

				}

				if(u8AddToQueue == 1)
				{
					while(xQueueSend(*(pxMBMap->xQueue), &u8AddToQueue, 1) != pdTRUE)
					{
						vTaskDelay(100/ portTICK_PERIOD_MS);
					}
				}
				vTaskDelay(100/ portTICK_PERIOD_MS);

			}
			//When scanning is stopped
			else
			{
				//set the restart flag
				bRestartFlag = true;

			}
		}
		else
		{
			printf("Scanning Stopped as Device Disconnected from WIFI(waiting 1 Sec).....\n");
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

static void vMbTcpMTCPReq(MbTcpMVariStruct * xRequest, uint8_t *pu8Buf, uint16_t u16timeout, uint8_t *u8DevIP)
{
	int iTcpSock = -1;
	time_t xTimeUnix;
	static uint16_t u16PktCnt = 0;
	uint8_t u8DataLen = 0;
	uint8_t u8Buf[255] = {0};

	//index and total byte count reg for Tx and Rx buff
	uint8_t u8MbTcpMTotalTx;
	uint8_t u8MbTcpMRxIndex,u8MbTcpMTotalRx;

	u8MbTcpMTotalRx = 0;

	//Error type in the Received Repsonse
	errMbTcpMErrorType xResponseResult;

	//wait for inter frame 3.5 character delay in this case
	vTaskDelay(1);

	//reset the Tx buff index
	u8MbTcpMTotalTx=0;

	//Memory clear for the buffer
	memset(pu8Buf, 0, 255);

	//take the Transaction-ID (Trans ID)
	pu8Buf[u8MbTcpMTotalTx++] = (u16PktCnt >> 8);
	pu8Buf[u8MbTcpMTotalTx++] = (u16PktCnt & 0x00FF);
	u16PktCnt ++;

	//take the Protocol-ID (Proto ID - for modbus its Zero)
	pu8Buf[u8MbTcpMTotalTx++] = 0;
	pu8Buf[u8MbTcpMTotalTx++] = 0;
	//take the Length (after this bytes)
	pu8Buf[u8MbTcpMTotalTx++] = 0;
	pu8Buf[u8MbTcpMTotalTx++] = 0;


	//take the Unit-ID (Slave ID)
	pu8Buf[u8MbTcpMTotalTx++] = xRequest->ucSlaveID;
	u8DataLen++;

	//take starting address
	pu8Buf[u8MbTcpMTotalTx++] = xRequest->xFuncType;
	u8DataLen++;

	//take the starting address
	pu8Buf[u8MbTcpMTotalTx++] = xRequest->usStartingAdd.cVal[1];
	pu8Buf[u8MbTcpMTotalTx++] = xRequest->usStartingAdd.cVal[0];
	u8DataLen+=2;

	switch(xRequest->xFuncType)
	{
	case eReadCoil:
	case eReadInput:
		pu8Buf[u8MbTcpMTotalTx++] = xRequest->usNoOfRegs.cVal[1];
		pu8Buf[u8MbTcpMTotalTx++] = xRequest->usNoOfRegs.cVal[0];
		u8DataLen+=2;

		//now fix the reply length
		if( ( xRequest->usNoOfRegs.iVal % 8) == 0 )
		{
			u8MbTcpMTotalRx = 5 + ( ( xRequest->usNoOfRegs.iVal ) >> 3);
		}
		else
		{
			u8MbTcpMTotalRx = 6 + ( ( xRequest->usNoOfRegs.iVal ) >> 3);
		}
		break;


	case eReadHoldReg:
	case eReadInputReg:
		pu8Buf[u8MbTcpMTotalTx++] = xRequest->usNoOfRegs.cVal[1];
		pu8Buf[u8MbTcpMTotalTx++] = xRequest->usNoOfRegs.cVal[0];
		u8DataLen+=2;
		u8MbTcpMTotalRx = 5 + ( ( xRequest->usNoOfRegs.iVal ) << 1);

		break;

	case eWriteSingleCoil:
		//Require implementation
		break;
	case eWriteSingleHReg:
		//Require implementation
		break;
	case eWriteMultiCoil:
		//Require implementation
		break;
	case eWriteMultiHReg:
		//Require implementation
		break;
	default:
		//Require implementation
		break;
	}


	// Add length of data to be sent
	pu8Buf[4] = (u8DataLen >> 8);
	pu8Buf[5] = (u8DataLen & 0x00FF);

	//	printf("%25s >>Data Buffer : ", __func__);
	//	for(int i = 0; i < u8MbTcpMTotalTx ; i++)
	//		printf("%02X ", pu8Buf[i]);
	//	printf("\n");


	// Calculate CRC16 checksum for Modbus-Serial-Line-PDU.
	//	usCRC16.iVal = usMbTcpMCRC16( ( unsigned char * ) pu8Buf, u8MbTcpMTotalTx );
	//	pu8Buf[u8MbTcpMTotalTx++] = ( uint8_t )( usCRC16.cVal[0] );
	//	pu8Buf[u8MbTcpMTotalTx++] = ( uint8_t )( usCRC16.cVal[1] );


#ifdef _MBDEBG_

	printf("%s >> \n", __FUNCTION__);
	for(int i =0 ; i < u8MbTcpMTotalTx; i++)
	{
		printf("[%2d] [%2X]\n", i, pu8Buf[i]);

	}
	printf("%s >> \n", __FUNCTION__);

#endif

	//	//Flush Rx buffer first
	//	uart_flush(xComPort);
	//
	//	//Transmit data
	//	uart_write_bytes(xComPort, (const char *) pu8Buf, u8MbTcpMTotalTx);
	//	//Wait till the data is is trasmitted
	//	uart_wait_tx_done(xComPort, 20/portTICK_PERIOD_MS);
	//
	//	//	for(int i = 0 ; i < u8MbTcpMTotalTx; i++)
	//	//	{
	//	//		printf("Tx >> %2d  %2x \r\n", i, pu8Buf[i]);
	//	//	}

	//variable for holding recieved byte length
	int iRxLen = -1;

	//printf("%25s >> Device IP To be Queried is : %s \n\n", __func__, u8DevIP);
	if( vCreateTCPConnection(u8DevIP,u16timeout, &iTcpSock)  < 0)
	{
		printf("%25s >> Error in TCP socket creation\n", __func__);

		//delay of 1 ms
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
	else
	{

		int err = send(iTcpSock, pu8Buf, u8MbTcpMTotalTx, 0);
		if (err < 0) {
			printf("\n\n%25s >> Error occured during sending: errno %d\n\n", __func__, errno);
			//break;
		}

		//clear the buffer
		memset(pu8Buf, 0, 255);

		//Set the recieve buffer index
		u8MbTcpMRxIndex = 0;

		//Start recieve process
		//Timeout variable
		TickType_t xTmeoutTicks = pdMS_TO_TICKS(u16timeout);
		TickType_t xStartTicks = xTaskGetTickCount();

		iRxLen = recv(iTcpSock, pu8Buf, 255, 0);
		// Error occured during receiving
		if (iRxLen < 0) {
			printf("\n\n%25s >> recv failed: errno %d\n\n", __func__, errno);
		}
		if(TimeoutOccured(xStartTicks, u16timeout))
		{
			printf("\n\n%25s >> Timeout Occured \n\n", __func__);
		}
		//		printf("\n\n%25s >> recv data : ", __func__);
		//		for(int i =0 ; i < iRxLen; i++)
		//		{
		//			printf("%02X ", pu8Buf[i]);
		//		}
		//		printf("\n");
	}
	// Close socket as we create another
	if (iTcpSock != -1) {
		//printf("%25s >> Shutting down socket and restarting...", __func__);
		//printf("%25s >> Close Socket : %d\r\n",__func__,iTcpSock);
		shutdown(iTcpSock, 0);
		close(iTcpSock);
		iTcpSock = 0;

		//delay of 1 ms
		//vTaskDelay(100 / portTICK_PERIOD_MS);
	}

	//Initially setting no error
	xResponseResult = err_NoError;

	if((iRxLen == -1) | (iRxLen == 0))
	{
		xResponseResult = err_Timeout;
	}
	//	else
	//	{
	////		//wait for 1 tick (10 ms ) --> Inter character delay
	////		xTmeoutTicks = 1;
	////		while(u8MbTcpMRxIndex < u8MbTcpMTotalRx)
	////		{
	////			iRxLen = uart_read_bytes(xComPort, &pu8Buf[u8MbTcpMRxIndex++], 1, xTmeoutTicks);
	////			if((iRxLen == -1) | (iRxLen == 0))
	////			{
	////				xResponseResult = err_IncharTimeout;
	////				break;
	////			}
	////		}
	//
	////		printf("\n\n%25s >> Skipping Else part for further reading...\n\n", __func__);
	//	}

	//If there is no error check CRC
	if(xResponseResult == err_NoError)
	{

#ifdef _MBDEBG_
		printf("%s >> Received data \n", __FUNCTION__);
		for(int i = 0; i < u8MbTcpMRxIndex; i++ )
		{
			printf("%s >> [%2d] [%2X]\n",__FUNCTION__, i, pu8Buf[i]);
		}
#endif

		//		//Check for the CRC Error
		//		usCRC16.iVal = usMbTcpMCRC16( ( unsigned char * ) pu8Buf, (u8MbTcpMTotalRx-2));
		//
		//		if((pu8Buf[u8MbTcpMTotalRx-2] == ( unsigned char )( usCRC16.cVal[0] )) &&
		//				(pu8Buf[u8MbTcpMTotalRx-1] == ( unsigned char )( usCRC16.cVal[1] )))
		//		{
		//			xResponseResult = err_NoError;
		//		}
		//		else
		//		{
		//			xResponseResult = err_CRC;
		//		}

	}

	//Check for other modbus error
	//if((xResponseResult != err_NoError) && ((pu8Buf[0x07] & 0x80) != 0))
	if((pu8Buf[0x07] & 0x80)  != 0)
	{
		switch(pu8Buf[0x08])
		{
		//Illegal function
		case MbTcpM_Exce_Ill_Func:
			xResponseResult = err_IllFunc;
			break;

			//Illegal address
		case MbTcpM_Exce_Ill_Addr:
			xResponseResult = err_IllAddr;
			break;

			//Illegal Data
		case MbTcpM_Exce_Ill_Data:
			xResponseResult = err_IllData;
			break;

			//Slave_Fail
		case MbTcpM_Exce_Slave_Fail:
			xResponseResult = err_SerFail;
			break;

			//Acknowlege
		case MbTcpM_Exce_Acknowlege:
			xResponseResult=err_Ack;
			break;

			//Slave busy
		case MbTcpM_Exce_Busy:
			xResponseResult=err_Busy;
			break;

			//Parity Error
		case MbTcpM_Exce_Mem_Parity_Err:
			xResponseResult=err_MemParity;
			break;

			//Gateway path
		case MbTcpM_Exce_Gateway_Path:
			xResponseResult=err_GateFail;
			break;
			//Gateway targe
		case MbTcpM_Exce_Gateway_Target_Unavl:
			xResponseResult=err_GateUnavl;
			break;
		}
	}

#ifdef	_MBDEBG_
	switch(xResponseResult)
	{
	case err_NoError :
		//printf("%s >> Recesption is successful\n", __FUNCTION__);
		break;

	case err_Timeout:
		printf("%s >> Timeout Error \n", __FUNCTION__);
		break;

	case err_IncharTimeout:
		printf("%s >> Inter-character timeout\n", __FUNCTION__);
		break;

	case err_IllFunc:
		printf("%s >> Illegal function \n", __FUNCTION__);
		break;

	case err_IllAddr:
		printf("%s >> Illegal Address \n", __FUNCTION__);
		break;

	case err_IllData:
		printf("%s >> Illegal Data \n", __FUNCTION__);
		break;

	case err_SerFail:
		printf("%s >> Slave Fail \n", __FUNCTION__);
		break;

	case err_Ack:
		printf("%s >> Acknowlege \n", __FUNCTION__);
		break;

	case err_Busy:
		printf("%s >> Slave Busy \n", __FUNCTION__);
		break;

	case err_MemParity:
		printf("%s >> Parity Error\n", __FUNCTION__);
		break;

	case err_GateFail:
		printf("%s >> Gateway Path \n", __FUNCTION__);
		break;
	case err_GateUnavl:
		printf("%s >> Gateway Target\n", __FUNCTION__);
		break;


	default :
		printf("%s Unknown Error\n", __FUNCTION__);
		break;

	}
#endif
	//If no error then upload recieved response
	if(xResponseResult == err_NoError)
	{
		switch(xRequest->xFuncType)
		{
		case eReadCoil:
			break;
		case eReadInput:
			break;
		case eReadHoldReg:
		case eReadInputReg:
			if(xRequest->u16ReceivedData != NULL)
			{
				uint16_t u16Temp;
				for(int i = 0; i < xRequest->usNoOfRegs.iVal ; i++)
				{
					u16Temp = 0;
					//For Modbus RTU
					//u16Temp = ((uint16_t)pu8Buf[3+i*2]) << 8;
					//u16Temp = u16Temp | (uint16_t)pu8Buf[3+1+i*2];
					//Fot Modbus TCP
					u16Temp = ((uint16_t)pu8Buf[9+i*2]) << 8;
					u16Temp = u16Temp | (uint16_t)pu8Buf[9+1+i*2];

					xRequest->u16ReceivedData[i] = u16Temp;
				}
				//memcpy((uint8_t*)xRequest.u16ReceivedData, &pu8Buf[3], pu8Buf[2]);
			}
			break;
		default :
			break;

		}

	}

	//Upload the error status
	xRequest->xResponseError = xResponseResult;

	//upload the timestamp
	time(&xTimeUnix);
	xRequest->u64TimeStampUnix = (uint64_t)xTimeUnix;

}


void vModbusTcpSlaveInit (MBTcpMap *pxMBMap, char *pcName )
{

	//	uart_config_t uart_config = {
	//			.baud_rate = pxMBMap->u32BRate,
	//			.data_bits = pxMBMap->xBits,
	//			.parity    = pxMBMap->xParity,
	//			.stop_bits = pxMBMap->xStopBits,
	//			.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	//	};
	//
	//	uart_param_config(pxMBMap->xComPort, &uart_config);
	//
	//	if(pxMBMap->xComPort == UART_NUM_1)
	//		uart_set_pin(pxMBMap->xComPort, MBUS1_TXD, MBUS1_RXD, MBUS1_RTS, MBUS1_CTS);
	//
	//	if(pxMBMap->xComPort == UART_NUM_2)
	//	{
	//		//assign pins
	//		uart_set_pin(pxMBMap->xComPort, MBUS2_TXD, MBUS2_RXD, MBUS2_RTS, MBUS2_CTS);
	//	}

	//	uart_driver_install(pxMBMap->xComPort, 255 * 2, 0, 0, NULL, 0);
	//	uart_set_mode(pxMBMap->xComPort,UART_MODE_RS485_HALF_DUPLEX);


	xTaskCreate(
			vMbTcpM_Task,
			pcName,
			1024*5,
			pxMBMap,          // Pass the index as a task argument
			5,
			NULL );
}




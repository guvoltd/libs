/*
 * mb_tcp_map.c
 *
 *  Created on: 06-June-2020
 *      Author: urvil
 */

#include "mb_tcp_map.h"
#include "deviceInfo.h"
#include "datamanage.h"
#include "adc.h"


void vInitMap(MBTcpMap * pxMapID,
		MbTcpMVariStruct *xMbQueryEntry,
		uart_port_t xComport, // TODO add TCP socket
		xUARTConf *xUART,    // TODO add TCP socket
		uint8_t *u8buf,
		uint16_t *u16ResultBuf,
		uint16_t u16SizeResultBuf,
		QueueHandle_t * xQueueData,
		QueueHandle_t * xQueue1,
		QueueHandle_t * xPwrAvgQueue)
{
	//First Read the data from the flash and get the query details
	//Read in for loop and set the data in xMbQueryEntry

	//Register count for calculating the offset in storage memory
	uint16_t u16RCount = 0;

	//Length of the map
	//**//	pxMapID->u16Length = xUART->u8MapEntries;

	//Port configuration
	pxMapID->u32BRate = xUART->u32BaudRate;

	pxMapID->xStopBits = (uart_stop_bits_t)xUART->u8StopBits;

	pxMapID->xParity = (uart_parity_t)xUART->u8Parity;

	pxMapID->xBits = (uart_word_length_t)xUART->u8DataBits;

	//set the port
	pxMapID->xComPort = (uart_port_t)xComport;

	//Set buffers
	pxMapID->pu8Buf = u8buf;

	//set timeout
	pxMapID->u16MbTimeout = xUART->u8Timeout * 1000;

	//Set result buffer
	pxMapID->u16MemoryForSave = u16ResultBuf;
	pxMapID->u16MemorySize = u16SizeResultBuf;

	pxMapID->xFastScanTicks = pdMS_TO_TICKS(xUART->u16SamplingSpeedFast*1000*60);
	pxMapID->xMedScanTicks = pdMS_TO_TICKS(xUART->u16SamplingSpeedMed*1000*60);
	pxMapID->xSlowScanTicks = pdMS_TO_TICKS(xUART->u16SamplingSpeedLow*1000*60);

	pxMapID->u8DeviceNo = xUART->u8DeviceEntries;

	//pass the queue handle
	pxMapID->xQueue = xQueueData;

	pxMapID->xPoAQueue = xQueue1;

	pxMapID->xPowerAvgQueue = xPwrAvgQueue;

	printf("%s << No of Device Entry : %d\n",__FUNCTION__,pxMapID->u8DeviceNo);

	xMBDeviceList *xDeviceData = calloc(pxMapID->u8DeviceNo,sizeof(xMBDeviceList));

	uint16_t u16MapSlave=0;   // MAP LENGTH ISSUE Solution

	for(uint8_t u8Id=0; u8Id < xUART->u8DeviceEntries; u8Id++)
	{
		//get slave id
		xDeviceData[u8Id].u8SlaveId = xUART->xDeviceMapObj[u8Id].u8SlaveId;

		//get the deviceId
		strcpy(xDeviceData[u8Id].cDeviceId,xUART->xDeviceMapObj[u8Id].cDeviceId);

		//set device Index
		xDeviceData[u8Id].u8DeviceIndex = u8Id;

		//get the device type
		xDeviceData[u8Id].u8DeviceType = xUART->xDeviceMapObj[u8Id].u8DeviceType;

		//reset PoA flag initially
		xDeviceData[u8Id].bPoAFlag = false;

		//reset Daily Energy flag initiallt
		xDeviceData[u8Id].bDailyEnergy = false;

		//count total modbus map entries
		pxMapID->u16Length += xUART->xDeviceMapObj[u8Id].u16MapEntries;

		for(uint16_t u16Id1=0; u16Id1 < xUART->xDeviceMapObj[u8Id].u16MapEntries; u16Id1++)
		{
			xMbQueryEntry[u16Id1+u16MapSlave].ucSlaveID = xUART->xDeviceMapObj[u8Id].u8SlaveId;
			//			xMbQueryEntry[u16Id1+u16MapSlave].u8DeviceIndex = u8Id;

			//printf("%s >> SlaveID,xMbQueryEntry[%d].ucSlaveID : %d\n",__FUNCTION__,u16Id1+u16MapSlave,xMbQueryEntry[u16Id1+u16MapSlave].ucSlaveID);
			//			printf("%s >> xMbQuery[%d] = %d \n",__FUNCTION__, u8Id1+u16MapSlave,xUART->xDeviceMapObj[u8Id].u8SlaveId);

			//			strcpy(xMbQueryEntry[u16Id1+u16MapSlave].cMapDeviceId, xUART->xDeviceMapObj[u8Id].cDeviceId);
			//			xMbQueryEntry[u16Id1+u16MapSlave].u8DeviceType = xUART->xDeviceMapObj[u8Id].u8DeviceType;
		}
		u16MapSlave += xUART->xDeviceMapObj[u8Id].u16MapEntries;
	}

	printf("%s >> Map Length[%d] : %d\n",__FUNCTION__,xComport,pxMapID->u16Length);

	//set the Daily Energy flag to false  initially
	//	pxMapID->bDailyEnergy = false;

	//maping the modbus map
	for(uint16_t i=0; i < pxMapID->u16Length; i++)
	{

		//if Parameter 400 is enable
		//if(strcmp(pParamName[xUART->xMBMapE[i].u16ParaType],pParamName[47]) == 0)
		if(xUART->xMBMapE[i].u16ParaType == 47)
		{
			for(uint8_t u8Dev=0;u8Dev < xUART->u8DeviceEntries; u8Dev++)
			{

				if(xMbQueryEntry[i].ucSlaveID == xDeviceData[u8Dev].u8SlaveId)
				{
					//enable the PoA Flag
					xDeviceData[u8Dev].bPoAFlag = true;
				}
			}
		}

		//check for the Daily energy Parameter
		if(xUART->xMBMapE[i].u16ParaType == 16 )
		{

			for(uint8_t u8Dev=0;u8Dev < 10; u8Dev++)
			{

				if(xMbQueryEntry[i].ucSlaveID == xDeviceData[u8Dev].u8SlaveId)
				{
					//enable the Daily Energy Flag
					xDeviceData[u8Dev].bDailyEnergy = true;
				}
			}

		}

		if( ( u16RCount + (xUART->xMBMapE[i].u16NoOfReg) ) > pxMapID->u16MemorySize )
		{
			printf("%s >> pxMapId.u16Longth : %d\n",__FUNCTION__,i);
			pxMapID->u16Length = i;
			break;
		}

		//assign slave id
		//		 xMbQueryEntry[i].ucSlaveID = xUART->xDeviceMapObj[0].u8SlaveId;

		//assign function cde
		xMbQueryEntry[i].xFuncType = xUART->xMBMapE[i].u8FunctionCode;

		//		printf("%s >> Index[%d] xUART_FCode[%d] FCode[%d]\n", __FUNCTION__,
		//				i,
		//				xUART->xMBMapE[i].u8FunctionCode,
		//				xMbQueryEntry[i].xFuncType);

		//assign starting address to read
		xMbQueryEntry[i].usStartingAdd.iVal = xUART->xMBMapE[i].u16StartADDR;

		//Find the number of the registers
		xMbQueryEntry[i].usNoOfRegs.iVal = xUART->xMBMapE[i].u16NoOfReg;

		//Data type
		//		xMbQueryEntry[i].u8DataType = xUART->xMBMapE[i].u16ParaType;

		//parameter type
		xMbQueryEntry[i].u16ParameterType = xUART->xMBMapE[i].u16ParaType;


		strcpy(xMbQueryEntry[i].cUnit,xUART->xMBMapE[i].cParaUnit);

		xMbQueryEntry[i].u8DataType = xUART->xMBMapE[i].u8DType;

		xMbQueryEntry[i].fFactor = xUART->xMBMapE[i].fFactor;

		//Switch case for the assigning scanning speed
		//Here directly assign the number of the ticks to be wait
		switch(xUART->xMBMapE[i].u8ScanSpeed)
		{

		case 0:
			xMbQueryEntry[i].u8ScanCategory = eScanSlow;
			if(xUART->xMBMapE[i].u16ParaType == 47)
				u8PoAScanCount = xUART->u16SamplingSpeedFast;
			break;

		case 1:
			xMbQueryEntry[i].u8ScanCategory = eScanMed;
			if(xUART->xMBMapE[i].u16ParaType == 47)
				u8PoAScanCount = xUART->u16SamplingSpeedFast;
			break;

		case 2:
			xMbQueryEntry[i].u8ScanCategory = eScanFast;
			if(xUART->xMBMapE[i].u16ParaType == 47)
				u8PoAScanCount = xUART->u16SamplingSpeedFast;
			break;

		default:
			xMbQueryEntry[i].u8ScanCategory = eScanSlow | eScanMed | eScanFast;

		}

		//Get the memory offset
		xMbQueryEntry[i].u16MemOffset = u16RCount;

		//data saving location in the buffer
		xMbQueryEntry[i].u16ReceivedData = &u16ResultBuf[xMbQueryEntry[i].u16MemOffset];

		u16RCount += xUART->xMBMapE[i].u16NoOfReg;

	}

	pxMapID->u16RCount = u16RCount;
	pxMapID->pxMBQuery = xMbQueryEntry;
	pxMapID->pxDeviceData = xDeviceData;

	//debug POA Count
	printf("%25s >> POA Count : %d\n",__func__,u8PoAScanCount);

}


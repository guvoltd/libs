/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
 */

#ifndef	__MB_TCP_M_H__
#define	__MB_TCP_M_H__


#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h" // TODO TCP SOCKET
#include "esp_err.h"

#include "GenDef.h"


//#define MBUS1_TXD  (GPIO_NUM_23)
//#define MBUS1_RXD  (GPIO_NUM_19)
//#define MBUS1_RTS  (GPIO_NUM_12)
//#define MBUS1_CTS  (UART_PIN_NO_CHANGE)
//
//
//#define MBUS2_TXD  (GPIO_NUM_5)
//#define MBUS2_RXD  (GPIO_NUM_18)
//#define MBUS2_RTS  (GPIO_NUM_13)
//#define MBUS2_CTS  (UART_PIN_NO_CHANGE)

#define MBUSTimeout 1000

/* --------------- Enum and structure used for Variable Map -------------*/
//rtu master response error enum
typedef enum
{
	//error No error response ok
	err_NoError,

	//error occured when no char received after request
	err_Timeout,

	//error when response frame is not complete
	err_IncharTimeout,

	//errro when crc in resopnse doesnot match
	err_CRC,

	//error indicating that we need to check the exception code
	err_Exception,

	//error when slave doesn't supprot the function code
	err_IllFunc,

	//error when address requeste is not allowed in slave
	err_IllAddr,

	//error when containd data fieled is not in range
	err_IllData,

	//error when service is failed in slave
	err_SerFail,

	//error for acknolege(This is not error)
	err_Ack,

	//error for slave busy
	err_Busy,

	//error for memmory parity failed
	err_MemParity,

	//error for gatway path unavilable
	err_GateUnavl,

	//error for gatway target device failed to respond
	err_GateFail

}errMbTcpMErrorType;


//Modbus Master Rtu Function Code type
typedef enum
{
	eReadCoil=0x01,
	eReadInput=0x02,
	eReadHoldReg=0x03,
	eReadInputReg=0x04,
	eWriteSingleCoil=0x05,
	eWriteSingleHReg=0x06,
	eWriteMultiCoil=0x0F,
	eWriteMultiHReg=0x10,
}eMbTcpM_FuncCodeType;

enum
{
	eScanSlow = (1 << 0),
	eScanMed = (1 << 1),
	eScanFast = (1 << 2),
	ePOAScan = (1 << 3),
	eInvStateScan = (1 << 4),
	ePowerAvgScan= (1 << 5)
};
//callback funtion type if required to be called after the request
typedef void (*funcMbTcpMReqCBType)(void);

#define DEVICE_NAME_ID_LENGTH 15

typedef struct _MBDeiveList_
{
	uint64_t u64TimeStamp[5];		//holds the timestamp of the scan query
	char cInvName[DEVICE_NAME_ID_LENGTH+1];              // Inverter Name
	char cDeviceId[DEVICE_NAME_ID_LENGTH+1];				// DeviceId
	uint16_t u16ScanData[5];		//holds last 5 Inverter state scan data (when state change)
	uint16_t u16State;				//current inverter scan data
	uint8_t u8ScanIndex;			//index for the scan data store
	uint8_t u8DeviceIndex;			//device index
	uint8_t u8DeviceType;			//device type
	uint8_t u8SlaveId;				//SlaveId
	uint8_t u8ErrorState;			// Scan error
	uint8_t u8Counter;      		//counter for holding the Inverter scan time count at every 1 minutue
	bool bPoAFlag;					// PoA Flag
	bool bDailyEnergy;				// Manual entry for daily energy flag
	float fLTEnergy;				//holds Lifetime enrgy at every 2 hour for checking if energy generation is continue or fault occured
	TickType_t xTwohourTick;		// Holds the timer tick for checking the lifetime energy at 2 hour
	bool bOnRebootLtStore;			// for storing the lifetime energy for the first time on reboot
	uint8_t u8SendMail;			// Send mail flag on inverter fault occured
}xMBDeviceList;

//Modbus RTU Variable structure
typedef struct
{
	//Slave ID from where this variable synced
	unsigned char ucSlaveID;

	//Type of the vairable (MODBUS Function Type)
	eMbTcpM_FuncCodeType xFuncType;

	//Starting address
	un_i2cval usStartingAdd;

	//Length of the variable
	un_i2cval usNoOfRegs;

	//Data Array Pointer 16bit type
	//un_i2cval * usData;
	uint16_t *u16ReceivedData;

	//stores the response error
	errMbTcpMErrorType xResponseError;

	//latest time stamp when this variables updated (in case of write)
	uint64_t u64TimeStampUnix;

	//function pointer if call back is requred after the request
	funcMbTcpMReqCBType funcMbTcpMReqCB;

	//Add offset for storing result
	uint16_t u16MemOffset;

	//Data type
	uint8_t	u8DataType;

	//scanning catagory i.e fast/slow/medium
	uint8_t	 u8ScanCategory; 	//0 - slow	1-med	2-fast

	//Parameter type
	uint16_t  u16ParameterType;

	//unit type
	char cUnit[16];

	//Multiplication Factor
	float fFactor;


}MbTcpMVariStruct;

typedef struct _MBTcpMap_
{
	MbTcpMVariStruct * pxMBQuery;			//Query structure pointer
	uint16_t		   u16Length;			//Number of the modbus queries in the pointer

	uart_port_t		   xComPort;			//Com Port on which the modbus is to be execute
	uint8_t			 * pu8Buf;				//buffer of 256 byte for hodling data temporary
	uint32_t			u32BRate;			//Baurd rate of comport
	uart_word_length_t	xBits;				//Number of bits
	uart_stop_bits_t	xStopBits;			//Number of stop bits
	uart_parity_t		xParity;			//Parity information
	uint8_t				u8char35delay; 		//3.5 character delay in ms

	uint16_t		 *	u16MemoryForSave;	//Memory for saving read results
	uint16_t			u16MemorySize;		//No of 16-bit location in the memory that saves results
	uint16_t			u16RCount;

	uint16_t			u16MbTimeout;
	uint8_t				u8DataType;

	TickType_t			xFastScanTicks;
	TickType_t			xSlowScanTicks;
	TickType_t			xMedScanTicks;

	uint8_t u8DeviceNo;

	xMBDeviceList *pxDeviceData;

	QueueHandle_t	 *	xQueue;
	QueueHandle_t	 *	xPoAQueue;
	QueueHandle_t	 *	xPowerAvgQueue;

}MBTcpMap;

extern char cLoggerIPPart[16];

void vModbusTcpSlaveInit (MBTcpMap *pxMBMap, char *pcName);

//used to update a Modbus Variable
void vMbTcpMUpdateVariable ( MbTcpMVariStruct xRequest );

int vCreateTCPConnection(uint8_t *u8DevIP,uint16_t u16Timeout, int *iTcpSock);
#endif
/* [] END OF FILE */

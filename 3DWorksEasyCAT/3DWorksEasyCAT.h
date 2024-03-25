#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project 3DWorksEasyCAT.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	24
#define CUST_BYTE_NUM_IN	40
#define TOT_BYTE_NUM_ROUND_OUT	24
#define TOT_BYTE_NUM_ROUND_IN	40


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		uint32_t    outDigitalSet0;
		uint32_t    outDigitalSet1;
		uint8_t     outAnalog00;
		uint8_t     outAnalog01;
		uint8_t     outAnalog02;
		uint8_t     outAnalog03;
		uint8_t     outAnalog04;
		uint8_t     outAnalog05;
		uint8_t     outAnalog06;
		uint8_t     outAnalog07;
		uint8_t     outAnalog08;
		uint8_t     outAnalog09;
		uint8_t     outAnalog10;
		uint8_t     outAnalog11;
		uint8_t     outAnalog12;
		uint8_t     outAnalog13;
		uint8_t     outAnalog14;
		uint8_t     outAnalog15;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		uint32_t    inDigitalSet0;
		uint32_t    inDigitalSet1;
		uint32_t    inToggleSet0;
		uint32_t    inToggleSet1;
		int32_t     mpg0;
		int32_t     mpg1;
		uint8_t     inAnalog00;
		uint8_t     inAnalog01;
		uint8_t     inAnalog02;
		uint8_t     inAnalog03;
		uint8_t     inAnalog04;
		uint8_t     inAnalog05;
		uint8_t     inAnalog06;
		uint8_t     inAnalog07;
		uint8_t     inAnalog08;
		uint8_t     inAnalog09;
		uint8_t     inAnalog10;
		uint8_t     inAnalog11;
		uint8_t     inAnalog12;
		uint8_t     inAnalog13;
		uint8_t     inAnalog14;
		uint8_t     inAnalog15;
	}Cust;
} PROCBUFFER_IN;

#endif
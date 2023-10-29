/*
 *  usb2084_pi.c
 *
 *  v 0.0.0 2014.4.15 by Winson Chen
 *
 *  create
 *
 */

#include "ICPDAS_USBIO.h"

int main()
{
	int res,i;
	int DevNum;
	BYTE BoardID = 0x1;
	
	printf("USB I/O Library version: %s\n",USBIO_GetLibraryVersion());

	res = USBIO_OpenDevice(BoardID, &DevNum);

	if(res)
	{
		printf("open /dev/hidraw%d failed! Erro: %d\r\n",DevNum,res);
		return 0;
	}

//	USBIO_LoadDefault(DevNum);

/*
	BYTE tst;
	USBIO_PI_GetTotalSupportType(DevNum, &tst);
	printf("Total support type amount is %d\n",tst);
*/
/*
	BYTE o_bySupportTypeCode[8];

	USBIO_PI_GetSupportTypeCode(DevNum, o_bySupportTypeCode);
	for(i = 0; i < 8; i++)
		printf("Support type code is 0x%02x\n",o_bySupportTypeCode[i]);
*/
/*
	BYTE gtc[8],stc[8];
        for(i = 0; i < 8; i++)
		stc[i] = 0x50;
	USBIO_PI_SetTypeCode(DevNum, stc);
	//USBIO_PI_SetTypeCodeToChannel(DevNum, 0, 0x55);
	USBIO_PI_GetTypeCode(DevNum, gtc);  //default is 0x50
	for(i = 0; i < 8; i++)
		printf("Type code is 0x%02x\n",gtc[i]);
*/
/*
	BYTE tmode[8],smode[8];
	for(i = 0; i < 8; i++)
		smode[i] = 0;
	USBIO_PI_SetTriggerMode(DevNum, smode);
	USBIO_PI_SetTriggerModeToChannel(DevNum, 0, 0);
	USBIO_PI_GetTriggerMode(DevNum, tmode);  //default is 0
	for(i = 0; i < 8; i++)
		printf("Trigger Mode is 0x%02x\n",tmode[i]);
*/
/*
	BYTE gflag[2], sflag[2];
        for(i = 0 ;i < 2; i++)
		sflag[i] = 0x5b;
	USBIO_PI_SetChIsolatedFlag(DevNum, sflag);
	USBIO_PI_SetChIsolatedFlagToChannel(DevNum, 5, 0x1);
	USBIO_PI_GetChIsolatedFlag(DevNum, gflag);  //default is 0
	for(i = 0 ;i < 2; i++)
		printf("Flag is %2x\n",gflag[i]);
*/
/*
	BYTE gLPEnable[1],sLPEnable[1];
	sLPEnable[0] = 0x00;
	USBIO_PI_SetLPFilterEnable(DevNum, sLPEnable);
	USBIO_PI_SetLPFilterEnableToChannel(DevNum, 5, 0x0);
	USBIO_PI_GetLPFilterEnable(DevNum, gLPEnable);  //default is 0
		printf("LPFilterEnable %02x\n",gLPEnable[0]);
*/
/*
	WORD gLPWidth[8], sLPWidth[8];
	for(i = 0 ;i < 8; i++)
		 sLPWidth[i] = 1;
	USBIO_PI_SetLPFilterWidth(DevNum, sLPWidth);
	USBIO_PI_SetLPFilterWidthToChannel(DevNum, 4, 1);
	USBIO_PI_GetLPFilterWidth(DevNum, gLPWidth);  //default is 1
	for(i = 0; i < 8; i ++)
		printf("LPFilterWidth is %d\n",gLPWidth[i]);
*/

	DWORD PIValue[8];
	BYTE ChStatus[8],ClrMask[1];
	//ClrMask[0] = 0xff;  //select clear which Channel
	USBIO_PI_ReadValue(DevNum, PIValue, ChStatus);
	//USBIO_PI_ClearSingleChCount(DevNum, 0);
	//USBIO_PI_ClearChCount(DevNum, ClrMask);
	//USBIO_PI_ClearSingleChStatus(DevNum, 0);
	for(i = 0; i < 8; i++)
		printf("PIValue is %d Status %d\n",PIValue[i], ChStatus[i]);

/*
	DWORD CntValue[8];
	BYTE ChaStatus[8];
	USBIO_PI_ReadCntValue(DevNum, CntValue, ChaStatus);
	USBIO_PI_ClearSingleChStatus(DevNum, 0);  //no use
	for(i = 0; i < 8; i++)
		printf("CntValue is %d Status %d\n",CntValue[i],ChaStatus[i]);
*/
/*
	float FreqValue[8];
	BYTE ChStatus[8];
	USBIO_PI_ReadFreqValue(DevNum, FreqValue, ChStatus);
	USBIO_PI_ClearSingleChStatus(DevNum, 2);  //test is no use
	for(i = 0; i < 8; i++)
		printf("FreqValue is %.5f Status %d\n",FreqValue[i], ChStatus[i]);
*/


	res = USBIO_CloseDevice(DevNum);

        if(res)
        {
                printf("close /dev/hidraw%d failed! Erro: %d\r\n",DevNum,res);
                return 0;
        }

	return 0;
}

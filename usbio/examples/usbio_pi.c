/*
 *  usb2084_pi.c
 *
 *  v 0.0.0 2014.5.5 by Winson Chen
 *
 *  create
 *
 */

#include"ICPDAS_USBIO.h"

int main()
{
	usleep(1000);
	int res,i;
	int DevNum;
	BYTE BoardID = 0x1;
	BYTE total_pi,TypeCode[8],ChStatus[8];
	BYTE status[3][12] = {"CHSTA_GOOD","CHSTA_OVER","CHSTA_UNDER"};
			   /*0x50	  0x51	      0x54		0x55		  0x56*/
	BYTE type[5][20] = {"Up counter","Frequency","Up/Down counter","Pulse/Direction","AB phase"};
	DWORD PIValue[8] = {0};
		
	printf("USB I/O Library version: %s\n",USBIO_GetLibraryVersion());

	res = USBIO_OpenDevice(BoardID, &DevNum);

	if(res)
	{
		printf("open /dev/hidraw%d failed! Erro: %d\r\n",DevNum,res);
		return 0;
	}
	//USBIO_SetUserDefinedBoardID(DevNum, 50);
	USBIO_GetPITotal(DevNum, &total_pi);
	printf("USB-2084 PI Number : %d\n\n",total_pi);
	/*BYTE gtc[8],stc[8];
        for(i = 0; i < 8; i++)
               stc[i] = 0x50;
	
	USBIO_PI_SetTypeCode(DevNum, stc);
	*/
	//USBIO_PI_SetTypeCodeToChannel(DevNum, 6, 0x50);

	USBIO_PI_GetTypeCode(DevNum, TypeCode);
	USBIO_PI_ReadValue(DevNum, PIValue, ChStatus);

	printf("        Type Code       Value      Status\n");

	for(i = 0; i < total_pi; i++)
	{
		if(TypeCode[i] >= 0x54)
			TypeCode[i] -= 0x52;
		else
			TypeCode[i] -= 0x50;

		printf("Ch%d %*s, %*d,   %s\n", i, 16, type[TypeCode[i]], 7, PIValue[i], status[ChStatus[i]]);
	}


	res = USBIO_CloseDevice(DevNum);

	if(res)
	{
		printf("close /dev/hidraw%d failed! Erro: %d\r\n",DevNum,res);
		return 0;
	}

	return 0;
}


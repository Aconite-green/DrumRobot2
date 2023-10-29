/*
 *  usbio_ai.c
 *
 *  v 0.0.0 2018.8.7 by Winson Chen
 *
 *    create
 *
 */

#include "Global.h"

int main()
{
	int res, DevNum, i;
        int DeviceID = USB2026;
        BYTE BoardID = 0x1;
        BYTE module_name[15], total_ai, o_byAIChStatus[USBIO_AI_MAX_CHANNEL];
	float o_fAIValue[USBIO_AI_MAX_CHANNEL];
	DWORD o_dwAIValue[USBIO_AI_MAX_CHANNEL];
	BYTE status[3][15] = {"Channel Good","Channel Over","Channel Under"};


	printf("USB I/O Library Version : %s\n", USBIO_GetLibraryVersion());
	usleep(100);
	res = USBIO_OpenDevice(DeviceID, BoardID, &DevNum);

        if (res)
        {
		printf("open Device failed! Erro : 0x%x\r\n", res);
                return 0;
        }

	USBIO_ModuleName(DevNum, module_name);

	USBIO_GetAITotal(DevNum, &total_ai);
	printf("%s AI Number : %d\n", module_name, total_ai);

	USBIO_AI_ReadValueFloat(DevNum, o_fAIValue);
	printf("Analog input float value:\n");
	for(i = 0; i < total_ai; i++)
	{
		printf("CH%2d %.5f  ", i, o_fAIValue[i]);
		if(i % 4  == 3)
			printf("\n");

	}
	printf("\n");

	USBIO_AI_ReadValueHex(DevNum, o_dwAIValue);

	printf("Analog input Hex value:\n");
	for(i = 0; i < total_ai; i++)
	{
		printf("CH%2d 0x%08x ", i, o_dwAIValue[i]);
		if(i % 4  == 3)
			printf("\n");
	}
	printf("\n");

	USBIO_AI_ReadValueFloatWithChSta(DevNum, o_fAIValue, o_byAIChStatus);

	printf("Analog input float value with channel status:\n");
	for(i = 0; i < total_ai; i++)
	{
		printf("CH%2d %.5f, %s  ", i, o_fAIValue[i], status[o_byAIChStatus[i]]);
		if(i % 4  == 3)
			printf("\n");
	}
	printf("\n");

	USBIO_AI_ReadValueHexWithChSta(DevNum, o_dwAIValue, o_byAIChStatus);
	printf("Analog input Hex value with channel status:\n");
	for(i = 0; i < total_ai; i++)
	{
		printf("CH%2d %08x, %s  ", i, o_dwAIValue[i], status[o_byAIChStatus[i]]);
		if(i % 4  == 3)
			printf("\n");
	}
	printf("\n");

	res = USBIO_CloseDevice(DevNum);

	if(res)
	{
		printf("close %s with Board iD %d failed! Erro : %d\r\n", module_name, BoardID, res);
                return 0;
        }

	return 0;
}

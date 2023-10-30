/*
 *  usbio_do.c
 *
 *  v 0.0.0 2018.7.31 by Winson Chen
 *
 *  create
 *
 */

#include "Global.h"

int main()
{
        int res, DevNum, i;
	//int DeviceID = USB2055;
	int DeviceID = USB2026;
	BYTE BoardID = 0x1;
	BYTE total_di, module_name[15];
	DWORD DIValue, o_dwDICntValue[USBIO_DI_MAX_CHANNEL];

        printf("USB I/O Library Version : %s\n", USBIO_GetLibraryVersion());
	
	res = USBIO_OpenDevice(DeviceID, BoardID, &DevNum);

        if (res)
        {
		printf("open Device failed! Erro : 0x%x\r\n", res);
                return res;
        }

	printf("Demo usbio_di DevNum = %d\n", DevNum);
	USBIO_ModuleName(DevNum, module_name);

	USBIO_GetDITotal(DevNum, &total_di);
	printf("%s DI number: %d\n\n",module_name, total_di);

	while(1)
	{
		printf("Press ESC to exit.\n\n");

		if(getchar() == 27)
			break;

		USBIO_DI_ReadValue(DevNum, &DIValue);

	        for(i = 0; i < total_di; i++)
        	{
	                if((DIValue >> i) & 1)
                        	printf("Ch%2d DI  On   ",i);
                	else
        	                printf("Ch%2d DI Off   ",i);

			if(i % 4  == 3)
				printf("\n");
	        }
		printf("\n");

		printf("Each DI channel counter value:\n");
		USBIO_DI_ReadCounterValue(DevNum, o_dwDICntValue);

		for(i = 0; i < total_di; i++)
		{
			printf("CH%2d  %11u   ", i, o_dwDICntValue[i]);

			if(i % 8  == 7)
				printf("\n");
		}

	}
        res = USBIO_CloseDevice(DevNum);

        if (res)
        {
		printf("close %s with Board iD %d failed! Erro : %d\r\n", module_name, BoardID, res);
                return res;
        }

	return 0;
}

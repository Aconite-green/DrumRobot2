/*
 *  usbio_do.c
 *
 *  v 0.0.0 2018.7.31 by Winson Chen
 *
 *  create
 *
 */

#include "ICPDAS_USBIO.h"

int main()
{
        int res, i = 0, DevNum;
	int DeviceID = USB2026;
	//int DeviceID = USB2045_32;
	BYTE BoardID = 0x1;
	DWORD DOValue = 0;
	BYTE module_name[15], total_do;

        printf("USB I/O Library Version : %s\n", USBIO_GetLibraryVersion());
	
	res = USBIO_OpenDevice(DeviceID, BoardID, &DevNum);

	if (res)
	{
		printf("open Device failed! Erro : 0x%x\r\n", res);
		return res;
	}

	USBIO_ModuleName(DevNum, module_name);

	USBIO_GetDOTotal(DevNum, &total_do);
	printf("%s DO number: %d\n",module_name, total_do);

		DOValue = 0;
	while(1)
	{
		printf("Press ESC to exit.\n\n");

		printf("Enter DO value(0~%.0f):",pow(2,total_do)-1);
		scanf("%u",&DOValue);
		if(getchar() == 27)
			break;
		
		res = USBIO_DO_WriteValue(DevNum, &DOValue);
                if(res)
                {
                        printf("API USBIO_DO_WriteValue error 0x%x \n", res);
                        return res;
                }
		DOValue++;
		if(DOValue == 64)
			DOValue = 0;

		usleep(200000);
	}

	DOValue = 0;        //Disable all DO channel
        USBIO_DO_WriteValue(DevNum, &DOValue);

        res = USBIO_CloseDevice(DevNum);

        if (res)
        {
                printf("close %s with Board iD %d failed! Erro : %d\r\n", module_name, BoardID, res);
                return res;
        }

	return 0;
}

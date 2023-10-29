/*
 *  usbio_do_channel.c
 *
 *  v 0.0.0 2018.8.6 by Winson Chen
 *
 *  create
 *
 */

#include "ICPDAS_USBIO.h"

int main()
{
        int res, DevNum, i, DOChannel, DOStatus;
	int DeviceID = USB2045_32;
	BYTE BoardID = 0x1;
	BYTE TotalDo, module_name[15];
	DWORD DOValue, do_conf, tmp;

        printf("USB I/O Library Version : %s\n", USBIO_GetLibraryVersion());
	
	res = USBIO_OpenDevice(DeviceID, BoardID, &DevNum);

        if (res)
        {
		printf("open Device failed! Erro : 0x%x\r\n", res);
                return res;
        }

	USBIO_ModuleName(DevNum, module_name);

	USBIO_GetDOTotal(DevNum, &TotalDo);
	printf("%s DO number: %d\n",module_name, TotalDo);

	while(1)
	{
		printf("Press ESC to exit.\n\n");

		printf("Enter DO value(0~%d):", TotalDo-1);
		scanf("%d",&DOChannel);
		if(getchar() == 27)
			break;
		printf("DO channel %d status(0 Disable, 1 Enable):",DOChannel);
		scanf("%d",&DOStatus);

		res = USBIO_DO_WriteValueToChannel(DevNum, DOChannel, DOStatus);
                if(res)
                {
                        printf("API USBIO_DO_WriteValueToChannel error\n");
                        return res;
                }

		res = USBIO_DO_ReadValue(DevNum, &do_conf);
		if(res)
		{
			printf("API USBIO_DO_ReadValue error 0x%x\n", res);
			return res;
		}

		printf("Do read back enable status\n");
		for(i = 0; i < TotalDo; i++)
		{
			tmp = (do_conf >> i) & 0x1;
			if(tmp)
				printf("CH%2d Enable\n", i);
			//else
			//	printf("CH%2d Disable", i);
		}
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

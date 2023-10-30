/*
 *  usbio_do.c
 *
 *  v 0.0.0 2018.7.31 by Winson Chen
 *
 *  create
 *
 */

#include "Global.h"
void Show();

int main()
{
        int res, DevNum, i, ch, options;
	int DeviceID = USB2026;
	BYTE BoardID = 0x1;
	BYTE module_name[15], total_di;
	DWORD o_dwDICntValue[USBIO_DI_MAX_CHANNEL], i_dwCntClrMask, o_dwEdgeTrig, i_dwEdgeTrig, i_dwInverse, o_dwInverse;
	WORD i_wFilterWidth, o_wFilterWidth;

        printf("USB I/O Library Version : %s\n", USBIO_GetLibraryVersion());
	
	res = USBIO_OpenDevice(DeviceID, BoardID, &DevNum);

        if (res)
        {
		printf("open Device failed! Erro : 0x%x\r\n", res);
                return res;
        }

	USBIO_ModuleName(DevNum, module_name);
	USBIO_GetDITotal(DevNum, &total_di);
	printf("%s DI number: %d\n",module_name, total_di);

	Show();
	while(1)
	{
		printf("\nPress ESC to exit.\n\n");
		
		scanf("%d",&options);
		if(getchar() == 27)
			break;

		switch(options)
		{
			case 1:
        			printf("Write Clear Counter By Channel (0~%d):",total_di-1);
				scanf("%d", &ch);
				if(ch < 0 || ch > total_di-1)
				{
					printf("Wrong value\n");
					break;
				}

				USBIO_DI_WriteClearCounter(DevNum, ch);

				printf("Each DI channel counter value:\n");
				USBIO_DI_ReadCounterValue(DevNum, o_dwDICntValue);

				for(i = 0; i < total_di; i++)
				{
					printf("CH%2d  %11u   ", i, o_dwDICntValue[i]);

					if(i % 4  == 3)
						printf("\n");
				}
			break;
			case 2:
				printf("Write Clear Counter By Mask (0~%.0f):",pow(2,total_di)-1);
				scanf("%u",&i_dwCntClrMask);
				USBIO_DI_WriteClearCounterByMask(DevNum, i_dwCntClrMask);

				printf("Each DI channel counter value:\n");
				USBIO_DI_ReadCounterValue(DevNum, o_dwDICntValue);

				for(i = 0; i < total_di; i++)
                                {
                                        printf("CH%2d  %1u   ", i, o_dwDICntValue[i]);

                                        if(i % 4  == 3)
                                                printf("\n");
                                }
				
			break;
			case 3:
				printf("Set All Channel Counter Edge Trigger:(0 Falling, 1 Rising):");
				scanf("%u",&i_dwEdgeTrig);

				if(i_dwEdgeTrig < 0 || i_dwEdgeTrig > 1)
				{
					printf("Wrong value.\n");
					break;
				}
				USBIO_DI_SetCntEdgeTrigger(DevNum, i_dwEdgeTrig);

			break;
			case 4:
				printf("Get All channel Counter Edge Trigger\n");
				USBIO_DI_GetCntEdgeTrigger(DevNum, &o_dwEdgeTrig);
				if(o_dwEdgeTrig)
					printf("Rising\n");
				else
					printf("Falling\n");
				
			break;
			case 5:
				printf("Set Digital Filter Width(unit 0.1ms):");
				scanf("%hd",&i_wFilterWidth);
				USBIO_DI_SetDigitalFilterWidth(DevNum, i_wFilterWidth);
			break;
			case 6:
				printf("Get Digital Filter Width(unit 0.1ms):\n");
				USBIO_DI_GetDigitalFilterWidth(DevNum, &o_wFilterWidth);
				printf("%d\n",o_wFilterWidth);
			break;
			case 7:
				printf("Set Digital Value Inverse(0 no Inverse, 1 Inverse):");
				scanf("%d",&i_dwInverse);
				if(i_dwInverse < 0 || i_dwInverse > 1)
				{
					printf("Wrong value.\n");
					break;
				}
				USBIO_DI_SetDigitalValueInverse(DevNum, i_dwInverse);
			break;
			case 8:
				printf("Get Digital Value Inverse\n");
				USBIO_DI_GetDigitalValueInverse(DevNum, &o_dwInverse);
				if(o_dwInverse)
					printf("DI Value Inverse.\n");
				else
					printf("DI Value not Inverse.\n");
			break;
			default:
				Show();
			break;
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

void Show()
{
	printf("Digital input configure options.\n");
	printf("1. Write Clear Counter By Channel\n");
	printf("2. Write Clear Counter By Mask\n");
	printf("3. Set All Channel Counter Edge Trigger\n");
	printf("4. Get All channel Counter Edge Trigger\n");
	printf("5. Set Digital Filter Width\n");
	printf("6. Get Digital Filter Width\n");
	printf("7. Set Digital Value Inverse\n");
	printf("8. Get Digital Value Inverse\n");
	printf("Others number show this list.\n");
}

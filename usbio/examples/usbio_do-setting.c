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
        int res, DevNum, i, j, ch, status;
	int DeviceID = USB2026;
	//int DeviceID = USB2045_32;
	BYTE BoardID = 0x1;
	BYTE module_name[15], byChPwrOn[USBIO_DO_MAX_CHANNEL], total_do;
	BYTE i_bySafetyEnable[(USBIO_DO_MAX_CHANNEL + 7) / 8], o_bySafetyEnable[(USBIO_DO_MAX_CHANNEL + 7) / 8];
	BYTE i_bySafetyValue[(USBIO_DO_MAX_CHANNEL + 7) / 8], o_bySafetyValue[(USBIO_DO_MAX_CHANNEL + 7) / 8];
	DWORD options, i_dwInverse, o_dwInverse;

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

	Show();

	while(1)
	{
		printf("\nPress ESC to exit.\n\n");
		
		scanf("%u",&options);
		if(getchar() == 27)
			break;

		switch(options)
		{
			case 1:
				printf("Set PowerOn Enable\n");
				printf("Input power on enable value(0~%.0f):",pow(2,total_do)-1);
				scanf("%u",&options);
				if(options > pow(2,total_do)-1 || options < 0)
				{
					printf("Invalud value\n");
					break;
				}
				for(i = 0; i < total_do; i++)
					byChPwrOn[i] = (options >> i) & 0x1;
				USBIO_DO_SetPowerOnEnable(DevNum, byChPwrOn);
			break;
			case 2:
				printf("Set PowerOn Enable To Channel\n");
				printf("select channel:");
				scanf("%d",&ch);
				printf("Enable 1/Disable 0:");
				scanf("%d",&status);
				USBIO_DO_SetPowerOnEnableToChannel(DevNum, ch, status);
			break;
			case 3:
				printf("Get PowerOn Enable value\n");
				USBIO_DO_GetPowerOnEnable(DevNum, byChPwrOn);
				for(i = 0; i < total_do; i++)
				{
					printf("CH%2d%2d  ",i, byChPwrOn[i]);

					if(i % 8  == 7)
						printf("\n");
				}
			break;
			case 4:
				printf("Set Safety Enable");
				printf("Input safety enable value(0~%.0f):",pow(2,total_do)-1);
				scanf("%u",&options);
				if(options > pow(2,total_do)-1 || options < 0)
				{
					printf("Invalud value\n");
					break;
				}

				for(i = 0; i < (total_do + 7) >> 3; i++)
					i_bySafetyEnable[i] = (options >> i * 8) & 0xff;

				USBIO_DO_SetSafetyEnable(DevNum, i_bySafetyEnable);
			break;
			case 5:
				j = 0;
				printf("Get Safety Enable\n");
				USBIO_DO_GetSafetyEnable(DevNum, o_bySafetyEnable);
				for(i = 0; i < (total_do + 7) >> 3 ; i++)
				{
					for(ch = 0; ch < 8; ch++)
					{
						status = (o_bySafetyEnable[i] >> ch) & 0x1;
						if(status)
							printf("CH%02d.  Enable  ", j);
						else
							printf("CH%02d. Disable  ", j);
						j++;
					}
					printf("\n");
				}
			break;
			case 6:
				printf("Set Safety Value\n");
				printf("Input safety value(0~%.0f):",pow(2,total_do)-1);
				scanf("%u",&options);
				if(options > pow(2,total_do)-1 || options < 0)
				{
					printf("Invalud value\n");
					break;
				}

				for(i = 0; i < (total_do + 7) >> 3; i++)
					i_bySafetyValue[i] = (options >> i * 8) & 0xff;

				USBIO_DO_SetSafetyValue(DevNum, i_bySafetyValue);
			break;
			case 7:
				j = 0;
				printf("Get Safety Value\n");
				USBIO_DO_GetSafetyValue(DevNum, o_bySafetyValue);
				for(i = 0; i < (total_do + 7) >> 3 ; i++)
				{
					for(ch = 0; ch < 8; ch++)
					{
						status = (o_bySafetyValue[i] >> ch) & 0x1;
						if(status)
							printf("CH%02d.  On  ", j);
						else
							printf("CH%02d. Off  ", j);
						j++;
					}
					printf("\n");
				}
			break;
			case 8:
				printf("Set Digital Output Inverse(0 no inverse, 1 yes):");
				scanf("%u",&i_dwInverse);
				USBIO_DO_SetDigitalOutputInverse(DevNum, i_dwInverse);
			break;
			case 9:
				printf("Get Digital Output Inverse\n");
				USBIO_DO_GetDigitalOutputInverse(DevNum, &o_dwInverse);
				if(o_dwInverse)
					printf("Digital Output is inverse\n");
				else
					printf("Digital Output is not inverse\n");
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
	printf("Digital output configure options.\n");
	printf("1. Set PowerOn Enable\n");
	printf("2. Set PowerOn Enable To Channel\n");
	printf("3. Get PowerOn Enable\n");
	printf("4. Set Safety Enable\n");
	printf("5. Get Safety Enable\n");
	printf("6. Set Safety Value\n");
	printf("7. Get Safety Value\n");
	printf("8. Set Digital Output Inverse\n");
	printf("9. Get Digital Output Inverse\n");
	printf("Others number show this list.\n");
}

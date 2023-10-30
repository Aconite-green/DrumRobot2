/*
 *  usbio_module_info.c
 *
 *  v 0.0.0 2018.8.8 by Winson Chen
 *
 *    create
 *
 */

#include "Global.h"
void Show();

int main()
{
	int res, DevNum, i, options, val;
        int DeviceID = USB2026;
        //int DeviceID = USB2019;
        BYTE BoardID = 0x1;
        BYTE module_name[15], o_bySupportIOMask, total_ch, i_byBID;
	DWORD DID, o_dwCommTimeout, i_dwCommTimeout, o_dwSoftWDTTimeout, i_dwSoftWDTTimeout;
	WORD o_wFwVer;
	BYTE IO_type[6][3] = {"DO","DI","AO","AI","PO","PI"};
	BYTE o_byDeviceNickName[USBIO_NICKNAME_LENGTH] = {0}, byNickName[USBIO_NICKNAME_LENGTH] = {0};;
	BOOL i_bEnable;

	printf("USB I/O Library Version : %s\n", USBIO_GetLibraryVersion());

	res = USBIO_OpenDevice(DeviceID, BoardID, &DevNum);

        if (res)
        {
		printf("open Device failed! Erro : 0x%x\r\n", res);
                return 0;
        }

	USBIO_ModuleName(DevNum, module_name);

	USBIO_GetSupportIOMask(DevNum, &o_bySupportIOMask);
	printf("%s support IO type:\n", module_name);
	for(i = 0; i < 6; i++)
	{
		val = (o_bySupportIOMask >> i) & 0x1;
		if(val)
		{
			printf("%s ",IO_type[i]);
			switch(i)
			{
				case 0:
					USBIO_GetDOTotal(DevNum, &total_ch);
				break;
				case 1:
					USBIO_GetDITotal(DevNum, &total_ch);
				break;
				case 2:
					USBIO_GetAOTotal(DevNum, &total_ch);
				break;
				case 3:
					USBIO_GetAITotal(DevNum, &total_ch);
				break;
				case 4:
					USBIO_GetPOTotal(DevNum, &total_ch);
				break;
				case 5:
					USBIO_GetPITotal(DevNum, &total_ch);
				break;
			}

			printf("%d channels\n", total_ch);
		}
	}
	printf("\n");

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
				printf("Set User Defined BoardID(valid value 16~127)\n");
				printf("Your rotary switch must now point to 0, this setting will succeed.\n");
				scanf("%hhd",&i_byBID);
				USBIO_SetUserDefinedBoardID(DevNum, i_byBID);
				printf("You have to power off %s module if you want to use new Board ID.", module_name);
			break;
			case 2:
				printf("Get Firmware Version\n");
				USBIO_GetFwVer(DevNum, &o_wFwVer);
				printf("%d\n",o_wFwVer);
			break;
			case 3:
				printf("Get Device ID\n");
				USBIO_GetDeviceID(DevNum, &DID);
				printf("0x%x\n",DID);
			break;
			case 4:
				printf("Set Device NickName:\n");
				scanf("%s",byNickName);
				USBIO_SetDeviceNickName(DevNum, byNickName);
			break;
			case 5:
				printf("Get Device NickName:\n");
				USBIO_GetDeviceNickName(DevNum, o_byDeviceNickName);
				printf("%s\n", o_byDeviceNickName);
			break;
			case 6:
				printf("Set Communication Timeout(unit ms):\n");
				scanf("%u",&i_dwCommTimeout);
				USBIO_SetCommTimeout(DevNum, i_dwCommTimeout);

			break;
			case 7:
				printf("Get Communication Timeout\n");
				USBIO_GetCommTimeout(DevNum, &o_dwCommTimeout);
				printf("%ums\n", o_dwCommTimeout);
			break;
			case 8:
				printf("Set AutoReset WDT(0 Disable, 1 Enable):\n");
				scanf("%d",&i_bEnable);
				USBIO_SetAutoResetWDT(DevNum, i_bEnable);
			break;
			case 9:
				printf("Set Soft WDT Timeout(100~1800000 unit ms):\n");
				scanf("%u",&i_dwSoftWDTTimeout);
				USBIO_SetSoftWDTTimeout(DevNum, i_dwSoftWDTTimeout);

			break;
			case 10:
				printf("Get Soft WDT Timeout\n");
				USBIO_GetSoftWDTTimeout(DevNum, &o_dwSoftWDTTimeout);
				printf("%ums",o_dwSoftWDTTimeout);
			break;
			default:
				Show();
			break;
		}
	}

	res = USBIO_CloseDevice(DevNum);

	if(res)
	{
		printf("close %s with Board iD %d failed! Erro : %d\r\n", module_name, BoardID, res);
                return 0;
        }

	return 0;
}

void Show()
{
	printf("1.  Set User Defined BoardID\n");
	printf("2.  Get Firmware Version\n");
	printf("3.  Get Device ID\n");
	printf("4.  Set Device NickName\n");
	printf("5.  Get Device NickName\n");
	printf("6.  Set Communication Timeout\n");
	printf("7.  Get Communication Timeout\n");
	printf("8.  Set AutoReset WDT\n");
	printf("9.  Set Soft WDT Timeout\n");
	printf("10. Get Soft WDT Timeout\n");
	printf("Others number show this list.\n");
}

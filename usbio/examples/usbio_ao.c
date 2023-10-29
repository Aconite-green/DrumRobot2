/*
 *  usbio_ao.c
 *
 *  v 0.0.0 2022.6.28 by Winson Chen
 *
 *  create
 *
 */

#include "Global.h"
void Show();

int main()
{
	int DeviceID = USB2026;
	BYTE BoardID = 0x1;
	BYTE module_name[15], total_ao, i_byChEnable[(USBIO_AO_MAX_CHANNEL + 7) / 8];;
        int iErrCode, ch, DevNum, options;
	BYTE byChannel = 0;
	DWORD dwAOValue = 0xffff;
	float fAOValue = 5.1;
	DWORD o_dwAOExpValue[USBIO_AO_MAX_CHANNEL]; 
	DWORD dwAOValue_a[USBIO_AO_MAX_CHANNEL];
	float fAOValue_a[USBIO_AO_MAX_CHANNEL];

        printf("USB I/O Library Version : %s\n", USBIO_GetLibraryVersion());
	
	iErrCode = USBIO_OpenDevice(DeviceID, BoardID, &DevNum);

	if (iErrCode)
	{
		printf("open Device failed! Erro : 0x%x\r\n", iErrCode);
		return iErrCode;
	}

	USBIO_ModuleName(DevNum, module_name);

	USBIO_GetAOTotal(DevNum, &total_ao);
	printf("%s AO number: %d\n",module_name, total_ao);

	//Set all channel enable
	for(ch = 0; ch <(total_ao + 7) / 8; ch++)
		i_byChEnable[ch] = pow(2,total_ao)-1;
	USBIO_AO_SetChEnable(DevNum, i_byChEnable);


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
				USBIO_AO_WriteValueHexToChannel(DevNum, byChannel, dwAOValue);
				printf("Set Ch%d value 0x%x\n", byChannel, dwAOValue);
			break;
			case 2:
				for(ch = 0; ch < total_ao; ch++)
					dwAOValue_a[ch] = 0xff;

				printf("Set All Channel 0x%04x\n", dwAOValue_a[0]);
				USBIO_AO_WriteValueHex(DevNum, dwAOValue_a);
			break;
			case 3:
				USBIO_AO_WriteValueFloatToChannel(DevNum, byChannel, fAOValue);
				printf("Set Ch%d value %f\n", byChannel, fAOValue);
			break;
			case 4:
				for(ch = 0; ch < total_ao; ch++)
					fAOValue_a[ch] = 2;

				printf("Set All Channel %f\n", fAOValue_a[1]);
				USBIO_AO_WriteValueFloat(DevNum, fAOValue_a);
			break;
			default:
				Show();
			break;
		}
	}

        iErrCode = USBIO_CloseDevice(DevNum);

        if (iErrCode)
        {
                printf("close %s with Board iD %d failed! Erro : %d\r\n", module_name, BoardID, iErrCode);
                return iErrCode;
        }

	return 0;
}

void Show()
{
	printf("1. Write AO expect value to specifying channel in double word (digital) format.\n");
	printf("2. Write AO expect value to all channels in double word (digital) format.\n");
	printf("3. Write AO expect value to specifying channel in float (analog) format.\n");
	printf("4. Write AO expect value to all channels in float (analog) format..\n");
}

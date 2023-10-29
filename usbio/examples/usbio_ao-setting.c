/*
 *  usbio_ao-setting.c
 *
 *  v 0.0.0 2022.7.25 by Winson Chen
 *
 *    create
 *
 */

#include "Global.h"
void Show();
void TypeCode_List();

int main()
{
        int DeviceID = USB2026;
	int res, DevNum, i, j, options, val, ch;
        BYTE BoardID = 0x1;
        BYTE module_name[15], total_ao, o_byCJCEnable, i_byCJCEnable, o_byFilterRejection, i_byFilterRejection;
	BYTE byChannel;
	DWORD dwSetPowerOnValue, dwSetSafetyValue;
	BYTE o_byResolution[USBIO_AO_MAX_CHANNEL], o_byWireDetectEnable, i_byWireDetectEnable;
	BYTE o_byTotalSupportType, o_bySupportTypeCode[USBIO_MAX_SUPPORT_TYPE], i_byChEnable[(USBIO_AO_MAX_CHANNEL + 7) / 8];
	BYTE i_byTypeCodes[USBIO_AO_MAX_CHANNEL], o_byTypeCode[USBIO_AO_MAX_CHANNEL], o_byChEnable[(USBIO_AO_MAX_CHANNEL + 7)/8];
	BYTE o_byPwrOnEnable[USBIO_AO_MAX_CHANNEL], i_bySetPwrOnEnable[USBIO_AO_MAX_CHANNEL];
	BYTE o_bySafetyEnable [(USBIO_AO_MAX_CHANNEL + 7) / 8], i_bySetSafetyEnable[(USBIO_AO_MAX_CHANNEL + 7) / 8];
	DWORD o_dwAOExpValue[USBIO_AO_MAX_CHANNEL];
	DWORD o_dwPwrOnValue[USBIO_AO_MAX_CHANNEL], o_dwPowerOnValue[USBIO_AO_MAX_CHANNEL];
	DWORD dwSetSafetyValue_a[USBIO_AO_MAX_CHANNEL], o_dwSafetyValue[USBIO_AO_MAX_CHANNEL];
	float o_fAOExpValue[USBIO_AO_MAX_CHANNEL], o_fPwrOnValue[USBIO_AO_MAX_CHANNEL], fSetPowerOnValue_a[USBIO_AO_MAX_CHANNEL], fSetSafetyValue_a[USBIO_AO_MAX_CHANNEL];
	float fSetPowerOnValue, fSetSafetyValue, o_fSafetyValue[USBIO_AO_MAX_CHANNEL];
	int iErrCode;

	printf("USB I/O Library Version : %s\n", USBIO_GetLibraryVersion());

	res = USBIO_OpenDevice(DeviceID, BoardID, &DevNum);

        if (res)
        {
		printf("open Device failed! Erro : 0x%x\r\n", res);
                return 0;
        }

	USBIO_ModuleName(DevNum, module_name);

	USBIO_GetAOTotal(DevNum, &total_ao);
	printf("%s AO Number : %d\n", module_name, total_ao);

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
				printf("Get Total Support Type\n");

				if(ERR_NO_ERR != (iErrCode = USBIO_AO_GetTotalSupportType(DevNum, &o_byTotalSupportType)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}

				printf("%d\n",o_byTotalSupportType);

			break;
			case 2:
				printf("Get Support Type Code(options 29 present Type code list.)\n");

				if(ERR_NO_ERR != (iErrCode = USBIO_AO_GetTotalSupportType(DevNum, &o_byTotalSupportType)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}

				if(ERR_NO_ERR != (iErrCode = USBIO_AO_GetSupportTypeCode(DevNum, o_bySupportTypeCode)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}

				for(i = 0; i < o_byTotalSupportType; i++)
					printf("0x%02x\n", o_bySupportTypeCode[i]);
			break;
			case 3:
        			printf("Get each channel Type Code\n");

				if(ERR_NO_ERR != (iErrCode = USBIO_AO_GetTypeCode(DevNum, o_byTypeCode)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}

				for(ch = 0; ch < total_ao; ch++)
					printf("Ch%d 0x%02x  ", ch, o_byTypeCode[ch]);
				printf("\n");
			break;
			case 4:
				printf("Set Type Code To Channel\n");
				printf("Select channel(0~%d):",total_ao-1);
				scanf("%d",&ch);
				printf("Input type code(can refer option 29 result): 0x");
				scanf("%x",&val);
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_SetTypeCodeToChannel(DevNum, ch, val)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}
			break;
			case 5:
				printf("Set Type Code For All Channel\n");
				printf("Input type code(can refer option 29 result): 0x");
				scanf("%x",&val);
				for(ch = 0; ch < total_ao; ch++)
					i_byTypeCodes[ch] = val;
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_SetTypeCode(DevNum, i_byTypeCodes)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}
			break;
			case 6:
				printf("set 0 all channel disable, set %.0f all channel enable\n",pow(2,total_ao)-1);
				printf("Set Channel Enable(0~%.0f):", pow(2,total_ao)-1);
				scanf("%d",&val);
				for(i = 0; i <(total_ao + 7) / 8; i++)
					i_byChEnable[i] = val;
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_SetChEnable(DevNum, i_byChEnable)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}
			break;
			case 7:
				printf("Get Channel Enable\n");
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_GetChEnable(DevNum, o_byChEnable)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}

				j = 0;
				for(i = 0; i < (total_ao + 7) / 8; i++)
				{
					for(ch = 0; ch < total_ao; ch++)
					{
						val = (o_byChEnable[i] >> ch) & 0x1;
						if(val)
							printf("Ch%d Enable  ",j);
						else
							printf("Ch%d Disable  ",j);
						j++;
					}

					printf("\n");
				}
			break;
			case 8:
				printf("Read AO expect value in double word (digital) format.\n");
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_ReadExpValueHex(DevNum, o_dwAOExpValue)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}

				for(ch = 0; ch < total_ao; ch++)
					printf("ch%d. 0x%04x\n",ch, o_dwAOExpValue[ch]);
			break;
			case 9:
				printf("Read the real AO expect value.\n");
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_ReadExpValueFloat(DevNum, o_fAOExpValue)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}

				for(ch = 0; ch < total_ao; ch++)
					printf("ch%d. %04f\n",ch, o_fAOExpValue[ch]);
			break;
			case 10:
				printf("Read AO current value in double word (digital) format.\n");
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_ReadCurValueHex(DevNum, o_dwAOExpValue)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}

				for(ch = 0; ch < total_ao; ch++)
					printf("ch%d. 0x%04x\n",ch, o_dwAOExpValue[ch]);
			break;
			case 11:
				printf("Read the real AO current value.\n");
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_ReadCurValueFloat(DevNum, o_fAOExpValue)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}

				for(ch = 0; ch < total_ao; ch++)
					printf("ch%d. %04f\n",ch, o_fAOExpValue[ch]);
			break;
			case 12:
				printf("Set Power-On Enable\n");
				printf("Set Channel Power-On Enable(0~%.0f):", pow(2,total_ao)-1);
				scanf("%d",&val);
				for(ch = 0; ch < total_ao; ch++)
					i_bySetPwrOnEnable[ch] = (val >> ch) & 0x1;
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_SetPowerOnEnable(DevNum, i_bySetPwrOnEnable)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}
			break;
			case 13:
				printf("Get Power-On Enable\n");
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_GetPowerOnEnable(DevNum, o_byPwrOnEnable)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}

				for(ch = 0; ch < total_ao; ch++)
				{
					if(o_byPwrOnEnable[ch])
						printf("ch%d Enable\n", ch);
					else
						printf("ch%d Disable\n", ch);
				}
			break;
			case 14:
				printf("Set AO Power-On Value to specifying channel in double word (digital) format\n");
				byChannel = 0;
				dwSetPowerOnValue = 0xfff;
				printf("Set Ch%d 0x%x\n",byChannel, dwSetPowerOnValue);
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_SetPowerOnValueHexToChannel(DevNum, byChannel, dwSetPowerOnValue)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}

				printf("Set Ch%d 0x%x\n",byChannel, dwSetPowerOnValue);
			break;
			case 15:
				printf("Set AO Power-On Value to all channels in double word (digital) format\n");
				for(ch = 0; ch < total_ao; ch++)
					o_dwPowerOnValue[ch] = 0xffff;

				printf("Set All channels 0x%x\n",o_dwPowerOnValue[0]);
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_SetPowerOnValueHex(DevNum, o_dwPowerOnValue)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}
			break;
			case 16:
				printf("Set AO Power-On Value to specifying channel in float (analog) format\n");
				byChannel = 0;
				fSetPowerOnValue = 5;
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_SetPowerOnValueFloatToChannel(DevNum, byChannel, fSetPowerOnValue)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}

				printf("Set Ch%d %f\n",byChannel, fSetPowerOnValue);

			break;
			case 17:
				printf("Set AO Power-On Value to all channels in float (analog) format\n");
				for(ch = 0; ch < total_ao; ch++)
					fSetPowerOnValue_a[ch] = 3;

				printf("Set All channels %f\n",fSetPowerOnValue_a[0]);
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_SetPowerOnValueFloat(DevNum, fSetPowerOnValue_a)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}
				
			break;
			case 18:
				printf("Get Power-On Value. Each channel takes one unit of DWORD array\n");
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_GetPowerOnValueHex(DevNum, o_dwPwrOnValue)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}

				for(ch = 0; ch < total_ao; ch++)
					printf("Ch%d 0x%04x\n",ch, o_dwPwrOnValue[ch]);
			break;
			case 19:
				printf("Get Power-On Value. Each channel takes one unit of Float array.\n");
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_GetPowerOnValueFloat(DevNum,o_fPwrOnValue)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}

				for(ch = 0; ch < total_ao; ch++)
					printf("Ch%d %04f\n",ch, o_fPwrOnValue[ch]);
			break;
			case 20:
				printf("Get Safety Enable. Each byte indicates 8 channels\n");
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_GetSafetyEnable(DevNum, o_bySafetyEnable)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}
				
				j = 0;

				for(ch = 0; ch < (USBIO_AO_MAX_CHANNEL + 7) / 8; ch++)
				{
					//printf("0x%02x\n", o_bySafetyEnable[ch]);

					for(i = 0; i < 8; i++)	//printf each bit
					{
						if(o_bySafetyEnable[ch] > i)
							printf("Ch%d Enable\n",j++);
						else
							printf("Ch%d Disable\n",j++);
					}
						//printf("Ch%d %d\n",j++, (o_bySafetyEnable[ch] > i));
				}
			break;
			case 21:
				printf("Set Safety Enable. Each byte indicates 8 channels\n");
				i_bySetSafetyEnable[0] = 2;	//set ch0~ch1

				printf("Set value %d\n",i_bySetSafetyEnable[0]);

				if(ERR_NO_ERR != (iErrCode = USBIO_AO_SetSafetyEnable(DevNum, i_bySetSafetyEnable)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}
			break;
			case 22:
				printf("Get Safety value.Each channel takes one unit of DWORD array.\n");
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_GetSafetyValueHex(DevNum, o_dwSafetyValue)))
				{
					printf("Error 0x%x\n", iErrCode);
					break;
				}

				for(ch = 0; ch < total_ao; ch++)
					printf("Ch%d 0x%04x\n",ch, o_dwSafetyValue[ch]);
			break;
			case 23:
				printf("Get Safety value.Each channel takes one unit of float array.\n");
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_GetSafetyValueFloat(DevNum, o_fSafetyValue)))
                                {
                                        printf("Error 0x%x\n", iErrCode);
                                        break;
                                }

				for(ch = 0; ch < total_ao; ch++)
					printf("Ch%d %04f\n",ch, o_fSafetyValue[ch]);
			break;
			case 24:
				printf("Set AO Safety Value to specifying channel in double word (digital) format.\n");
				byChannel = 0;
				dwSetSafetyValue = 0xff;
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_SetSafetyValueHexToChannel(DevNum, byChannel, dwSetSafetyValue)))
                                {
                                        printf("Error 0x%x\n", iErrCode);
                                        break;
                                }

				printf("Set Ch%d value 0x%04x", byChannel, dwSetSafetyValue);
			break;
			case 25:
				printf("Set AO Safety Value to all channels in double word (digital)format.\n");
				for(ch = 0; ch < total_ao; ch++)
					dwSetSafetyValue_a[ch] = 0xffff;

				if(ERR_NO_ERR != (iErrCode = USBIO_AO_SetSafetyValueHex(DevNum, dwSetSafetyValue_a)))
                                {
                                        printf("Error 0x%x\n", iErrCode);
                                        break;
                                }

				printf("Set Value 0x%04x\n",dwSetSafetyValue_a[0]);
			break;
			case 26:
				printf("Set AO Safety Value to specifying channel in float (analog) format.\n");
				byChannel = 0;
				fSetSafetyValue = 6.7;
				
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_SetSafetyValueFloatToChannel(DevNum, byChannel, fSetSafetyValue)))
                                {
                                        printf("Error 0x%x\n", iErrCode);
                                        break;
                                }

				printf("Set Ch%d value %f", byChannel, fSetSafetyValue);
			break;
			case 27:
				printf("Set AO Safety Value to all channels in float (analog) format.\n");
				for(ch = 0; ch < total_ao; ch++)
					fSetSafetyValue_a[ch] = 3.9;
				
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_SetSafetyValueFloat(DevNum, fSetSafetyValue_a)))
                                {
                                        printf("Error 0x%x\n", iErrCode);
                                        break;
                                }

				printf("Set Value %f\n",fSetSafetyValue_a[0]);
			break;
			case 28:
				printf("Get Resolution\n");
				if(ERR_NO_ERR != (iErrCode = USBIO_AO_GetResolution(DevNum, o_byResolution)))
                                {
                                        printf("Error 0x%x\n", iErrCode);
                                        break;
                                }
				
				for(ch = 0; ch < total_ao; ch++)
				{
					printf("Ch%d %d  ", ch, o_byResolution[ch]);
					if(ch % 8 == 7)
						printf("\n");
				}
			break;
			case 29:
				TypeCode_List();
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
	printf("1.  Get Total Support Type\n");
	printf("2.  Get Support Type Code\n");
	printf("3.  Get Type Code\n");
	printf("4.  Set Type Code To Channel\n");
	printf("5.  Set Type Code For All Channel\n\n");
	printf("=====================================================================\n");
	printf("6.  Set Ch Enable\n");
	printf("7.  Get Ch Enable\n");
	printf("8.  Read AO expect value in double word (digital) format\n");
	printf("9.  Read the real AO expect value (float) format\n");
	printf("10. Read AO current value in double word (digital) format\n");
	printf("11. Read the real AO current value (float) format\n\n");
	printf("=====================================================================\n");
	printf("12. Set Power-On Enable\n");
	printf("13. Get Power-On Enable\n");
	printf("14. Set AO Power-On Value to specifying channel in double word (digital) format\n");
	printf("15. Set AO Power-On Value to all channels in double word (digital) format\n");
	printf("16. Set AO Power-On Value to specifying channel in float (analog) format\n");
	printf("17. Set AO Power-On Value to all channels in float (analog) format\n");
	printf("18. Get Power-On Value. Each channel takes one unit of DWORD array\n");
	printf("19. Get Power-On Value. Each channel takes one unit of Float array.\n\n");
	printf("=====================================================================\n");
	printf("20. Get Safety Enable. Each byte indicates 8 channels\n");
	printf("21. Set Safety Enable. Each byte indicates 8 channels\n");
	printf("22. Get Safety value.Each channel takes one unit of DWORD array.\n");
	printf("23. Get Safety value.Each channel takes one unit of float array.\n");
	printf("24. Set AO Safety Value to specifying channel in double word (digital) format.\n");
	printf("25. Set AO Safety Value to all channels in double word (digital) format.\n");
	printf("26. Set AO Safety Value to specifying channel in float (analog) format.\n");
	printf("27. Set AO Safety Value to all channels in float (analog) format.\n\n");
	printf("=====================================================================\n");
	printf("28. Get Resolution\n");
	printf("29. Analog Output Type Code List\n");
	printf("Others number show this list.\n");

}

void TypeCode_List()
{
	printf("Analog Output Type Code\n");
	printf("Code 	Input Type \n");
	printf("-------------------------\n");
	printf("0x30 	0~+20mA\n");
	printf("0x31 	4~+20mA\n");
	printf("0x32 	0V~+10V\n");
	printf("0x33 	-10V~+10V\n");
	printf("0x34 	0V~+5V\n");
	printf("0x35 	-5V~+5V");
}



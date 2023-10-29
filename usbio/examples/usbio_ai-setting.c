/*
 *  usbio_ai-setting.c
 *
 *  v 0.0.0 2018.8.8 by Winson Chen
 *
 *    create
 *
 */

#include "Global.h"
void Show();
void TypeCode_List();

int main()
{
	int res, DevNum, i, j, options, val, ch;
        int DeviceID = USB2026;
        BYTE BoardID = 0x1;
        BYTE module_name[15], total_ai, o_byCJCEnable, i_byCJCEnable, o_byFilterRejection, i_byFilterRejection;
	BYTE o_byResolution[USBIO_AI_MAX_CHANNEL], o_byWireDetectEnable, i_byWireDetectEnable;
	BYTE o_byTotalSupportType, o_bySupportTypeCode[USBIO_MAX_SUPPORT_TYPE], i_byChEnable[(USBIO_AI_MAX_CHANNEL + 7) / 8];
	BYTE i_byTypeCodes[USBIO_AI_MAX_CHANNEL], o_byTypeCode[USBIO_AI_MAX_CHANNEL], o_byChEnable[(USBIO_AI_MAX_CHANNEL + 7)/8];
	float o_fChCJCOffset [USBIO_AI_MAX_CHANNEL], i_fChCJCOffset[USBIO_AI_MAX_CHANNEL], CJC;
	float o_fCJCOffset, i_fCJCOffset, o_fCJCValue;

	printf("USB I/O Library Version : %s\n", USBIO_GetLibraryVersion());

	res = USBIO_OpenDevice(DeviceID, BoardID, &DevNum);

        if (res)
        {
		printf("open Device failed! Erro : 0x%x\r\n", res);
                return 0;
        }

	USBIO_ModuleName(DevNum, module_name);

	USBIO_GetAITotal(DevNum, &total_ai);
	printf("%s AI Number : %d\n", module_name, total_ai);

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

				USBIO_AI_GetTotalSupportType(DevNum, &o_byTotalSupportType);
				printf("%d\n",o_byTotalSupportType);

			break;
			case 2:
				printf("Get Support Type Code(options 21 present Type code list.)\n");

				USBIO_AI_GetTotalSupportType(DevNum, &o_byTotalSupportType);
				USBIO_AI_GetSupportTypeCode(DevNum, o_bySupportTypeCode);
				for(i = 0; i < o_byTotalSupportType; i++)
					printf("0x%02x\n", o_bySupportTypeCode[i]);
			break;
			case 3:
				printf("Set Type Code\n");
				printf("Input type code(can refer option 2 result):");
				scanf("%d",&val);
				for(i = 0; i < total_ai; i++)
					i_byTypeCodes[i] = val;
				USBIO_AI_SetTypeCode(DevNum, i_byTypeCodes);
					
			break;
			case 4:
				printf("Set Type Code To Channel\n");
				printf("Select channel(0~%d):",total_ai-1);
				scanf("%d",&ch);
				printf("Input type code(can refer option 2 result):");
				scanf("%d",&val);
				USBIO_AI_SetTypeCodeToChannel(DevNum, ch, val);
				
			break;
			case 5:
        			printf("Get each channel Type Code\n");

				USBIO_AI_GetTypeCode(DevNum, o_byTypeCode);
				for(i = 0; i < total_ai; i++)
					printf("CH%d 0x%02x  ", i, o_byTypeCode[i]);
				printf("\n");
			break;
			case 6:
				printf("Set Channel CJC Offset(The valid range of offset is -40.96 ~ +40.95)\n");
				scanf("%f",&CJC);
				for(i = 0; i < total_ai; i++)
					i_fChCJCOffset[i] = CJC;

				USBIO_AI_SetChCJCOffset(DevNum, i_fChCJCOffset);
				
			break;
			case 7:
				printf("Set Channel CJC Offset To Channel(The valid range of offset is -40.96 ~ +40.95)\n");
				printf("Select channel(0~%d):",total_ai-1);
				scanf("%d",&ch);
				printf("Set CJC offset:");
				scanf("%f",&CJC);
				USBIO_AI_SetChCJCOffsetToChannel(DevNum, ch, CJC);
			break;
			case 8:
				printf("Get Channel CJC Offset\n");
				USBIO_AI_GetChCJCOffset(DevNum, o_fChCJCOffset);
				for(i = 0; i < total_ai; i++)
				{
					printf("CH%d %.3f ", i, o_fChCJCOffset[i]);
					if(i % 4 == 3)
						printf("\n");
				}

			break;
			case 9:
				printf("Set Channel Enable(0~%.0f):", pow(2,total_ai)-1);
				scanf("%d",&val);
				for(i = 0; i <(total_ai + 7) / 8; i++)
					i_byChEnable[i] = val;
				USBIO_AI_SetChEnable(DevNum, i_byChEnable);

			break;
			case 10:
				printf("Get Channel Enable\n");
				USBIO_AI_GetChEnable(DevNum, o_byChEnable);
				j = 0;
				for(i = 0; i < (total_ai + 7) / 8; i++)
				{
					for(ch = 0; ch < total_ai; ch++)
					{
						val = (o_byChEnable[i] >> ch) & 0x1;
						if(val)
							printf("CH%d Enable  ",j);
						else
							printf("CH%d Disable  ",j);
						j++;
					}
					printf("\n");
				}
			break;
			case 11:
				printf("Set CJC Offset(The valid range of offset is -40.96 ~ +40.95):\n");
				scanf("%f",&i_fCJCOffset);
				USBIO_AI_SetCJCOffset(DevNum, i_fCJCOffset);
			break;
			case 12:
				printf("Get CJC Offset\n");
				USBIO_AI_GetCJCOffset(DevNum, &o_fCJCOffset);
				printf("%.5f\n", o_fCJCOffset);
			break;
			case 13:
				printf("Set CJC Enable(0 Disable, 1 Enable):\n");
				scanf("%hhu",&i_byCJCEnable);
				USBIO_AI_SetCJCEnable(DevNum, i_byCJCEnable);
			break;
			case 14:
				printf("Get CJC Enable\n");
				USBIO_AI_GetCJCEnable(DevNum, &o_byCJCEnable);  //default is 1
				if(o_byCJCEnable)
					printf("CJC Enable\n");
				else
					printf("CJC Disable\n");

			break;
			case 15:
				printf("Read CJC Value\n");
				USBIO_AI_ReadCJCValue(DevNum, &o_fCJCValue);
				printf("Current CJC value on %s is %.5f\n",module_name, o_fCJCValue);
			break;
			case 16:
				printf("Set Filter Rejection(0->60Hz, 1->50Hz):\n");
				scanf("%hhu",&i_byFilterRejection);
				USBIO_AI_SetFilterRejection(DevNum, i_byFilterRejection);
			break;
			case 17:
				printf("Get Filter Rejection\n");
				USBIO_AI_GetFilterRejection(DevNum, &o_byFilterRejection);
				if(o_byFilterRejection)
					printf("50Hz\n");
				else
					printf("60Hz\n");
			break;
			case 18:
				printf("Set Wire Detect Enable(0 Disable, 1 Enable):\n");
				scanf("%hhu",&i_byWireDetectEnable);
				USBIO_AI_SetWireDetectEnable(DevNum, i_byWireDetectEnable);
			break;
			case 19:
				printf("Get Wire Detect Enable\n");
				USBIO_AI_GetWireDetectEnable(DevNum, &o_byWireDetectEnable);
				if(o_byWireDetectEnable)
					printf("Enable\n");
				else
					printf("Disable\n");
			break;
			case 20:
				printf("Get Resolution\n");
				USBIO_AI_GetResolution(DevNum, o_byResolution);
				for(i = 0; i < total_ai; i++)
				{
					printf("CH%d %d  ", i, o_byResolution[i]);
					if(i % 8 == 7)
						printf("\n");
				}
			break;
			case 21:
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
	printf("3.  Set Type Code\n");
	printf("4.  Set Type Code To Channel\n");
	printf("5.  Get Type Code\n");
	printf("6.  Set Ch CJC Offset\n");
	printf("7.  Set Ch CJC Offset To Channel\n");
	printf("8.  Get Ch CJC Offset\n");
	printf("9.  Set Ch Enable\n");
	printf("10. Get Ch Enable\n");
	printf("11. Set CJC Offset\n");
	printf("12. Get CJC Offset\n");
	printf("13. Set CJC Enable\n");
	printf("14. Get CJC Enable\n");
	printf("15. Read CJC Value\n");
	printf("16. Set Filter Rejection\n");
	printf("17. Get Filter Rejection\n");
	printf("18. Set Wire Detect Enable\n");
	printf("19. Get Wire Detect Enable\n");
	printf("20. Get Resolution\n");
	printf("21. Analog Input Type Code List\n");
	printf("Others number show this list.\n");

}

void TypeCode_List()
{
	printf("Analog Input Type Code\n");
	printf("Code Input Type 		Code Input Type\n");
	printf("------------------------------------------------------------------\n");
	printf("0x00 -15  mV ~ +15  mV 		0x17 Type L TC, -200 ~ +800%c%c\n",0xa2,0x4a);
	printf("0x01 -50  mV ~ +50  mV 		0x18 Type M TC, -200 ~ +100%c%c\n",0xa2,0x4a);
	printf("0x02 -100 mV ~ +100 mV 		0x19 Type LDIN43710 TC, -200 ~ +900%c%c\n",0xa2,0x4a);
	printf("0x03 -500 mV ~ +500 mV	 	0x1A 0 ~ +20 mA\n");
	printf("0x04 -1   V ~ +1   V 		0x1B -150 V ~ +150 V\n");
	printf("0x05 -2.5 V ~ +2.5 V 		0x1C -50 V ~ +50 V\n");
	printf("0x06 -20 mA ~ +20 mA 		0x20 Pt 100, .=.00385, -100 ~ +100%c%c\n",0xa2,0x4a);
	printf("0x07 +4  mA ~ +20 mA 		0x21 Pt 100, .=.00385, 0 ~ +100%c%c\n",0xa2,0x4a);
	printf("0x08 -10  V ~ +10  V 		0x22 Pt 100, .=.00385, 0 ~ +200%c%c\n",0xa2,0x4a);
	printf("0x09 -5   V ~ +5   V 		0x23 Pt 100, .=.00385, 0 ~ +600%c%c\n",0xa2,0x4a);
	printf("0x0A -1   V ~ +1   V 		0x24 Pt 100, .=.003916, -100 ~ +100%c%c\n",0xa2,0x4a);
	printf("0x0B -500 mV ~ +500 mV 		0x25 Pt 100, .=.003916, 0 ~ +100%c%c\n",0xa2,0x4a);
	printf("0x0C -150 mV ~ +150 mV 		0x26 Pt 100, .=.003916, 0 ~ +200%c%c\n",0xa2,0x4a);
	printf("0x0D -20 mA ~ +20 mA 		0x27 Pt 100, .=.003916, 0 ~ +600%c%c\n",0xa2,0x4a);
	printf("0x0E Type J TC, -210 ~ +760%c%c 	0x28 Nickel 120, -80 ~ +100%c%c\n",0xa2,0x4a,0xa2,0x4a);
	printf("0x0F Type K TC, -210 ~ +1372%c%c 	0x29 Nickel 120, 0 ~ +100%c%c\n",0xa2,0x4a,0xa2,0x4a);
	printf("0x10 Type T TC, -270 ~ +400%c%c 	0x2A Pt 1000, .=.00392, -200 ~ +600%c%c\n",0xa2,0x4a,0xa2,0x4a);
	printf("0x11 Type E TC, -270 ~ +1000%c%c 	0x2B Cu 100, .=.00421, -20 ~ +150%c%c\n",0xa2,0x4a,0xa2,0x4a);
	printf("0x12 Type R TC, 0 ~ +1768%c%c 	0x2C Cu 100, .=.00427, 0 ~ +200%c%c\n",0xa2,0x4a,0xa2,0x4a);
	printf("0x13 Type S TC, 0 ~ +1768%c%c 	0x2D Cu 1000, .=.00421, -20 ~ +150%c%c\n",0xa2,0x4a,0xa2,0x4a);
	printf("0x14 Type B TC, 0 ~ +1820%c%c 	0x2E Pt 100, .=.00385, -200 ~ +200%c%c\n",0xa2,0x4a,0xa2,0x4a);
	printf("0x15 Type N TC, -270 ~ +1300%c%c 	0x2F Pt 100, .=.003916, -200 ~ +200%c%c\n",0xa2,0x4a,0xa2,0x4a);
	printf("0x16 Type C TC, 0 ~ +2320%c%c\n",0xa2,0x4a);
}



/*
 *  usb2055_dio.c
 *
 *  v 0.0.0 208.7.26 by Winson Chen
 *
 *  create
 *  Display USB-2000 series module Device ID and Board ID.
 *
 */

#include "ICPDAS_USBIO.h"

int main()
{
	USBIO_list list[MAX_USB_DEVICES];
	char count, res = 0, i;

        printf("USB I/O Library Version : %s\n", USBIO_GetLibraryVersion());
	res  = USBIO_ListDevice(list,&count);
	if(res < 0)
		return ERR_USBDEV_ERROR_OPENFILE;

	printf("   Device ID	Board ID\n");

	for(i = 0; i < count; i++)
	{
		printf("%d. ",i);
		switch(list[i].DeviceID)
		{
			case USB2019:
				printf("USB2019	0x%x\n",list[i].BoardID);
			break;
			case USB2026:
				printf("USB2026	0x%x\n",list[i].BoardID);
			break;
			case USB2045:
				printf("USB2045	0x%x\n",list[i].BoardID);
			break;
			case USB2051:
				printf("USB2051	0x%x\n",list[i].BoardID);
			break;
			case USB2055:
				printf("USB2055	0x%x\n",list[i].BoardID);
			break;
			case USB2060:
				printf("USB2060	0x%x\n",list[i].BoardID);
			break;
			case USB2064:
				printf("USB2064	0x%x\n",list[i].BoardID);
			break;
			case USB2084:
				printf("USB2084	0x%x\n",list[i].BoardID);
			break;
			case USB2055_32:
				printf("USB2055_32	0x%x\n",list[i].BoardID);
			break;
			case USB2068_18:
				printf("USB2068_18	0x%x\n",list[i].BoardID);
			break;
			case USB2045_32:
				printf("USB2045_32	0x%x\n",list[i].BoardID);
			break;
			case USB2051_32:
				printf("USB2051_32	0x%x\n",list[i].BoardID);
			break;
			case USB2060_24:
				printf("USB2060_24	0x%x\n",list[i].BoardID);
			break;
			case USB2064_16:
				printf("USB2064_16	0x%x\n",list[i].BoardID);
			break;
			case USB2069_18:
				printf("USB2069_18	0x%x\n",list[i].BoardID);
			break;
			default:
				printf("Not support device in this demo.\n");
			break;
		}
	}
	return 0;
}

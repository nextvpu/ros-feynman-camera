#include <stdio.h>
#include "feynman_sdk.h"
#ifdef _WINDOWS
#include <process.h>
#pragma comment(lib, "..\\x64\\Debug\\lib\\libusb-1.0.lib")
#else
#include <pthread.h>
#include <unistd.h>
#endif

#include "ring_queue.h"
#include "libusb.h"

#include <stdlib.h>
#pragma pack(1)

#define FEYNMAN_VID 0x4E56
#define FEYNMAN_PID 0x5055
#define FEYNMAN_ENDPOINT_IN (LIBUSB_ENDPOINT_IN + 1)
#define FEYNMAN_ENDPOINT_OUT (LIBUSB_ENDPOINT_OUT + 1)

#define USB_START_REQUEST 0x01
#define USB_START_RESPONSE 0x02

#define USB_STOP_REQUEST 0x03
#define USB_STOP_RESPONSE 0x04

#define USB_RESET_REQUEST 0x05
#define USB_RESET_RESPONSE 0x06

typedef struct
{
	int timeout;
	FEYNMAN_COMMAND_SUB_TYPE cmdsubtype;
	int len;
	void *data;
} sendcmd_t;
typedef struct _pkt_info_custom_t
{
	unsigned char buffer[USB_PACKET_MAX_SIZE];
	int len;
} pkt_info_custom_t;

static Ring_Queue *otherpktqueue = NULL;
static Ring_Queue *depthpktqueue = NULL;
static Ring_Queue *irpktqueue = NULL;
static Ring_Queue *rgbpktqueue = NULL;
static Ring_Queue *imupktqueue = NULL;
static Ring_Queue *savepktqueue = NULL;
static Ring_Queue *sendqueue = NULL;
////////////////////
const unsigned char colortable[256][3] = {{16, 128, 128},
										  {26, 170, 122},
										  {26, 173, 121},
										  {27, 176, 121},
										  {28, 178, 120},
										  {28, 181, 120},
										  {29, 184, 119},
										  {29, 186, 119},
										  {30, 189, 119},
										  {30, 192, 118},
										  {31, 194, 118},
										  {32, 196, 118},
										  {32, 197, 117},
										  {32, 199, 117},
										  {33, 201, 117},
										  {33, 203, 117},
										  {33, 204, 116},
										  {34, 206, 116},
										  {34, 208, 116},
										  {35, 209, 116},
										  {35, 211, 115},
										  {35, 213, 115},
										  {36, 215, 114},
										  {37, 217, 115},
										  {36, 218, 114},
										  {37, 220, 114},
										  {37, 222, 114},
										  {38, 223, 113},
										  {38, 225, 113},
										  {39, 226, 113},
										  {39, 229, 113},
										  {39, 231, 112},
										  {40, 233, 112},
										  {40, 235, 111},
										  {40, 236, 111},
										  {41, 238, 111},
										  {41, 239, 110},
										  {41, 240, 110},
										  {43, 239, 109},
										  {45, 238, 107},
										  {47, 237, 106},
										  {49, 235, 104},
										  {51, 234, 103},
										  {53, 233, 101},
										  {55, 232, 100},
										  {57, 231, 98},
										  {59, 230, 97},
										  {61, 228, 96},
										  {63, 227, 94},
										  {65, 226, 93},
										  {67, 225, 92},
										  {69, 224, 90},
										  {71, 222, 89},
										  {73, 221, 87},
										  {75, 220, 86},
										  {77, 219, 84},
										  {79, 218, 84},
										  {81, 217, 81},
										  {83, 216, 80},
										  {86, 214, 78},
										  {88, 213, 77},
										  {90, 212, 75},
										  {92, 211, 74},
										  {94, 210, 72},
										  {96, 208, 71},
										  {98, 207, 69},
										  {100, 206, 68},
										  {102, 205, 66},
										  {104, 204, 65},
										  {106, 203, 63},
										  {108, 202, 63},
										  {110, 200, 60},
										  {112, 199, 59},
										  {114, 198, 57},
										  {116, 197, 56},
										  {118, 196, 54},
										  {120, 194, 53},
										  {122, 193, 51},
										  {124, 192, 50},
										  {126, 191, 48},
										  {128, 190, 47},
										  {130, 189, 45},
										  {131, 188, 44},
										  {134, 186, 43},
										  {136, 185, 41},
										  {138, 184, 40},
										  {140, 183, 39},
										  {142, 182, 37},
										  {144, 181, 36},
										  {146, 180, 34},
										  {148, 178, 33},
										  {150, 177, 31},
										  {152, 176, 30},
										  {154, 175, 28},
										  {156, 174, 27},
										  {158, 173, 25},
										  {160, 171, 24},
										  {162, 170, 22},
										  {164, 169, 21},
										  {166, 168, 19},
										  {168, 167, 18},
										  {170, 166, 16},
										  {171, 164, 18},
										  {171, 162, 20},
										  {172, 159, 22},
										  {173, 157, 25},
										  {173, 155, 26},
										  {174, 153, 28},
										  {174, 150, 31},
										  {175, 148, 32},
										  {176, 146, 34},
										  {177, 143, 37},
										  {177, 141, 39},
										  {178, 138, 41},
										  {179, 136, 43},
										  {179, 134, 45},
										  {180, 131, 47},
										  {180, 129, 49},
										  {181, 127, 51},
										  {182, 124, 54},
										  {182, 122, 55},
										  {183, 119, 57},
										  {183, 117, 59},
										  {184, 115, 61},
										  {184, 112, 63},
										  {185, 110, 65},
										  {186, 108, 67},
										  {187, 106, 69},
										  {187, 103, 71},
										  {188, 101, 73},
										  {189, 99, 75},
										  {189, 96, 77},
										  {190, 94, 79},
										  {190, 91, 82},
										  {191, 89, 84},
										  {191, 87, 85},
										  {191, 85, 86},
										  {192, 83, 88},
										  {192, 81, 90},
										  {193, 78, 91},
										  {193, 76, 93},
										  {193, 74, 94},
										  {194, 72, 96},
										  {194, 70, 98},
										  {194, 67, 99},
										  {195, 65, 101},
										  {196, 66, 102},
										  {197, 68, 103},
										  {199, 72, 103},
										  {201, 76, 104},
										  {203, 80, 105},
										  {204, 84, 105},
										  {207, 92, 105},
										  {209, 96, 106},
										  {211, 100, 106},
										  {213, 107, 106},
										  {214, 107, 108},
										  {215, 106, 109},
										  {215, 106, 110},
										  {216, 105, 112},
										  {217, 105, 113},
										  {218, 104, 114},
										  {219, 104, 116},
										  {220, 103, 118},
										  {221, 103, 120},
										  {222, 102, 121},
										  {223, 101, 123},
										  {224, 101, 125},
										  {225, 100, 127},
										  {226, 100, 128},
										  {227, 99, 130},
										  {228, 98, 132},
										  {228, 98, 133},
										  {227, 93, 134},
										  {226, 88, 134},
										  {225, 84, 135},
										  {224, 79, 136},
										  {223, 74, 137},
										  {222, 69, 138},
										  {221, 64, 138},
										  {220, 60, 139},
										  {219, 55, 140},
										  {218, 50, 141},
										  {210, 17, 146},
										  {208, 18, 148},
										  {207, 19, 149},
										  {205, 19, 150},
										  {204, 21, 151},
										  {202, 21, 152},
										  {200, 22, 153},
										  {199, 22, 154},
										  {198, 24, 155},
										  {196, 25, 156},
										  {195, 26, 157},
										  {193, 26, 159},
										  {192, 27, 160},
										  {190, 28, 161},
										  {188, 29, 162},
										  {187, 29, 163},
										  {185, 31, 164},
										  {184, 32, 165},
										  {182, 32, 166},
										  {181, 33, 167},
										  {179, 34, 168},
										  {178, 35, 170},
										  {176, 36, 171},
										  {175, 37, 172},
										  {173, 38, 173},
										  {172, 39, 174},
										  {170, 39, 175},
										  {169, 40, 176},
										  {167, 41, 177},
										  {166, 42, 178},
										  {164, 43, 180},
										  {163, 44, 181},
										  {161, 45, 182},
										  {160, 46, 183},
										  {158, 46, 184},
										  {157, 47, 185},
										  {155, 48, 186},
										  {154, 49, 187},
										  {152, 50, 188},
										  {151, 51, 189},
										  {149, 52, 191},
										  {148, 53, 192},
										  {146, 53, 193},
										  {145, 54, 194},
										  {143, 55, 195},
										  {142, 56, 196},
										  {140, 57, 197},
										  {139, 58, 198},
										  {137, 59, 199},
										  {136, 60, 200},
										  {134, 60, 202},
										  {133, 61, 203},
										  {131, 62, 204},
										  {130, 63, 205},
										  {128, 64, 206},
										  {126, 65, 207},
										  {125, 66, 208},
										  {123, 67, 209},
										  {122, 67, 210},
										  {120, 68, 212},
										  {115, 71, 216},
										  {109, 75, 220},
										  {104, 78, 224},
										  {98, 81, 228},
										  {93, 84, 232},
										  {92, 85, 233},
										  {91, 85, 233},
										  {90, 86, 234},
										  {89, 87, 235},
										  {88, 87, 235},
										  {87, 88, 236},
										  {86, 88, 237},
										  {16, 128, 128}};
#define log_printf printf

extern int update_result;
extern uint8_t upgrade_update_flag[256];
/*void usb_upgrade_callback(FEYNMAN_UPGRADE_SUB_TYPE type, void *data, uint32_t size)
{
	extern int32_t get_upgrade_update_flag_index(FEYNMAN_UPGRADE_SUB_TYPE type);
	int32_t index = get_upgrade_update_flag_index(type);
	if (index >= 0)
	{
		switch (type)
		{
		case FEYNMAN_COMMAND_USB_UPGRADE_FEYNMAN_RETURN:
		{
			update_result = ((s_feynman_upgrade_result *)data)->result;

			log_printf("recv upgrade feynman ack, %s\r\n",
					   (update_result == 1) ? "success" : ((update_result == -1) ? "err,retry" : ((update_result == -2) ? "can't write, stop" : "can't operate")));

			upgrade_update_flag[index] = 0;

			break;
		}
		case FEYNMAN_COMMAND_USB_UPGRADE_LIB_RETURN:
		{
			update_result = ((s_feynman_upgrade_result *)data)->result;

			log_printf("recv upgrade lib ack, %s\r\n",
					   (update_result == 1) ? "success" : ((update_result == -1) ? "err,retry" : ((update_result == -2) ? "can't write, stop" : "can't operate")));

			upgrade_update_flag[index] = 0;

			break;
		}
		case FEYNMAN_COMMAND_USB_UPGRADE_FILE_RETURN:
		{
			update_result = ((s_feynman_upgrade_result *)data)->result;

			log_printf("recv upgrade file ack, %s\r\n",
					   (update_result == 1) ? "success" : ((update_result == -1) ? "err,retry" : ((update_result == -2) ? "can't write, stop" : "can't operate")));

			upgrade_update_flag[index] = 0;

			break;
		}
		}
	}
}*/

int feynman_getyuvfromindex(int index, unsigned char *py, unsigned char *pu, unsigned char *pv)
{
	if (index < 255 && index >= 0)
	{
		*py = colortable[index][0];
		*pu = colortable[index][1];
		*pv = colortable[index][2];
		return 0;
	}
	else
	{
		return -1;
	}
}
//////////////////////////////////////

int g_thread_running_flag = 0;
static uint8_t *g_usb_buf = NULL;

struct libusb_device_handle *g_h_dev_usb;

static void *connectuserdata;

static void usb_hal_close()
{
	libusb_close(g_h_dev_usb);
	libusb_exit(NULL);
}
void feynman_init()
{
	printf("init in feynman!\n");
}
void feynman_deinit()
{
	printf("deinit in feynman!\n");
}
int usb_hal_write(uint8_t *data, int len, int *bytesTransffered, int32_t timeout)
{
	if (feynman_hasconnect())
	{

		int32_t ret = libusb_bulk_transfer(g_h_dev_usb, FEYNMAN_ENDPOINT_OUT, (unsigned char *)data, len, bytesTransffered, timeout);

		return ret;
	}
	else
	{
		return -1;
	}
}
static int send_one_packet(uint16_t type, uint16_t sub_type, uint32_t len, const uint8_t *pData, int timeout)
{
	int bytesTransffered;
	FEYNMAN_USBHeaderDataPacket *usbPacket = (FEYNMAN_USBHeaderDataPacket *)calloc(1, sizeof(FEYNMAN_USBHeaderDataPacket) + len);
	int ret = 0;
	uint32_t checksum = 0;

	if (len > ONE_PACKET_DATA_MAX_SIZE)
	{
		printf("packet data too long\r\n");
		return -1;
	}
	//	memset(&usbPacket, 0, sizeof(usbPacket));

	usbPacket->magic[0] = 'N';
	usbPacket->magic[1] = 'E';
	usbPacket->magic[2] = 'X';
	usbPacket->magic[3] = 'T';
	usbPacket->magic[4] = '_';
	usbPacket->magic[5] = 'V';
	usbPacket->magic[6] = 'P';
	usbPacket->magic[7] = 'U';
	usbPacket->type = type;
	usbPacket->sub_type = sub_type;
	usbPacket->len = len;

	if (len > 0)
	{
		for (uint32_t i = 0; i < len; i++)
		{
			checksum += pData[i];
		}
		memcpy((void *)usbPacket->data, pData, len);
	}
	usbPacket->checksum = checksum;

	int bytes_transffered = 0;

	ret = usb_hal_write((uint8_t *)usbPacket, sizeof(FEYNMAN_USBHeaderDataPacket) + len, &bytesTransffered, timeout);
	free(usbPacket);
	if (ret < 0 || bytesTransffered != (sizeof(FEYNMAN_USBHeaderDataPacket) + len))
	{
		printf("[%s]: ret = %d\n", __FUNCTION__, ret);
		return -1;
	}
	return 0;
}
/*
typedef struct
{
uint32_t exposure_mode; //0-�Զ��ع�  1-�ֶ��ع�
int32_t exposure_time[2];  //us, ���ڵ���0ʱ��Ч��С��0��ʾ�������ع�ʱ��
int32_t digital_gain[2];   //���ڵ���0ʱ��Ч��С��0��ʾ��������������
}s_feynman_ir_sensor_exposure_infor;
*/
void feynman_setirautoexposure()
{
	if (feynman_hasconnect())
	{
		s_feynman_ir_sensor_exposure_infor autoexposure;
		autoexposure.exposure_mode = 0;
		autoexposure.exposure_time[0] = 0;
		autoexposure.exposure_time[1] = 0;
		send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_SET_IR_IMAGE_SENEOR_EXPOSURE_COMMAND, sizeof(s_feynman_ir_sensor_exposure_infor), (const uint8_t *)&autoexposure, 2000);
	}
}
void feynman_getirexposure(int timeout)
{
	if (NULL == sendqueue)
	{
		sendqueue = new Ring_Queue(9, sizeof(sendcmd_t));
	}
	if (feynman_hasconnect())
	{
		sendcmd_t *p = 0;
		p = (sendcmd_t *)sendqueue->SOLO_Write();
		if (p)
		{
			memset(p, 0, sizeof(sendcmd_t));
			p->cmdsubtype = FEYNMAN_COMMAND_GET_IR_IMAGE_SENSOR_EXPOSURE_COMMAND;
			p->data = NULL;
			p->len = 0;
			p->timeout = timeout;
			sendqueue->SOLO_Write_Over();
		}
	}
}
void feynman_setirmanualexposure(int leftus, int leftgain, int rightus, int rightgain)
{
	if (feynman_hasconnect())
	{
		s_feynman_ir_sensor_exposure_infor manualexposure;
		manualexposure.exposure_mode = 1;
		manualexposure.exposure_time[0] = leftus;
		manualexposure.digital_gain[0] = leftgain;
		manualexposure.exposure_time[1] = rightus;
		manualexposure.digital_gain[1] = rightgain;
		send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_SET_IR_IMAGE_SENEOR_EXPOSURE_COMMAND, sizeof(s_feynman_ir_sensor_exposure_infor), (const uint8_t *)&manualexposure, 2000);
	}
}
static s_feynman_depth_config s_depthconfig = {0, 0, 0, 0, 0};
void feynman_setresolutionfps(FEYNMAN_SENSOR_RESOLUTION_TYPE res, unsigned int fps)
{
	if (feynman_hasconnect())
	{
		s_feynman_sensor_resolution_fps_infor info;
		info.resolution = res;
		info.fps = fps;
		send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_SET_IR_IMAGE_SENSOR_RESOLUTION_FPS_COMMAND, sizeof(s_feynman_sensor_resolution_fps_infor), (const uint8_t *)&info, 2000);
	}
}
void feynman_setdepthmode(int mode)
{
	s_depthconfig.app_depth_mode = mode;
	send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_SET_DEPTH_CONFIGURATION_COMMAND, sizeof(s_feynman_depth_config), (const uint8_t *)&s_depthconfig, 2000);
}
void feynman_stoppipeline()
{
	s_feynman_pipeline_cmd cmd;
	cmd.status = 2;
	send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_USB_PIPELINE_COMMAND, sizeof(s_feynman_pipeline_cmd), (const uint8_t *)&cmd, 2000);
}
void feynman_startpipeline()
{
	if (feynman_hasconnect())
	{
		s_feynman_pipeline_cmd cmd;
		cmd.status = 1;
		send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_USB_PIPELINE_COMMAND, sizeof(s_feynman_pipeline_cmd), (const uint8_t *)&cmd, 2000);
	}
}
void feynman_resetpipeline()
{
	if (feynman_hasconnect())
	{
		s_feynman_pipeline_cmd cmd;
		cmd.status = 3;
		send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_USB_PIPELINE_COMMAND, sizeof(s_feynman_pipeline_cmd), (const uint8_t *)&cmd, 2000);
	}
}
void feynman_transferrgb(int enable)
{
	if (feynman_hasconnect())
	{
		s_feynman_img_depth_rgb_transfer tmptransfer;
		tmptransfer.status = enable;

		send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_USB_IMG_DEPTH_RGB_TRANSFER_COMMAND, sizeof(s_feynman_img_depth_rgb_transfer), (const uint8_t *)&tmptransfer, 2000);
	}
}
void feynman_transferir(int enable)
{
	s_feynman_img_depth_ir_transfer cmd;
	cmd.status = enable;
	send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_USB_IMG_DEPTH_IR_TRANSFER_COMMAND, sizeof(s_feynman_pipeline_cmd), (const uint8_t *)&cmd, 2000);
}
void feynman_transferdepth(int mode)
{ //0.off 1.depth 2.disparity
	if (feynman_hasconnect())
	{
		if (mode == 0)
		{
			s_feynman_img_depth_depth_transfer cmd;
			cmd.status = 0;
			send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_USB_IMG_DEPTH_DEPTH_TRANSFER_COMMAND, sizeof(s_feynman_pipeline_cmd), (const uint8_t *)&cmd, 2000);
		}
		else if (mode == 1)
		{
			s_feynman_img_depth_depth_transfer cmd;
			cmd.status = 1;
			send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_USB_IMG_DEPTH_DEPTH_TRANSFER_COMMAND, sizeof(s_feynman_pipeline_cmd), (const uint8_t *)&cmd, 2000);

			s_feynman_depth_img_source srccmd;
			srccmd.source = 1;
			send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_SET_DEPTH_DATA_SOURCE_COMMAND, sizeof(s_feynman_depth_img_source), (const uint8_t *)&cmd, 2000);
		}
		else if (mode == 2)
		{
			s_feynman_img_depth_depth_transfer cmd;
			cmd.status = 1;
			send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_USB_IMG_DEPTH_DEPTH_TRANSFER_COMMAND, sizeof(s_feynman_img_depth_depth_transfer), (const uint8_t *)&cmd, 2000);

			s_feynman_depth_img_source srccmd;
			srccmd.source = 0;
			send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_SET_DEPTH_DATA_SOURCE_COMMAND, sizeof(s_feynman_depth_img_source), (const uint8_t *)&srccmd, 2000);
		}
	}
}
void feynman_setmode(int mode)
{ //0.vi 1.vpss 2.depth 3.cnn 4.upgrade
	if (feynman_hasconnect())
	{
		s_feynman_run_config cmd;
		cmd.app_run_mode = mode;
		send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_SET_RUN_CONFIGURATION_COMMAND, sizeof(s_feynman_run_config), (const uint8_t *)&cmd, 2000);
	}
}
void feynman_setvi()
{
	s_feynman_ir_img_source cmd;
	cmd.source = 0;
	send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_SET_IR_DATA_SOURCE_COMMAND, sizeof(s_feynman_ir_img_source), (const uint8_t *)&cmd, 2000);
}
void feynman_setvpss()
{
	s_feynman_ir_img_source cmd;
	cmd.source = 1;
	send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_SET_IR_DATA_SOURCE_COMMAND, sizeof(s_feynman_ir_img_source), (const uint8_t *)&cmd, 2000);
}
void feynman_setdepthdenoise(int on)
{
	s_depthconfig.app_depth_denoise = on;
	send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_SET_DEPTH_CONFIGURATION_COMMAND, sizeof(s_feynman_depth_config), (const uint8_t *)&s_depthconfig, 2000);
}
void feynman_setdepthfusion(int on)
{
	s_depthconfig.app_depth_fusion = on;
	send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_SET_DEPTH_CONFIGURATION_COMMAND, sizeof(s_feynman_depth_config), (const uint8_t *)&s_depthconfig, 2000);
}
void feynman_setdepthzoom(int on)
{
	s_depthconfig.app_depth_zoom = on;
	send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_SET_DEPTH_CONFIGURATION_COMMAND, sizeof(s_feynman_depth_config), (const uint8_t *)&s_depthconfig, 2000);
}
void feynman_setdepthstitch(int on)
{
	s_depthconfig.app_depth_stitch = on;
	send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_SET_DEPTH_CONFIGURATION_COMMAND, sizeof(s_feynman_depth_config), (const uint8_t *)&s_depthconfig, 2000);
}
void feynman_imuenable(int enable)
{
	if (enable)
	{
		s_feynman_imu_transfer tmptransfer;
		tmptransfer.status = 1;

		send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_USB_IMU_TRANSFER_COMMAND, sizeof(s_feynman_imu_transfer), (const uint8_t *)&tmptransfer, 2000);
	}
	else
	{
		s_feynman_imu_transfer tmptransfer;
		tmptransfer.status = 0;
		send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_USB_IMU_TRANSFER_COMMAND, sizeof(s_feynman_imu_transfer), (const uint8_t *)&tmptransfer, 2000);
	}
}
static libusb_device_handle *my_open_device_with_vid_pid(
	libusb_context *ctx, uint16_t vendor_id, uint16_t product_id, int bus, int device)
{
	struct libusb_device **devs;
	struct libusb_device *found = NULL;
	struct libusb_device *dev;
	struct libusb_device_handle *dev_handle = NULL;
	size_t i = 0;
	int r;

	if (libusb_get_device_list(ctx, &devs) < 0)
	{
		printf("fail to get device list!\n");
		return NULL;
	}

	while ((dev = devs[i++]) != NULL)
	{
		struct libusb_device_descriptor desc;
		r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0)
			goto out;
		int tmpbus = libusb_get_bus_number(dev);
		int tmpdevice = libusb_get_device_address(dev);
		if (tmpdevice == device && tmpbus == bus && desc.idVendor == vendor_id && desc.idProduct == product_id)
		{
			found = dev;
			break;
		}
	}

	if (found)
	{
		r = libusb_open(found, &dev_handle);
		if (r < 0)
		{
			printf("fail to open dev_handle!%d\n", r);
			dev_handle = NULL;
		}
	}

out:
	libusb_free_device_list(devs, 1);
	printf("to the end,return dev_handle!\n");
	return dev_handle;
}
void feynman_refresh(DEVICECALLBACK callback, void *userdata)
{
	libusb_device **devs;
	int r;
	ssize_t cnt;

	r = libusb_init(NULL);
	if (r < 0)
		return; // r;
	cnt = libusb_get_device_list(NULL, &devs);
	if (cnt < 0)
	{
		libusb_exit(NULL);
		return; // (int)cnt;
	}
	printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
	libusb_device *dev;
	int i = 0, j = 0;
	uint8_t path[8];

	while ((dev = devs[i++]) != NULL)
	{
		struct libusb_device_descriptor desc;
		int r = libusb_get_device_descriptor(dev, &desc);
		if (r < 0)
		{
			fprintf(stderr, "failed to get device descriptor");
			libusb_exit(NULL);
			return;
		}
		int bus = libusb_get_bus_number(dev);
		int device = libusb_get_device_address(dev);
		if (desc.idVendor == FEYNMAN_VID && desc.idProduct == FEYNMAN_PID)
		{
			/*	libusb_device_handle* handler = libusb_open_device_with_vid_pid(NULL, 0x0525, 0xa4a2);
			if (handler == NULL) {
			printf("fail to open usb device!\n");
			}
			else {
			printf("open usb device ok!");
			}*/
			//if (0 == ret) {
			static unsigned char *tmpbuf = 0;
			if (0 == tmpbuf)
				tmpbuf = (unsigned char *)malloc(128);
			struct libusb_device_descriptor des;
			struct libusb_device_handle *dev_handle = libusb_open_device_with_vid_pid(NULL, FEYNMAN_VID, FEYNMAN_PID);

			//	libusb_get_string_descriptor_ascii(dev_handle, desc.iProduct, tmpbuf, 128);
			//	printf("got kepler!:%s\n", tmpbuf);
			sprintf((char *)tmpbuf, "feynman-%d-%d", bus, device);
			callback((const char *)tmpbuf, userdata);
			libusb_close(dev_handle);
			//		libusb_close(handler);

			//	}
			//	else {
			//		printf("open usb ret:%d\n", ret);
			//	}
		}
		printf("%04x:%04x (bus %d, device %d)",
			   desc.idVendor, desc.idProduct, bus, device);

		r = libusb_get_port_numbers(dev, path, sizeof(path));
		if (r > 0)
		{
			printf(" path: %d", path[0]);
			for (j = 1; j < r; j++)
				printf(".%d", path[j]);
		}
		printf("\n");
	}
	printf("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
	libusb_free_device_list(devs, 1);

	libusb_exit(NULL);
}
static int usb_hal_read(uint8_t *data, int len, int *bytesTransffered)
{
	int32_t timeout = 2000;

	int ret = libusb_bulk_transfer(g_h_dev_usb, FEYNMAN_ENDPOINT_IN, (unsigned char *)data, len, bytesTransffered, timeout);

	//printf("has recv:%d bytes\n", *bytesTransffered);
	if (*bytesTransffered > 0)
	{
		ret = *bytesTransffered;
	}
	return ret;
}

void feynman_setprojector(int status)
{
	if (feynman_hasconnect())
	{
		send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_SET_PROJECTOR_CURRENT_COMMAND, sizeof(int), (const uint8_t *)&status, 2000);
	}
	/*if (NULL == sendqueue)
	{
		sendqueue = new Ring_Queue(9, sizeof(sendcmd_t));
	}
	if (feynman_hasconnect())
	{
		sendcmd_t *p = 0;
		p = (sendcmd_t *)sendqueue->SOLO_Write();
		if (p)
		{
			memset(p, 0, sizeof(sendcmd_t));
			p->cmdsubtype = FEYNMAN_COMMAND_SET_PROJECTOR_CURRENT_COMMAND;
			int *pstatus = (int *)malloc(sizeof(int));
			*pstatus = status;
			p->data = pstatus;
			p->len = sizeof(int);
			p->timeout = 2000;

			sendqueue->SOLO_Write_Over();
		}
	}*/
}
static void on_usb_data_receive(uint8_t *data, int len, FRAMECALLBACK callback)
{
	if (len < sizeof(FEYNMAN_USBHeaderDataPacket))
	{
		printf("data too short:%d<=%d\n", len, sizeof(FEYNMAN_USBHeaderDataPacket));
		return;
	}

	if (0 != strncmp("NEXT_VPU", (const char *)data, strlen("NEXT_VPU")))
	{
		printf("first 8 character is not NEXT_VPU!\n");
		return;
	}

	FEYNMAN_USBHeaderDataPacket *tmppack = (FEYNMAN_USBHeaderDataPacket *)data;
	if (tmppack->len != (len - sizeof(FEYNMAN_USBHeaderDataPacket)))
	{
		printf("len not equal:%d,%d\n", len - sizeof(FEYNMAN_USBHeaderDataPacket), tmppack->len);
		return;
	}

	/*	if (tmppack->type == FEYNMAN_COMMAND_DATA)
	{ //&&tmppack->sub_type== FEYNMAN_COMMAND_GET_DEVICE_SN_RETURN) {
		if (tmppack->sub_type == FEYNMAN_COMMAND_GET_DEPTH_CONFIGURATION_RETURN ||
			tmppack->sub_type == FEYNMAN_COMMAND_SET_DEPTH_CONFIGURATION_RETURN)
		{
			s_depthconfig = *(s_feynman_depth_config *)tmppack->data;
		}

		callback(tmppack, connectuserdata);
	}
	else if (tmppack->type == FEYNMAN_LOG_DATA)
	{
		callback(tmppack, connectuserdata);
	}
	else if (tmppack->type == FEYNMAN_CNN_DATA)
	{
		callback(tmppack, connectuserdata);
	}
	else */
	if (tmppack->type == FEYNMAN_IMAGE_DATA)
	{ //here,we will save data to rgb/depth/ir/save queue
		if (NULL == rgbpktqueue)
		{
			rgbpktqueue = new Ring_Queue(32, sizeof(pkt_info_custom_t));
		}
		if (NULL == depthpktqueue)
		{
			depthpktqueue = new Ring_Queue(32, sizeof(pkt_info_custom_t));
		}
		if (NULL == irpktqueue)
		{
			irpktqueue = new Ring_Queue(32, sizeof(pkt_info_custom_t));
		}
		if (NULL == savepktqueue)
		{
			savepktqueue = new Ring_Queue(32, sizeof(pkt_info_custom_t));
		}

		pkt_info_custom_t *p = 0;
		p = (pkt_info_custom_t *)savepktqueue->SOLO_Write();
		if (p)
		{
			p->len = len;
			memcpy((unsigned char *)p->buffer, data, len);
			savepktqueue->SOLO_Write_Over();
		}
		else
		{
			//printf("fail image one!\n");
		}

		if (tmppack->sub_type == FEYNMAN_DEPTH_IMAGE)
		{
			pkt_info_custom_t *p = 0;
			p = (pkt_info_custom_t *)depthpktqueue->SOLO_Write();
			if (p)
			{
				p->len = len;
				memcpy((unsigned char *)p->buffer, data, len);
				depthpktqueue->SOLO_Write_Over();
			}
			else
			{
				//printf("fail image one!\n");
			}
		}
		if (tmppack->sub_type == FEYNMAN_DEPTH_IMAGE_LEFT_RAW)
		{
			pkt_info_custom_t *p = 0;
			p = (pkt_info_custom_t *)depthpktqueue->SOLO_Write();
			if (p)
			{
				p->len = len;
				memcpy((unsigned char *)p->buffer, data, len);
				depthpktqueue->SOLO_Write_Over();
			}
			else
			{
				//printf("fail image one!\n");
			}
			p = (pkt_info_custom_t *)irpktqueue->SOLO_Write();
			if (p)
			{
				p->len = len;
				memcpy((unsigned char *)p->buffer, data, len);
				irpktqueue->SOLO_Write_Over();
			}
			else
			{
				//printf("fail image one!\n");
			}
		}
		if (tmppack->sub_type == FEYNMAN_DEPTH_IMAGE_RIGHT_RAW ||
			tmppack->sub_type == FEYNMAN_IR_IMAGE_LEFT_VI ||
			tmppack->sub_type == FEYNMAN_IR_IMAGE_RIGHT_VI ||
			tmppack->sub_type == FEYNMAN_IR_IMAGE_LEFT_VPSS ||
			tmppack->sub_type == FEYNMAN_IR_IMAGE_RIGHT_VPSS)
		{
			pkt_info_custom_t *p = 0;
			p = (pkt_info_custom_t *)irpktqueue->SOLO_Write();
			if (p)
			{
				p->len = len;
				memcpy((unsigned char *)p->buffer, data, len);
				irpktqueue->SOLO_Write_Over();
			}
			else
			{
				//printf("fail image one!\n");
			}
		}
		if (tmppack->sub_type == FEYNMAN_RGB_IMAGE_SINGLE)
		{
			pkt_info_custom_t *p = 0;
			p = (pkt_info_custom_t *)rgbpktqueue->SOLO_Write();
			if (p)
			{
				p->len = len;
				memcpy((unsigned char *)p->buffer, data, len);
				rgbpktqueue->SOLO_Write_Over();
			}
			else
			{
				//printf("fail image one!\n");
			}
		}
	}
	else if (tmppack->type == FEYNMAN_IMU_DATA)
	{
		if (NULL == imupktqueue)
		{
			imupktqueue = new Ring_Queue(9, sizeof(pkt_info_custom_t));
		}
		pkt_info_custom_t *p = 0;
		p = (pkt_info_custom_t *)imupktqueue->SOLO_Write();
		if (p)
		{
			p->len = len;
			memcpy((unsigned char *)p->buffer, data, len);
			imupktqueue->SOLO_Write_Over();
		}
	}
	else
	{

		callback(tmppack, connectuserdata);
		/*		if (NULL == rawpktqueue)
		{
			rawpktqueue = new Ring_Queue(9, sizeof(pkt_info_custom_t));
		}
		pkt_info_custom_t *p = 0;
		p = (pkt_info_custom_t *)rawpktqueue->SOLO_Write();
		if (p)
		{
			p->len = len;
			memcpy((unsigned char *)p->buffer, data, len);
			rawpktqueue->SOLO_Write_Over();
		}*/
	}
}

//pthread_cond_t  g_cond  = PTHREAD_COND_INITIALIZER;

#ifdef _WINDOWS
static unsigned __stdcall saveprocessthread(void *param)
{
#else
static void *saveprocessthread(void *param)
{
#endif
	FRAMECALLBACK callback = (FRAMECALLBACK)param;
	if (NULL == savepktqueue)
	{
		savepktqueue = new Ring_Queue(9, sizeof(pkt_info_custom_t));
	}

#ifdef _WINDOWS
	HANDLE waitevent = ::CreateEvent(NULL, FALSE, FALSE, NULL);
#endif
	while (g_thread_running_flag)
	{
		pkt_info_custom_t *p = 0;
		p = (pkt_info_custom_t *)savepktqueue->SOLO_Read();
		if (p)
		{
			int len = p->len;
			static uint8_t *tmpbuf = (uint8_t *)malloc(1280 * 800 * 3);
			memcpy(tmpbuf, (unsigned char *)p->buffer, len);
			savepktqueue->SOLO_Read_Over();

			callback(tmpbuf, connectuserdata);
			//	free(tmpbuf);
		}
		else
		{
#ifdef _WINDOWS
			WaitForSingleObject(waitevent, 5);
#else
			/*	struct timeval now;
			struct timespec outtime;
			gettimeofday(&now, NULL);
			outtime.tv_sec = now.tv_sec + 5;
			outtime.tv_nsec = now.tv_usec * 1000; 
    		int ret = pthread_cond_timedwait(&g_cond,NULL, &outtime);*/
			usleep(10000 * 1);
#endif
		}
	}

#ifdef _WINDOWS
	CloseHandle(waitevent);
#endif
	return 0;
}

#ifdef _WINDOWS
static unsigned __stdcall imuprocessthread(void *param)
{
#else
static void *imuprocessthread(void *param)
{
#endif
	FRAMECALLBACK callback = (FRAMECALLBACK)param;
	if (NULL == imupktqueue)
	{
		imupktqueue = new Ring_Queue(9, sizeof(pkt_info_custom_t));
	}

#ifdef _WINDOWS
	HANDLE waitevent = ::CreateEvent(NULL, FALSE, FALSE, NULL);
#endif
	while (g_thread_running_flag)
	{
		pkt_info_custom_t *p = 0;
		p = (pkt_info_custom_t *)imupktqueue->SOLO_Read();
		if (p)
		{
			int len = p->len;
			uint8_t *tmpbuf = (uint8_t *)malloc(1280 * 800);
			memcpy(tmpbuf, (unsigned char *)p->buffer, len);
			imupktqueue->SOLO_Read_Over();

			callback(tmpbuf, connectuserdata);
			//		free(tmpbuf);
		}
		else
		{
#ifdef _WINDOWS
			WaitForSingleObject(waitevent, 5);
#else
			/*	struct timeval now;
			struct timespec outtime;
			gettimeofday(&now, NULL);
			outtime.tv_sec = now.tv_sec + 5;
			outtime.tv_nsec = now.tv_usec * 1000; 
    		int ret = pthread_cond_timedwait(&g_cond,NULL, &outtime);*/
			usleep(10000 * 1);
#endif
		}
	}

#ifdef _WINDOWS
	CloseHandle(waitevent);
#endif
	return 0;
}

#ifdef _WINDOWS
static unsigned __stdcall rgbprocessthread(void *param)
{
#else
static void *
rgbprocessthread(void *param)
{
#endif
	FRAMECALLBACK callback = (FRAMECALLBACK)param;
	if (NULL == rgbpktqueue)
	{
		rgbpktqueue = new Ring_Queue(32, sizeof(pkt_info_custom_t));
	}

#ifdef _WINDOWS
	HANDLE waitevent = ::CreateEvent(NULL, FALSE, FALSE, NULL);
#endif
	while (g_thread_running_flag)
	{
		pkt_info_custom_t *p = 0;
		p = (pkt_info_custom_t *)rgbpktqueue->SOLO_Read();
		if (p)
		{
			int len = p->len;
			static uint8_t *tmpbuf = (uint8_t *)malloc(1280 * 800 * 3);
			memcpy(tmpbuf, (unsigned char *)p->buffer, len);
			rgbpktqueue->SOLO_Read_Over();

			callback(tmpbuf, connectuserdata);
			//	free(tmpbuf);
		}
		else
		{
#ifdef _WINDOWS
			WaitForSingleObject(waitevent, 5);
#else
			/*	struct timeval now;
			struct timespec outtime;
			gettimeofday(&now, NULL);
			outtime.tv_sec = now.tv_sec + 5;
			outtime.tv_nsec = now.tv_usec * 1000; 
    		int ret = pthread_cond_timedwait(&g_cond,NULL, &outtime);*/
			usleep(10000 * 1);
#endif
		}
	}

#ifdef _WINDOWS
	CloseHandle(waitevent);
#endif
	return 0;
}

#ifdef _WINDOWS
static unsigned __stdcall irprocessthread(void *param)
{
#else
static void *
irprocessthread(void *param)
{
#endif
	FRAMECALLBACK callback = (FRAMECALLBACK)param;
	if (NULL == irpktqueue)
	{
		irpktqueue = new Ring_Queue(32, sizeof(pkt_info_custom_t));
	}

#ifdef _WINDOWS
	HANDLE waitevent = ::CreateEvent(NULL, FALSE, FALSE, NULL);
#endif
	while (g_thread_running_flag)
	{
		pkt_info_custom_t *p = 0;
		p = (pkt_info_custom_t *)irpktqueue->SOLO_Read();
		if (p)
		{
			int len = p->len;
			static uint8_t *tmpbuf = (uint8_t *)malloc(1280 * 800 * 2);
			memcpy(tmpbuf, (unsigned char *)p->buffer, len);
			irpktqueue->SOLO_Read_Over();

			callback(tmpbuf, connectuserdata);
			//	free(tmpbuf);
		}
		else
		{
#ifdef _WINDOWS
			WaitForSingleObject(waitevent, 5);
#else
			/*	struct timeval now;
			struct timespec outtime;
			gettimeofday(&now, NULL);
			outtime.tv_sec = now.tv_sec + 5;
			outtime.tv_nsec = now.tv_usec * 1000; 
    		int ret = pthread_cond_timedwait(&g_cond,NULL, &outtime);*/
			usleep(10000 * 1);
#endif
		}
	}

#ifdef _WINDOWS
	CloseHandle(waitevent);
#endif
	return 0;
}
void feynman_setpointcloudtransfer(int willtransfer)
{
	if (feynman_hasconnect())
	{
		s_feynman_img_pointcloud_transfer tmptransfer;
		tmptransfer.status = willtransfer;

		send_one_packet(FEYNMAN_COMMAND_DATA, FEYNMAN_COMMAND_USB_IMG_POINTCLOUD_TRANSFER_COMMAND, sizeof(s_feynman_img_pointcloud_transfer), (const uint8_t *)&tmptransfer, 2000);
	}
}
#ifdef _WINDOWS
static unsigned __stdcall depthprocessthread(void *param)
{
#else
static void *depthprocessthread(void *param)
{
#endif
	FRAMECALLBACK callback = (FRAMECALLBACK)param;
	if (NULL == depthpktqueue)
	{
		depthpktqueue = new Ring_Queue(32, sizeof(pkt_info_custom_t));
	}

#ifdef _WINDOWS
	HANDLE waitevent = ::CreateEvent(NULL, FALSE, FALSE, NULL);
#endif
	while (g_thread_running_flag)
	{
		pkt_info_custom_t *p = 0;
		p = (pkt_info_custom_t *)depthpktqueue->SOLO_Read();
		if (p)
		{
			int len = p->len;
			static uint8_t *tmpbuf = (uint8_t *)malloc(1280 * 800 * 3);
			memcpy(tmpbuf, (unsigned char *)p->buffer, len);
			depthpktqueue->SOLO_Read_Over();
			if (callback != NULL)
				callback(tmpbuf, connectuserdata);
			//			free(tmpbuf);
		}
		else
		{
#ifdef _WINDOWS
			WaitForSingleObject(waitevent, 5);
#else
			/*	struct timeval now;
			struct timespec outtime;
			gettimeofday(&now, NULL);
			outtime.tv_sec = now.tv_sec + 5;
			outtime.tv_nsec = now.tv_usec * 1000; 
    		int ret = pthread_cond_timedwait(&g_cond,NULL, &outtime);*/
			usleep(10000 * 1);
#endif
		}
	}

#ifdef _WINDOWS
	CloseHandle(waitevent);
#endif
	return 0;
}
#ifdef _WINDOWS
static unsigned __stdcall otherprocessthread(void *param)
{
#else
static void *otherprocessthread(void *param)
{
#endif
	FRAMECALLBACK callback = (FRAMECALLBACK)param;
	if (NULL == otherpktqueue)
	{
		otherpktqueue = new Ring_Queue(9, sizeof(pkt_info_custom_t));
	}

#ifdef _WINDOWS
	HANDLE waitevent = ::CreateEvent(NULL, FALSE, FALSE, NULL);
#endif
	while (g_thread_running_flag)
	{
		pkt_info_custom_t *p = 0;
		p = (pkt_info_custom_t *)otherpktqueue->SOLO_Read();
		if (p)
		{
			int len = p->len;
			uint8_t *tmpbuf = (uint8_t *)malloc(len);
			memcpy(tmpbuf, (unsigned char *)p->buffer, len);
			otherpktqueue->SOLO_Read_Over();

			callback(tmpbuf, connectuserdata);
			free(tmpbuf);
		}
		else
		{
#ifdef _WINDOWS
			WaitForSingleObject(waitevent, 5);
#else
			/*	struct timeval now;
			struct timespec outtime;
			gettimeofday(&now, NULL);
			outtime.tv_sec = now.tv_sec + 5;
			outtime.tv_nsec = now.tv_usec * 1000; 
    		int ret = pthread_cond_timedwait(&g_cond,NULL, &outtime);*/
			usleep(1000 * 1);
#endif
		}
	}

#ifdef _WINDOWS
	CloseHandle(waitevent);
#endif
	return 0;
}

#ifdef _WINDOWS
static unsigned __stdcall usb_loop_send(void *pParam)
{
#else
static void *usb_loop_send(void *pParam)
{
#endif
	if (NULL == sendqueue)
	{
		sendqueue = new Ring_Queue(9, sizeof(sendcmd_t));
	}

	while (g_thread_running_flag)
	{
		sendcmd_t *p = 0;
		p = (sendcmd_t *)sendqueue->SOLO_Read();
		if (p)
		{
			if (feynman_hasconnect())
			{
				send_one_packet(FEYNMAN_COMMAND_DATA, p->cmdsubtype, p->len, (const uint8_t *)p->data, p->timeout);
			}
			if (p->data != NULL)
				free(p->data);
			sendqueue->SOLO_Read_Over();
		}
#ifdef _WINDOWS
		Sleep(1000);
#else
		usleep(1000 * 1000);
#endif
	}
	return 0;
}
#ifdef _WINDOWS
static unsigned __stdcall usb_loop_recv(void *pParam)
{
#else
static void *usb_loop_recv(void *pParam)
{
#endif
	printf("enter usb_loop_recv thread\n");
	FRAMECALLBACK callback = (FRAMECALLBACK)pParam;
	//depthtool* toolobj = (depthtool*)pParam;
	if (g_usb_buf == NULL)
		g_usb_buf = (uint8_t *)malloc(sizeof(uint8_t) * USB_PACKET_MAX_SIZE);

	static unsigned char *tmpbuf = NULL;
	if (NULL == tmpbuf)
		tmpbuf = (unsigned char *)malloc(USB_PACKET_MAX_SIZE);
	int index = 0;
	int currentpacketlen = 0;

	while (g_thread_running_flag)
	{
		int bytesTransffered = 0;
		int32_t ret = usb_hal_read(tmpbuf, USB_PACKET_MAX_SIZE /*sizeof(toolobj->g_usb_buf)*/, &bytesTransffered);

		if (ret == 0)
		{
			printf("received nothing!!!!\n");
			continue;
		}
		else if (ret == LIBUSB_ERROR_TIMEOUT)
		{
			printf("just timeout!!!\n");
			continue;
		}
		else if (ret < 0)
		{
			printf("ret:%d and not equal LIBUSB_ERROR_TIMEOUT,something bad happened!\n", ret);
			g_thread_running_flag = 0;
			continue;
		}

		if ((bytesTransffered >= sizeof(FEYNMAN_USBHeaderDataPacket)) && (index == 0) && (0 == strncmp("NEXT_VPU", (const char *)tmpbuf, 8)))
		{ //ͷ��
			memcpy(g_usb_buf, tmpbuf, bytesTransffered);
			index += bytesTransffered;
			currentpacketlen = ((FEYNMAN_USBHeaderDataPacket *)g_usb_buf)->len;
			if (index == (currentpacketlen + sizeof(FEYNMAN_USBHeaderDataPacket)))
			{

				on_usb_data_receive(g_usb_buf, index, callback);
				index = 0;
			}
		}
		else if (index >= sizeof(FEYNMAN_USBHeaderDataPacket))
		{
			if ((index + bytesTransffered) <= USB_PACKET_MAX_SIZE)
			{
				memcpy(g_usb_buf + index, tmpbuf, bytesTransffered);
				index += bytesTransffered;
				if (index == (currentpacketlen + sizeof(FEYNMAN_USBHeaderDataPacket)))
				{
					on_usb_data_receive(g_usb_buf, index, callback);
					index = 0;
				}
				else if (index > (currentpacketlen + sizeof(FEYNMAN_USBHeaderDataPacket)))
				{
					printf("fail data!!!\n");
					index = 0;
				}
			}
			else
			{
				printf("false data!\n");
				index = 0;
			}
		}
	}
	if (g_thread_running_flag == 0)
	{

		usb_hal_close();
	}
	return 0;
}

static int usb_hal_open(int bus, int device)
{
	int ret;
	char str[64];
	memset(str, 0, sizeof(str));
	ret = libusb_init(NULL);
	if (ret < 0)
	{
		printf("usb init failed");
		return -1;
	}
	g_h_dev_usb = my_open_device_with_vid_pid(NULL, FEYNMAN_VID, FEYNMAN_PID, bus, device);
	if (g_h_dev_usb == NULL)
	{
		printf("open device failed");
		libusb_exit(NULL);
		return -1;
	}
	ret = libusb_claim_interface(g_h_dev_usb, 0);
	if (ret < 0)
	{
		printf("claim interface failed");
		libusb_exit(NULL);
		return -1;
	}
	return 0;
}
static BOOL s_hasconnect = FALSE;
#ifdef _WINDOWS
static HANDLE recvthread, processthread;
static HANDLE sendthread;
#else
static pthread_t recvthread, depththread, rgbthread, imuthread, irthread, savethread, sendthread;
#endif
void feynman_waitfordisconnect()
{
	pthread_join(recvthread, NULL);
	pthread_join(irthread, NULL);
	pthread_join(rgbthread, NULL);
	pthread_join(imuthread, NULL);
	pthread_join(depththread, NULL);
	pthread_join(savethread, NULL);
	pthread_join(sendthread, NULL);

	s_hasconnect = FALSE;
	g_thread_running_flag = 0;
}

void feynman_disconnectcamera()
{
	if (feynman_hasconnect())
	{
		s_hasconnect = FALSE;
		g_thread_running_flag = 0;
#ifdef _WINDOWS
		DWORD retfirst = WaitForSingleObject(recvthread, INFINITE);
		DWORD retsecond = WaitForSingleObject(processthread, INFINITE);
		DWORD retthird = WaitForSingleObject(sendthread, INFINITE);
		printf("ret:%d:%d\n", retfirst, retsecond, retthird);
#else
		pthread_join(recvthread, NULL);
		pthread_join(irthread, NULL);
		pthread_join(rgbthread, NULL);
		pthread_join(imuthread, NULL);
		pthread_join(depththread, NULL);
		pthread_join(savethread, NULL);
		pthread_join(sendthread, NULL);
#endif
	}
}
BOOL feynman_hasconnect()
{
	return s_hasconnect;
}
static void c_split(char *src, const char *separator, int maxlen, char **dest, int *num)
{
	char *pNext;
	int count = 0;
	if (src == NULL || strlen(src) == 0)
		return;
	if (separator == NULL || strlen(separator) == 0)
		return;
	pNext = strtok(src, separator);
	while (pNext != NULL && count < maxlen)
	{
		*dest++ = pNext;
		++count;
		pNext = strtok(NULL, separator);
	}
	*num = count;
}
static unsigned int id_data_process(void *data, int len)
{
	FEYNMAN_USBHeaderDataPacket *packet = (FEYNMAN_USBHeaderDataPacket *)data;
	if (packet->type == FEYNMAN_DEVICE_DATA && packet->sub_type == FEYNMAN_DEVICE_DATA_ALL)
	{
		s_feynman_device_info *tmpinfo = (s_feynman_device_info *)packet->data;
		return tmpinfo->device_id.device_id;
	}
	return 0xffffffff;
}
unsigned int feynman_connectcameraforid(const char *devicename)
{
	int ret = -1;
	char *msg = strdup(devicename);
	int num = 0;
	int bus, device;
	char *split_buf[512] = {0};
	int i = 0;
	c_split((char *)devicename, "-", 512, split_buf, &num);
	if (num < 3)
	{
		printf("fail to split devicename!\n");
		free(msg);
		return 0xffffffff;
	}
	bus = atoi(split_buf[1]);
	device = atoi(split_buf[2]);
	free(msg);
	//////////////////////////////////////////////////////////////////////
	if (usb_hal_open(bus, device) >= 0)
	{
		ret = 1;
		printf("enter usb_loop_recv thread\n");
		if (g_usb_buf == NULL)
			g_usb_buf = (uint8_t *)malloc(sizeof(uint8_t) * USB_PACKET_MAX_SIZE);

		static unsigned char *tmpbuf = NULL;
		if (NULL == tmpbuf)
			tmpbuf = (unsigned char *)malloc(USB_PACKET_MAX_SIZE);
		int index = 0;
		int currentpacketlen = 0;

		g_thread_running_flag = 1;
		while (g_thread_running_flag)
		{
			int bytesTransffered = 0;
			int32_t ret = usb_hal_read(tmpbuf, USB_PACKET_MAX_SIZE /*sizeof(toolobj->g_usb_buf)*/, &bytesTransffered);

			if (ret <= 0)
			{
				continue;
			}

			if ((bytesTransffered >= sizeof(FEYNMAN_USBHeaderDataPacket)) && (index == 0) && (0 == strncmp("NEXT_VPU", (const char *)tmpbuf, 8)))
			{ //头包
				memcpy(g_usb_buf, tmpbuf, bytesTransffered);
				index += bytesTransffered;
				currentpacketlen = ((FEYNMAN_USBHeaderDataPacket *)g_usb_buf)->len;
				if (index == (currentpacketlen + sizeof(FEYNMAN_USBHeaderDataPacket)))
				{
					unsigned tmpid = id_data_process(g_usb_buf, index);
					if (0xffffffff != tmpid)
					{
						usb_hal_close();
						return tmpid;
					}
					index = 0;
				}
			}
			else if (index >= sizeof(FEYNMAN_USBHeaderDataPacket))
			{
				if ((index + bytesTransffered) <= USB_PACKET_MAX_SIZE)
				{
					memcpy(g_usb_buf + index, tmpbuf, bytesTransffered);
					index += bytesTransffered;
					if (index == (currentpacketlen + sizeof(FEYNMAN_USBHeaderDataPacket)))
					{
						unsigned tmpid = id_data_process(g_usb_buf, index);
						if (0xffffffff != tmpid)
						{
							usb_hal_close();
							return tmpid;
						}
						index = 0;
					}
					else if (index > (currentpacketlen + sizeof(FEYNMAN_USBHeaderDataPacket)))
					{
						printf("fail data!!!\n");
						index = 0;
					}
				}
				else
				{
					printf("false data!\n");
					index = 0;
				}
			}
		}
		if (g_thread_running_flag == 0)
		{

			usb_hal_close();
		}
	}

	return 0xffffffff;
}

BOOL feynman_connectcamera(const char *devicename,
						   FRAMECALLBACK imucallback,	//only imu
						   FRAMECALLBACK savecallback,	//all image data
						   FRAMECALLBACK depthcallback, //only depth and depth's left ir
						   FRAMECALLBACK ircallback,	//only ir left and right
						   FRAMECALLBACK rgbcallback,	//only rgb and depth
						   FRAMECALLBACK othercallback, //cnn and log and other
						   void *userdata)
{
	int ret = -1;
	//pthread_t pid;
	//pthread_t pid_upacket;
	//HANDLE thread;
	//HANDLE thread_upacket;
	char *msg = strdup(devicename);
	int num = 0;
	int bus, device;
	char *split_buf[512] = {0};
	int i = 0;
	printf("will split devicename!%s\n", devicename);
	c_split((char *)devicename, "-", 512, split_buf, &num);
	if (num < 3)
	{
		printf("fail to split devicename!%s\n", devicename);
		free(msg);
		return FALSE;
	}
	bus = atoi(split_buf[1]);
	device = atoi(split_buf[2]);
	free(msg);

	connectuserdata = userdata;

	if (usb_hal_open(bus, device) >= 0)
	{
		ret = 1;

		g_thread_running_flag = 1;

		//pthread_create(&pid, NULL, usb_loop_recv, NULL);
		//pthread_create(&pid_upacket, NULL, usb_loop_upacket, NULL);

#ifdef _WINDOWS
		unsigned int threadID;
		recvthread = (HANDLE)_beginthreadex(NULL, 0, usb_loop_recv, (LPVOID)callback, 0, &threadID);
		processthread = (HANDLE)_beginthreadex(NULL, 0, dataprocessthread, (LPVOID)callback, 0, &threadID);
		sendthread = (HANDLE)_beginthreadex(NULL, 0, usb_loop_send, (LPVOID)NULL, 0, &threadID);
#else
		//	pthread_t threadID;
		pthread_create(&recvthread, 0, usb_loop_recv, (void *)othercallback);
		if (depthcallback != NULL)
			pthread_create(&depththread, 0, depthprocessthread, (void *)depthcallback);
		if (ircallback != NULL)
			pthread_create(&irthread, 0, irprocessthread, (void *)ircallback);
		if (rgbcallback != NULL)
			pthread_create(&rgbthread, 0, rgbprocessthread, (void *)rgbcallback);
		if (imucallback != NULL)
			pthread_create(&imuthread, 0, imuprocessthread, (void *)imucallback);
		if (savecallback != NULL)
			pthread_create(&savethread, 0, saveprocessthread, (void *)savecallback);
		pthread_create(&sendthread, 0, usb_loop_send, (void *)0);
#endif
		s_hasconnect = TRUE;
		//HANDLE thread_upacket=(HANDLE)_beginthreadex(NULL, 0, &depthtool::usb_loop_upacket, (LPVOID)this, 0, &threadID);
		return TRUE;
	}
	return FALSE;
}
void feynman_getcamparam(int timeout)
{
	if (NULL == sendqueue)
	{
		sendqueue = new Ring_Queue(9, sizeof(sendcmd_t));
	}
	if (feynman_hasconnect())
	{
		sendcmd_t *p = 0;
		p = (sendcmd_t *)sendqueue->SOLO_Write();
		if (p)
		{
			memset(p, 0, sizeof(sendcmd_t));
			p->cmdsubtype = FEYNMAN_COMMAND_GET_CAM_PARAM_COMMAND;
			p->data = NULL;
			p->len = 0;
			p->timeout = timeout;
			sendqueue->SOLO_Write_Over();
		}
	}
}
void feynman_getprojector(int timeout)
{
	if (NULL == sendqueue)
	{
		sendqueue = new Ring_Queue(9, sizeof(sendcmd_t));
	}
	if (feynman_hasconnect())
	{
		sendcmd_t *p = 0;
		p = (sendcmd_t *)sendqueue->SOLO_Write();
		if (p)
		{
			memset(p, 0, sizeof(sendcmd_t));
			p->cmdsubtype = FEYNMAN_COMMAND_GET_PROJECTOR_CURRENT_COMMAND;
			p->data = NULL;
			p->len = 0;
			p->timeout = timeout;

			sendqueue->SOLO_Write_Over();
		}
	}
}
void feynman_getdepthconfig(int timeout)
{
	if (NULL == sendqueue)
	{
		sendqueue = new Ring_Queue(9, sizeof(sendcmd_t));
	}
	if (feynman_hasconnect())
	{
		sendcmd_t *p = 0;
		p = (sendcmd_t *)sendqueue->SOLO_Write();
		if (p)
		{
			memset(p, 0, sizeof(sendcmd_t));
			p->cmdsubtype = FEYNMAN_COMMAND_GET_DEPTH_CONFIGURATION_COMMAND;
			p->data = NULL;
			p->len = 0;
			p->timeout = timeout;

			sendqueue->SOLO_Write_Over();
		}
	}
}
void feynman_getrunconfig(int timeout)
{
	if (NULL == sendqueue)
	{
		sendqueue = new Ring_Queue(9, sizeof(sendcmd_t));
	}
	if (feynman_hasconnect())
	{
		sendcmd_t *p = 0;
		p = (sendcmd_t *)sendqueue->SOLO_Write();
		if (p)
		{
			memset(p, 0, sizeof(sendcmd_t));
			p->cmdsubtype = FEYNMAN_COMMAND_GET_RUN_CONFIGURATION_COMMAND;
			p->data = NULL;
			p->len = 0;
			p->timeout = timeout;
			sendqueue->SOLO_Write_Over();
		}
	}
}
/*
void feynman_setcapture(IMGTYPE imgtype, BOOL willcapture) {
	if (willcapture) {
		uint8_t current_mode_ = CURRENT_MODE_CAPTURE_AFTER_DSP;
		send_one_packet(CMD_TYPE_CONTROL, CONTROL_SET_CURRENT_MODE, 1, (const uint8_t*)&current_mode_);
	}
	else {
		uint8_t current_mode_ = CURRENT_MODE_NULL;
		send_one_packet(CMD_TYPE_CONTROL, CONTROL_SET_CURRENT_MODE, 1, (const uint8_t*)&current_mode_);
	}
}*/

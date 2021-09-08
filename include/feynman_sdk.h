#ifndef __FEYNMAN__
#define __FEYNMAN__
#ifdef __cplusplus
extern "C"
{
#endif
#define TRUE 1
#define FALSE 0
#ifdef _WINDOWS
	typedef unsigned short uint16_t;
	typedef unsigned char uint8_t;
	typedef unsigned int uint32_t;
	typedef int int32_t;
	typedef short int16_t;
	typedef unsigned long long uint64_t;
#else
#include <stdint.h>
#endif
	typedef signed short NVP_S16;
	typedef unsigned long long NVP_U64;
	typedef float NVP_FLOAT;

	typedef int BOOL;
#define FEYNMAN_ONE_PACKET_DATA_MAX_SIZE (1280 * 800 * 2 + 4)
#define USB_PACKET_MAX_SIZE (1024 * 1024 * 8)
#define ONE_PACKET_DATA_MAX_SIZE (USB_PACKET_MAX_SIZE - 16)
#define PREVIEW_FRAME_DATA_MAX_SIZE (ONE_PACKET_DATA_MAX_SIZE - sizeof(FrameHeader))

	/** FEYNMAN_PRIMARY_TYPE */
	typedef enum
	{
		FEYNMAN_IMAGE_DATA,		  /**< image data 0  */
		FEYNMAN_COMMAND_DATA,	  /**< command data 1  */
		FEYNMAN_IMU_DATA,		  /**< imu data 2  */
		FEYNMAN_DEVICE_DATA,	  /**< device data 3  */
		FEYNMAN_CALIBRATION_DATA, /**< calibration data 4  */
		FEYNMAN_TEST_DATA,		  /**< test data 5  */
		FEYNMAN_UPGRADE_DATA,	  /**< upgrade data 6  */
		FEYNMAN_LOG_DATA,		  /**< log data 7 */
		FEYNMAN_CNN_DATA,		  /**< cnn data 8 */
		FEYNMAN_USER_DATA,		  /**< user define data 9 */
	} FEYNMAN_PRIMARY_TYPE;

	/** FEYNMAN_IMAGE_FORMAT_TYPE */
	typedef enum
	{
		FEYNMAN_IMAGE_FORMAT_RAW10, /**< image format raw10 0  */
		FEYNMAN_IMAGE_FORMAT_YUV2,	/**< image format yuv2 1  */
		FEYNMAN_IMAGE_FORMAT_JPEG,	/**< image format jpeg 2  */
		FEYNMAN_IMAGE_FORMAT_MJPG,	/**< image format mjpg 3  */
		FEYNMAN_IMAGE_FORMAT_INT16, /**< image format int16 4  */
	} FEYNMAN_IMAGE_FORMAT_TYPE;

	/** FEYNMAN_IMAGE_SUB_TYPE */
	typedef enum
	{
		FEYNMAN_IR_IMAGE_LEFT_VI = 0x00, /**< ir sensor left, YUV,δ���궨У׼ 0  */
		FEYNMAN_IR_IMAGE_RIGHT_VI,		 /**< ir sensor right, YUV��δ���궨У׼ 1  */
		FEYNMAN_IR_IMAGE_LEFT_VPSS,		 /**< ir sensor left, YUV���궨У׼�� 2  */
		FEYNMAN_IR_IMAGE_RIGHT_VPSS,	 /**< ir sensor right, YUV���궨У׼�� 3  */
		FEYNMAN_IR_IAMGE_DUAL,			 /**< ir sensor dual��feynman��Ŀ��������Ч 4  */
		FEYNMAN_RGB_IMAGE_SINGLE,		 /**< rgb sensor single, ���� 5  */
		FEYNMAN_RGB_IMAGE_LEFT,			 /**< rgb sensor left��feynman��Ŀ��������Ч 6  */
		FEYNMAN_RGB_IMAGE_RIGHT,		 /**< rgb sensor right��feynman��Ŀ��������Ч 7  */
		FEYNMAN_RGB_IMAGE_DUAL,			 /**< rgb sensorr dual��feynman��Ŀ��������Ч 8  */
		FEYNMAN_DISPARITYIMAGE,			 /**< depth���Ӳ�ͼ��16bit RAW 9  */
		FEYNMAN_DEPTH_IMAGE,			 /**< depth���Ӳ�ͼ��RAW���ݣ�16bit RAW 10  */
		FEYNMAN_DEPTH_IMAGE_LEFT_RAW,	 /**< ����depth����ͼRAW���ݣ�8bit RAW 11  */
		FEYNMAN_DEPTH_IMAGE_RIGHT_RAW,	 /**< ����depth����ͼRAW���ݣ�8bit RAW 12  */
	} FEYNMAN_IMAGE_SUB_TYPE;

	/** FEYNMAN_IMU_DATASUB_TYPE */
	typedef enum
	{
		FEYNMAN_IMU_DATA_ACC = 0x00, /**< imu data of accel 0  */
		FEYNMAN_IMU_DATA_GYO,		 /**< imu data of gyro 1  */
		FEYNMAN_IMU_DATA_MANG,		 /**< imu data of magnet 2  */
		FEYNMAN_IMU_DATA_ALL,		 /**< imu data of all 3  */
		FEYNMAN_IMU_DATA_RES		 /**< imu data of res 4  */
	} FEYNMAN_IMU_DATASUB_TYPE;

	/** FEYNMAN_COMMAND_SUB_TYPE */
	typedef enum
	{
		FEYNMAN_COMMAND_GET_DEVICE_ID_COMMAND = 0x00,						/**< get device id 0  */
		FEYNMAN_COMMAND_GET_DEVICE_ID_RETURN,								/**< return get device id 1  */
		FEYNMAN_COMMAND_GET_DEVICE_SN_COMMAND = 0x02,						/**< get device sn 2  */
		FEYNMAN_COMMAND_GET_DEVICE_SN_RETURN,								/**< return get device sn 3  */
		FEYNMAN_COMMAND_GET_PROJECTOR_CURRENT_COMMAND = 0x10,				/**< get projector current status 16  */
		FEYNMAN_COMMAND_GET_PROJECTOR_CURRENT_RETURN,						/**< return get projector current status 17  */
		FEYNMAN_COMMAND_SET_PROJECTOR_CURRENT_COMMAND = 0x12,				/**< set projector current status 18  */
		FEYNMAN_COMMAND_SET_PROJECTOR_CURRENT_RETURN,						/**< return set projector current status 19  */
		FEYNMAN_COMMAND_GET_IR_IMAGE_SENSOR_RESOLUTION_FPS_COMMAND = 0x20,	/**< get ir sensor resolution/fps  32 */
		FEYNMAN_COMMAND_GET_IR_IMAGE_SENSOR_RESOLUTION_FPS_RETURN,			/**<return get ir sensor resolution and fps 33  */
		FEYNMAN_COMMAND_SET_IR_IMAGE_SENSOR_RESOLUTION_FPS_COMMAND = 0x22,	/**< set ir sensor resolution/fps 34  */
		FEYNMAN_COMMAND_SET_IR_IMAGE_SENSOR_RESOLUTION_FPS_RETURN,			/**< return set ir sensor resolution and fps 35  */
		FEYNMAN_COMMAND_GET_IR_IMAGE_SENSOR_EXPOSURE_COMMAND = 0x24,		/**< get ir image ir sensor exposure 36  */
		FEYNMAN_COMMAND_GET_IR_IMAGE_SENSOR_EXPOSURE_RETURN,				/**< return get ir image sensor exposure 37  */
		FEYNMAN_COMMAND_SET_IR_IMAGE_SENEOR_EXPOSURE_COMMAND = 0x26,		/**< set ir image sensor exposure 38  */
		FEYNMAN_COMMAND_SET_IR_IMAGE_SENSOR_EXPOSURE_RETURN,				/**< return set ir  image senor exposure 39  */
		FEYNMAN_COMMAND_GET_IR_IMAGE_SENSOR_ROI_COMMAND = 0x28,				/**< get ir image sensor ROI(Region of interest) 40  */
		FEYNMAN_COMMAND_GET_IR_IMAGE_SENSOR_ROI_RETURN,						/**< return get ir image sensor ROI(Region of interest) 41  */
		FEYNMAN_COMMAND_SET_IR_IMAGE_SENSOR_ROI_COMMAND = 0x2A,				/**< set ir image sensor ROI(Region of interest) 42  */
		FEYNMAN_COMMAND_SET_IR_IMAGE_SENOSR_ROI_RETURN,						/**< return set ir  image sensor ROI(Region of interest) 43  */
		FEYNMAN_COMMAND_GET_RGB_IMAGE_SENSOR_RESOLUTION_FPS_COMMAND = 0x30, /**< get rgb image sensor resolutoin and fps 48  */
		FEYNMAN_COMMAND_GET_RGB_IMAGE_SENSOR_RESOLUTION_FPS_RETURN,			/**< return get rgb image sensor resolution and fps 49  */
		FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_RESOLUTION_FPS_COMMAND = 0x32, /**< set rgb image sensor resolution  and  fps 50  */
		FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_RESOLUTION_FPS_RETURN,			/**< return set rgb image sensor resolution and fps 51  */
		FEYNMAN_COMMAND_GET_RGB_IMAGE_SENSOR_EXPOSURE_COMMAND = 0x34,		/**< get rgb image sensor exposure 52  */
		FEYNMAN_COMMAND_GET_RGB_IMAGE_SENSOR_EXPOSURE_RETURN,				/**< return rgb image sensor exposure 53  */
		FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_EXPOSURE_COMMAND = 0x36,		/**< set rbg image sensor exposure 54  */
		FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_EXPOSURE_RETURN,				/**< return set rgb image sensor exposure 55  */
		FEYNMAN_COMMAND_GET_RGB_IMAGE_SENSOR_AWB_COMMAND = 0x38,			/**< get rgb image sensor awb 56  */
		FEYNMAN_COMMAND_GET_RGB_IAMGE_SENSOR_AWB_RETURN,					/**< return get rgb image sensor awb 57  */
		FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_AWB_COMMAND = 0x3A,			/**< set rgb image sensor awb 58  */
		FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_AWB_RETURN,					/**< return rgb image sensor awb 59  */
		FEYNMAN_COMMAND_GET_RGB_IMAGE_SENSOR_DIGITAL_GAIN_COMMAND = 0x3C,	/**< get rgb image sensor digital gain 60  */
		FEYNMAN_COMMAND_GET_RGB_IMAGE_SENSOR_DIGITAL_GAIN_RETURN,			/**< return get rgb image sensor digital gain 61  */
		FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_DIGITAL_GAIN_COMMAND = 0x3E,	/**< set rgb image sensor digital gain 62 */
		FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_DIGITAL_GAIN_RETURN,			/**< return set rgb image sensor digital gain 63  */
		FEYNMAN_COMMAND_GET_RGB_IMAGE_SENSOR_ANALOG_GAIN_COMMAND = 0x40,	/**< get rbg image sensor analog gain 64 */
		FEYNMAN_COMMAND_GET_RGB_IMAGE_SENSOR_ANALOG_GAIN_RETURN,			/**< return get rbg image sensor analog gai 65  */
		FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_ANALOG_GAIN_COMMAND = 0x42,	/**< set rgb image sensor analog gain 66  */
		FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_ANALOG_GAIN_RETURN,			/**< return set rgb image sensor analog gain 67  */
		FEYNMAN_COMMAND_GET_RGB_IMAGE_SENSOR_DATA_FORMAT_COMMAND = 0x50,	/**< get rbg image sensor data format 80  */
		FEYNMAN_COMMAND_GET_RGB_IMAGE_SENSOR_DATA_FORMAT_RETURN,			/**< return get rbg image sensor data format 81  */
		FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_DATA_FORMAT_COMMAND = 0x52,	/**< set rgb image sensor data format 82  */
		FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_DATA_FORMAT_RETURN,			/**< return set rgb image sensor data format 83  */
		FEYNMAN_COMMAND_GET_IR_IMAGE_SENSOR_DATA_FORMAT_COMMAND = 0x54,		/**< get ir image sensor data format 84  */
		FEYNMAN_COMMAND_GET_IR_IMAGE_SENSOR_DATA_FORMAT_RETURN,				/**< return get ir image sensor data format 85  */
		FEYNMAN_COMMAND_SET_IR_IMAGE_SENSOR_DATA_FORMAT_COMMAND = 0x56,		/**< set ir image sensor data format 86  */
		FEYNMAN_COMMAND_SET_IR_IMAGE_SENSOR_DATA_FORMAT_RETURN,				/**< return set ir image sensor data format 87  */
		FEYNMAN_COMMAND_GET_DEPTH_DATA_FORMAT_COMMAND = 0x58,				/**< get depth data format 88  */
		FEYNMAN_COMMAND_GET_DEPTH_DATA_FORMAT_RETURN,						/**< return get depth data format 89  */
		FEYNMAN_COMMAND_SET_DEPTH_DATA_FORMAT_COMMAND = 0x5A,				/**< set depth data format 90  */
		FEYNMAN_COMMAND_SET_DEPTH_DATA_FORMAT_RETURN,						/**< return set depth data format 91  */
		FEYNMAN_COMMAND_GET_RGBD_DATA_FORMAT_COMMAND = 0x5C,				/**< get rgbd data format 92  */
		FEYNMAN_COMMAND_GET_RGBD_DATA_FORMAT_RETURN,						/**< return get rgbd data format 93  */
		FEYNMAN_COMMAND_SET_RGBD_DATA_FORMAT_COMMAND = 0x5E,				/**< set rgbd data format 94  */
		FEYNMAN_COMMAND_SET_RGBD_DATA_FORMAT_RETURN,						/**< return set rgbd data format 95  */
		FEYNMAN_COMMAND_GET_OPERATING_MODE_COMMAND = 0x70,					/**< get operating mode 112  */
		FEYNMAN_COMMAND_GET_OPERATING_MODE_RETURN,							/**< return get operating mode 113  */
		FEYNMAN_COMMAND_SET_OPERATING_MODE_COMMAND = 0x72,					/**< set operating mode 114  */
		FEYNMAN_COMMAND_SET_OPERATING_MODE_RETURN,							/**< return set operating mode 115  */
		FEYNMAN_COMMAND_GET_CALIBRATION_MODE_COMMAND = 0x80,				/**< get calibration paramter 128  */
		FEYNMAN_COMMAND_GET_CALIBRATION_MODE_RETURN,						/**< return get calibration paramter 129  */
		FEYNMAN_COMMAND_SET_CALIBRATION_PARAMTER_COMMAND = 0x82,			/**< set auto calibration 130  */
		FEYNMAN_COMMAND_SET_CALIBRATION_PARAMTER_RETURN,					/**< return set auto calibration 131  */
		FEYNMAN_COMMAND_GET_IMU_CONFIGURATION_COMMAND = 0x90,				/**< get imu configuration 144  */
		FEYNMAN_COMMAND_GET_IMU_CONFIGURATION_RETURN,						/**< return get imu configuration 145  */
		FEYNMAN_COMMAND_SET_IMU_CONFIGURATION_COMMAND = 0x92,				/**< set imu configuration 146  */
		FEYNMAN_COMMAND_SET_IMU_CONFIGURATION_RETURN,						/**< set imu configuration 147  */
		FEYNMAN_COMMAND_GET_DEPTH_CONFIGURATION_COMMAND = 0x94,				/**< get depth config 148  */
		FEYNMAN_COMMAND_GET_DEPTH_CONFIGURATION_RETURN,						/**< return get depth config 149  */
		FEYNMAN_COMMAND_SET_DEPTH_CONFIGURATION_COMMAND = 0x96,				/**< set depth config 150  */
		FEYNMAN_COMMAND_SET_DEPTH_CONFIGURATION_RETURN,						/**< return set depth config 151  */

		FEYNMAN_COMMAND_GET_RUN_CONFIGURATION_COMMAND = 0x98, /**< get run config 152  */
		FEYNMAN_COMMAND_GET_RUN_CONFIGURATION_RETURN,		  /**< return get run config 153  */
		FEYNMAN_COMMAND_SET_RUN_CONFIGURATION_COMMAND = 0x9A, /**< set run config 154  */
		FEYNMAN_COMMAND_SET_RUN_CONFIGURATION_RETURN,		  /**< return set run config 155  */

		FEYNMAN_COMMAND_SAVE_CONFIGURATION_COMMAND = 0x9C, /**< save config 156  */
		FEYNMAN_COMMAND_SAVE_CONFIGURATION_RETURN,		   /**< return save config 157  */

		FEYNMAN_COMMAND_SET_IR_DATA_SOURCE_COMMAND = 0xA0,			 /**< set ir image data source, 0-vi 1-vpss,after rectify 160  */
		FEYNMAN_COMMAND_SET_IR_DATA_SOURCE_RETURN,					 /**< return set ir image data source, 0-vi 1-vpss,after rectify 161  */
		FEYNMAN_COMMAND_SET_DEPTH_DATA_SOURCE_COMMAND = 0xA2,		 /**< set depth image data source, 0-DISPARITY raw, 1-DEPTH raw 162  */
		FEYNMAN_COMMAND_SET_DEPTH_DATA_SOURCE_RETURN,				 /**< return set depth image data source, 0-DISPARITY raw, 1-DEPTH raws 163  */
		FEYNMAN_COMMAND_USB_IMG_TRANSFER_COMMAND = 0xA4,			 /**< usb image tarnsfer 164  */
		FEYNMAN_COMMAND_USB_IMG_TRANSFER_RETURN,					 /**< return  usb image tarnsfer 165  */
		FEYNMAN_COMMAND_GET_CAM_PARAM_COMMAND = 0xA6,				 /**< get camera param 166  */
		FEYNMAN_COMMAND_GET_CAM_PARAM_RETURN,						 /**< return get camera param 167  */
		FEYNMAN_COMMAND_USB_IMU_TRANSFER_COMMAND = 0xA8,			 /**<imu transfer 168  */
		FEYNMAN_COMMAND_USB_IMU_TRANSFER_RETURN,					 /**< return imu transfer 169  */
		FEYNMAN_COMMAND_USB_PIPELINE_COMMAND = 0xAA,				 /**< start/close pipeline 170  */
		FEYNMAN_COMMAND_USB_PIPELINE_RETURN,						 /**< return start/close pipeline 171  */
		FEYNMAN_COMMAND_USB_IMG_DEPTH_IR_TRANSFER_COMMAND = 0xAC,	 /**< image depth ir transfer 172  */
		FEYNMAN_COMMAND_USB_IMG_DEPTH_IR_TRANSFER_RETURN,			 /**< return image depth ir transfer 173  */
		FEYNMAN_COMMAND_USB_IMG_DEPTH_DEPTH_TRANSFER_COMMAND = 0xAE, /**< image depth depth transfer 174  */
		FEYNMAN_COMMAND_USB_IMG_DEPTH_DEPTH_TRANSFER_RETURN			 /**< return image depth depth transfer 175  */
	} FEYNMAN_COMMAND_SUB_TYPE;

	/** FEYNMAN_UPGRADE_SUB_TYPE */
	typedef enum
	{
		FEYNMAN_COMMAND_USB_UPGRADE_FEYNMAN = 0x00, /**< upgrade feynman file 0  */
		FEYNMAN_COMMAND_USB_UPGRADE_FEYNMAN_RETURN, /**< upgrade feynman file return data type 1  */
		FEYNMAN_COMMAND_USB_UPGRADE_LIB,			/**< upgrade library file 2  */
		FEYNMAN_COMMAND_USB_UPGRADE_LIB_RETURN,		/**< upgrade library file return data type 3  */
		FEYNMAN_COMMAND_USB_UPGRADE_FILE,			/**< upgrade other type of file 4  */
		FEYNMAN_COMMAND_USB_UPGRADE_FILE_RETURN		/**< upgrade file return data type 5  */
	} FEYNMAN_UPGRADE_SUB_TYPE;

	/** FEYNMAN_SENSOR_RESOLUTION_TYPE */
	typedef enum
	{
		FEYNMAN_RESOLUTION_1280_800, /**< resolution 1280x800 0 */
		FEYNMAN_RESOLUTION_1280_720, /**< resolution 1280x800 1 */
		FEYNMAN_RESOLUTION_640_480,	 /**< resolution 1280x800 2 */
	} FEYNMAN_SENSOR_RESOLUTION_TYPE;

	/** FEYNMAN_DEVICE_DATASUB_TYPE */
	typedef enum
	{
		FEYNMAN_DEVICE_DATA_ALL = 0x00, /**< all data data type 0  */
	} FEYNMAN_DEVICE_DATASUB_TYPE;

	/** FEYNMAN_LOG_DATA_SUB_TYPE */
	typedef enum
	{
		FEYNMAN_LOG_ALL = 0x00, /**< log data type 0  */
	} FEYNMAN_LOG_DATA_SUB_TYPE;

	//for FEYNMAN_COMMAND_GET_RUN_CONFIGURATION_COMMAND & FEYNMAN_COMMAND_SET_RUN_CONFIGURATION_COMMAND
	typedef struct
	{
		int app_run_mode; //0-vi 1-vpss 2-depth 3-CNN demo
	} s_feynman_run_config;

	//for FEYNMAN_COMMAND_SAVE_CONFIGURATION_COMMAND
	typedef struct
	{
		int status; //0-save success, else save faile
	} s_feynman_save_config;

	/** FEYNMAN_CNN_DATASUB_TYPE */
	typedef enum
	{
		FEYNMAN_CNN_DATA_ALL = 0x00, /**< cnn data type 0  */
	} FEYNMAN_CNN_DATASUB_TYPE;

	/** FEYNMAN_USER_DATA_SUB_TYPE */
	typedef enum
	{
		FEYNMAN_USER_DATA_TO_PC = 0X00, /**< user defined data from camera to pc 0  */
		FEYNMAN_USER_DATA_TO_BOARD		/**< user defined data from pc to camera 1  */
	} FEYNMAN_USER_DATA_SUB_TYPE;

	/**@struct FEYNMAN_USBHeaderDataPacket
	* @brief usb packet struct \n
	* define usb communication packet struct
	*/
	typedef struct
	{
		uint8_t magic[8];  /**< "NEXT_VPU" magic word */
		uint16_t type;	   /**< primary type */
		uint16_t sub_type; /**< sub type */
		uint32_t checksum; /**< checksum */
		uint32_t len;	   /**< length of data member in bytes,len<FEYNMAN_ONE_PACKET_DATA_MAX_SIZE */
		uint8_t data[0];   /**< pure data,mean depends type and sub_type,length depends len */
	} FEYNMAN_USBHeaderDataPacket;

	/**@struct s_feynman_device_id
	* @brief feynman device id struct \n
	* define feynman device id struct,corresponding FEYNMAN_COMMAND_GET_DEVICE_ID_COMMAND
	*/
	typedef struct
	{
		uint32_t device_id; /**< device id of feynman */
	} s_feynman_device_id;

	/**@struct s_feynman_device_sn
	* @brief feynman device sn struct \n
	* define feynman device sn struct,corresponding FEYNMAN_COMMAND_GET_DEVICE_SN_COMMAND
	*/
	typedef struct
	{
		uint8_t sn[16]; /**< device sn of feynman,sn[15]Ϊż�����ǹ������� Ϊ������������ */
	} s_feynman_device_sn;

	/**@struct s_feynman_current_infor
	* @brief feynman device projector status \n
	* define feynman device projector status,corresponding FEYNMAN_COMMAND_GET_PROJECTOR_CURRENT_COMMAND/FEYNMAN_COMMAND_SET_PROJECTOR_CURRENT_COMMAND
	*/
	typedef struct
	{
		uint32_t crruent_value; /**< projector status of feynman device,0-�ر�Ͷ���� ����0-��Ͷ���������õ���ֵ */
	} s_feynman_current_infor;

	/**@struct s_feynman_depth_config
	* @brief feynman device depth config struct \n
	* define feynman device depth config,corresponding FEYNMAN_COMMAND_GET_DEPTH_CONFIGURATION_COMMAND/FEYNMAN_COMMAND_SET_DEPTH_CONFIGURATION_COMMAND
	*/
	typedef struct
	{
		int app_depth_mode;	   /**< depth mode,value:0,1,2,3 */
		int app_depth_denoise; /**< depth denoise,value:/0, 1 */
		int app_depth_fusion;  /**< depth fusion,value:0, 1 */
		int app_depth_zoom;	   /**< depth zoom,value:0,1 */
		int app_depth_stitch;  /**< depth stitch,value:0,1 */
	} s_feynman_depth_config;

	/**@struct s_feynman_sensor_resolution_fps_infor
	* @brief feynman device sensor resolution and fps information struct \n
	* define feynman device sensor resolution and fps information,corresponding FEYNMAN_COMMAND_GET_IR_IMAGE_SENSOR_RESOLUTION_FPS_COMMAND/FEYNMAN_COMMAND_SET_IR_IMAGE_SENSOR_RESOLUTION_FPS_COMMAND
	*/
	typedef struct
	{
		uint32_t resolution; /**< resolution */
		uint32_t fps;		 /**< frames per second */
	} s_feynman_sensor_resolution_fps_infor;

	/**@struct s_feynman_rgb_sensor_exposure_infor
	* @brief feynman device rgb sensor exposure information struct \n
	* define feynman device rgb sensor exposure information,corresponding FEYNMAN_COMMAND_GET_RGB_IMAGE_SENSOR_EXPOSURE_COMMAND/FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_EXPOSURE_COMMAND
	*/
	typedef struct
	{
		uint32_t exposure_time; /**< exposure time in milliseconds */
	} s_feynman_rgb_sensor_exposure_infor;

	/**@struct s_feynman_rgb_awb_infor
	* @brief feynman device rgb awb information struct \n
	* define feynman device rgb awb information,corresponding FEYNMAN_COMMAND_GET_RGB_IMAGE_SENSOR_AWB_COMMAND/FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_AWB_COMMAND
	*/
	typedef struct
	{
		uint32_t awb_value; /**< awb value */
	} s_feynman_rgb_awb_infor;

	/**@struct s_feynman_rgb_digit_gain_infor
	* @brief feynman device rgb digit gain information struct \n
	* define feynman device rgb digit gain information,corresponding FEYNMAN_COMMAND_GET_RGB_IMAGE_SENSOR_DIGITAL_GAIN_COMMAND/FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_DIGITAL_GAIN_COMMAND
	*/
	typedef struct
	{
		uint32_t digit_gain_value; /**< digit gain */
	} s_feynman_rgb_digit_gain_infor;

	/**@struct s_feynman_rgb_analog_gain_infor
	* @brief feynman device rgb analog gain information struct \n
	* define feynman device rgb analog gain information,corresponding FEYNMAN_COMMAND_GET_RGB_IMAGE_SENSOR_ANALOG_GAIN_COMMAND/FEYNMAN_COMMAND_SET_RGB_IMAGE_SENSOR_ANALOG_GAIN_COMMAND
	*/
	typedef struct
	{
		uint32_t analog_gain_value; /**< analog gain */
	} s_feynman_rgb_analog_gain_infor;

	/**@struct s_feynman_roi_infor
	* @brief feynman device region of interest information struct \n
	* define feynman device region of interest  information
	*/
	typedef struct
	{
		int x1; /**< left top point coordinate's x value of roi rectangle */
		int y1; /**< left top point coordinate's y value of roi rectangle */
		int x2; /**< right bottom point coordinate's y value of roi rectangle */
		int y2; /**< right bottom point coordinate's y value of roi rectangle */
	} s_feynman_roi_infor;

	/**@struct s_feynman_image_format_infor
	* @brief feynman device image format information struct \n
	* define feynman device image format  information
	*/
	typedef struct
	{
		uint32_t format; /**< image format */
	} s_feynman_image_format_infor;

	/**@struct s_feynman_depth_data_format_infor
	* @brief feynman device depth data format information struct \n
	* define feynman device depth data format  information
	*/
	typedef struct
	{
		uint32_t format; /**< depth data format */
	} s_feynman_depth_data_format_infor;

	/**@struct s_feynman_rgbd_data_format_infor
	* @brief feynman device rgbd data format information struct \n
	* define feynman device rgbd data format  information
	*/
	typedef struct
	{
		uint32_t format; /**< rgbd data format */
	} s_feynman_rgbd_data_format_infor;

	/**@struct s_feynman_operating_mode_infor
	* @brief feynman device operating mode information struct \n
	* define feynman device operating mode  information
	*/
	typedef struct
	{
		uint32_t mode; /**< operating mode */
	} s_feynman_operating_mode_infor;

	/**@struct s_feynman_calibrition_mode_infor
	* @brief feynman device calibrition mode information struct \n
	* define feynman device calibrition mode information
	*/
	typedef struct
	{
		int f_x; /**< calibration mode f_x */
		int f_y; /**< calibration mode f_y */
		int u_x; /**< calibration mode u_x */
		int u_y; /**< calibration mode u_y */
	} s_feynman_calibrition_mode_infor;

	/**@struct s_feynman_ir_img_source
	* @brief feynman device ir image source information struct \n
	* define feynman device ir image source information
	*/
	typedef struct
	{
		int source; /**< 0-vi  1-vpss,after rectify */
	} s_feynman_ir_img_source;

	/**@struct s_feynman_depth_img_source
	* @brief feynman device depth image source struct \n
	* define feynman device depth image source
	*/
	typedef struct
	{
		int source; /**< 0-disparity  1-depth 16bit raw */
	} s_feynman_depth_img_source;

	/**@struct s_feynman_imu_transfer
	* @brief feynman device imu transfer status struct \n
	* define feynman device  imu transfer status
	*/
	typedef struct
	{
		int status; /**< bit0:-0 stop transfer, ��1-start transfer,bit1:ACC,bit2:GYO,bit3:MANG.(0:do not transfer,1:transfer)����Ŀǰ��ȫ���ϴ���������1����0 */
	} s_feynman_imu_transfer;

	/**@struct s_feynman_cam_param
	* @brief feynman device camera param struct \n
	* define feynman device camera param
	*/
	typedef struct
	{
		int img_width;		   /**< image width */
		int img_height;		   /**< image height */
		int left_right_switch; /**< left right switch */
		int is_new_format;	   /**< 0-�ɵı궨�ļ���ʽ  1-�µı궨�ļ���ʽ */

		char camera_mode[16]; /**< ���ģ�� */
		int camera_number;	  /**<camera number */

		float left_sensor_focus[2];		  /**< [0] - X���򽹾�, [1] - Y���򽹾� */
		float left_sensor_photocenter[2]; /**< [0] - ����X����  [1]-����Y���� */

		float right_sensor_focus[2];	   /**< [0] - X���򽹾�, [1] - Y���򽹾� */
		float right_sensor_photocenter[2]; /**< [0] - ����X����  [1]-����Y���� */

		float rgb_sensor_focus[2];		 /**< [0] - X���򽹾�, [1] - Y���򽹾� */
		float rgb_sensor_photocenter[2]; /**< [0] - ����X����  [1]-����Y���� */

		float left2right_extern_param[12]; /**< �����µı궨��ʽ�����߳���λ��left2right_extern_param[9]�����ھɵı궨��ʽ�����߳���λ��left2right_extern_param[3] */
		float left2rgb_extern_param[12];   /**< left and rgb extern params */
	} s_feynman_cam_param;

	/**@struct IMU_CALIBRATION_DATA
	* @brief feynman device imu data struct \n
	* define feynman device imu data
	*/
	typedef struct
	{
		NVP_FLOAT s16X;
		NVP_FLOAT s16Y;
		NVP_FLOAT s16Z;
	} IMU_CALIBRATION_DATA; // accel gyro,mang��������

	/**@struct IC20948_RAW_DATA_STRUC
	* @brief feynman device imu data struct \n
	* define feynman device imu data
	*/
	typedef struct
	{
		IMU_CALIBRATION_DATA stGyroRawData;
		IMU_CALIBRATION_DATA stAccelRawData;
		IMU_CALIBRATION_DATA stMagnRawData;
		NVP_FLOAT s16TemRawData;
		NVP_U64 timestamp;
	} IC20948_CALIBRATION_DATA_STRUC; //һ��������IMU����

	/**@struct s_feynman_imu_data
	* @brief feynman device imu data struct \n
	* define feynman device imu data
	*/
	typedef struct
	{
		uint32_t data_type;							  /**<  bit0:gyro,bit1:accel,bit2:MANG ,bit3:temp.0-disable,1-enable,����typeʹ�����ɸѡ���� */
		uint32_t data_number;						  /**< ���δ������������ */
		IC20948_CALIBRATION_DATA_STRUC imu_data[128]; /**< ÿ�����ݰ�����gyo��xyz�������ݣ� acc��xyz�������ݣ�  mang���������ݣ� temp��һ������ (���512�����ݣ�ʵ��һ�δ���С��20��) */
	} s_feynman_imu_data;

	/**@struct s_feynman_upgrade_data
	* @brief feynman device upgrade data struct \n
	* define feynman device upgrade data struct
	*/
	typedef struct
	{
		uint32_t packet_numbers;	  /**< һ�����ٰ����� */
		uint32_t curr_packet_numbers; /**< ��ǰ�ǵڼ������� */
		uint32_t data_len;			  /**< ���ݳ��� */
		char path[256];				  /**< �ļ�·�� ����Feynman���� "/nextvpu/bin/arm" */
		char name[100];				  /**< �ļ����֣�����Feynman���� "feynman" */
		char data[0];				  /**< ��������� */
	} s_feynman_upgrade_data;

	/**@struct s_feynman_upgrade_result
	* @brief feynman device upgrade result struct \n
	* define feynman device upgrade result struct
	*/
	typedef struct
	{
		int result; /**< -2�޷����¸��ļ���-1����ʧ�ܣ�0-��ǰ�����³ɹ���  1-������� */
	} s_feynman_upgrade_result;

	/**@struct s_feynman_device_info
	* @brief feynman device info struct \n
	* define feynman device info struct
	*/
	typedef struct
	{
		int sensor_type;						   /**< 0-normal 1-wide-angle */
		float cpu_temperaure;					   /**< CPU�¶ȣ���λ���϶� */
		float projector_temperaure[2];			   /**< 0-projector1 data, 1-projector2 data */
		s_feynman_device_id device_id;			   /**< �豸ID�ţ���FEYNMAN_COMMAND_GET_DEVICE_ID_COMMAND�Ľṹһ�� */
		s_feynman_device_sn device_sn;			   /**< �豸SN�ţ���FEYNMAN_COMMAND_GET_DEVICE_SN_COMMAND�Ľṹһ�� */
		s_feynman_sensor_resolution_fps_infor fps; /**< sensor�ֱ��ʡ�֡����Ϣ */
		int depth_ir_transfer;					   /**< 0-stop transfer, 1-start transfer�� ��������depthʱ��Ч */
		int depth_depth_transfer;				   /**<0-stop transfer, 1-start transfer�� ��������depthʱ��Ч */
		int imu_transfer;						   /**< 0-stop transfer, 1-start transfer */
		int image_freq;							   /**< ͼ����յ�֡�� */
		s_feynman_run_config app_run_mode;
		int hardware_version[2]; //[0]-major version [1]-minor version
		char software_version[64];
	} s_feynman_device_info;

	/**@struct s_feynman_ir_sensor_exposure_infor
	* @brief feynman device ir sensor exposure infor struct \n
	* define feynman device ir sensor exposure infor struct
	*/
	typedef struct
	{
		uint32_t exposure_mode;	  /**< 0-�Զ��ع�  1-�ֶ��ع� */
		int32_t exposure_time[2]; /**< us, ���ڵ���0ʱ��Ч��С��0��ʾ�������ع�ʱ�� */
		int32_t digital_gain[2];  /**< ���ڵ���0ʱ��Ч��С��0��ʾ�������������� */
	} s_feynman_ir_sensor_exposure_infor;

	/**@struct s_feynman_cnn_sub_data
	* @brief feynman cnn sub data infor struct \n
	* define feynman cnn sub data infor struct
	*/
	typedef struct
	{
		int label;	 /**< label value of cnn result */
		float score; /**< score value of cnn result */
		float ymin;	 /**< bbox y coordination of left top point */
		float xmin;	 /**< bbox x coordination of left top point */
		float ymax;	 /**< bbox y coordination of right bottom point */
		float xmax;	 /**< bbox x coordination of right bottom point */
	} s_feynman_cnn_sub_data;

	/**@struct s_feynman_cnn_data
	* @brief feynman cnn data infor struct \n
	* define feynman cnn data infor struct
	*/
	typedef struct
	{
		int frame_id;					  /**< frame id of cnn result */
		int groups;						  /**< groups sum of cnn result */
		s_feynman_cnn_sub_data group[10]; /**< groups of cnn result */
	} s_feynman_cnn_data;

	/**@struct s_feynman_pipeline_cmd
	* @brief feynman start/stop pipeline infor struct \n
	* define feynman start/stop pipeline infor struct
	*/
	typedef struct
	{
		int status; /**< for FEYNMAN_COMMAND_USB_PIPELINE_COMMAND,0-do nothing, 1-start pipeline, 2-close pipeline, 3-restart pipeline, 4-restart application */
	} s_feynman_pipeline_cmd;

	/**@struct s_feynman_img_transfer
	* @brief feynman start/stop transfer infor struct \n
	* define feynman start/stop transfer infor struct
	*/
	typedef struct
	{
		int status; /**< for FEYNMAN_COMMAND_USB_IMG_TRANSFER_COMMAND,0-stop transfer, 1-start transfer */
	} s_feynman_img_transfer;

	/**@struct s_feynman_img_depth_ir_transfer
	* @brief feynman depth and ir start/stop transfer infor struct \n
	* define feynman depth and ir start/stop transfer infor struct
	*/
	typedef struct
	{
		int status; /**< for FEYNMAN_COMMAND_USB_IMG_DEPTH_IR_TRANSFER_COMMAND,0-stop transfer, 1-start transfer */
	} s_feynman_img_depth_ir_transfer;

	/**@struct s_feynman_img_depth_depth_transfer
	* @brief feynman depth start/stop transfer infor struct \n
	* define feynman depth start/stop transfer infor struct
	*/
	typedef struct
	{
		int status; /**< for FEYNMAN_COMMAND_USB_IMG_DEPTH_DEPTH_TRANSFER_COMMAND,0-stop transfer, 1-start transfer */
	} s_feynman_img_depth_depth_transfer;

	/**@struct FEYNMAN_USB_IMAGE_HEADER
	* @brief feynman data packet information struct \n
	* define  feynman data packet information struct,before real image data
	*/
	typedef struct
	{
		uint32_t group_id;	   /**< group_id of data frame,same group_id data frame captured at the same moment */
		uint32_t width;		   /**< width of image data */
		uint32_t height;	   /**< height of image data */
		uint64_t timestamp;	   /**< timestamp of data */
		uint32_t reserved[16]; /**< reserved for later data definition */
	} FEYNMAN_USB_IMAGE_HEADER;

	typedef void (*FRAMECALLBACK)(void *data, void *userdata);
	typedef void (*DEVICECALLBACK)(const char *devicename, void *userdata);

	/**
	* @brief		initial sdk
	* 
	* @note	Call it once at begining of application.
	* @return
	*	none \n
	* @par Sample
	* @code
	*	feynman_init();
	* @endcode
	*/
	void feynman_init();

	/**
	* @brief		deinitial sdk
	* 
	* @note	Call it once when application exit.
	* @return
	*	none \n
	* @par Sample
	* @code
	*	feynman_deinit();
	* @endcode
	*/
	void feynman_deinit();

	/**
	* @brief		enumurate feynman devices
	* @param[in]	callback : callback function,when feynman devices found,this function will be called.
	* @param[in]	userdata : pointer or something else as integer,the value will be pass through to callback function.
	* @note	Call it to discover feynman devices.
	* @return
	*	none \n
	* @par Sample
	* @code
	*  void mycallback(const char* devicename, void* userdata){
	*       printf("found device:%s\n",devicename);
	*       QWidget* THIS=(QWidget*)userdata;
	*       MyEvent* event=new MyEvent(QEvent::Type(QEvent::User + 5),  QStringLiteral(devicename));
	*       QApplication::postEvent(THIS, event);
	*  }
	*  ...
	*  feynman_refresh(mycallback,this);
	*   
	* @endcode
	*/
	void feynman_refresh(DEVICECALLBACK callback, void *userdata);

	/**
	* @brief		connect feynman device and framecallback function will be called when data received from the device
	* @param[in]	devicename : devicename of fenyman device,which you want to connect to.
	* @param[in]	framecallback : callback function,when data received from the device,this function will be called.
	* @param[in]	userdata : pointer or something else as integer,the value will be pass through to framecallback function.
	* @note	Call it to connect to feynman device.
	* @return
	*	none \n
	* @par Sample
	* @code
	*  void mycallback(void* data, void* userdata){
	*       QWidget* THIS=(QWidget*)userdata;
	*       //process data
	*  }
	*  ...
	*  feynman_connectcamera("feynman-12-456",mycallback,this);
	*
	* @endcode
	*/
	BOOL feynman_connectcamera(const char *devicename, FRAMECALLBACK framecallback, void *userdata);

	/**
	* @brief		disconnect feynman device
	* 
	* @note	Call it to disconnect to feynman device.
	* @return
	*	none \n
	* @par Sample
	* @code
	*  
	*  feynman_disconnectcamera();
	*
	* @endcode
	*/
	void feynman_disconnectcamera();

	/**
	* @brief		test if has connect to feynman device
	* 
	* @note	Call it test if has connect to feynman device.
	* @return
	*	 TRUE has connect to feynman device\n
	*	 FALSE not connect to feynman device\n
	* @par Sample
	* @code
	*
	*  if(feynman_hasconnect()){
	*       printf("has connect to feynman device!\n");
	*   }
	*
	* @endcode
	*/
	BOOL feynman_hasconnect();

	/**
	* @brief		set projector on or off
	* @param[in]	on : 1 means set projector on and 0 means set projector off
	* @note	Call it to set project on or off of feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will set projector on!\n");
	*  feynman_setprojector(1);
	*  printf("will set projector off!\n");
	*  feynman_setprojector(0);
	*
	* @endcode
	*/
	void feynman_setprojector(int on);

	/**
	* @brief		enable imu or disable imu data transfer.
	* @param[in]	on : 1 means enable imu data transfer and 0 means disable imu data transfer
	* @note	Call it to enable or disable imu data transfer.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will enable imu data transfer!\n");
	*  feynman_imuenable(1);
	*  printf("will disable imu data transfer!\n");
	*  feynman_imuenable(0);
	*
	* @endcode
	*/
	void feynman_imuenable(int on);

	/**
	* @brief		get yuv color value by index.
	* @param[in]	index : 0-255 interger
	* @param[inout]	py : pointer of unsigned char,corresponding y value(unsigned char) to index will be put
	* @param[inout]	pu : pointer of unsigned char,corresponding u value(unsigned char) to index will be put
	* @param[inout]	pv : pointer of unsigned char,corresponding v value(unsigned char) to index will be put
	* @note	Call it to get yuv color value by index.
	* @return
	*	 0 : Success to get yuv value of index \n
	*	 -1 : Failed to get yuv value of index \n
	* @par Sample
	* @code
	*  unsigned char y,u,v;
	*  printf("will get yuv value of index 23!\n");
	*  if(0==feynman_getyuvfromindex(23,&y,&u,&v)){
	*       printf("get yuv value of index 23 ok!\n);  
	*  }
	*
	* @endcode
	*/
	int feynman_getyuvfromindex(int index, unsigned char *py, unsigned char *pu, unsigned char *pv);

	/**
	* @brief		upgrade file in feynman device.
	* @param[in]	upgrade_type : FEYNMAN_UPGRADE_SUB_TYPE
	* @param[in]	dst_path : pointer of char,dest file path in device,not include filename itself
	* @param[in]	dst_filename : pointer of char,dest file name in device
	* @param[in]	local_filename : pointer of char,local file name,include absolute path or relative path
	* @note	Call it to upgrade file in feynman device.
	* @return
	*	 0 : Success to upgrade file \n
	*	 -1 : Failed to open file or file not exist \n
	*	 -2 : memory fault \n
	*	 -3 : error happened,stop transfer \n
	* @par Sample
	* @code
	*  
	*  printf("will upgrade file of feynman in /nextvpu/bin/arm of device!\n");
	*  if(0==feynman_upgrade(FEYNMAN_COMMAND_USB_UPGRADE_FEYNMAN,"/nextvpu/bin/arm","feynman","c:/feynman")){
	*       printf("upgrade file feynman ok!\n);
	*  }
	*
	* @endcode
	*/
	int feynman_upgrade(int upgrade_type, char *dst_path, char *dst_filename, char *local_filename);

	/**
	* @brief		set depth mode of feynman device.
	* @param[in]	mode : 0/1/2/3
	* @note	Call it to set depth mode of feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will set depth mode of feynman!\n");
	*  feynman_setdepthmode(0);
	*
	* @endcode
	*/
	void feynman_setdepthmode(int mode);

	/**
	* @brief		set transfer ir data or not of feynman device.
	* @param[in]	enable : 0/1
	* @note	Call it to set transfer ir data or not of feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will set transfer ir data of feynman!\n");
	*  feynman_transferir(1);
	*  printf("will stop transfer ir data of feynman!\n");
	*  feynman_transferir(0);
	*
	* @endcode
	*/
	void feynman_transferir(int enable);

	/**
	* @brief		set transfer depth data or not of feynman device.
	* @param[in]	enable : 0/1
	* @note	Call it to set transfer depth data or not of feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will set transfer depth data of feynman!\n");
	*  feynman_transferdepth(1);
	*  printf("will stop transfer depth data of feynman!\n");
	*  feynman_transferdepth(0);
	*
	* @endcode
	*/
	void feynman_transferdepth(int enable);

	/**
	* @brief		set transfer disparity data or not of feynman device.
	* @param[in]	enable : 0/1
	* @note	Call it to set transfer disparity data or not of feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will set transfer disparity data of feynman!\n");
	*  feynman_transferdisparity(1);
	*  printf("will stop transfer disparity data of feynman!\n");
	*  feynman_transferdisparity(0);
	*
	* @endcode
	*/
	void feynman_transferdisparity(int enable);

	/**
	* @brief		set depth denoise of feynman device.
	* @param[in]	on : 0 means denoise off and 1 means denoise on
	* @note	Call it to set depth denoise of feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will set depth denoise on!\n");
	*  feynman_setdepthdenoise(1);
	*
	* @endcode
	*/
	void feynman_setdepthdenoise(int on);

	/**
	* @brief		set depth fusion of feynman device.
	* @param[in]	on : 0 means fusion off and 1 means fusion on
	* @note	Call it to set depth fusion of feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will set depth fusion on!\n");
	*  feynman_setdepthfusion(1);
	*
	* @endcode
	*/
	void feynman_setdepthfusion(int on);

	/**
	* @brief		set depth zoom of feynman device.
	* @param[in]	on : 0 means zoom off and 1 means zoom on
	* @note	Call it to set depth zoom of feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will set depth zoom on!\n");
	*  feynman_setdepthzoom(1);
	*
	* @endcode
	*/
	void feynman_setdepthzoom(int on);

	/**
	* @brief		set depth stitch of feynman device.
	* @param[in]	on : 0 means stitch off and 1 means stitch on
	* @note	Call it to set depth stitch of feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will set depth stitch on!\n");
	*  feynman_setdepthstitch(1);
	*
	* @endcode
	*/
	void feynman_setdepthstitch(int on);

	/**
	* @brief		set depth auto exposure of feynman device.
	* 
	* @note	Call it to set ir auto exposure of feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will set or auto exposure on!\n");
	*  feynman_setirautoexposure();
	*
	* @endcode
	*/
	void feynman_setirautoexposure();

	/**
	* @brief		set depth manual exposure of feynman device.
	* @param[in]	leftus: left ir sensor exposure time in microseconds,less than 0 means ignore
	* @param[in]	leftgain: left ir sensor gain,less than 0 means ignore
	* @param[in]	rightus: right ir sensor exposure time in microseconds,less than 0 means ignore
	* @param[in]	rightgain: right ir sensor gain,less than 0 means ignore
	* @note	Call it to set ir manual exposure of feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will set ir manual exposure of left 200us right 300us,leave left gain and right gain not changed!\n");
	*  feynman_setirmanualexposure(200,-1,300,-1);
	*
	* @endcode
	*/
	void feynman_setirmanualexposure(int leftus, int leftgain, int rightus, int rightgain);

	/**
	* @brief		reset pipeline of feynman device.
	* 
	* @note	Call it to reset pipeline of feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will reset pipeline!\n");
	*  feynman_resetpipeline();
	*
	* @endcode
	*/
	void feynman_resetpipeline();

	/**
	* @brief		stop pipeline of feynman device.
	*
	* @note	Call it to stop pipeline of feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will reset pipeline!\n");
	*  feynman_resetpipeline();
	*
	* @endcode
	*/
	void feynman_stoppipeline();

	/**
	* @brief		start pipeline of feynman device.
	*
	* @note	Call it to start pipeline of feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will start pipeline!\n");
	*  feynman_startpipeline();
	*
	* @endcode
	*/
	void feynman_startpipeline();

	/**
	* @brief		set stream mode of feynman device.
	* @param[in]	mode : 0 means vi+(rgb) mode and 1 means sensor+rectify mode and 2 means depth mode and 3 means depth+cnn mode
	* @note	Call it to set stream mode of feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will set depth mode of feynman device!\n");
	*  feynman_setmode(2);
	*
	* @endcode
	*/
	void feynman_setmode(int mode);

	/**
	* @brief		connect feynman device to get id.
	* @param[in]	devicename : string from feynman_refresh's DEVICECALLBACK function's param
	* @note	Call it to get id of feynman device.
	* @return
	*	 feynman device id in unsigned int type \n
	* @par Sample
	* @code
	*
	*  printf("will get id of feynman device!\n");
	*  unsigned int deviceid=feynman_connectcameraforid("feynman-1-1");
	*
	* @endcode
	*/
	unsigned int feynman_connectcameraforid(const char *devicename);

	/**
	* @brief		switch stream to ir data from sensor of feynman device in sensor+rectify mode.
	* 
	* @note	Call it to switch stream to ir data of feynman device in sensor+rectify mode.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will switch stream to ir data  of feynman device in sensor+rectify mode!\n");
	*  feynman_setvi();
	*
	* @endcode
	*/
	void feynman_setvi();

	/**
	* @brief		switch stream to rectify data from sensor of feynman device in sensor+rectify mode.
	* 
	* @note	Call it to switch stream to rectify data of feynman device in sensor+rectify mode.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will switch stream to rectify data  of feynman device in sensor+rectify mode!\n");
	*  feynman_setvpss();
	*
	* @endcode
	*/
	void feynman_setvpss();

	/**
	* @brief		set to transfer rgb data of feynman device in sensor mode.
	* @param[in]	enable: 0 means stop and 1 means start transfer
	* @note	Call it to set to transfer rgb data of feynman device in sensor mode.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will set to transfer rgb data of feynman device in sensor mode!\n");
	*  feynman_transferrgb(1);
	*
	* @endcode
	*/
	void feynman_transferrgb(int enable);

	/**
	* @brief		send command of get exposure to feynman device,return data will be received in FRAMECALLBACK function.
	* @param[in]	timeout: write usb in {timeout} milliseconds
	* @note	Call it to send command of get exposure to feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will send command of get exposure to feynman device!\n");
	*  feynman_getirexposure(100);
	*
	* @endcode
	*/
	void feynman_getirexposure(int timeout);

	/**
	* @brief		send command of get camera param to feynman device,return data will be received in FRAMECALLBACK function.
	* @param[in]	timeout: write usb in {timeout} milliseconds
	* @note	Call it to send command of get camera param to feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will send command of get camera param to feynman device!\n");
	*  feynman_getcamparam(200);
	*
	* @endcode
	*/
	void feynman_getcamparam(int timeout);

	/**
	* @brief		send command of get projector status to feynman device,return data will be received in FRAMECALLBACK function.
	* @param[in]	timeout: write usb in {timeout} milliseconds
	* @note	Call it to send command of get projector status to feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will send command of get projector status to feynman device!\n");
	*  feynman_getprojector(200);
	*
	* @endcode
	*/
	void feynman_getprojector(int timeout);

	/**
	* @brief		send command of get depth config to feynman device,return data will be received in FRAMECALLBACK function.
	* @param[in]	timeout: write usb in {timeout} milliseconds
	* @note	Call it to send command of get depth config to feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will send command of get depth config to feynman device!\n");
	*  feynman_getdepthconfig(200);
	*
	* @endcode
	*/
	void feynman_getdepthconfig(int timeout);

	/**
	* @brief		send command of get run config to feynman device,return data will be received in FRAMECALLBACK function.
	* @param[in]	timeout: write usb in {timeout} milliseconds
	* @note	Call it to send command of get run config to feynman device.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will send command of get run config to feynman device!\n");
	*  feynman_getrunconfig(200);
	*
	* @endcode
	*/
	void feynman_getrunconfig(int timeout);

	/**
	* @brief		send command to set resolution and fps of sensors.
	* @param[in]	res: resolution enum value
	* @param[in]	fps: frames per second
	* @note	Call it to send command to set resolution and fps of sensors.
	* @return
	*	 none \n
	* @par Sample
	* @code
	*
	*  printf("will send command to set resolution and fps of sensors!\n");
	*  feynman_setresolutionfps(FEYNMAN_RESOLUTION_1280_720,30);
	*
	* @endcode
	*/
	void feynman_setresolutionfps(FEYNMAN_SENSOR_RESOLUTION_TYPE res, unsigned int fps);

#ifdef __cplusplus
}
#endif
#endif

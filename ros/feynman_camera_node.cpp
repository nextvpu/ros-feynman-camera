#include "feynman_sdk.h"

#include <string>
#include <ros/ros.h>
#include "feynman_camera/SaveDepth.h"
#include "feynman_camera/RemoveDark.h"
#include "feynman_camera/SetStreamMode.h"
#include "feynman_camera/SwitchRectify.h"
#include "feynman_camera/SetExposure.h"
#include "feynman_camera/SetDepthMode.h"
#include "feynman_camera/SetProjector.h"
#include "feynman_camera/EnablePointCloud.h"
#include "feynman_camera/EnableIMU.h"
#include "feynman_camera/temp_info.h"
#include "feynman_camera/imu_frame.h"
#include "feynman_camera/imu_info.h"
#include "feynman_camera/cnn_box.h"
#include "feynman_camera/cnn_info.h"
#include <dynamic_reconfigure/server.h>
#include "feynman_camera/resfpsConfig.h"
#include "yuv_rgb.h"
#include <std_msgs/String.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include "ring_queue.h"
#include <opencv2/opencv.hpp>
//#include <pcl/ros/conversions.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

#include <string.h>
#define MAXWIDTH 1280
#define MAXHEIGHT 800

double DEPTHMAX = 5000.0;
double DEPTHMIN = 100.0;

int g_runconfig = -1;
void resfpscallback(feynman_camera::resfpsConfig &config)
{
  ROS_INFO("Reconfigure Request: %s %d",
           config.resolution.c_str(),
           config.fps);

  if ((config.resolution != "1280x800" &&
       config.resolution != "1280x720" &&
       config.resolution != "640x480" &&
       config.resolution != "640x400" &&
       config.resolution != "320x200") ||
      config.fps != 30)
  {
    ROS_INFO("Only support 1280x800/1280x720/640x480/640x400/320x200 at 30fps!");
  }
  else
  {
    ROS_INFO("will change resolution!!!!\n");
    if (config.resolution == "1280x800")
      feynman_setresolutionfps(FEYNMAN_RESOLUTION_1280_800, config.fps);
    else if (config.resolution == "1280x720")
      feynman_setresolutionfps(FEYNMAN_RESOLUTION_1280_720, config.fps);
    else if (config.resolution == "640x480")
      feynman_setresolutionfps(FEYNMAN_RESOLUTION_640_480, config.fps);
    else if (config.resolution == "640x400")
      feynman_setresolutionfps(FEYNMAN_RESOLUTION_640_400, config.fps);
    else if (config.resolution == "320x200")
      feynman_setresolutionfps(FEYNMAN_RESOLUTION_320_200, config.fps);

    feynman_resetpipeline();
  }
}
////////////////////////////////////
using namespace cv;
unsigned char *g_leftdepth = NULL;

bool g_removedark = true;

float pixthreshold = 20;
float sigmathreshold = 30;

int32_t removeDarkOutliers(
    const cv::Mat &img,
    cv::Mat &depth,
    int32_t pixThr,
    int32_t sigmaThr,
    int8_t winsize)
{
  // img mean

  Mat meanI;
  boxFilter(img, meanI, -1, Size(winsize, winsize), Point(-1, -1), true, 2);

  // img - mean
  Mat diff;
  cv::absdiff(img, meanI, diff); // todo: check: uint8 sub

  // diff square
  Mat sDiff = diff.mul(diff); // todo: check: mul out of range
                              // box filter SAD
  Mat sad;
  boxFilter(sDiff, sad, -1, Size(winsize, winsize), Point(-1, -1), true, 2);
  // compare  mask1 mask2
  Mat mask1, mask2;
  mask1 = sad < sigmaThr;
  mask2 = meanI < pixThr;
  // mask and op
  Mat mask = mask1 & mask2;
  Mat nmask;
  mask.convertTo(nmask, CV_16UC1, 1, 0);
  // mask box filter

  // compare   //todo:simplify here
  Mat ndepth;
  depth.copyTo(ndepth, 255 - mask);
  depth = ndepth;
  return 0;
}

std::vector<s_feynman_cam_param>
    g_cameraparam;
// 0.obtain device id from launch file
// 1.when device plugin,enumrate device and open it and recv data from it
// 2.while device id is not the one request,close it
// 3.while device id is the one request,open it and continue recv data from it

bool handle_device_switchrectify_request(feynman_camera::SwitchRectifyRequest &req,
                                         feynman_camera::SwitchRectifyResponse &res)
{
  if (req.rectifyornot == "rectify")
  { // rectify
    feynman_setvpss();
  }
  else
  { // not rectify,just vi
    feynman_setvi();
  }
  return true;
}

typedef struct
{
  int filenums;
  char filepath[256];
} SAVEDEPTHTASK;

typedef struct
{
  int len;
  char filename[256];
  char data[1280 * 800 * 2];
} DEPTHDATAINFO;

Ring_Queue *taskqueue = NULL;

bool handle_removedark_request(feynman_camera::RemoveDarkRequest &req,
                               feynman_camera::RemoveDarkResponse &res)
{
  if (req.enableremovedark)
  {
    g_removedark = true;
    pixthreshold = (int32_t)(req.pixelthreshold * 255.0);
    sigmathreshold = (int32_t)(req.sigmathreshold * 255.0);
  }
  else
  {
    g_removedark = false;
  }
}
bool handle_savedepth_request(feynman_camera::SaveDepthRequest &req,
                              feynman_camera::SaveDepthResponse &res)
{
  // create task node and insert to queue of save depth task
  if (NULL == taskqueue)
  {
    taskqueue = new Ring_Queue(9, sizeof(SAVEDEPTHTASK));
  }
  SAVEDEPTHTASK *tmptask = NULL;
  while (tmptask == NULL)
  {
    tmptask = (SAVEDEPTHTASK *)taskqueue->SOLO_Write();
    if (tmptask)
    {
      strcpy(tmptask->filepath, req.filepath.c_str());
      tmptask->filenums = req.filenums;
      taskqueue->SOLO_Write_Over();
      break;
    }
  }
  printf("ok to add savedepth task!\n");
  // create tmp thread to save files
}
bool handle_device_streammode_request(feynman_camera::SetStreamModeRequest &req,
                                      feynman_camera::SetStreamModeResponse &res)
{
  if (req.mode == 0)
  { // vi
    // 0.vi 1.vpss 2.depth 3.cnn
    feynman_setmode(0);
    feynman_resetpipeline();
  }
  else if (req.mode == 1)
  { // vpss
    // 0.vi 1.vpss 2.depth 3.cnn
    feynman_setmode(1);
    feynman_resetpipeline();
  }
  else if (req.mode == 2)
  { // depth
    feynman_setmode(2);
    feynman_resetpipeline();
  }
  else if (req.mode == 3)
  { // cnn
    feynman_setmode(3);
    feynman_resetpipeline();
  }
  return true;
}

bool handle_device_exposure_request(feynman_camera::SetExposureRequest &req,
                                    feynman_camera::SetExposureResponse &res)
{
  if (req.isauto == 1)
  {
    feynman_setirautoexposure();
  }
  else
  {
    feynman_setirmanualexposure(req.leftexposureus, req.leftgain, req.rightexposureus, req.rightgain);
  }
  return true;
}
bool handle_device_depthmode_request(feynman_camera::SetDepthModeRequest &req,
                                     feynman_camera::SetDepthModeResponse &res)
{
  if (req.mode == "denoise")
  { // vpss
    // 0.vi 1.vpss 2.depth 3.cnn
    feynman_setdepthdenoise(1);
    feynman_resetpipeline();
  }
  else if (req.mode == "fusion")
  { // depth
    // 0.vi 1.vpss 2.depth 3.cnn
    feynman_setdepthfusion(1);
    feynman_resetpipeline();
  }
  else if (req.mode == "zoom")
  { // cnn
    feynman_setdepthzoom(1);
    feynman_resetpipeline();
  }
  else if (req.mode == "stitch")
  { // cnn
    feynman_setdepthstitch(1);
    feynman_resetpipeline();
  }
  return true;
}
bool handle_device_enablepointcloud_request(feynman_camera::EnablePointCloudRequest &req,
                                            feynman_camera::EnablePointCloudResponse &res)
{

  feynman_setpointcloudtransfer(req.enable);

  return true;
}
bool handle_device_enableimu_request(feynman_camera::EnableIMURequest &req,
                                     feynman_camera::EnableIMUResponse &res)
{

  feynman_imuenable(req.enable);

  return true;
}
bool handle_device_setprojector_request(feynman_camera::SetProjectorRequest &req,
                                        feynman_camera::SetProjectorResponse &res)
{

  feynman_setprojector(req.enable);

  return true;
}
/*
bool handle_device_cameraparam_request(feynman_camera::GetCameraParamRequest &req,
                                       feynman_camera::GetCameraParamResponse &res)
{
  if (g_cameraparam.size() == 1)
  {
    s_feynman_cam_param tmpparam = g_cameraparam[0];
    sensor_msgs::CameraInfo info;

    info.width = tmpparam.img_width;
    info.height = tmpparam.img_height;
    info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    info.D.resize(5, 0.0);
    info.D[0] = 0;
    info.D[1] = 0;
    info.D[2] = 0;
    info.D[3] = 0;
    info.D[4] = 0;

    info.K.assign(0.0);
    info.K[0] = tmpparam.left_sensor_focus[0];
    info.K[2] = tmpparam.left_sensor_photocenter[0];
    info.K[4] = tmpparam.left_sensor_focus[1];
    info.K[5] = tmpparam.left_sensor_photocenter[1];
    info.K[8] = 1.0;

    info.R.assign(0.0);
    //for (int i = 0; i < 9; i++)
    //{
    //  info.R[i] = p.r2l_r[i];
    //}
    info.R[0] = 1;
    info.R[4] = 1;
    info.R[8] = 1;

    info.P.assign(0.0);
    info.P[0] = info.K[0];
    info.P[2] = info.K[2];
    info.P[3] = (tmpparam.is_new_format ? tmpparam.left2right_extern_param[9] : tmpparam.left2right_extern_param[3]);
    info.P[5] = info.K[4];
    info.P[6] = info.K[5];
    info.P[7] = 0;
    info.P[10] = 1.0;
    info.P[11] = 0;
    // Fill in header
    info.header.stamp = ros::Time::now();
    info.header.frame_id = "feynman_camera";

    res.info = info;
    return true;
  }

  return false;
}*/
typedef struct
{
  char devicename[64];
  unsigned int device_id;
  unsigned int fps;
  char resolution[16];
  int connected;
  ros::Publisher leftircamerainfopublisher;
  ros::Publisher rightircamerainfopublisher;
  ros::Publisher rgbcamerainfopublisher;
  ros::Publisher depthrawpublisher;
  ros::Publisher depthrawleftpublisher;
  ros::Publisher depthrawrightpublisher;
  ros::Publisher temperaturepublisher;
  ros::Publisher dotcloudpublisher;
  ros::Publisher depthcamerainfopublisher;
  ros::Publisher rgbpublisher;
  ros::Publisher rgbrawpublisher;
  ros::Publisher sensorrawleftpublisher;
  ros::Publisher sensorrawrightpublisher;
  ros::Publisher rectifyleftpublisher;
  ros::Publisher rectifyrightpublisher;
  ros::Publisher cnnpublisher;
  ros::Publisher logpublisher;
  ros::Publisher imupublisher;
  ros::Publisher depthalignrgbpublisher;
  ros::Publisher depthalignrgbviewpublisher;
  ros::Publisher depthpseudopublisher;
  bool pubrgb;
  bool pubir;
  bool pubdotcloud;
  bool pubdepth;
  bool savedata;
  bool pubdepthalign;
  bool pubpseudo;
  bool confidence;
  bool pubimu;
} DEVICEINFO;

int hasstartpipeline = 0;
int g_hasgotparam = 0;
std::vector<FEYNMAN_USBHeaderDataPacket *> g_savedepth;
void savecallback(void *data, void *userdata)
{
  DEVICEINFO *info = (DEVICEINFO *)userdata;
  if (!info->savedata)
    return;
  FEYNMAN_USBHeaderDataPacket *tmppack = (FEYNMAN_USBHeaderDataPacket *)data;
  if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_DEPTH_IMAGE)
  {
    // ROS_INFO("depth data!\n");

    if (g_cameraparam.size() == 1)
    {

      int width = g_cameraparam[0].img_width;
      int height = g_cameraparam[0].img_height;

      if (info->confidence && g_leftdepth != NULL && g_removedark && (tmppack->len - sizeof(FEYNMAN_USB_IMAGE_HEADER)) == width * height * 2)
      {
        Mat *tempdepth = new Mat(height, width, CV_16U, Scalar::all(0));
        // 循环赋值
        for (int i = 0; i < height; i++)
        {
          for (int j = 0; j < width; j++)
          {
            tempdepth->at<uint16_t>(i, j) = *((uint16_t *)(tmppack->data + sizeof(FEYNMAN_USB_IMAGE_HEADER)) + i * width + j);
          }
        }
        Mat *templeftraw = new Mat(height, width, CV_8U, Scalar::all(0));
        // 循环赋值
        for (int i = 0; i < height; i++)
        {
          for (int j = 0; j < width; j++)
          {
            templeftraw->at<uint8_t>(i, j) = *((uint8_t *)(g_leftdepth) + i * width + j);
          }
        }
        int8_t winsize = 15;
        removeDarkOutliers(*templeftraw, *tempdepth, pixthreshold, sigmathreshold, winsize);

        for (int i = 0; i < height; i++)
        {
          for (int j = 0; j < width; j++)
          {
            *((uint16_t *)(tmppack->data + sizeof(FEYNMAN_USB_IMAGE_HEADER)) + i * width + j) = tempdepth->at<uint16_t>(i, j);
          }
        }
        delete templeftraw;
        delete tempdepth;
      }
    }
  }
  if (NULL == taskqueue)
  {
    taskqueue = new Ring_Queue(9, sizeof(SAVEDEPTHTASK));
  }
  static SAVEDEPTHTASK *tmptask = NULL;
  if (tmptask == NULL)
  {
    SAVEDEPTHTASK *task = (SAVEDEPTHTASK *)taskqueue->SOLO_Read();
    if (task)
    {
      tmptask = (SAVEDEPTHTASK *)malloc(sizeof(SAVEDEPTHTASK));
      memcpy(tmptask, task, sizeof(SAVEDEPTHTASK));
      taskqueue->SOLO_Read_Over();
    }
    else
    {
      usleep(1000);
    }
  }

  if (tmptask != NULL)
  {
    // save depth and rgb with same groupid
    FEYNMAN_USBHeaderDataPacket *tmppack = (FEYNMAN_USBHeaderDataPacket *)data;
    if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_DEPTH_IMAGE)
    {
      char filename[256];
      FEYNMAN_USB_IMAGE_HEADER *tmpheader = (FEYNMAN_USB_IMAGE_HEADER *)tmppack->data;
      bool hasrgb = false;
      for (int i = 0; i < g_savedepth.size(); i++)
      {
        FEYNMAN_USBHeaderDataPacket *mydata = g_savedepth[i];
        FEYNMAN_USB_IMAGE_HEADER *myheader = (FEYNMAN_USB_IMAGE_HEADER *)mydata->data;
        if (myheader->group_id == tmpheader->group_id && mydata->sub_type == FEYNMAN_RGB_IMAGE_SINGLE)
        { // save depth and rgb
          sprintf(filename, "%s/depth-%u-%llu.raw", tmptask->filepath, tmpheader->group_id, tmpheader->timestamp);
          printf("filenums:%d,will write file:%s\n", tmptask->filenums, filename);
          FILE *fp = fopen(filename, "wb");
          fwrite(tmppack->data + sizeof(FEYNMAN_USB_IMAGE_HEADER), 1, tmppack->len - sizeof(FEYNMAN_USB_IMAGE_HEADER), fp);
          fclose(fp);

          sprintf(filename, "%s/rgb-%u-%llu.raw", tmptask->filepath, myheader->group_id, myheader->timestamp);
          printf("filenums:%d,will write file:%s\n", tmptask->filenums, filename);
          fp = fopen(filename, "wb");
          fwrite(mydata->data + sizeof(FEYNMAN_USB_IMAGE_HEADER), 1, mydata->len - sizeof(FEYNMAN_USB_IMAGE_HEADER), fp);
          fclose(fp);

          tmptask->filenums--;
          if (tmptask->filenums == 0)
          {
            free(tmptask);
            tmptask = NULL;
          }
          hasrgb = true;
          break;
        }
      }
      if (!hasrgb)
      { // save depth to g_savedepth
        FEYNMAN_USBHeaderDataPacket *tobesaved = (FEYNMAN_USBHeaderDataPacket *)malloc(sizeof(FEYNMAN_USBHeaderDataPacket) + tmppack->len);
        memcpy(tobesaved, tmppack, sizeof(FEYNMAN_USBHeaderDataPacket) + tmppack->len);
        g_savedepth.push_back(tobesaved);
      }
    }
    else if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_RGB_IMAGE_SINGLE)
    {
      char filename[256];
      FEYNMAN_USB_IMAGE_HEADER *tmpheader = (FEYNMAN_USB_IMAGE_HEADER *)tmppack->data;
      bool hasdepth = false;
      for (int i = 0; i < g_savedepth.size(); i++)
      {
        FEYNMAN_USBHeaderDataPacket *mydata = g_savedepth[i];
        FEYNMAN_USB_IMAGE_HEADER *myheader = (FEYNMAN_USB_IMAGE_HEADER *)mydata->data;
        if (myheader->group_id == tmpheader->group_id && mydata->sub_type == FEYNMAN_DEPTH_IMAGE)
        { // save depth and rgb
          sprintf(filename, "%s/rgb-%u-%llu.raw", tmptask->filepath, tmpheader->group_id, tmpheader->timestamp);
          printf("filenums:%d,will write file:%s\n", tmptask->filenums, filename);
          FILE *fp = fopen(filename, "wb");
          fwrite(tmppack->data + sizeof(FEYNMAN_USB_IMAGE_HEADER), 1, tmppack->len - sizeof(FEYNMAN_USB_IMAGE_HEADER), fp);
          fclose(fp);

          sprintf(filename, "%s/depth-%u-%llu.raw", tmptask->filepath, myheader->group_id, myheader->timestamp);
          printf("filenums:%d,will write file:%s\n", tmptask->filenums, filename);
          fp = fopen(filename, "wb");
          fwrite(mydata->data + sizeof(FEYNMAN_USB_IMAGE_HEADER), 1, mydata->len - sizeof(FEYNMAN_USB_IMAGE_HEADER), fp);
          fclose(fp);

          tmptask->filenums--;
          if (tmptask->filenums == 0)
          {
            free(tmptask);
            tmptask = NULL;
          }
          hasdepth = true;
          break;
        }
      }
      if (!hasdepth)
      { // save depth to g_savedepth
        FEYNMAN_USBHeaderDataPacket *tobesaved = (FEYNMAN_USBHeaderDataPacket *)malloc(sizeof(FEYNMAN_USBHeaderDataPacket) + tmppack->len);
        memcpy(tobesaved, tmppack, sizeof(FEYNMAN_USBHeaderDataPacket) + tmppack->len);
        g_savedepth.push_back(tobesaved);
      }
    }
  }
}
void imucallback(void *data, void *userdata)
{
  DEVICEINFO *info = (DEVICEINFO *)userdata;
  FEYNMAN_USBHeaderDataPacket *tmppack = (FEYNMAN_USBHeaderDataPacket *)data;
  if (tmppack->type == FEYNMAN_IMU_DATA && tmppack->sub_type == FEYNMAN_IMU_DATA_ALL)
  {
    s_feynman_imu_data *tmpimudata = (s_feynman_imu_data *)tmppack->data;
    feynman_camera::imu_info imudata;

    if (sizeof(s_feynman_imu_data) == tmppack->len)
    {
      imudata.imu_frames.resize(tmpimudata->data_number);
      static uint64_t lastbegintimestamp = 0, lastendtimestamp = 0;
      int offset = tmpimudata->imu_data[0].timestamp - lastendtimestamp;
      // printf("big packet offset:%d\n", offset);
      if ((offset > 5000 || offset < 4000) && lastbegintimestamp != 0)
      {
        printf("warning,imu packet abnormal offset!!!:%d\n", offset);
      }
      lastbegintimestamp = tmpimudata->imu_data[0].timestamp;
      lastendtimestamp = tmpimudata->imu_data[tmpimudata->data_number - 1].timestamp;
      for (int i = 0; i < tmpimudata->data_number; i++)
      {
        /*    printf("====================================\n");
        printf("acc:%f,%f,%f\n", tmpimudata->imu_data[i].stAccelRawData.s16X, tmpimudata->imu_data[i].stAccelRawData.s16Y, tmpimudata->imu_data[i].stAccelRawData.s16Z);
        printf("gyo:%f,%f,%f\n", tmpimudata->imu_data[i].stGyroRawData.s16X, tmpimudata->imu_data[i].stGyroRawData.s16Y, tmpimudata->imu_data[i].stGyroRawData.s16Z);
        printf("mang:%f,%f,%f\n", tmpimudata->imu_data[i].stMagnRawData.s16X, tmpimudata->imu_data[i].stMagnRawData.s16Y, tmpimudata->imu_data[i].stMagnRawData.s16Z);
        printf("temp:%f\n", tmpimudata->imu_data[i].s16TemRawData);
        printf("timestamp:%lu\n", tmpimudata->imu_data[i].timestamp);
        printf("====================================\n");
        */
        imudata.imu_frames[i].accx = tmpimudata->imu_data[i].stAccelRawData.s16X;
        imudata.imu_frames[i].accy = tmpimudata->imu_data[i].stAccelRawData.s16Y;
        imudata.imu_frames[i].accz = tmpimudata->imu_data[i].stAccelRawData.s16Z;
        imudata.imu_frames[i].gyrx = tmpimudata->imu_data[i].stGyroRawData.s16X;
        imudata.imu_frames[i].gyry = tmpimudata->imu_data[i].stGyroRawData.s16Y;
        imudata.imu_frames[i].gyrz = tmpimudata->imu_data[i].stGyroRawData.s16Z;
        imudata.imu_frames[i].magx = tmpimudata->imu_data[i].stMagnRawData.s16X;
        imudata.imu_frames[i].magy = tmpimudata->imu_data[i].stMagnRawData.s16Y;
        imudata.imu_frames[i].magz = tmpimudata->imu_data[i].stMagnRawData.s16Z;
        imudata.imu_frames[i].temp = tmpimudata->imu_data[i].s16TemRawData;
        imudata.imu_frames[i].timestamp = tmpimudata->imu_data[i].timestamp;
      }
    }
    else
    {
      printf("imudata len invalid:%d!=%d\n", tmppack->len, sizeof(s_feynman_imu_data));
    }
    //   printf("will publish imu!!!\n");
    info->imupublisher.publish(imudata);
    //   printf("has publish imu!\n");
  }
}
void rgbcallback(void *data, void *userdata)
{
  // printf("enter rgbcallback!\n");
  DEVICEINFO *info = (DEVICEINFO *)userdata;
  if (!info->pubrgb)
    return;

  FEYNMAN_USBHeaderDataPacket *tmppack = (FEYNMAN_USBHeaderDataPacket *)data;
  if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_RGB_IMAGE_SINGLE)
  {
    if (g_cameraparam.size() == 1 && (tmppack->len - sizeof(FEYNMAN_USB_IMAGE_HEADER)) == g_cameraparam[0].img_width * g_cameraparam[0].img_height * 3 / 2)
    {
      s_feynman_cam_param theparam = g_cameraparam[0];
      sensor_msgs::Image new_image;

      new_image.header.frame_id = "feynman_camera/rgb";
      new_image.width = theparam.img_width;
      new_image.height = theparam.img_height;
      new_image.is_bigendian = 0;
      new_image.encoding = sensor_msgs::image_encodings::RGB8;
      new_image.step = 3 * new_image.width;

      std::size_t data_size = new_image.step * new_image.height;
      new_image.data.resize(data_size);

      int width = new_image.width;
      int height = new_image.height;
      static unsigned char *buffer = NULL;
      // printf("will malloc in rgb:%dx%dx3!\n", width, height);
      if (NULL == buffer)
        buffer = (unsigned char *)malloc(MAXWIDTH * MAXHEIGHT * 3);
      unsigned char *tmpimgdata = tmppack->data + sizeof(FEYNMAN_USB_IMAGE_HEADER);
      //  printf("will convert nv12 to rgb24!\n");
      nv12_rgb24_std(width, height, tmpimgdata, tmpimgdata + width * height, width, width, buffer, width * 3, YCBCR_601);
      //   printf("ok convert nv12 to rgb24!\n");
      unsigned short *in_ptr = reinterpret_cast<unsigned short *>(&new_image.data[0]);
      //   printf("will memcpy:%d!%d\n", data_size, width * height * 3);
      memcpy(in_ptr, buffer, data_size);
      //  printf("end memcpy!\n");
      new_image.header.stamp = ros::Time::now();
      if (g_runconfig >= 2) // depth or cnn_demo
      {
        info->rgbpublisher.publish(new_image);
      }
      else
      {
        info->rgbrawpublisher.publish(new_image);
      }
      //  printf("will publish camerainfo of rgb!\n");
      sensor_msgs::CameraInfo caminfo;

      caminfo.width = theparam.img_width;
      caminfo.height = theparam.img_height;
      caminfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      caminfo.D.resize(5, 0.0);
      caminfo.D[0] = 0;
      caminfo.D[1] = 0;
      caminfo.D[2] = 0;
      caminfo.D[3] = 0;
      caminfo.D[4] = 0;

      caminfo.K.assign(0.0);
      caminfo.K[0] = theparam.rgb_sensor_focus[0];
      caminfo.K[2] = theparam.rgb_sensor_photocenter[0];
      caminfo.K[4] = theparam.rgb_sensor_focus[1];
      caminfo.K[5] = theparam.rgb_sensor_photocenter[1];
      caminfo.K[8] = 1.0;

      caminfo.R.assign(0.0);
      // for (int i = 0; i < 9; i++)
      //{
      //   caminfo.R[i] = p.r2l_r[i];
      // }
      caminfo.R[0] = 1;
      caminfo.R[4] = 1;
      caminfo.R[8] = 1;

      caminfo.P.assign(0.0);
      caminfo.P[0] = caminfo.K[0];
      caminfo.P[2] = caminfo.K[2];
      caminfo.P[3] = (theparam.is_new_format ? theparam.left2rgb_extern_param[9] : theparam.left2rgb_extern_param[3]);
      caminfo.P[5] = caminfo.K[4];
      caminfo.P[6] = caminfo.K[5];
      caminfo.P[7] = 0;
      caminfo.P[10] = 1.0;
      caminfo.P[11] = 0;
      // Fill in header
      caminfo.header.stamp = new_image.header.stamp;
      caminfo.header.frame_id = "feynman_camera/rgb/camerainfo";

      info->rgbcamerainfopublisher.publish(caminfo);
      //   printf("after rgb publish!\n");
    }
  }
}
typedef struct
{
  unsigned char r;
  unsigned char g;
  unsigned char b;
} MYRGB;
MYRGB *g_colortable = NULL;

void calculatecolortable()
{
  if (NULL == g_colortable)
  {
    g_colortable = (MYRGB *)calloc(1, sizeof(MYRGB) * 65535);
  }
  memset(g_colortable, 0, sizeof(MYRGB) * 65535);

  for (int i = (int)DEPTHMIN; i <= (int)DEPTHMAX; i++)
  { //0.2m-5m
    int effectindex = (int)((float)(i - (int)DEPTHMIN) * 255.0 / (float)(DEPTHMAX - DEPTHMIN));
    //	if (effectindex > 255)effectindex = 255;
    if (effectindex == 255)
      effectindex = 254;

    unsigned char y = 0;
    unsigned char u = 0;
    unsigned char v = 0;
    int result = feynman_getyuvfromindex(effectindex, &y, &u, &v);
    //y = 26; u = 170; v = 122;
    if (result >= 0)
    {
      float fr = y + 1.4075 * (v - 128);

      float fg = y - 0.3455 * (u - 128) - 0.7169 * (v - 128);

      float fb = y + 1.779 * (u - 128);
      /*		static FILE* fp = fopen("table.txt", "wt");
				fprintf(fp,"table:%f,%f,%f\n", fr, fg, fb);
				fflush(fp);*/
      g_colortable[i].r = (unsigned char)fr;
      g_colortable[i].g = (unsigned char)fg;
      g_colortable[i].b = (unsigned char)fb;
    }
  }
}
void depthcallback(void *data, void *userdata)
{
  DEVICEINFO *info = (DEVICEINFO *)userdata;
  FEYNMAN_USBHeaderDataPacket *tmppack = (FEYNMAN_USBHeaderDataPacket *)data;
  if (!info->pubdepth)
    return;
  if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_DEPTH_IMAGE)
  {
#include <time.h>
    static unsigned int count = 0, lastcount;
    static struct timespec lasttp;
    if (count == 0)
    {
      clock_gettime(CLOCK_MONOTONIC, &lasttp);
      lastcount = 0;
    }
    else
    {
      if (count % 100 == 0)
      {
        struct timespec tp;
        clock_gettime(CLOCK_MONOTONIC, &tp);
        int millis = (tp.tv_sec - lasttp.tv_sec) * 1000 + (tp.tv_nsec - lasttp.tv_nsec) / 1000000;
        printf("fps:%f\n", (float)(count - lastcount) * 1000.0 / (float)millis);

        lasttp = tp;
        lastcount = count;
      }
    }
    count++;
  }
  // return;
  if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_DEPTH_IMAGE)
  {
    // ROS_INFO("depth data!\n");

    if (g_cameraparam.size() == 1)
    {

      int width = g_cameraparam[0].img_width;
      int height = g_cameraparam[0].img_height;

      if (info->confidence && g_leftdepth != NULL && g_removedark && (tmppack->len - sizeof(FEYNMAN_USB_IMAGE_HEADER)) == width * height * 2)
      {
        Mat tempdepth = Mat(height, width, CV_16U, Scalar::all(0));
        // 循环赋值
        for (int i = 0; i < height; i++)
        {
          for (int j = 0; j < width; j++)
          {
            tempdepth.at<uint16_t>(i, j) = *((uint16_t *)(tmppack->data + sizeof(FEYNMAN_USB_IMAGE_HEADER)) + i * width + j);
          }
        }
        Mat templeftraw = Mat(height, width, CV_8U, Scalar::all(0));
        // 循环赋值
        for (int i = 0; i < height; i++)
        {
          for (int j = 0; j < width; j++)
          {
            templeftraw.at<uint8_t>(i, j) = *((uint8_t *)(g_leftdepth) + i * width + j);
          }
        }
        int8_t winsize = 15;
        removeDarkOutliers(templeftraw, tempdepth, pixthreshold, sigmathreshold, winsize);

        for (int i = 0; i < height; i++)
        {
          for (int j = 0; j < width; j++)
          {
            *((uint16_t *)(tmppack->data + sizeof(FEYNMAN_USB_IMAGE_HEADER)) + i * width + j) = tempdepth.at<uint16_t>(i, j);
          }
        }
      }
    }
  }

  if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_DEPTH_IMAGE)
  {
    // ROS_INFO("depth data!\n");

    if (g_cameraparam.size() == 1 && (tmppack->len - sizeof(FEYNMAN_USB_IMAGE_HEADER)) == g_cameraparam[0].img_width * g_cameraparam[0].img_height * 2)
    {

      int width = g_cameraparam[0].img_width;
      int height = g_cameraparam[0].img_height;

      //   printf("will depthraw publish!\n");
      s_feynman_cam_param theparam = g_cameraparam[0];
      sensor_msgs::Image *new_image = new sensor_msgs::Image;

      new_image->header.frame_id = "feynman_camera/depth/image_raw";
      new_image->width = theparam.img_width;
      new_image->height = theparam.img_height;
      new_image->is_bigendian = 0;
      new_image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      new_image->step = sizeof(unsigned short) * new_image->width;

      std::size_t data_size = new_image->step * new_image->height;
      new_image->data.resize(data_size);

      unsigned short *in_ptr = reinterpret_cast<unsigned short *>(&new_image->data[0]);

      unsigned char *tmpimgdata = tmppack->data + sizeof(FEYNMAN_USB_IMAGE_HEADER);
      memcpy(in_ptr, tmpimgdata, data_size);
      new_image->header.stamp = ros::Time::now();
      static uint32_t depthseq = 0;
      new_image->header.seq = depthseq;
      depthseq++;
      info->depthrawpublisher.publish(*new_image);

      sensor_msgs::CameraInfo caminfo;

      caminfo.width = theparam.img_width;
      caminfo.height = theparam.img_height;
      caminfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      caminfo.D.resize(5, 0.0);
      caminfo.D[0] = 0;
      caminfo.D[1] = 0;
      caminfo.D[2] = 0;
      caminfo.D[3] = 0;
      caminfo.D[4] = 0;

      caminfo.K.assign(0.0);
      caminfo.K[0] = theparam.left_sensor_focus[0];
      caminfo.K[2] = theparam.left_sensor_photocenter[0];
      caminfo.K[4] = theparam.left_sensor_focus[1];
      caminfo.K[5] = theparam.left_sensor_photocenter[1];
      caminfo.K[8] = 1.0;

      caminfo.R.assign(0.0);
      // for (int i = 0; i < 9; i++)
      //{
      //   caminfo.R[i] = p.r2l_r[i];
      // }
      caminfo.R[0] = 1;
      caminfo.R[4] = 1;
      caminfo.R[8] = 1;

      caminfo.P.assign(0.0);
      caminfo.P[0] = caminfo.K[0];
      caminfo.P[2] = caminfo.K[2];
      caminfo.P[3] = (theparam.is_new_format ? theparam.left2right_extern_param[9] : theparam.left2right_extern_param[3]);
      caminfo.P[5] = caminfo.K[4];
      caminfo.P[6] = caminfo.K[5];
      caminfo.P[7] = 0;
      caminfo.P[10] = 1.0;
      caminfo.P[11] = 0;
      // Fill in header
      caminfo.header.stamp = new_image->header.stamp;
      caminfo.header.seq = new_image->header.seq;
      caminfo.header.frame_id = "feynman_camera/depth/camerainfo";

      delete new_image;
      info->depthcamerainfopublisher.publish(caminfo);

      //     printf("will dotcloud publish!\n");

      double LEFTCAMERAFX = 1048.0;
      double LEFTCAMERAFY = 1048.0;
      double CAMERAT = 23.590832;
      double LEFTCAMERAX = 174.361675;
      double LEFTCAMERAY = 297.007244;

      float DEPTHINRGBROTATE[9];
      float DEPTHINRGBOFFSET[3];
      float RGBFX;
      float RGBFY;
      float RGBX;
      float RGBY;
      //      printf("will get some param!\n");
      LEFTCAMERAX = theparam.left_sensor_photocenter[0];
      LEFTCAMERAY = theparam.left_sensor_photocenter[1];
      LEFTCAMERAFX = theparam.left_sensor_focus[0];
      LEFTCAMERAFY = theparam.left_sensor_focus[1];

      memcpy(&DEPTHINRGBROTATE[0], &theparam.left2rgb_extern_param[0], 9 * sizeof(float));
      memcpy(&DEPTHINRGBOFFSET[0], &theparam.left2rgb_extern_param[9], 3 * sizeof(float));
      RGBFX = theparam.rgb_sensor_focus[0];
      RGBFY = theparam.rgb_sensor_focus[1];
      RGBX = theparam.rgb_sensor_photocenter[0];
      RGBY = theparam.rgb_sensor_photocenter[1];

      //   printf("will get camerat!\n");
      if (theparam.is_new_format == 1)
      {
        CAMERAT = abs(theparam.left2right_extern_param[9]);
      }
      else
      {
        CAMERAT = abs(theparam.left2right_extern_param[3]);
      }

      //   printf("will set depth dotclould!%dx%d\n", theparam.img_width, theparam.img_height);
      uint16_t *tmppic = (uint16_t *)(tmppack->data + sizeof(FEYNMAN_USB_IMAGE_HEADER));
      // Fill in the cloud data

      /////////////////////////will publish dot cloud
      sensor_msgs::PointCloud2 *pnew_dotcloud = NULL;

      pcl::PointCloud<pcl::PointXYZ> *pcloud = NULL;

      if (info->pubdotcloud)
      {
        pnew_dotcloud = new sensor_msgs::PointCloud2;
        pcloud = new pcl::PointCloud<pcl::PointXYZ>;

        pcloud->width = theparam.img_width;
        pcloud->height = theparam.img_height; //此处也可以为cloud.width = 4; cloud.height = 2;
                                              //   printf("will resize points!\n");
        pcloud->points.resize(pcloud->width * pcloud->height);
      }

      static uint16_t *depthrgbmask = (uint16_t *)malloc(MAXWIDTH * MAXHEIGHT * sizeof(uint16_t));
      static uint8_t *depthrgbmaskview = (uint8_t *)malloc(MAXWIDTH * MAXHEIGHT * 3);
      static uint8_t *depthpseudo = (uint8_t *)malloc(MAXWIDTH * MAXHEIGHT * 3);
      memset(depthrgbmask, 0, width * height * sizeof(uint16_t));
      memset(depthrgbmaskview, 0, width * height * 3);
      memset(depthpseudo, 0, width * height * 3);
      // printf("will convert dotclould!\n");
      if (info->pubdepthalign || info->pubpseudo || info->pubdotcloud)
      {
        for (int row = 0; row < height; row++)
        {
          for (int col = 0; col < width; col++)
          {
            int index = row * width + col;
            uint16_t depth = *(tmppic + row * width + col);
            MYRGB color;
            color.r = 0;
            color.g = 0;
            color.b = 0;
            if (depth >= DEPTHMIN && depth <= DEPTHMAX)
            {
              color = *(g_colortable + depth);
              if (info->pubpseudo)
              {
                *(depthpseudo + (row * width + col) * 3) = color.r;
                *(depthpseudo + (row * width + col) * 3 + 1) = color.g;
                *(depthpseudo + (row * width + col) * 3 + 2) = color.b;
              }
            }
            else
            {
              if (info->pubpseudo)
              {
                *(depthpseudo + (row * width + col) * 3) = 0;
                *(depthpseudo + (row * width + col) * 3 + 1) = 0;
                *(depthpseudo + (row * width + col) * 3 + 2) = 0;
              }
            }

            double X = ((double)col - LEFTCAMERAX) * (double)depth / LEFTCAMERAFX;
            double Y = ((double)row - LEFTCAMERAY) * (double)depth / LEFTCAMERAFY;
            double Z = (double)depth;
            if (info->pubdotcloud)
            {
              pcloud->points[index].x = X / 1000.0;
              pcloud->points[index].y = Y / 1000.0;
              pcloud->points[index].z = Z / 1000.0;
            }
            ////////////////////////////////////////////////
            /////////////////��ʼ��depthdotcloud->rgbdotcloud
            if (depth > 0 && info->pubdepthalign)
            {
              float rgbx = (DEPTHINRGBROTATE[0] * X + DEPTHINRGBROTATE[1] * Y + DEPTHINRGBROTATE[2] * Z + DEPTHINRGBOFFSET[0]) / 1000.0;
              float rgby = (DEPTHINRGBROTATE[3] * X + DEPTHINRGBROTATE[4] * Y + DEPTHINRGBROTATE[5] * Z + DEPTHINRGBOFFSET[1]) / 1000.0;
              float rgbz = (DEPTHINRGBROTATE[6] * X + DEPTHINRGBROTATE[7] * Y + DEPTHINRGBROTATE[8] * Z + DEPTHINRGBOFFSET[2]) / 1000.0;
              /////////////////rgbdotcloud->rgb
              if (rgbz > 0.0001)
              {
                float tmpx = rgbx / rgbz, tmpy = rgby / rgbz;

                int resultu = (int)(tmpx * RGBFX + RGBX + 0.5);
                int resultv = (int)(tmpy * RGBFY + RGBY + 0.5);
                //	printf("resultuv:%d,%d\n", resultu, resultv);
                if (resultu >= 0 && resultu < width && resultv >= 0 && resultv < height)
                {
                  *(depthrgbmask + (resultv * width + resultu)) = depth;
                  *(depthrgbmaskview + (resultv * width + resultu) * 3) = color.r;
                  *(depthrgbmaskview + (resultv * width + resultu) * 3 + 1) = color.g;
                  *(depthrgbmaskview + (resultv * width + resultu) * 3 + 2) = color.b;
                }
              }
            }
          }
        }
      }
      if (info->pubdotcloud)
      {
        // printf("after convert dotclould!\n");
        // Convert the cloud to ROS message
        pcl::toROSMsg(*pcloud, *pnew_dotcloud);

        // printf("after torosmsg!\n");
        pnew_dotcloud->header.frame_id = "odom";

        info->dotcloudpublisher.publish(*pnew_dotcloud);
        delete pnew_dotcloud;
        delete pcloud;
      }
      /////////////////////////////////
      if (info->pubdepthalign)
      {
        sensor_msgs::Image align_image;

        align_image.header.frame_id = "feynman_camera/depthalignrgb";
        align_image.width = theparam.img_width;
        align_image.height = theparam.img_height;
        align_image.is_bigendian = 0;
        align_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        align_image.step = sizeof(unsigned short) * align_image.width;

        data_size = align_image.step * align_image.height;
        align_image.data.resize(data_size);

        in_ptr = reinterpret_cast<unsigned short *>(&align_image.data[0]);

        memcpy(in_ptr, depthrgbmask, data_size);

        info->depthalignrgbpublisher.publish(align_image);

        /////////////////////////////////////////////
        sensor_msgs::Image alignimage;

        alignimage.header.frame_id = "feynman_camera/depthalignrgbview";
        alignimage.width = theparam.img_width;
        alignimage.height = theparam.img_height;
        alignimage.is_bigendian = 0;
        alignimage.encoding = sensor_msgs::image_encodings::RGB8;
        alignimage.step = 3 * alignimage.width;

        data_size = alignimage.step * alignimage.height;
        alignimage.data.resize(data_size);

        width = alignimage.width;
        height = alignimage.height;
        //   printf("ok convert nv12 to rgb24!\n");
        unsigned char *inalign_ptr = reinterpret_cast<unsigned char *>(&alignimage.data[0]);
        //   printf("will memcpy:%d!%d\n", data_size, width * height * 3);
        memcpy(inalign_ptr, depthrgbmaskview, data_size);
        //  printf("end memcpy!\n");
        info->depthalignrgbviewpublisher.publish(alignimage);
      }

      if (info->pubpseudo)
      {
        sensor_msgs::Image pseudoimage;

        pseudoimage.header.frame_id = "feynman_camera/depthpseudo";
        pseudoimage.width = theparam.img_width;
        pseudoimage.height = theparam.img_height;
        pseudoimage.is_bigendian = 0;
        pseudoimage.encoding = sensor_msgs::image_encodings::RGB8;
        pseudoimage.step = 3 * pseudoimage.width;

        data_size = pseudoimage.step * pseudoimage.height;
        pseudoimage.data.resize(data_size);

        width = pseudoimage.width;
        height = pseudoimage.height;
        //   printf("ok convert nv12 to rgb24!\n");
        unsigned char *inpseudo_ptr = reinterpret_cast<unsigned char *>(&pseudoimage.data[0]);
        //   printf("will memcpy:%d!%d\n", data_size, width * height * 3);
        memcpy(inpseudo_ptr, depthpseudo, data_size);
        //  printf("end memcpy!\n");
        info->depthpseudopublisher.publish(pseudoimage);
      }
    }
  }
}
void ircallback(void *data, void *userdata)
{
  // printf("enter ircallback!\n");
  DEVICEINFO *info = (DEVICEINFO *)userdata;
  FEYNMAN_USBHeaderDataPacket *tmppack = (FEYNMAN_USBHeaderDataPacket *)data;

  if (!info->pubir)
    return;

  if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_IR_IMAGE_LEFT_VI)
  {
    // ROS_INFO("depth left raw!\n");
    if (g_cameraparam.size() == 1 && (tmppack->len - sizeof(FEYNMAN_USB_IMAGE_HEADER)) == g_cameraparam[0].img_width * g_cameraparam[0].img_height * 3 / 2)
    {
      s_feynman_cam_param theparam = g_cameraparam[0];
      sensor_msgs::Image new_image;

      new_image.header.frame_id = "feynman_camera/sensorrawleft";
      new_image.width = theparam.img_width;
      new_image.height = theparam.img_height;
      new_image.is_bigendian = 0;
      new_image.encoding = sensor_msgs::image_encodings::RGB8;
      new_image.step = 3 * new_image.width;

      std::size_t data_size = new_image.step * new_image.height;
      new_image.data.resize(data_size);

      int width = new_image.width;
      int height = new_image.height;
      static unsigned char *rgb = NULL;
      if (NULL == rgb)
        rgb = (unsigned char *)malloc(MAXWIDTH * MAXHEIGHT * 3);
      unsigned char *tmpimgdata = tmppack->data + sizeof(FEYNMAN_USB_IMAGE_HEADER);
      yuv420_rgb24_std(width, height, tmpimgdata, tmpimgdata + width * height, tmpimgdata + width * height + width * height / 4, width, width / 2, rgb, width * 3, YCBCR_601);

      unsigned short *in_ptr = reinterpret_cast<unsigned short *>(&new_image.data[0]);
      memcpy(in_ptr, rgb, width * height * 3);

      info->sensorrawleftpublisher.publish(new_image);
    }
  }
  else if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_IR_IMAGE_RIGHT_VI)
  {
    // ROS_INFO("depth left raw!\n");
    if (g_cameraparam.size() == 1 && (tmppack->len - sizeof(FEYNMAN_USB_IMAGE_HEADER)) == g_cameraparam[0].img_width * g_cameraparam[0].img_height * 3 / 2)
    {
      s_feynman_cam_param theparam = g_cameraparam[0];
      sensor_msgs::Image new_image;

      new_image.header.frame_id = "feynman_camera/sensorrawright";
      new_image.width = theparam.img_width;
      new_image.height = theparam.img_height;
      new_image.is_bigendian = 0;
      new_image.encoding = sensor_msgs::image_encodings::RGB8;
      new_image.step = 3 * new_image.width;

      std::size_t data_size = new_image.step * new_image.height;
      new_image.data.resize(data_size);

      int width = new_image.width;
      int height = new_image.height;
      static unsigned char *rgb = NULL;
      if (NULL == rgb)
        rgb = (unsigned char *)malloc(MAXWIDTH * MAXHEIGHT * 3);
      unsigned char *tmpimgdata = tmppack->data + sizeof(FEYNMAN_USB_IMAGE_HEADER);
      yuv420_rgb24_std(width, height, tmpimgdata, tmpimgdata + width * height, tmpimgdata + width * height + width * height / 4, width, width / 2, rgb, width * 3, YCBCR_601);

      unsigned short *in_ptr = reinterpret_cast<unsigned short *>(&new_image.data[0]);
      memcpy(in_ptr, rgb, width * height * 3);

      info->sensorrawrightpublisher.publish(new_image);
    }
  }
  else if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_IR_IMAGE_LEFT_VPSS)
  {
    // ROS_INFO("depth left raw!\n");
    if (g_cameraparam.size() == 1 && (tmppack->len - sizeof(FEYNMAN_USB_IMAGE_HEADER)) == g_cameraparam[0].img_width * g_cameraparam[0].img_height * 3 / 2)
    {
      s_feynman_cam_param theparam = g_cameraparam[0];
      sensor_msgs::Image new_image;

      new_image.header.frame_id = "feynman_camera/rectifyleft";
      new_image.width = theparam.img_width;
      new_image.height = theparam.img_height;
      new_image.is_bigendian = 0;
      new_image.encoding = sensor_msgs::image_encodings::RGB8;
      new_image.step = 3 * new_image.width;

      std::size_t data_size = new_image.step * new_image.height;
      new_image.data.resize(data_size);

      int width = new_image.width;
      int height = new_image.height;
      static unsigned char *rgb = NULL;
      if (NULL == rgb)
        rgb = (unsigned char *)malloc(MAXWIDTH * MAXHEIGHT * 3);
      unsigned char *tmpimgdata = tmppack->data + sizeof(FEYNMAN_USB_IMAGE_HEADER);
      yuv420_rgb24_std(width, height, tmpimgdata, tmpimgdata + width * height, tmpimgdata + width * height + width * height / 4, width, width / 2, rgb, width * 3, YCBCR_601);

      unsigned short *in_ptr = reinterpret_cast<unsigned short *>(&new_image.data[0]);
      memcpy(in_ptr, rgb, width * height * 3);

      info->rectifyleftpublisher.publish(new_image);
    }
  }
  else if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_IR_IMAGE_RIGHT_VPSS)
  {
    // ROS_INFO("depth left raw!\n");
    if (g_cameraparam.size() == 1 && (tmppack->len - sizeof(FEYNMAN_USB_IMAGE_HEADER)) == g_cameraparam[0].img_width * g_cameraparam[0].img_height * 3 / 2)
    {
      s_feynman_cam_param theparam = g_cameraparam[0];
      sensor_msgs::Image new_image;

      new_image.header.frame_id = "feynman_camera/rectifyright";
      new_image.width = theparam.img_width;
      new_image.height = theparam.img_height;
      new_image.is_bigendian = 0;
      new_image.encoding = sensor_msgs::image_encodings::RGB8;
      new_image.step = 3 * new_image.width;

      std::size_t data_size = new_image.step * new_image.height;
      new_image.data.resize(data_size);

      int width = new_image.width;
      int height = new_image.height;
      static unsigned char *rgb = NULL;
      if (NULL == rgb)
        rgb = (unsigned char *)malloc(MAXWIDTH * MAXHEIGHT * 3);
      unsigned char *tmpimgdata = tmppack->data + sizeof(FEYNMAN_USB_IMAGE_HEADER);
      yuv420_rgb24_std(width, height, tmpimgdata, tmpimgdata + width * height, tmpimgdata + width * height + width * height / 4, width, width / 2, rgb, width * 3, YCBCR_601);

      unsigned short *in_ptr = reinterpret_cast<unsigned short *>(&new_image.data[0]);
      memcpy(in_ptr, rgb, width * height * 3);

      info->rectifyrightpublisher.publish(new_image);
    }
  }
  else if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_DEPTH_IMAGE_LEFT_RAW)
  {
    // ROS_INFO("depth left raw!\n");
    if (g_cameraparam.size() == 1 && (tmppack->len - sizeof(FEYNMAN_USB_IMAGE_HEADER)) == g_cameraparam[0].img_width * g_cameraparam[0].img_height)
    {
      unsigned char *tmpimgdata = tmppack->data + sizeof(FEYNMAN_USB_IMAGE_HEADER);
      int width = g_cameraparam[0].img_width;
      int height = g_cameraparam[0].img_height;

      if (g_leftdepth == NULL)
      {
        g_leftdepth = (uint8_t *)malloc(MAXWIDTH * MAXHEIGHT);
      }
      memcpy(g_leftdepth, tmpimgdata, width * height);

      s_feynman_cam_param theparam = g_cameraparam[0];
      sensor_msgs::Image new_image;

      new_image.header.frame_id = "feynman_camera/depthleftraw";
      new_image.width = theparam.img_width;
      new_image.height = theparam.img_height;
      new_image.is_bigendian = 0;
      new_image.encoding = sensor_msgs::image_encodings::MONO8;
      new_image.step = new_image.width;

      std::size_t data_size = new_image.step * new_image.height;
      new_image.data.resize(data_size);
      /*
      static unsigned char *rgb = NULL;
      //   printf("will malloc in depth left raw!\n");
      if (NULL == rgb)
        rgb = (unsigned char *)malloc(width * height * 3);
      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col++)
        {
          *(rgb + row * width * 3 + col * 3) = *(tmpimgdata + row * width + col);
          *(rgb + row * width * 3 + col * 3 + 1) = *(tmpimgdata + row * width + col);
          *(rgb + row * width * 3 + col * 3 + 2) = *(tmpimgdata + row * width + col);
        }
      }*/
      unsigned char *in_ptr = reinterpret_cast<unsigned char *>(&new_image.data[0]);
      memcpy(in_ptr, tmpimgdata, width * height);
      new_image.header.stamp = ros::Time::now();
      info->depthrawleftpublisher.publish(new_image);

      //  printf("will publish camerainfo of left ir!\n");
      sensor_msgs::CameraInfo caminfo;

      caminfo.width = theparam.img_width;
      caminfo.height = theparam.img_height;
      caminfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      caminfo.D.resize(5, 0.0);
      caminfo.D[0] = 0;
      caminfo.D[1] = 0;
      caminfo.D[2] = 0;
      caminfo.D[3] = 0;
      caminfo.D[4] = 0;

      caminfo.K.assign(0.0);
      caminfo.K[0] = theparam.left_sensor_focus[0];
      caminfo.K[2] = theparam.left_sensor_photocenter[0];
      caminfo.K[4] = theparam.left_sensor_focus[1];
      caminfo.K[5] = theparam.left_sensor_photocenter[1];
      caminfo.K[8] = 1.0;

      caminfo.R.assign(0.0);
      // for (int i = 0; i < 9; i++)
      //{
      //   caminfo.R[i] = p.r2l_r[i];
      // }
      caminfo.R[0] = 1;
      caminfo.R[4] = 1;
      caminfo.R[8] = 1;

      caminfo.P.assign(0.0);
      caminfo.P[0] = caminfo.K[0];
      caminfo.P[2] = caminfo.K[2];
      caminfo.P[3] = (theparam.is_new_format ? theparam.left2right_extern_param[9] : theparam.left2right_extern_param[3]);
      caminfo.P[5] = caminfo.K[4];
      caminfo.P[6] = caminfo.K[5];
      caminfo.P[7] = 0;
      caminfo.P[10] = 1.0;
      caminfo.P[11] = 0;
      // Fill in header
      caminfo.header.stamp = new_image.header.stamp;
      caminfo.header.frame_id = "feynman_camera/leftir/camerainfo";

      info->leftircamerainfopublisher.publish(caminfo);
    }
  }
  else if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_DEPTH_IMAGE_RIGHT_RAW)
  {
    // ROS_INFO("depth right raw!\n");
    if (g_cameraparam.size() == 1 && (tmppack->len - sizeof(FEYNMAN_USB_IMAGE_HEADER)) == g_cameraparam[0].img_width * g_cameraparam[0].img_height)
    {
      s_feynman_cam_param theparam = g_cameraparam[0];
      sensor_msgs::Image new_image;

      new_image.header.frame_id = "feynman_camera/depthrightraw";
      new_image.width = theparam.img_width;
      new_image.height = theparam.img_height;
      new_image.is_bigendian = 0;
      new_image.encoding = sensor_msgs::image_encodings::MONO8;
      new_image.step = new_image.width;

      std::size_t data_size = new_image.step * new_image.height;
      new_image.data.resize(data_size);

      int width = new_image.width;
      int height = new_image.height;
      unsigned char *tmpimgdata = tmppack->data + sizeof(FEYNMAN_USB_IMAGE_HEADER);
      /* static unsigned char *rgb = NULL;

      //   printf("will malloc in depth right raw!\n");
      if (NULL == rgb)
        rgb = (unsigned char *)malloc(width * height * 3);
      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col++)
        {
          *(rgb + row * width * 3 + col * 3) = *(tmpimgdata + row * width + col);
          *(rgb + row * width * 3 + col * 3 + 1) = *(tmpimgdata + row * width + col);
          *(rgb + row * width * 3 + col * 3 + 2) = *(tmpimgdata + row * width + col);
        }
      }*/
      unsigned char *in_ptr = reinterpret_cast<unsigned char *>(&new_image.data[0]);
      memcpy(in_ptr, tmpimgdata, width * height);

      new_image.header.stamp = ros::Time::now();
      info->depthrawrightpublisher.publish(new_image);

      //    printf("will publish camerainfo of right ir!\n");
      sensor_msgs::CameraInfo caminfo;

      caminfo.width = theparam.img_width;
      caminfo.height = theparam.img_height;
      caminfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      caminfo.D.resize(5, 0.0);
      caminfo.D[0] = 0;
      caminfo.D[1] = 0;
      caminfo.D[2] = 0;
      caminfo.D[3] = 0;
      caminfo.D[4] = 0;

      caminfo.K.assign(0.0);
      caminfo.K[0] = theparam.right_sensor_focus[0];
      caminfo.K[2] = theparam.right_sensor_photocenter[0];
      caminfo.K[4] = theparam.right_sensor_focus[1];
      caminfo.K[5] = theparam.right_sensor_photocenter[1];
      caminfo.K[8] = 1.0;

      caminfo.R.assign(0.0);
      // for (int i = 0; i < 9; i++)
      //{
      //   caminfo.R[i] = p.r2l_r[i];
      // }
      caminfo.R[0] = 1;
      caminfo.R[4] = 1;
      caminfo.R[8] = 1;

      caminfo.P.assign(0.0);
      caminfo.P[0] = caminfo.K[0];
      caminfo.P[2] = caminfo.K[2];
      caminfo.P[3] = (theparam.is_new_format ? theparam.left2right_extern_param[9] : theparam.left2right_extern_param[3]);
      caminfo.P[5] = caminfo.K[4];
      caminfo.P[6] = caminfo.K[5];
      caminfo.P[7] = 0;
      caminfo.P[10] = 1.0;
      caminfo.P[11] = 0;
      // Fill in header
      caminfo.header.stamp = new_image.header.stamp;
      caminfo.header.frame_id = "feynman_camera/rightir/camerainfo";

      info->rightircamerainfopublisher.publish(caminfo);
    }
  }
}
void othercallback(void *data, void *userdata)
{
  DEVICEINFO *info = (DEVICEINFO *)userdata;
  FEYNMAN_USBHeaderDataPacket *tmppack = (FEYNMAN_USBHeaderDataPacket *)data;

  if (hasstartpipeline == 0)
  {
    hasstartpipeline = 1;
    ROS_INFO("will start pipeline!\n");
    feynman_startpipeline();
  }
  static time_t lastseconds = 0;
  if ((time(NULL) - lastseconds) > 5)
  {
    printf("will get camparam!\n");
    lastseconds = time(NULL);
    feynman_getirexposure(100);
    feynman_getprojector(100);

    feynman_getdepthconfig(100);
    feynman_getrunconfig(100);
    //  if (!g_hasgotparam)
    feynman_getcamparam(100);
  }

  if (tmppack->type == FEYNMAN_USER_DATA && tmppack->sub_type == FEYNMAN_USER_DATA_TO_PC)
  {
    ROS_INFO("receive user deined data!");
  }
  else if (tmppack->type == FEYNMAN_DEVICE_DATA && tmppack->sub_type == FEYNMAN_DEVICE_DATA_ALL)
  {
    s_feynman_device_info *tmpinfo = (s_feynman_device_info *)tmppack->data;
    g_runconfig = tmpinfo->app_run_mode.app_run_mode;
    feynman_camera::temp_info tempinfo;
    tempinfo.cputemp = tmpinfo->cpu_temperaure;
    tempinfo.projectortemp = tmpinfo->projector_temperaure[0];
    // printf("camera fps:%d\n", tmpinfo->fps.fps);

    info->temperaturepublisher.publish(tempinfo);
    static bool g_isfirsttime = true;
    if (tmpinfo->fps.resolution == 0)
    {
      ROS_INFO("recv resolution:1280x800\n");
    }
    else if (tmpinfo->fps.resolution == 1)
    {
      ROS_INFO("recv resolution:1280x720\n");
    }
    else if (tmpinfo->fps.resolution == 2)
    {
      ROS_INFO("recv resolution:640x480\n");
    }
    else if (tmpinfo->fps.resolution == 3)
    {
      ROS_INFO("recv resolution:640x400\n");
    }
    else if (tmpinfo->fps.resolution == 4)
    {
      ROS_INFO("recv resolution:320x200\n");
    }
    else
    {
      ROS_INFO("recv resolution:unknown:%d\n", tmpinfo->fps.resolution);
    }

    if (((0 == strcmp(info->resolution, "1280x800") && tmpinfo->fps.resolution != 0) ||
         (0 == strcmp(info->resolution, "1280x720") && tmpinfo->fps.resolution != 1) ||
         (0 == strcmp(info->resolution, "640x480") && tmpinfo->fps.resolution != 2) ||
         (0 == strcmp(info->resolution, "640x400") && tmpinfo->fps.resolution != 3) ||
         (0 == strcmp(info->resolution, "320x200") && tmpinfo->fps.resolution != 4) ||
         info->fps != tmpinfo->fps.fps) &&
        g_isfirsttime)
    {
      g_isfirsttime = false;
      printf("will change resolution only once!!!\n");
      if (0 == strcmp(info->resolution, "1280x800"))
        feynman_setresolutionfps(FEYNMAN_RESOLUTION_1280_800, info->fps);
      else if (0 == strcmp(info->resolution, "1280x720"))
        feynman_setresolutionfps(FEYNMAN_RESOLUTION_1280_720, info->fps);
      else if (0 == strcmp(info->resolution, "640x480"))
        feynman_setresolutionfps(FEYNMAN_RESOLUTION_640_480, info->fps);
      else if (0 == strcmp(info->resolution, "640x400"))
        feynman_setresolutionfps(FEYNMAN_RESOLUTION_640_400, info->fps);
      else if (0 == strcmp(info->resolution, "320x200"))
        feynman_setresolutionfps(FEYNMAN_RESOLUTION_320_200, info->fps);

      feynman_resetpipeline();
    }
  }
  else if (tmppack->type == FEYNMAN_COMMAND_DATA && tmppack->sub_type == FEYNMAN_COMMAND_GET_CAM_PARAM_RETURN)
  {
    s_feynman_cam_param *tmpinfo = (s_feynman_cam_param *)tmppack->data;

    if (g_cameraparam.size() == 1 && tmpinfo->img_width != 0)
    {
      s_feynman_cam_param theparam = g_cameraparam[0];
      if (0 != memcmp(&theparam, tmpinfo, sizeof(s_feynman_cam_param)))
      {
        g_cameraparam.clear();
        g_cameraparam.push_back(*tmpinfo);
      }
    }
    else if (g_cameraparam.size() == 0 && tmpinfo->img_width != 0)
    {
      g_cameraparam.push_back(*tmpinfo);
    }
    // ROS_INFO("recv camera param:%dx%d\n", tmpinfo->img_width, tmpinfo->img_height);
  }

  else if (tmppack->type == FEYNMAN_LOG_DATA && tmppack->sub_type == FEYNMAN_LOG_ALL)
  {
    char *tmpstr = (char *)calloc(1, tmppack->len + 1);
    memcpy(tmpstr, tmppack->data, tmppack->len);
    std_msgs::String logmsg;
    logmsg.data = tmpstr;
    free(tmpstr);

    info->logpublisher.publish(logmsg);
  }

  else if (tmppack->type == FEYNMAN_CNN_DATA && tmppack->sub_type == FEYNMAN_CNN_DATA_ALL)
  {
    s_feynman_cnn_data *tmpcnndata = (s_feynman_cnn_data *)tmppack->data;
    if (tmpcnndata->groups > 0)
    {
      /*
              typedef struct
              {
                int frame_id; //< frame id of cnn result
                int groups; //< groups sum of cnn result
                s_feynman_cnn_sub_data group[10];// groups of cnn result
              }s_feynman_cnn_data;
        */

      feynman_camera::cnn_info cnninfo;
      cnninfo.frameid = tmpcnndata->frame_id;
      cnninfo.cnn_box.resize(tmpcnndata->groups);
      for (int i = 0; i < tmpcnndata->groups; i++)
      {
        cnninfo.cnn_box[i].xmin = tmpcnndata->group[i].xmin;
        cnninfo.cnn_box[i].ymin = tmpcnndata->group[i].ymin;
        cnninfo.cnn_box[i].xmax = tmpcnndata->group[i].xmax;
        cnninfo.cnn_box[i].ymax = tmpcnndata->group[i].ymax;
      }

      info->cnnpublisher.publish(cnninfo);
    }
  }
}
void callback(const char *devicename, void *userdata)
{
  DEVICEINFO *tmpdev = (DEVICEINFO *)userdata;
  ROS_INFO("will connect to device:%s\n", devicename);
  char *tmpdevicename = strdup(devicename);
  int tmpid = feynman_connectcameraforid(tmpdevicename);
  free(tmpdevicename);
  if (tmpid != 0xffffffff)
  {
    DEVICEINFO *thedev = (DEVICEINFO *)userdata;
    ROS_INFO("want devid:%u,got device!!!%u\n", thedev->device_id, tmpid);

    if (thedev->device_id == tmpid)
    {
      strcpy(tmpdev->devicename, devicename);
      tmpdev->connected = 1;
    }
  }
}
void *devicethread(void *param)
{
  DEVICEINFO *info = (DEVICEINFO *)param;
  feynman_init();
  while (1)
  {
    while (info->connected == 0)
    {
      ROS_INFO("will refresh usb device!\n");
      usleep(1000 * 1000);
      feynman_refresh(callback, info);
    }
    ROS_INFO("device:%d connected!\n", info->device_id);

    feynman_connectcamera(info->devicename,
                          info->pubimu ? imucallback : NULL,
                          info->savedata ? savecallback : NULL,
                          info->pubdepth ? depthcallback : NULL,
                          info->pubir ? ircallback : NULL,
                          info->pubrgb ? rgbcallback : NULL,
                          othercallback, info);
    ROS_INFO("after connect camera!\n");
    feynman_waitfordisconnect();
    info->connected = 0;
  }
  return 0;
}
int main(int argc, char *argv[])
{
  ROS_INFO("enter main func!\n");
  ros::init(argc, argv, "feynman_camera");
  ros::NodeHandle node_obj;

  dynamic_reconfigure::Server<feynman_camera::resfpsConfig> dynamicresfpsserver;
  dynamic_reconfigure::Server<feynman_camera::resfpsConfig>::CallbackType f;
  f = boost::bind(&resfpscallback, _1); //绑定回调函数
  dynamicresfpsserver.setCallback(f);   //为服务器设置回调函数， 节点程序运行时会调用一次回调函数来输出当前的参数配置情况

  ROS_INFO("will get device_id from launch!\n");
  int device_id = 305419896;
  int fps = 30;
  std::string resolutionstr = "";

  DEVICEINFO *info = (DEVICEINFO *)calloc(1, sizeof(DEVICEINFO));

  if (!ros::param::get("/feynmannode/pubrgb", info->pubrgb))
  {
    printf("fail to get pubrgb from param!");
    exit(0);
  }
  if (!ros::param::get("/feynmannode/pubir", info->pubir))
  {
    printf("fail to get pubir from param!");
    exit(0);
  }
  if (!ros::param::get("/feynmannode/pubdepth", info->pubdepth))
  {
    printf("fail to get pubdepth from param!");
    exit(0);
  }
  if (!ros::param::get("/feynmannode/savedata", info->savedata))
  {
    printf("fail to get savedata from param!");
    exit(0);
  }
  if (!ros::param::get("/feynmannode/pubdepthalign", info->pubdepthalign))
  {
    printf("fail to get pubdepthalign from param!");
    exit(0);
  }
  if (!ros::param::get("/feynmannode/pubdotcloud", info->pubdotcloud))
  {
    printf("fail to get pubdotcloud from param!");
    exit(0);
  }

  if (!ros::param::get("/feynmannode/confidence", info->confidence))
  {
    printf("fail to get confidence from param!");
    exit(0);
  }

  if (!ros::param::get("/feynmannode/pubpseudo", info->pubpseudo))
  {
    printf("fail to get pubpseudo from param!");
    exit(0);
  }

  if (!ros::param::get("/feynmannode/pubimu", info->pubimu))
  {
    printf("fail to get pubimu from param!");
    exit(0);
  }

  if (!ros::param::get("/feynmannode/device_id", device_id))
  {
    printf("fail to get device id from param!\n");
    exit(0);
  }
  //  usleep(10*1000*1000);
  ROS_INFO("got device_id:%d from launch!\n", device_id);

  if (!ros::param::get("/feynmannode/resolution", resolutionstr))
  {
    printf("fail to get resolution from param!\n");
    exit(0);
  }
  ROS_INFO("got resolution:%s from launch!\n", resolutionstr.c_str());

  if (!ros::param::get("/feynmannode/fps", fps))
  {
    printf("fail to get fps from param!");
    exit(0);
  }

  ROS_INFO("got fps:%d from launch!\n", fps);
  if ((resolutionstr != "1280x800" &&
       resolutionstr != "1280x720" &&
       resolutionstr != "640x480" &&
       resolutionstr != "640x400" &&
       resolutionstr != "320x200") ||
      fps != 30)
  {
    printf("Only support 1280x800/1280x700/640x480/640x400/320x200 at 30fps!\n");
    exit(0);
  }

  char tmpparamsstr[64];
  sprintf(tmpparamsstr, "feynman_camera/%d/streammode", device_id);

  ros::ServiceServer devicesetsteammodeservice = node_obj.advertiseService(tmpparamsstr, handle_device_streammode_request);

  sprintf(tmpparamsstr, "feynman_camera/%d/savedepth", device_id);

  ros::ServiceServer savedepthservice = node_obj.advertiseService(tmpparamsstr, handle_savedepth_request);

  sprintf(tmpparamsstr, "feynman_camera/%d/removedark", device_id);

  ros::ServiceServer removedarkservice = node_obj.advertiseService(tmpparamsstr, handle_removedark_request);

  sprintf(tmpparamsstr, "feynman_camera/%d/switchrectify", device_id);

  ros::ServiceServer deviceswitchrectifyservice = node_obj.advertiseService(tmpparamsstr, handle_device_switchrectify_request);

  sprintf(tmpparamsstr, "feynman_camera/%d/depthmode", device_id);
  ros::ServiceServer devicesetdepthmodeservice = node_obj.advertiseService(tmpparamsstr, handle_device_depthmode_request);

  // sprintf(tmpparamsstr, "feynman_camera/%d/cameraparam", device_id);
  // ros::ServiceServer devicegetparamservice = node_obj.advertiseService(tmpparamsstr, handle_device_cameraparam_request);
  sprintf(tmpparamsstr, "feynman_camera/%d/setprojector", device_id);
  ros::ServiceServer devicesetprojectorservice = node_obj.advertiseService(tmpparamsstr, handle_device_setprojector_request);

  sprintf(tmpparamsstr, "feynman_camera/%d/enablepointcloud", device_id);
  ros::ServiceServer enablepointcloudservice = node_obj.advertiseService(tmpparamsstr, handle_device_enablepointcloud_request);

  sprintf(tmpparamsstr, "feynman_camera/%d/enableimu", device_id);
  ros::ServiceServer enableimuservice = node_obj.advertiseService(tmpparamsstr, handle_device_enableimu_request);

  sprintf(tmpparamsstr, "feynman_camera/%d/exposure", device_id);
  ros::ServiceServer devicesetexposureservice = node_obj.advertiseService(tmpparamsstr, handle_device_exposure_request);

  sprintf(tmpparamsstr, "/feynman_camera/%d/leftir/camera_info", device_id);
  info->leftircamerainfopublisher = node_obj.advertise<sensor_msgs::CameraInfo>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/rightir/camera_info", device_id);
  info->rightircamerainfopublisher = node_obj.advertise<sensor_msgs::CameraInfo>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/rgb/camera_info", device_id);
  info->rgbcamerainfopublisher = node_obj.advertise<sensor_msgs::CameraInfo>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/depth/image_raw", device_id);
  info->depthrawpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/depthalignrgb/image_raw", device_id);
  info->depthalignrgbpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/depthalignrgb/image_color", device_id);
  info->depthalignrgbviewpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/depth/image_color", device_id);
  info->depthpseudopublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/leftir/image_rect", device_id);
  info->depthrawleftpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);
  sprintf(tmpparamsstr, "/feynman_camera/%d/rightir/image_rect", device_id);
  info->depthrawrightpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);
  sprintf(tmpparamsstr, "/feynman_camera/%d/temperature", device_id);
  info->temperaturepublisher = node_obj.advertise<feynman_camera::temp_info>(tmpparamsstr, 10);
  sprintf(tmpparamsstr, "/feynman_camera/%d/imu", device_id);
  info->imupublisher = node_obj.advertise<feynman_camera::imu_info>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/depth/dotcloud", device_id);
  info->dotcloudpublisher = node_obj.advertise<sensor_msgs::PointCloud2>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/depth/camera_info", device_id);
  info->depthcamerainfopublisher = node_obj.advertise<sensor_msgs::CameraInfo>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/rgb/image_rect_color", device_id);
  info->rgbpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/rgb/image_color", device_id);
  info->rgbrawpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/leftir/image_color", device_id);
  info->sensorrawleftpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/rightir/image_color", device_id);
  info->sensorrawrightpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/leftir/image_rect_color", device_id);
  info->rectifyleftpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/rightir/image_rect_color", device_id);
  info->rectifyrightpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/cnn_info", device_id);
  info->cnnpublisher = node_obj.advertise<feynman_camera::cnn_info>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/cameralog", device_id);
  info->logpublisher = node_obj.advertise<std_msgs::String>(tmpparamsstr, 10);

  info->device_id = device_id;
  info->fps = fps;
  strcpy(info->resolution, resolutionstr.c_str());

  calculatecolortable();

  pthread_t devicethreadid;
  pthread_create(&devicethreadid, NULL, devicethread, info);
  // getchar();
  ros::spin();
  feynman_deinit();
  return 0;
}

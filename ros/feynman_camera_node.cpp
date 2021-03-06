#include "feynman_sdk.h"

#include <string>
#include <ros/ros.h>
#include "feynman_camera/SetStreamMode.h"
#include "feynman_camera/SwitchRectify.h"
#include "feynman_camera/SetExposure.h"
#include "feynman_camera/SetDepthMode.h"
#include "feynman_camera/SetProjector.h"
#include "feynman_camera/GetCameraParam.h"
#include "feynman_camera/temp_info.h"
#include "feynman_camera/cnn_box.h"
#include "feynman_camera/cnn_info.h"
#include "yuv_rgb.h"
#include <std_msgs/String.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

//#include <pcl/ros/conversions.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>

#include <string.h>
////////////////////////////////////
std::vector<s_feynman_cam_param>
    g_cameraparam;
//0.obtain device id from launch file
//1.when device plugin,enumrate device and open it and recv data from it
//2.while device id is not the one request,close it
//3.while device id is the one request,open it and continue recv data from it

bool handle_device_switchrectify_request(feynman_camera::SwitchRectifyRequest &req,
                                         feynman_camera::SwitchRectifyResponse &res)
{
  if (req.rectifyornot == "rectify")
  { //rectify
    feynman_setvpss();
  }
  else
  { //not rectify,just vi
    feynman_setvi();
  }
  return true;
}

bool handle_device_streammode_request(feynman_camera::SetStreamModeRequest &req,
                                      feynman_camera::SetStreamModeResponse &res)
{
  if (req.mode == 0)
  { //vi
    //0.vi 1.vpss 2.depth 3.cnn
    feynman_setmode(0);
    feynman_resetpipeline();
  }
  else if (req.mode == 1)
  { //vpss
    //0.vi 1.vpss 2.depth 3.cnn
    feynman_setmode(1);
    feynman_resetpipeline();
  }
  else if (req.mode == 2)
  { //depth
    feynman_setmode(2);
    feynman_resetpipeline();
  }
  else if (req.mode == 3)
  { //cnn
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
  { //vpss
    //0.vi 1.vpss 2.depth 3.cnn
    feynman_setdepthdenoise(1);
    feynman_resetpipeline();
  }
  else if (req.mode == "fusion")
  { //depth
    //0.vi 1.vpss 2.depth 3.cnn
    feynman_setdepthfusion(1);
    feynman_resetpipeline();
  }
  else if (req.mode == "zoom")
  { //cnn
    feynman_setdepthzoom(1);
    feynman_resetpipeline();
  }
  else if (req.mode == "stitch")
  { //cnn
    feynman_setdepthstitch(1);
    feynman_resetpipeline();
  }
  return true;
}
bool handle_device_setprojector_request(feynman_camera::SetProjectorRequest &req,
                                        feynman_camera::SetProjectorResponse &res)
{

  feynman_setprojector(req.enable);

  return true;
}
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
    /*for (int i = 0; i < 9; i++)
    {
      info.R[i] = p.r2l_r[i];
    }*/
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
}
typedef struct
{
  char devicename[64];
  unsigned int device_id;
  int connected;
  ros::Publisher depthrawpublisher;
  ros::Publisher depthrawleftpublisher;
  ros::Publisher depthrawrightpublisher;
  ros::Publisher temperaturepublisher;
  ros::Publisher dotcloudpublisher;
  ros::Publisher rgbpublisher;
  ros::Publisher sensorrawleftpublisher;
  ros::Publisher sensorrawrightpublisher;
  ros::Publisher rectifyleftpublisher;
  ros::Publisher rectifyrightpublisher;
  ros::Publisher cnnpublisher;
  ros::Publisher logpublisher;
} DEVICEINFO;

int hasstartpipeline = 0;
int g_hasgotparam = 0;
void framecallback(void *data, void *userdata)
{
  if (hasstartpipeline == 0)
  {
    hasstartpipeline = 1;
    ROS_INFO("will start pipeline!\n");
    feynman_startpipeline();
  }
  static time_t lastseconds = 0;
  if ((time(NULL) - lastseconds) > 5)
  {
    lastseconds = time(NULL);
    feynman_getirexposure(100);
    feynman_getprojector(100);

    feynman_getdepthconfig(100);
    feynman_getrunconfig(100);
    //  if (!g_hasgotparam)
    feynman_getcamparam(100);
  }

  DEVICEINFO *info = (DEVICEINFO *)userdata;
  FEYNMAN_USBHeaderDataPacket *tmppack = (FEYNMAN_USBHeaderDataPacket *)data;
  if (tmppack->type == FEYNMAN_DEVICE_DATA && tmppack->sub_type == FEYNMAN_DEVICE_DATA_ALL)
  {
    s_feynman_device_info *tmpinfo = (s_feynman_device_info *)tmppack->data;
    feynman_camera::temp_info tempinfo;
    tempinfo.cputemp = tmpinfo->cpu_temperaure;
    tempinfo.projectortemp = tmpinfo->projector_temperaure[0];

    info->temperaturepublisher.publish(tempinfo);
    ROS_INFO("recv deviceid:0x%X\n", tmpinfo->device_id);
  }
  else if (tmppack->type == FEYNMAN_COMMAND_DATA && tmppack->sub_type == FEYNMAN_COMMAND_GET_CAM_PARAM_RETURN)
  {
    s_feynman_cam_param *tmpinfo = (s_feynman_cam_param *)tmppack->data;

    if (g_cameraparam.size() == 1)
    {
      s_feynman_cam_param theparam = g_cameraparam[0];
      if (0 != memcmp(&theparam, tmpinfo, sizeof(s_feynman_cam_param)))
      {
        g_cameraparam.clear();
        g_cameraparam.push_back(*tmpinfo);
      }
    }
    else
    {
      g_cameraparam.push_back(*tmpinfo);
    }
    ROS_INFO("recv camera param:%dx%d\n", tmpinfo->img_width, tmpinfo->img_height);
  }
  else if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_IR_IMAGE_LEFT_VI)
  {
    //ROS_INFO("depth left raw!\n");
    if (g_cameraparam.size() == 1)
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
        rgb = (unsigned char *)malloc(width * height * 3);
      yuv420_rgb24_std(width, height, tmppack->data, tmppack->data + width * height, tmppack->data + width * height + width * height / 4, width, width / 2, rgb, width * 3, YCBCR_601);

      unsigned short *in_ptr = reinterpret_cast<unsigned short *>(&new_image.data[0]);
      memcpy(in_ptr, rgb, width * height * 3);

      info->sensorrawleftpublisher.publish(new_image);
    }
  }
  else if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_IR_IMAGE_RIGHT_VI)
  {
    //ROS_INFO("depth left raw!\n");
    if (g_cameraparam.size() == 1)
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
        rgb = (unsigned char *)malloc(width * height * 3);
      yuv420_rgb24_std(width, height, tmppack->data, tmppack->data + width * height, tmppack->data + width * height + width * height / 4, width, width / 2, rgb, width * 3, YCBCR_601);

      unsigned short *in_ptr = reinterpret_cast<unsigned short *>(&new_image.data[0]);
      memcpy(in_ptr, rgb, width * height * 3);

      info->sensorrawrightpublisher.publish(new_image);
    }
  }
  else if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_IR_IMAGE_LEFT_VPSS)
  {
    //ROS_INFO("depth left raw!\n");
    if (g_cameraparam.size() == 1)
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
        rgb = (unsigned char *)malloc(width * height * 3);
      yuv420_rgb24_std(width, height, tmppack->data, tmppack->data + width * height, tmppack->data + width * height + width * height / 4, width, width / 2, rgb, width * 3, YCBCR_601);

      unsigned short *in_ptr = reinterpret_cast<unsigned short *>(&new_image.data[0]);
      memcpy(in_ptr, rgb, width * height * 3);

      info->rectifyleftpublisher.publish(new_image);
    }
  }
  else if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_IR_IMAGE_RIGHT_VPSS)
  {
    //ROS_INFO("depth left raw!\n");
    if (g_cameraparam.size() == 1)
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
        rgb = (unsigned char *)malloc(width * height * 3);
      yuv420_rgb24_std(width, height, tmppack->data, tmppack->data + width * height, tmppack->data + width * height + width * height / 4, width, width / 2, rgb, width * 3, YCBCR_601);

      unsigned short *in_ptr = reinterpret_cast<unsigned short *>(&new_image.data[0]);
      memcpy(in_ptr, rgb, width * height * 3);

      info->rectifyrightpublisher.publish(new_image);
    }
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
  else if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_DEPTH_IMAGE_LEFT_RAW)
  {
    //ROS_INFO("depth left raw!\n");
    if (g_cameraparam.size() == 1)
    {
      s_feynman_cam_param theparam = g_cameraparam[0];
      sensor_msgs::Image new_image;

      new_image.header.frame_id = "feynman_camera/depthleftraw";
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
        rgb = (unsigned char *)malloc(width * height * 3);
      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col++)
        {
          *(rgb + row * width * 3 + col * 3) = *(tmppack->data + row * width + col);
          *(rgb + row * width * 3 + col * 3 + 1) = *(tmppack->data + row * width + col);
          *(rgb + row * width * 3 + col * 3 + 2) = *(tmppack->data + row * width + col);
        }
      }
      unsigned short *in_ptr = reinterpret_cast<unsigned short *>(&new_image.data[0]);
      memcpy(in_ptr, rgb, width * height * 3);

      info->depthrawleftpublisher.publish(new_image);
    }
  }
  else if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_DEPTH_IMAGE_RIGHT_RAW)
  {
    // ROS_INFO("depth right raw!\n");
    if (g_cameraparam.size() == 1)
    {
      s_feynman_cam_param theparam = g_cameraparam[0];
      sensor_msgs::Image new_image;

      new_image.header.frame_id = "feynman_camera/depthrightraw";
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
        rgb = (unsigned char *)malloc(width * height * 3);
      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col++)
        {
          *(rgb + row * width * 3 + col * 3) = *(tmppack->data + row * width + col);
          *(rgb + row * width * 3 + col * 3 + 1) = *(tmppack->data + row * width + col);
          *(rgb + row * width * 3 + col * 3 + 2) = *(tmppack->data + row * width + col);
        }
      }
      unsigned short *in_ptr = reinterpret_cast<unsigned short *>(&new_image.data[0]);
      memcpy(in_ptr, rgb, width * height * 3);

      info->depthrawrightpublisher.publish(new_image);
    }
  }
  else if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_DEPTH_IMAGE)
  {
    // ROS_INFO("depth data!\n");

    if (g_cameraparam.size() == 1)
    {
      s_feynman_cam_param theparam = g_cameraparam[0];
      sensor_msgs::Image new_image;

      new_image.header.frame_id = "feynman_camera/depthraw";
      new_image.width = theparam.img_width;
      new_image.height = theparam.img_height;
      new_image.is_bigendian = 0;
      new_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      new_image.step = sizeof(unsigned short) * new_image.width;

      std::size_t data_size = new_image.step * new_image.height;
      new_image.data.resize(data_size);

      unsigned short *in_ptr = reinterpret_cast<unsigned short *>(&new_image.data[0]);
      memcpy(in_ptr, tmppack->data, tmppack->len);

      info->depthrawpublisher.publish(new_image);
      /////////////////////////will publish dot cloud
      sensor_msgs::PointCloud2 new_dotcloud;
      pcl::PointCloud<pcl::PointXYZ> cloud;

      double LEFTCAMERAF = 1048.0;
      double CAMERAT = 23.590832;
      double LEFTCAMERAX = 174.361675;
      double LEFTCAMERAY = 297.007244;

      LEFTCAMERAX = theparam.left_sensor_photocenter[0];
      LEFTCAMERAY = theparam.left_sensor_photocenter[1];
      LEFTCAMERAF = theparam.left_sensor_focus[0];
      if (theparam.is_new_format == 1)
      {
        CAMERAT = abs(theparam.left2right_extern_param[9]);
      }
      else
      {
        CAMERAT = abs(theparam.left2right_extern_param[3]);
      }

      unsigned int *tmppic = (unsigned int *)tmppack->data;
      // Fill in the cloud data
      cloud.width = theparam.img_width;
      cloud.height = theparam.img_height; //??????????????????cloud.width = 4; cloud.height = 2;
      cloud.points.resize(cloud.width * cloud.height);
      int width = cloud.width;
      int height = cloud.height;
      for (int row = 0; row < height; row++)
      {
        for (int col = 0; col < width; col++)
        {
          int index = row * width + col;

          //	DOTINFO* info = &(g_dotcloud[row*width+col]);
          uint16_t depth = *(tmppic + row * width + col);

          double X = col - LEFTCAMERAX;
          double Y = row - LEFTCAMERAY;
          double Z = LEFTCAMERAF;
          double W = LEFTCAMERAF * CAMERAT / (double)depth;

          cloud.points[index].x = X / W;
          cloud.points[index].y = Y / W;
          cloud.points[index].z = Z / W;
        }
      }

      //Convert the cloud to ROS message
      pcl::toROSMsg(cloud, new_dotcloud);
      new_dotcloud.header.frame_id = "odom";

      info->dotcloudpublisher.publish(new_dotcloud);
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
  else if (tmppack->type == FEYNMAN_IMAGE_DATA && tmppack->sub_type == FEYNMAN_RGB_IMAGE_SINGLE)
  {
    if (g_cameraparam.size() == 1)
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
      if (NULL == buffer)
        buffer = (unsigned char *)malloc(width * height * 3);
      nv12_rgb24_std(width, height, tmppack->data, tmppack->data + width * height, width, width, buffer, width * 3, YCBCR_601);

      unsigned short *in_ptr = reinterpret_cast<unsigned short *>(&new_image.data[0]);

      memcpy(in_ptr, buffer, data_size);

      info->rgbpublisher.publish(new_image);
    }
  }
  else
  {
    ROS_INFO("other data!type:%d,subtype:%d\n", tmppack->type, tmppack->sub_type);
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

int main(int argc, char *argv[])
{
  ROS_INFO("enter main func!\n");
  ros::init(argc, argv, "feynman_camera");
  ros::NodeHandle node_obj;

  ROS_INFO("will get device_id from launch!\n");
  int device_id = 305419896;
  ros::param::get("/feynmannode/device_id", device_id);
  //  usleep(10*1000*1000);
  ROS_INFO("got device_id:%d from launch!\n", device_id);
  char tmpparamsstr[64];
  sprintf(tmpparamsstr, "feynman_camera/%d/streammode", device_id);

  ros::ServiceServer devicesetsteammodeservice = node_obj.advertiseService(tmpparamsstr, handle_device_streammode_request);

  sprintf(tmpparamsstr, "feynman_camera/%d/switchrectify", device_id);

  ros::ServiceServer deviceswitchrectifyservice = node_obj.advertiseService(tmpparamsstr, handle_device_switchrectify_request);

  sprintf(tmpparamsstr, "feynman_camera/%d/depthmode", device_id);
  ros::ServiceServer devicesetdepthmodeservice = node_obj.advertiseService(tmpparamsstr, handle_device_depthmode_request);

  sprintf(tmpparamsstr, "feynman_camera/%d/cameraparam", device_id);
  ros::ServiceServer devicegetparamservice = node_obj.advertiseService(tmpparamsstr, handle_device_cameraparam_request);
  sprintf(tmpparamsstr, "feynman_camera/%d/setprojector", device_id);
  ros::ServiceServer devicesetprojectorservice = node_obj.advertiseService(tmpparamsstr, handle_device_setprojector_request);

  sprintf(tmpparamsstr, "feynman_camera/%d/exposure", device_id);
  ros::ServiceServer devicesetexposureservice = node_obj.advertiseService(tmpparamsstr, handle_device_exposure_request);

  DEVICEINFO *info = (DEVICEINFO *)calloc(1, sizeof(DEVICEINFO));

  sprintf(tmpparamsstr, "/feynman_camera/%d/depth", device_id);
  info->depthrawpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);
  sprintf(tmpparamsstr, "/feynman_camera/%d/depth_raw_left", device_id);
  info->depthrawleftpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);
  sprintf(tmpparamsstr, "/feynman_camera/%d/depth_raw_right", device_id);
  info->depthrawrightpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);
  sprintf(tmpparamsstr, "/feynman_camera/%d/temperature", device_id);
  info->temperaturepublisher = node_obj.advertise<feynman_camera::temp_info>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/dotcloud", device_id);
  info->dotcloudpublisher = node_obj.advertise<sensor_msgs::PointCloud2>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/rgb", device_id);
  info->rgbpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/sensor_raw_left", device_id);
  info->sensorrawleftpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/sensor_raw_right", device_id);
  info->sensorrawrightpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/sensor_rectify_left", device_id);
  info->rectifyleftpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/sensor_rectify_right", device_id);
  info->rectifyrightpublisher = node_obj.advertise<sensor_msgs::Image>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/cnn_info", device_id);
  info->cnnpublisher = node_obj.advertise<feynman_camera::cnn_info>(tmpparamsstr, 10);

  sprintf(tmpparamsstr, "/feynman_camera/%d/cameralog", device_id);
  info->logpublisher = node_obj.advertise<std_msgs::String>(tmpparamsstr, 10);

  feynman_init();

  info->device_id = device_id;
  while (info->connected == 0)
  {
    ROS_INFO("will refresh usb device!\n");
    usleep(1000 * 1000);
    feynman_refresh(callback, info);
  }
  ROS_INFO("device:%d connected!\n", info->device_id);
  feynman_connectcamera(info->devicename, framecallback, info);
  ROS_INFO("after connect camera!\n");
  // getchar();
  ros::spin();
  feynman_deinit();
  return 0;
}

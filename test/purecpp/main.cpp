#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "feynman_sdk.h"
typedef struct
{
  char devicename[64];
  unsigned int device_id;
  unsigned int fps;
  char resolution[16];
  int connected;
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
void othercallback(void *data, void *userdata)
{

printf("othercallback!\n");

}

void depthcallback(void *data, void *userdata)
{

printf("depthcallback!\n");


}

void imucallback(void *data, void *userdata)
{

printf("imucallback!\n");


}



void ircallback(void *data, void *userdata)
{


printf("ircallback!\n");

}


void savecallback(void *data, void *userdata)
{

printf("savecallback!\n");


}


void rgbcallback(void *data, void *userdata)
{

printf("rgbcallback!\n");

}




void callback(const char *devicename, void *userdata)
{
  DEVICEINFO *tmpdev = (DEVICEINFO *)userdata;
  printf("will connect to device:%s\n", devicename);

  int tmpid = feynman_connectcameraforid(devicename);

  if (tmpid != 0xffffffff)
  {
    DEVICEINFO *thedev = (DEVICEINFO *)userdata;
    printf("want devid:%u,got device!!!%u\n", thedev->device_id, tmpid);

    if (thedev->device_id == tmpid)
    {
      strcpy(tmpdev->devicename, devicename);
      tmpdev->connected = 1;
    }
  }
}

int main(int argc,const char* argv[]){
  DEVICEINFO *info = (DEVICEINFO *)calloc(1, sizeof(DEVICEINFO));
  info->device_id=305419896;
  feynman_init();
  while (1)
  {
    while (info->connected == 0)
    {
      printf("will refresh usb device!\n");
      usleep(1000 * 1000);
      feynman_refresh(callback, info);
    }
    printf("device:%d connected!\n", info->device_id);

    feynman_connectcamera(info->devicename,
                          imucallback,
                          savecallback,
                          depthcallback,
                          ircallback,
                          rgbcallback,
                          othercallback, info);
    printf("after connect camera!\n");
    feynman_waitfordisconnect();
    info->connected = 0;
  }

	printf("hello,world!\n");
	return 0;
}

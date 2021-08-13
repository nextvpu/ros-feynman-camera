#include "../include/feynman_sdk.h"
#include <stdio.h>
int hasstartpipeline = 0;
void framecallback(void *data, void *userdata)
{
  printf("rev data from device!\n");
  if (hasstartpipeline == 0)
  {
    hasstartpipeline = 1;

    feynman_startpipeline();
  }
}
void callback(const char *devicename, void *userdata)
{

  feynman_connectcamera(devicename, framecallback, userdata);
}
int main(int argc, const char *argv[])
{
  feynman_init();
  feynman_refresh(callback, NULL);
  printf("press enter to exit!\n");
  getchar();
  return 0;
}
#include "feynman_sdk.h"
#include <stdio.h>

void framecallback(void *data, void *userdata)
{
  printf("rev data from device!\n");
}

void callback(const char *devicename, void *userdata)
{

  feynman_connectcamera(devicename, framecallback, NULL, NULL, NULL, NULL, NULL, userdata);
}

int main(int argc, const char *argv[])
{
  feynman_init();
  feynman_refresh(callback, NULL);
  printf("press enter to exit!\n");
  getchar();
  return 0;
}

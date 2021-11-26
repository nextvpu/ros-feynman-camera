#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include "cppapi.h"

#include <iostream>
using namespace std;

DeviceList::DeviceList()
{
	total = 0;
}

DeviceInfo *DeviceList::Find(unsigned int devid)
{
	for (int i = 0; i < total; i++)
	{
		DeviceInfo &di = devices[i];
		if (di.id == devid)
			return &di;
	}
	return nullptr;
}

bool DeviceList::Add(unsigned int devid, const char *devname)
{
	if (Find(devid))
		return true;
	if (total == MAX_DEVICES)
		return false;
	DeviceInfo &di = devices[total++];
	di.id = devid;
	strcpy(di.name, devname);
	return true;
}

void refresh_callback(const char *devname, void *userdata)
{
	DeviceList *devlist = (DeviceList *)userdata;
	int deviceid = feynman_connectcameraforid(devname);
	if (deviceid == 0xffffffff)
		return;
	devlist->Add(deviceid, devname);
}

void Feynman::EnumDevices(DeviceList *devlist, int timeout)
{
	while (timeout--)
	{
		feynman_refresh(refresh_callback, devlist);
		if (devlist->total > 0)
			return;
		usleep(1000 * 1000);
	}
}

Feynman::Feynman()
{
	feynman_init();
}

Feynman::~Feynman()
{
	feynman_deinit();
}

void imu_callback(void *data, void *userdata)
{
	cout << "imu_callback" << endl;
}

void save_callback(void *data, void *userdata)
{
	cout << "save_callback" << endl;
}

void depth_callback(void *data, void *userdata)
{
	cout << "depth_callback" << endl;
}

void ir_callback(void *data, void *userdata)
{
	cout << "ir_callback" << endl;
}

void rgb_callback(void *data, void *userdata)
{
	cout << "rgb_callback" << endl;
}

void other_callback(void *data, void *userdata)
{
	cout << "other_callback" << endl;
}

void Feynman::Connect(const char *devname)
{
	feynman_connectcamera(devname,
						  imu_callback,
						  save_callback,
						  depth_callback,
						  ir_callback,
						  rgb_callback,
						  other_callback,
						  this);
}

void Feynman::Disconnect()
{
	feynman_disconnectcamera();
	feynman_waitfordisconnect();
}

bool Feynman::IsConnected()
{
	return feynman_hasconnect();
}

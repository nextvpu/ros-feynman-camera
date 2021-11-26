#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include "cppapi.h"

#include <iostream>
using namespace std;

DeviceList Feynman::g_devlist;

ostream& operator<<(ostream& os, const DeviceInfo& di)
{
	os << "ID: " << di.id << "; Name: " << di.name << endl;
	return os;
}

ostream& operator<<(ostream& os, const DeviceList& dl)
{
	os << "Total Devices: " << dl.total << endl;
	for (int i = 0; i < dl.total; i++)
		os << dl.devices[i];
	return os;
}

DeviceList::DeviceList()
: total(-1)
{
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

int Feynman::EnumDevices(int timeout, DeviceList* devlist)
{
	if (devlist == nullptr)
		devlist = &g_devlist;
	devlist->total = 0;
	while (timeout--)
	{
		feynman_refresh(refresh_callback, devlist);
		if (devlist->total > 0)
		{
			return devlist->total;
		}
		usleep(1000 * 1000);
	}
	return devlist->total;
}

Feynman::Feynman()
{
	feynman_init();
	ResetCounters();
}

Feynman::~Feynman()
{
	feynman_deinit();
}

void imu_callback(void *data, void *userdata)
{
	((Feynman*)userdata)->CallbackIMU(data);
}

void save_callback(void *data, void *userdata)
{
	((Feynman*)userdata)->CallbackSave(data);
}

void depth_callback(void *data, void *userdata)
{
	((Feynman*)userdata)->CallbackDepth(data);
}

void ir_callback(void *data, void *userdata)
{
	((Feynman*)userdata)->CallbackIR(data);
}

void rgb_callback(void *data, void *userdata)
{
	((Feynman*)userdata)->CallbackRGB(data);
}

void other_callback(void *data, void *userdata)
{
	((Feynman*)userdata)->CallbackOther(data);
}

bool Feynman::Connect(const char *devname)
{
	if (devname == nullptr)
	{
		if (g_devlist.total < 0)
		{
			EnumDevices(5);
		}
		if (g_devlist.total == 0)
		{
			cout << "No camera found!" << endl;
			return false;
		}
		devname = g_devlist.devices[0].name;
	}
	ResetCounters();
	return feynman_connectcamera(devname,
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

void Feynman::ResetCounters()
{
	for (int i = 0; i < MAX_COUNTERS; i++)
		m_ccb[i] = 0;
}

void Feynman::CallbackIMU(void* data)
{
	m_ccb[0]++;
}

void Feynman::CallbackSave(void* data)
{
	m_ccb[1]++;
}

void Feynman::CallbackDepth(void* data)
{
	m_ccb[2]++;
}

void Feynman::CallbackIR(void* data)
{
	m_ccb[3]++;
}

void Feynman::CallbackRGB(void* data)
{
	m_ccb[4]++;
}

void Feynman::CallbackOther(void* data)
{
	m_ccb[5]++;
}

void Feynman::ShowCounters()
{
	cout << "[Counters]" << endl;
	cout << "  IMU: " << m_ccb[0] << endl;
	cout << " Save: " << m_ccb[1] << endl;
	cout << "Depth: " << m_ccb[2] << endl;
	cout << "   IR: " << m_ccb[3] << endl;
	cout << "  RGB: " << m_ccb[4] << endl;
	cout << "Other: " << m_ccb[5] << endl;
}


#pragma once

#include "feynman_sdk.h"

#define MAX_DEV_NAME 64
#define MAX_DEVICES 4

struct DeviceInfo
{
	unsigned int id;
	char name[MAX_DEV_NAME];
};

struct DeviceList
{
    int total;
    DeviceInfo devices[MAX_DEVICES];

public:
    DeviceList();
    DeviceInfo* Find(unsigned int devid);
    bool Add(unsigned int devid, const char* devname);

};

class Feynman
{
public:
	static int EnumDevices(int timeout=10, DeviceList* devlist=nullptr);
	static DeviceList g_devlist;

public:
	Feynman();
	~Feynman();

	bool Connect(const char* devname=nullptr);
	void Disconnect();
	bool IsConnected();
};


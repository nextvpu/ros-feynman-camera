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
	static void EnumDevices(DeviceList* devlist, int timeout=10);

public:
	Feynman();
	~Feynman();

	void Connect(const char* devname);
	void Disconnect();
	bool IsConnected();
};

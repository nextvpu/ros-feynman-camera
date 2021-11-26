#pragma once

#include "feynman_sdk.h"
#include <iostream>

#define MAX_DEV_NAME 64
#define MAX_DEVICES 4
#define MAX_COUNTERS 6

struct DeviceInfo
{
	unsigned int id;
	char name[MAX_DEV_NAME];

	friend std::ostream& operator<<(std::ostream& os, const DeviceInfo& di);
};

struct DeviceList
{
    int total;
    DeviceInfo devices[MAX_DEVICES];

public:
    DeviceList();
    DeviceInfo* Find(unsigned int devid);
    bool Add(unsigned int devid, const char* devname);

	friend std::ostream& operator<<(std::ostream& os, const DeviceList& dl);
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

public:
	void CallbackIMU(void* data);
	void CallbackSave(void* data);
	void CallbackDepth(void* data);
	void CallbackIR(void* data);
	void CallbackRGB(void* data);
	void CallbackOther(void* data);
public:
	void ResetCounters();
	void ShowCounters();
	int GetCounter(int index) const { return m_ccb[index]; }
protected:
	int m_ccb[MAX_COUNTERS];
};


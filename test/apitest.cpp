#include <gtest/gtest.h>
#include "cppapi.h"

#include <iostream>
using namespace std;

static DeviceList DevList;

TEST(FeynmanTest, EnumDevice) {
	Feynman f;
    f.EnumDevices(&DevList, 15);
    ASSERT_GE(DevList.total, 1);
}

TEST(FeynmanTest, Connect) {
	Feynman f;
    ASSERT_GE(DevList.total, 1);
    const char* devname = DevList.devices[0].name;
    f.Connect(devname);
    bool connected = f.IsConnected();
    EXPECT_TRUE(connected);
}

TEST(FeynmanTest, Disconnect) {
	Feynman f;
    ASSERT_GE(DevList.total, 1);
    const char* devname = DevList.devices[0].name;
    f.Connect(devname);
    bool connected = f.IsConnected();
    EXPECT_TRUE(connected);
    f.Disconnect();
    connected = f.IsConnected();
    EXPECT_FALSE(connected);
}

TEST(FeynmanTest, Callback) {
	Feynman f;
    ASSERT_GE(DevList.total, 1);
    const char* devname = DevList.devices[0].name;
    f.Connect(devname);
	usleep(1000000 * 20);
    f.Disconnect();
    bool connected = f.IsConnected();
    EXPECT_FALSE(connected);
}

int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


#include <gtest/gtest.h>
#include "cppapi.h"

#include <iostream>
using namespace std;

TEST(FeynmanTest, EnumDevice) {
	Feynman f;
    int devices = f.EnumDevices(10);
	cout << f.g_devlist;
    ASSERT_GE(devices, 1);
}

TEST(FeynmanTest, Connect) {
	Feynman f;
    bool ok = f.Connect();
    EXPECT_TRUE(ok);
    bool connected = f.IsConnected();
    EXPECT_TRUE(connected);
}

TEST(FeynmanTest, Disconnect) {
	Feynman f;
    bool ok = f.Connect();
    EXPECT_TRUE(ok);
    bool connected = f.IsConnected();
    EXPECT_TRUE(connected);
    f.Disconnect();
    connected = f.IsConnected();
    EXPECT_FALSE(connected);
}

TEST(FeynmanTest, Callback) {
	Feynman f;
    bool ok = f.Connect();
	f.ShowCounters();
    EXPECT_TRUE(ok);
	usleep(1000000 * 4);
	f.ShowCounters();
	EXPECT_EQ(f.GetCounter(0), 0);
	for (int i = 1; i < 6; i++)
	{
		EXPECT_GT(f.GetCounter(i), 0);
	}
    f.Disconnect();
    bool connected = f.IsConnected();
    EXPECT_FALSE(connected);
}

int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


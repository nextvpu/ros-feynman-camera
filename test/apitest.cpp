#include <gtest/gtest.h>
#include "cppapi.h"

#include <iostream>
using namespace std;

TEST(FeynmanTest, EnumDevice) {
	Feynman f;
    int devices = f.EnumDevices(10);
    ASSERT_GE(devices, 1);
}

TEST(FeynmanTest, Connect) {
	Feynman f;
    f.Connect();
    bool connected = f.IsConnected();
    EXPECT_TRUE(connected);
}

TEST(FeynmanTest, Disconnect) {
	Feynman f;
    f.Connect();
    bool connected = f.IsConnected();
    EXPECT_TRUE(connected);
    f.Disconnect();
    connected = f.IsConnected();
    EXPECT_FALSE(connected);
}

TEST(FeynmanTest, Callback) {
	Feynman f;
    f.Connect();
	usleep(1000000 * 10);
    f.Disconnect();
    bool connected = f.IsConnected();
    EXPECT_FALSE(connected);
}

int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


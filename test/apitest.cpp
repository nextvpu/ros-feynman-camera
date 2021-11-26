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
    EXPECT_TRUE(ok);
	usleep(1000000 * 10);
    f.Disconnect();
    bool connected = f.IsConnected();
    EXPECT_FALSE(connected);
}

int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


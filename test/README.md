# API Test

API test code based on [GoogleTest](https://github.com/google/googletest)

## Build and run

```bash
./setup.sh
mkdir build
cd build
cmake ..
make
./apitest
```

## Command line options

```bash
./apitest --gtest_filter=*.EnumDevices
export GTEST_FILTER=*
./apitest --gtest_list_tests
export GTEST_FILTER=*.Callback
./apitest --gtest_list_tests
```


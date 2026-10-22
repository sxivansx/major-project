# major-project LiDAR bridge. See README.md.
#
#   make          build build/s2bridge (builds the vendored SDK first if needed)
#   make sdk      build only the Slamtec SDK static library
#   make clean    remove build/

CXX      := clang++
PLATFORM := $(shell uname -s)

SDK      := rplidar_sdk
SDK_LIB  := $(SDK)/output/$(PLATFORM)/Release/libsl_lidar_sdk.a

CXXFLAGS := -std=c++17 -O2 -Wall -Wextra
CPPFLAGS := -I $(SDK)/sdk/include -I $(SDK)/sdk/src
LDLIBS   := -lpthread
ifeq ($(PLATFORM),Linux)
LDLIBS   += -lrt
endif

.PHONY: all sdk clean

all: build/s2bridge

build/s2bridge: bridge/main.cpp $(SDK_LIB) | build
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -o $@ $< $(SDK_LIB) $(LDLIBS)

build:
	mkdir -p $@

sdk: $(SDK_LIB)

$(SDK_LIB):
	$(MAKE) -C $(SDK)

clean:
	rm -rf build

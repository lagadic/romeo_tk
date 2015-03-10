Romeo_tk
====================

# Installation

## Prerequisites
* [Install Aldebaran SDK C++](http://jokla.me/install-sdk-c-naoqi/) 

# Howto build in debug

	$ qibuild configure -c toolchain_romeo -Dvisp_naoqi_DIR=/local/soft/romeo/cpp/workspace/visp_naoqi/build-toolchain_romeo/sdk/cmake -DVISP_DIR=/local/soft/ViSP/ViSP-build-release
`	$ qibuild make -c toolchain_romeo

# Howto build in release

	$ qibuild configure --release -c toolchain_romeo -Dvisp_naoqi_DIR=/local/soft/romeo/cpp/workspace/visp_naoqi/build-toolchain_romeo/sdk/cmake -DVISP_DIR=/local/soft/ViSP/ViSP-build-release
`	$ qibuild make --release -c toolchain_romeo

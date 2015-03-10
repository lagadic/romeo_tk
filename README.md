Romeo_tk
====================

# Installation

## Prerequisites
* [Install Aldebaran SDK C++](http://jokla.me/install-sdk-c-naoqi/)
* [Install VispNaoqi](http://jokla.me/visp_naoqi/)

## How to build romeo_tk

* Clone the repository: `git clone http://www.github.com/lagadic/romeo_tk.git`
* Go via terminal in the folder romeo_tk:

`$ qibuild configure -c toolchain_romeo -Dvisp_naoqi_DIR=/change_with_your_path/visp_naoqi/build-toolchain_romeo/sdk/cmake -DVISP_DIR=/change_with_your_path/ViSP/ViSP-build-release`

`$ qibuild make -c toolchain_romeo`

where `toolchain_romeo` is the name of the toolchain you created previously, `-Dvisp_naoqi_DIR` has to point to the folder /sdk/cmake in the build folder of the project visp_naoqi, `-DVISP_DIR` points to the build directory of ViSP

* If you want to build in release run:

`$ qibuild configure --release -c toolchain_romeo -Dvisp_naoqi_DIR=/change_with_your_path/visp_naoqi/build-toolchain_romeo/sdk/cmake -DVISP_DIR=/change_with_your_path/ViSP/ViSP-build-release`

`$ qibuild make --release -c toolchain_romeo`

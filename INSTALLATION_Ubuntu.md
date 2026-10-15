# Installation on Ubuntu 22.04 / 24.04 / 26.04

I tested corolib originally on an Ubuntu MATE 22.04 installation in VirtualBox. 
The g++ version was 11.2.0.

Currently, I am using Ubuntu 22.04, Ubuntu 24.04 and Ubuntu 24.04 in WSL (Windows Subsystem for Linux) 
on a Windows 11 laptop.

The g++ version is 11.4.0 on Ubuntu 22.04, 13.3.0 on Ubuntu 24.04, and 15.2.0 on Ubuntu 24.06.

The g++ version should be >= 10 to have coroutine support.

## Install Ubuntu on WSL on Windows 11

The information in this section is only applicable if you do not have a native Ubuntu computer.

Open a Windows Terminal or PowerShell window. The Terminal opens with a PowerShell tab.

To install, e.g., Ubuntu 24.04, enter on the command line:
```
wsl --install -d Ubuntu-24.04
```
See https://documentation.ubuntu.com/wsl/en/latest/guides/install-ubuntu-wsl2/ 
and https://learn.microsoft.com/en-us/windows/wsl/install for more information.

The installation script asks for a Unix username and a password.

After installation, open a new Terminal window.
In the drop-down menu, you will find an entry "Ubuntu 24.04.1 LTS" (the minor version may change over time).
Select this menu item. It will open a tab running Ubuntu.

To verify the installed Ubuntu version, type on the command line:
```
lsb_release -a
```
Update and upgrade the Ubuntu installation:
```
sudo apt update
sudo apt upgrade
```
The first command will ask for the password you entered during the installation.

## Install build tools

There are a few things to install to be able to build corolib.
These are described next.

### gcc

If gcc is not installed:
```
prompt$ gcc
Command 'gcc' not found, but can be installed with:
sudo apt install gcc
prompt$
```
Solution:
```
sudo apt install gcc
```
### CMake

If cmake is not installed:
```
prompt$ cmake
Command 'cmake' not found, but can be installed with:
sudo snap install cmake  # version 3.31.4, or
sudo apt  install cmake  # version 3.27.8-1build1
See 'snap info cmake' for additional versions.
prompt$
```
Solution:
```
sudo apt install cmake
```
or, using -y to provide an automatic yes to prompts:
```
sudo apt install -y cmake
```
### build-essential

Also install build-essential (otherwise you will have to do it later anyway):
```
    sudo apt-get install build-essential
```
## corolib

Copy or clone corolib (https://github.com/JohanVanslembrouck/corolib) to your computer.

I propose to build corolib as follows:
```
cd <path>/corolib
mkdir build
cd build
cmake ../
make -j 4
```
or (better):
```
cd <path>/corolib
cd ..
mkdir corolib-build
cd corolib-build
cmake ../corolib
make -j 4
```
This second alternative keeps your build directory completely apart from the source code.

When using the same corolib source code directory to build corolib both on Windows and on WSL Ubuntu (using /mnt/...),
it occurred that Visual Studio got confused by the content of the Ubuntu build directory.

Go to [tutorial](./examples/tutorial) or any of the [studies](./studies) subdirectories and start the executables.
(See the README.md files in these directories.)

## Boost

Install Boost:
```
sudo apt-get update
sudo apt-get -y install libboost-dev
```
The Boost include files are in
```
/usr/include/boost
```
On my computer, the Boost libraries (version 1.74.0) are located in
```
/usr/lib/x86_64-linux-gnu/
```
After installing Boost ASIO, set the BOOST_INSTALLED flag to TRUE in the top-level CMakelists.txt file
if you want to build the Boost examples in [boost](./examples/boost) and [corolab](./studies/corolab).

## Qt

To install and configure qtcreator on Ubuntu 22.04 or Ubuntu 24.04, follow the instructions described in
https://askubuntu.com/questions/1404263/how-do-you-install-qt-on-ubuntu22-04:
```
sudo apt install -y qtcreator qtbase5-dev qt5-qmake cmake
```

### Building with cmake

Set the QT5_INSTALLED flag to TRUE in the top-level CMakelists.txt file
to build the Qt5 examples in [qt5](./examples/qt5).

Because Qt6 is not installed, in file examples/qt5/CMakeLists.txt change
```
set(USE_QT5 FALSE)
set(USE_QT6 TRUE)
```
to
```
set(USE_QT5 TRUE)
set(USE_QT6 FALSE)
```

### Building with QtCreator

Alternatively, you can use QtCreator. Select the qt5.pro file in the top-level directory.

Because the compiler used by QtCreator does not support C++ concepts, a modification is necessary
in file include/corolib/config.h. Change
```
#define USE_ASYNC_BASE 0
#define USE_WHEN_TYPE 1
```
to
```
#define USE_ASYNC_BASE 1
#define USE_WHEN_TYPE 0
```
Although optional, you can also make this change when building with cmake to use the same settings.

## gRPC

Starting points: https://github.com/grpc/grpc, https://github.com/grpc/grpc/tree/master/src/cpp

Building gRPC using CMake with FetchContent, see https://github.com/grpc/grpc/tree/master/src/cpp#fetchcontent, did not work on my computer.

Rather, I used the instructions under https://grpc.io/docs/languages/cpp/quickstart/#install-grpc
```
export MY_INSTALL_DIR=$HOME/.local
mkdir -p $MY_INSTALL_DIR
export PATH="$MY_INSTALL_DIR/bin:$PATH"

sudo apt install -y build-essential autoconf libtool pkg-config
```
Note: I used MY_INSTALL_DIR=$HOME/grpcbin as installation directory.

Clone the grpc repo and its submodules to a local directory:
```
git clone --recurse-submodules -b v1.55.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc

cd grpc
mkdir -p cmake/build
pushd cmake/build
cmake -DgRPC_INSTALL=ON \
  -DgRPC_BUILD_TESTS=OFF \
  -DCMAKE_INSTALL_PREFIX=$MY_INSTALL_DIR \
  ../..
make -j 4
make install
popd
```
Note: I cloned grpc to $HOME/grpcsrc.

Add the following lines to .bashrc for future use:
```
export PATH=$HOME/.local/bin:$PATH
```
or, in my setup:
```
export PATH=$HOME/grpcbin/bin:$PATH
```
After installing gRPC, set the GRPC_INSTALLED flag to TRUE in the top-level CMakelists.txt file
if you want to build the gRPC examples in [grpc](./examples/grpc).

## ROS 2

Please follow the instructions at https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

## TAO

Starting point: https://download.dre.vanderbilt.edu/

Download the file ACE+TAO-7.1.1.tar.gz and place it in (e.g.) your home directory, $HOME.

Unzip the file:
```
tar xvf ACE+TAO-7.1.1.tar.gz
```
This will produce a directory ACE_wrappers.
Consequently, the full path is $HOME\ACE_wrappers.

Open the file $HOME\ACE_wrappers\ACE-INSTALL.html in a web browser and navigate to the section 
"Building and Installing ACE on UNIX".

Follow the instructions in that section. In short:

Define the ACE_ROOT environment variable:
```
export ACE_ROOT=$HOME/ACE_wrappers 
```
Create a configuration file, $ACE_ROOT/ace/config.h, 
that includes the appropriate platform/compiler-specific header configurations from the ACE source directory:
```
#include "ace/config-linux.h"
```
Create a build configuration file, $ACE_ROOT/include/makeinclude/platform_macros.GNU,
that contains the appropriate platform/compiler-specific Makefile configurations, e.g.,
```
include $(ACE_ROOT)/include/makeinclude/platform_linux.GNU
```
Set LD_LIBRARY_PATH to the directory where the binary version of the ACE library will be built into:
```
export LD_LIBRARY_PATH=$ACE_ROOT/lib:$LD_LIBRARY_PATH
```
In a terminal window, navigate to $ACE_ROOT and build ACE:
```
cd $ACE_ROOT
make
```
Building ACE in my WSL environment took about 10 minutes.

Open the file ...\ACE_wrappers\TAO\TAO-INSTALL.html in a web browser and follow the instructions in section 
"On UNIX platforms". In short:

Build GPERF as follows:
```
cd $ACE_ROOT/apps/gperf/src
make
```
Set the TAO_ROOT environment variable to $ACE_ROOT/TAO:
```
export TAO_ROOT=$ACE_ROOT/TAO
```
Build TAO:
```
cd $TAO_ROOT
make
```
Building TAO takes a long time (about 2 hours in my WSL environment). I did not use e.g. make -j 4.

Add the following lines to .bashrc:
```
export ACE_ROOT=$HOME/ACE_wrappers
export TAO_ROOT=$ACE_ROOT/TAO
export PATH=$ACE_ROOT/bin:$PATH
export LD_LIBRARY_PATH=$ACE_ROOT/lib:$LD_LIBRARY_PATH
```
After installing TAO, set the TAO_INSTALLED flag to TRUE in the top-level CMakelists.txt file
if you want to build the TAO examples in [tao](./examples/tao).

## libevent

The installation instructions can be found here: https://github.com/libevent/libevent/blob/master/Documentation/Building.md#linux-deb-like-ubuntudebian

libevent was already installed on my Ubuntu installation, so I did not have to do anything.

Set the LIBEVENT_INSTALLED flag to TRUE in the top-level CMakelists.txt file
if you want to build the libevent examples in [libevent](./examples/libevent).

## curl

curl is by default installed on Ubuntu (and other Linux) distributions, so I did not have to do anything.

Set the CURL_INSTALLED flag to TRUE in the top-level CMakelists.txt file 
if you want to build the curl examples in [curl](./examples/curl).

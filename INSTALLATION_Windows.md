# Installation on Windows 10 or 11

I originally developed corolib with Visual Studio 2019 and Qt Creator 4.12.0 on a Windows 10 laptop.

Since May 2023, I have been using Visual Studio 2022 and Qt Creator 10.0.1 on a Windows 11 laptop.

In October 2026, I started using Visual Studio 2026 and Qt Creator 20.0.2 on the same Windows 11 laptop.

## corolib

Copy or clone corolib (https://github.com/JohanVanslembrouck/corolib) to your computer.

Building the Visual Studio projects makes use of the CMake support of Visual Studio:
```
File -> Open -> CMake...
```
and select the CMakeLists.txt in the top-level folder (directory).

Go to [tutorial](./examples/tutorial) or any of the [studies](./studies) subdirectories and start the executables.
(See the README.md files in these directories.)

## Boost

I originally used Boost 1.70 (now Boost 1.82), but many other versions will do.

Starting point: https://www.boost.org/

Download release 1.82 from https://boostorg.jfrog.io/artifactory/main/release/1.82.0/source/

I copied boost_1_82_0.zip to C:\local\boost and unzipped it into this directory.

In a Windows terminal (e.g. Command Prompt or PowerShell), go to C:\local\boost\boost_1_82_0

Follow the instructions in https://www.boost.org/doc/libs/1_82_0/more/getting_started/windows.html#simplified-build-from-source

Step 1: .\bootstrap.bat

This will generate b2.exe

Step 2: .\b2.exe

b2.exe generates the libraries we will need. Its execution takes a while.

The root directory for the include files is C:/local/boost/boost_1_82_0.

The root directory for the libraries is C:/local/boost/boost_1_82_0/stage/lib.

In the top-level CMakeLists.txt, adapt BOOST_ROOT to your own installation of the Boost library:
```
set(BOOST_ROOT C:/local/boost/boost_1_82_0/)
```
After installing Boost, set the BOOST_INSTALLED flag to TRUE in the top-level CMakelists.txt file
if you want to build the Boost examples in [boost](./examples/boost) and [corolab](./studies/corolab).

## Qt

The example code for Qt has been developed with Qt5.
Originally, the Qt examples were built using QtCreator only, using .pro files.
Afterwards, I added CMakeLists.txt files so that the Qt examples can also be built from Visual Studio.

I will describe two installations (one from May 2023 and one from October 2026),
because they involve different actions.
The version depends on the time you make the installation.

### Qt5 installation (May 2023)

Starting points: https://www.qt.io/download, https://www.qt.io/download-open-source

Download the Qt Online Installer for your operating system (macOS, Windows, Linux) 
(e.g., for Windows: qt-unified-windows-x64-4.5.2-online.exe).

Run the installer. I installed Qt in C:\Qt.

Edit the Environment Variables and add the following directory to the Path variable:
```
C:\Qt\5.15.2\msvc2019_64\bin
```
This directory contains the .dll files the applications need at run-time.

The version (5.15.2 in my case) will depend on your installation.

In contract to Visual Studio 2026, I did not have to configure Qt 5.12.0
in Visual Studio 2022 (see the section describing the Qt6 installation).

You can build the Qt examples from the top-level CMakeLists.txt file.
In addition, you can also build them from the qt5.pro file in the top-level directory.

#### Building with Visual Studio 2022 or 2026 using cmake

After installing Qt5, set the QT5_INSTALLED flag to TRUE in the top-level CMakelists.txt file
if you want to build the Qt5 examples in [qt5](./examples/qt5).

To use the Qt5 libraries, change the following settings in examples\Qt5\CMakeLists.txt from
```
set(USE_QT5 FALSE)
set(USE_QT6 TRUE)
```
to
```
set(USE_QT5 TRUE)
set(USE_QT6 FALSE)
```

#### Building with Qt Creator 10.0.1

Because Qt Creator 10.0.1 uses MSVC 2019 and because of the use of C++ 20 concepts
in some files in include\corolib, a modification is necessary in file include\corolib\config.h
to use the original implementation using inheritance instead of concepts
(which seem to be not supported by MSVC 2019).

Change the following lines ...
```
#define USE_ASYNC_BASE 0
#define USE_WHEN_TYPE 1
```
into ...
```
#define USE_ASYNC_BASE 1
#define USE_WHEN_TYPE 0
```
This modification is not necessary anymore with a more recent installation of Qt.
This installation is described next.

### Qt6 installation (October 2026)

Starting point: https://www.qt.io/development/download-qt-installer-oss

Download the Qt Online Installer for your operating system (macOS, Windows, Linux) 
(e.g., for Windows: qt-online-installer-windows-x64-4.11.0.exe).

Run the installer. I installed Qt in C:\Qt.

The installer installs only MinGW 13.1.0 64-bit as compiler.

If you also want to build the Qt examples with Visual Studio, an MSVC compiler
must be installed. This compiler can also be used from Qt Creator.

This requires 3 steps:

1. Add MSVC 2022 64-bit to Qt 6.12
2. Install Qt Visual Studio Tools
3. Configure Qt 6.12 in VS2026

These steps will now described in more details.

Step 1. Add MSVC 2022 64-bit to Qt 6.12

Start the Qt Maintenance Tool, located in C:\Qt\MaintenanceTool.exe

In "Maintenance actions", select "Add or remove components"

In "Customize", we see the following items:
```
Preview
Developer Tools
Extensions
Qt for Development
Visual Creation Tools
```
Expand "Qt for Development". "MinGW 13.1.0 64-bit" is already checked.
```
Qt for Development	
		Qt 6.12.0
			MinGW 13.1.0 64-bit
```
Also check
```
			MSVC 2022 64-bit
```

Step 2. Install Qt Visual Studio Tools

Start Visual Studio 2026. Do this:

1.	Extensions -> Manage Extensions
2.	Search for Qt Visual Studio Tools
3.	Select it and choose Download
4.	Restart Visual Studio to complete the installation. 

After restarting, you should have a new top-level menu:
```
Extensions -> Qt VS Tools
```

Step 3. Configure Qt 6.12 in VS2026

In Visual Studio 2026:

1.	Select Extensions -> Qt VS Tools -> Qt Versions.
2.	Click Add.
3.	For Location, point it at folder with the qmake.exe belonging to your Qt installation. 

For example, this might be:
```
C:\Qt\6.12.0\msvc2022_64\bin\
```
4.	Give it a recognizable name, such as:
```
6.12.0_msvc2022_64
```
5.	Select that version and click Default if you want new projects to use it automatically.

Alternatively, use Autodetect button to auto-detect and fill in the locations and names.

Edit the Environment Variables and add the following directory to the Path variable:
```
C:\Qt\6.12.0\msvc2022_64\bin
```
This directory contains the .dll files the applications need at run-time.

#### Building with Visual Studio 2022 or 2026 using cmake

After installing Qt6, set the QT5_INSTALLED flag to TRUE in the top-level CMakelists.txt file
if you want to build the Qt5 examples in [qt5](./examples/qt5).

No changes are necessary to build the Qt examples using Visual Studio.

#### Building with Qt Creator 20.0.2

QtCreator allows building the examples using MinGW (default) or MSVC.
Deselect MinGW and select MSVC instead.

To use MinGW, a change is necessary to the qt5.pri file in the home directory:

Change
```
win32 {
QMAKE_CXXFLAGS += /await:strict
}
unix {
QMAKE_CXXFLAGS += -fcoroutines
}
```
into
```
win32 {
QMAKE_CXXFLAGS += -fcoroutines
}
unix {
QMAKE_CXXFLAGS += -fcoroutines
}
```
In other words, when using MinGW, QMAKE_CXXFLAGS is the same on Windows and on Ubuntu.

## gRPC

Starting points: https://github.com/grpc/grpc, https://github.com/grpc/grpc/tree/master/src/cpp

Building gRPC using CMake with FetchContent, see https://github.com/grpc/grpc/tree/master/src/cpp#fetchcontent, did not work on my computer.

Rather, I used the instructions under https://github.com/grpc/grpc/tree/master/src/cpp#install-using-vcpkg-package
```
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg
```
I cloned vcpkg.git into C:\local, giving C:\local\vcpkg.

Note: this, of course, requires the installation of Git. If this is not yet the case, see https://gitforwindows.org/
```
./bootstrap-vcpkg.bat
./vcpkg install grpc
```
The execution of this last command may take 0.5 hour to 1.5 hours or even more, depending on your computer.

Finally, I added the following folders to my PATH:
```
C:\local\vcpkg\installed\x64-windows\tools\protobuf
C:\local\vcpkg\installed\x64-windows\tools\grpc
```
After installing gRPC, set the GRPC_INSTALLED flag to TRUE in the top-level CMakelists.txt file
if you want to build the gRPC examples in [grpc](./examples/grpc).

## ROS 2

I did not manage to install ROS 2 on Windows 11.
I tried to follow the instructions at https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html.
According to https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html#system-requirements, only Windows 10 is supported.

See below for an installation on Ubuntu 22.04 LTS.

## TAO

Starting point: https://download.dre.vanderbilt.edu/

Download the file ACE+TAO-7.1.1.zip and place it in a directory (e.g.) C:\local. 
This is the directory that already contains boost and vcpkg, see above.

Unzip the file. This will produce a directory ACE_wrappers.
Consequently, the full path is C:\local\ACE_wrappers.
I will refer to this directory as ...\ACE_wrappers from now on.

Open the file ...\ACE_wrappers\ACE-INSTALL.html in a web browser and navigate to the section 
"Building and Installing ACE on Windows" and, in case of the use of Visual Studio, to 
"Building and Installing ACE on Windows with Microsoft Visual Studio".

Follow the instructions in that section. In short:

In directory ...\ACE_wrappers\ace, create a file called config.h and enter the following line in this file:
```
#include "ace/config-win32.h"
```
Open the file ...\ACE_wrappers\ACE_vs2019.sln with Visual Studio 2019 or 2022 (or ACE_vs2017.sln with Visual Studio 2017).

Visual Studio 2022 reports the use of an earlier version of the Visual Studio platform toolset 
and proposes to upgrade the projects to target the latest Microsoft toolset. I upgraded to the latest toolset.

Build the solution. This builds ACE (The ADAPTIVE Communication Environment). We still have to build TAO (The ACE ORB).

Open the file ...\ACE_wrappers\TAO\TAO-INSTALL.html in a web browser and follow the instructions in section 
"On Windows NT and Windows 2000 and Windows XP". In short:

Open the file ...\ACE_wrappers\TAO\TAO_vs2019.sln with Visual Studio and build the solution.

Edit the Environment Variables and add the following directories to the Path variable:
```
C:\local\ACE_wrappers\lib
C:\local\ACE_wrappers\bin
C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\<VERSION>\bin\Hostx64\x64
```
The VERSION changes with every upgrade of Visual Studio (2022).

The last line is needed if you want to run the TAO IDL compiler (tao_idl in C:\local\ACE_wrappers\bin) from the command line.
If the last line is not present, running tao_idl will fail with the message that CL.exe is not found.

After installing TAO, set the TAO_INSTALLED flag to TRUE in the top-level CMakelists.txt file
if you want to build the TAO examples in [tao](./examples/tao).

## libevent

The build instructions can be found here: https://github.com/libevent/libevent/blob/master/Documentation/Building.md#building-on-windows

Clone or download the source code for libevent from https://github.com/libevent/libevent.
I placed libevent in folder C:\local, leading to C:\local\libevent.
```
cd C:\local\libevent
md build
cd build
cmake -G "Visual Studio 17" ..
cmake --build . --config Release
```
Visual Studio 17 corresponds to Visual Studio 2022.

Edit the Environment Variables and add the following directory to the Path variable:
```
C:\local\libevent\build\bin\Release
```
After installing libevent, set the LIBEVENT_INSTALLED flag to TRUE in the top-level CMakelists.txt file
if you want to build the libevent examples in examples/libevent.

## curl

Download the source code of the latest release of curl from https://curl.se/download.html.
Early September 2024, I downloaded curl-8.9.1.zip.

I placed curl-8.9.1.zip in folder C:\local and unzipped it, leading to C:\local\curl-8.9.1.

Build curl as follows:
```
cd C:\local\curl-8.9.1
md build
cd build
cmake -G "Visual Studio 17" ..
cmake --build . --config Release
```
Visual Studio 17 corresponds to Visual Studio 2022.

Edit the Environment Variables and add the following directory to the Path variable:
```
C:\local\curl-8.9.1\build\bin\Release
```
After installing curl, set the CURL_INSTALLED flag to TRUE in the top-level CMakelists.txt file 
if you want to build the curl examples in [curl](./examples/curl).

# Scripts

This directory contains 2 scripts (copy.bat for Windows and copy.sh for Linux)
that copy run.bat and run.sh scripts
and (if applicable) configuration files from their source directory to the corresponding build directory.

The paths to the build directory may have to be adapted to your local build set-up.
For example, for Linux builds, the assumption is that corolib is built from directory corolib-build,
which is a sibling of directory corolib, i.e.,
directories corolib (or corolib-master) and corolib-build are subdirectories of the same parent directory.

The 2 other scripts (hide-cs-projects.bat and unhide-cs-projects.bat) are for Windows with Visual Studio only.

hide-cs-projects.bat adds extension .hidden to the .sln and .csproj files of C# projects in ..\studies\control-flow-cs and its subfolders.
This prevents them from being visible in the C++ Build menu. These C# projects cannot be built from the top folder anyway, and
their presence removes the 'Build All', 'Rebuild All' and 'Clean All' options from the menu. Only 'Install corolib' is present
to build the corolib project.

unhide-cs-projects.bat restores the original names by removing the .hidden file name extension. The C# projects can then be built
with Visual Studio from ..\studies\control-flow-cs.

This directory contains 2 scripts (copy.bat for Windows and copy.sh for Linux)
that copy run.bat and run.sh scripts
and (if applicable) configuration files from their source directory to the corresponding build directory.

The paths to the build directory may have to be adapted to your local build set-up.
For example, for Linux builds, the assumption is that corolib is built from directory corolib-build,
which is a sibling of directory corolib, i.e.,
directories corolib (or corolib-master) and corolib-build are subdirectories of the same parent directory.

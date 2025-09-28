
@echo off

if exist ..\out\build\x64-Debug\examples\tutorial\ (
    echo copy ..\examples\tutorial\run.bat ..\out\build\x64-Debug\examples\tutorial\.
    copy ..\examples\tutorial\run.bat ..\out\build\x64-Debug\examples\tutorial\.
) else (
    echo ..\out\build\x64-Debug\examples\tutorial\ does not exist.
    echo Please build the project.
)

if exist ..\out\build\x64-Debug\examples\boost\various\ (
    echo copy ..\examples\boost\various\run.bat ..\out\build\x64-Debug\examples\boost\various\.
    copy ..\examples\boost\various\run.bat ..\out\build\x64-Debug\examples\boost\various\.
) else (
    echo ..\out\build\x64-Debug\examples\boost\various\ does not exist.
    echo "set(BOOST_INSTALLED TRUE)" in CMakeList.txt and rebuild.
)

if exist ..\out\build\x64-Debug\examples\cppcoro\ (
    echo copy ..\examples\cppcoro\examples-cc\run.bat ..\out\build\x64-Debug\examples\cppcoro\examples-cc\.
    copy ..\examples\cppcoro\examples-cc\run.bat ..\out\build\x64-Debug\examples\cppcoro\examples-cc\.
    echo copy ..\examples\cppcoro\examples-cl\run.bat ..\out\build\x64-Debug\examples\cppcoro\examples-cl\.
    copy ..\examples\cppcoro\examples-cl\run.bat ..\out\build\x64-Debug\examples\cppcoro\examples-cl\.
) else (
    echo ..\out\build\x64-Debug\examples\cppcoro\ does not exist.
    echo "set(CPPCORO_INSTALLED TRUE)" in CMakeList.txt and rebuild.
)

if exist ..\out\build\x64-Debug\examples\qt5\ (
    echo copying files to ..\out\build\x64-Debug\examples\qt5\ subdirectories.
    copy ..\examples\qt5\clientserver11\*.bat ..\out\build\x64-Debug\examples\qt5\clientserver11\.
    copy ..\examples\qt5\clientserver11\*.cfg ..\out\build\x64-Debug\examples\qt5\clientserver11\.
    copy ..\examples\qt5\various\run.bat ..\out\build\x64-Debug\examples\qt5\various\.
) else (
    echo ..\out\build\x64-Debug\examples\qt5\ does not exist.
    echo "set(QT5_INSTALLED TRUE)" in CMakeList.txt.
)

if exist ..\out\build\x64-Debug\examples\grpc\cpp\ (
    echo copying files to ..\out\build\x64-Debug\examples\grpc\cpp\ subdirectories.
    copy ..\examples\grpc\cpp\helloworld\run.bat ..\out\build\x64-Debug\examples\grpc\cpp\helloworld\.
    copy ..\examples\grpc\cpp\hellostreamingworld\run.bat ..\out\build\x64-Debug\examples\grpc\cpp\hellostreamingworld\.
    copy ..\examples\grpc\cpp\multiplex\run.bat ..\out\build\x64-Debug\examples\grpc\cpp\multiplex\.
    copy ..\examples\grpc\cpp\route_guide\route_guide_db.json ..\out\build\x64-Debug\examples\grpc\cpp\route_guide\.
    copy ..\examples\grpc\cpp\route_guide\run.bat ..\out\build\x64-Debug\examples\grpc\cpp\route_guide\.
) else (
    echo ..\out\build\x64-Debug\examples\grpc\ does not exist.
    echo "set(GRPC_INSTALLED TRUE)" in CMakeList.txt and rebuild.
)

if exist ..\out\build\x64-Debug\studies\corolab\ (
    echo copying files to ..\out\build\x64-Debug\studies\ subdirectories.
    copy ..\studies\cfsms\run.bat ..\out\build\x64-Debug\studies\cfsms\.
    copy ..\studies\corolab\run.bat ..\out\build\x64-Debug\studies\corolab\.
    copy ..\studies\final_suspend\run.bat ..\out\build\x64-Debug\studies\final_suspend\.
    copy ..\studies\initial_suspend\run.bat ..\out\build\x64-Debug\studies\initial_suspend\.
    copy ..\studies\transform\run.bat ..\out\build\x64-Debug\studies\transform\.
    copy ..\studies\why-coroutines\run.bat ..\out\build\x64-Debug\studies\why-coroutines\.
    copy ..\studies\why-coroutines2\run.bat ..\out\build\x64-Debug\studies\why-coroutines2\.
) else (
    echo ..\out\build\x64-Debug\studies\  does not exist.
    echo "set(BUILD_STUDIES TRUE)" in CMakeList.txt and rebuild.
)

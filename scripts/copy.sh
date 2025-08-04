#!/bin/sh

if [ -d "../../corolib-build/examples/tutorial/" ]; then
    echo "cp ../examples/tutorial/run.sh ../../corolib-build/examples/tutorial/."
    cp ../examples/tutorial/run.sh ../../corolib-build/examples/tutorial/.
    chmod ug+x ../../corolib-build/examples/tutorial/run.sh
else
    echo "../../corolib-build/examples/tutorial/ does not exist."
    echo "Please build the project."
fi

if [ -d "../../corolib-build/examples/boost/" ]; then
    echo "cp ../examples/boost/various/run.sh ../../corolib-build/examples/boost/various/."
    cp ../examples/boost/various/run.sh ../../corolib-build/examples/boost/various/.
    chmod ug+x ../../corolib-build/examples/boost/various/run.sh
else
    echo "../../corolib-build/examples/boost/ does not exist."
    echo "set(BOOST_INSTALLED TRUE) in CMakeList.txt and rebuild."
fi

if [ -d "../../corolib-build/examples/qt5/" ]; then
    echo "copying files to ../../corolib-build/examples/qt5/ subdirectories."
    cp ../examples/qt5/clientserver11/*.cfg ../../corolib-build/examples/qt5/clientserver11/.
    cp ../examples/qt5/clientserver11/*.sh ../../corolib-build/examples/qt5/clientserver11/.
    chmod ug+x ../../corolib-build/examples/qt5/clientserver11/*.sh
 
    cp ../examples/qt5/various/run.sh ../../corolib-build/examples/qt5/various/.
    chmod ug+x ../../corolib-build/examples/qt5/various/run.sh
else
    echo "../../corolib-build/examples/qt5/ does not exist."
    echo "set(QT5_INSTALLED TRUE) in CMakeList.txt and rebuild."
fi

if [ -d "../../corolib-build/examples/grpc/" ]; then
    echo "copying files to ../../corolib-build/examples/grpc/ subdirectories"
    cp ../examples/grpc/cpp/helloworld/run.sh ../../corolib-build/examples/grpc/cpp/helloworld/.
    chmod ug+x ../../corolib-build/examples/grpc/cpp/helloworld/run.sh

    cp ../examples/grpc/cpp/hellostreamingworld/run.sh ../../corolib-build/examples/grpc/cpp/hellostreamingworld/.
    chmod ug+x ../../corolib-build/examples/grpc/cpp/hellostreamingworld/run.sh

    cp ../examples/grpc/cpp/multiplex/run.sh ../../corolib-build/examples/grpc/cpp/multiplex/.
    chmod ug+x ../../corolib-build/examples/grpc/cpp/multiplex/run.sh

    cp ../examples/grpc/cpp/route_guide/route_guide_db.json ../../corolib-build/examples/grpc/cpp/route_guide/.
    cp ../examples/grpc/cpp/route_guide/run.sh ../../corolib-build/examples/grpc/cpp/route_guide/.
    chmod ug+x ../../corolib-build/examples/grpc/cpp/route_guide/run.sh
else
    echo "../../corolib-build/examples/grpc/ does not exist."
    echo "set(GRPC_INSTALLED TRUE) in CMakeList.txt and rebuild."
fi

if [ -d "../../corolib-build/studies/" ]; then
    echo "copying files to ../../corolib-build/studies/ subdirectories."

    cp ../studies/corolab/run.sh ../../corolib-build/studies/corolab
    chmod ug+x  ../../corolib-build/studies/corolab/run.sh

    cp ../studies/final_suspend/run.sh ../../corolib-build/studies/final_suspend/.
    chmod ug+x ../../corolib-build/studies/final_suspend/run.sh

    cp ../studies/initial_suspend/run.sh ../../corolib-build/studies/initial_suspend/.
    chmod ug+x ../../corolib-build/studies/initial_suspend/run.sh

    cp ../studies/transform/run.sh ../../corolib-build/studies/transform/.
    chmod ug+x ../../corolib-build/studies/transform/run.sh

    cp ../studies/why-coroutines/run.sh ../../corolib-build/studies/why-coroutines
    chmod ug+x ../../corolib-build/studies/why-coroutines/run.sh

    cp ../studies/why-coroutines2/run.sh ../../corolib-build/studies/why-coroutines2/.
    chmod ug+x ../../corolib-build/studies/why-coroutines2/run.sh
else
    echo "../../corolib-build/studies/ does not exist."
    echo "set(BUILD_STUDIES TRUE) in CMakeList.txt and rebuild."
fi

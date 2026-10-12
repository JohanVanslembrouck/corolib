TEMPLATE = subdirs

CONFIG += ordered
CONFIG += c++20

SUBDIRS += \
        lib \
        examples/qt5 \
        examples/tutorial \
        studies/rvo

examples/qt5.depends = lib
examples/tutorial.depends = lib

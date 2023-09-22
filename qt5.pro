TEMPLATE = subdirs

CONFIG += ordered

SUBDIRS += \
        lib \
        examples/qt5 \
        examples/tutorial

examples/qt5.depends = lib
examples/tutorial.depends = lib

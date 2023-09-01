TEMPLATE = subdirs

SUBDIRS += \
        lib \
        examples/qt5

examples/qt5.depends = lib

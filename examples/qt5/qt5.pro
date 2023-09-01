TEMPLATE = subdirs

CONFIG += ordered

SUBDIRS += \
        common \
        clientserver11 \
        clientserver12 \
        various

#clientserver11.depends = common
#clientserver12.depends = common
#various.depends = common

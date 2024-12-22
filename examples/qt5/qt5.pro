TEMPLATE = subdirs

SUBDIRS += \
        common \
        clientserver10 \
        clientserver11 \
        clientserver12 \
        various

clientserver10.depends = common
clientserver11.depends = common
clientserver12.depends = common
various.depends = common


ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

NAME=asm3_sensor_driven



#This has to be included last
include $(MKFILES_ROOT)/qtargets.mk

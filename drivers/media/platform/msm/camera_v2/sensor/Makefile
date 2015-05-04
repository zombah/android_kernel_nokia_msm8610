ccflags-y += -Idrivers/media/platform/msm/camera_v2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/msm_vb2
ccflags-y += -Idrivers/media/platform/msm/camera_v2/camera
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/io
ccflags-y += -Idrivers/media/platform/msm/camera_v2/sensor/cci
obj-$(CONFIG_MSMB_CAMERA) += cci/ io/ csiphy/ csid/ actuator/ flash/ eeprom/
obj-$(CONFIG_MSM_CAMERA_SENSOR) += msm_sensor.o
obj-$(CONFIG_S5K3L1YX) += s5k3l1yx.o
obj-$(CONFIG_IMX135) += imx135.o
obj-$(CONFIG_IMX134) += imx134.o
obj-$(CONFIG_OV8825) += ov8825.o
obj-$(CONFIG_OV8865) += ov8865.o
obj-$(CONFIG_s5k4e1) += s5k4e1.o
obj-$(CONFIG_OV12830) += ov12830.o
obj-$(CONFIG_OV2720) += ov2720.o
obj-$(CONFIG_OV9724) += ov9724.o
obj-$(CONFIG_HI256) += hi256.o
obj-$(CONFIG_OV5648) += ov5648.o
obj-$(CONFIG_MT9M114) += mt9m114.o
obj-$(CONFIG_SP1628) += sp1628.o
obj-$(CONFIG_GC0339) += gc0339.o

#because we all think back camera(smiapp) is the default first camera device,
#now it is probed dependency.The reason why the order in a Makefile matter is 
#how the current kernel build system works and how the linkers works. To make 
#a long story short, the build system takes all object files in obj-y and link 
#them together. It is highly environment dependent but there is high probability 
#that the linker place first object file in the obj-y in lower address, thus called earlier.  
#If you just want your driver to be called earlier than other drivers in the same directory, 
#this is simplest way to do it.
obj-$(CONFIG_SMIAPP) += smiapp.o
obj-$(CONFIG_SMIA65PP) += smia65pp.o
obj-$(CONFIG_OV7695) += ov7695.o
obj-$(CONFIG_HM03D5) += hm03d5.o

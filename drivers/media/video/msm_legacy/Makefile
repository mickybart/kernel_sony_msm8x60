GCC_VERSION      := $(shell $(CONFIG_SHELL) $(PWD)/scripts/gcc-version.sh $(CROSS_COMPILE)gcc)
EXTRA_CFLAGS += -Idrivers/media/video/msm_legacy/io
obj-$(CONFIG_MSM_CAMERA) += io/
ifeq ($(CONFIG_MSM_CAMERA_V4L2),y)
  EXTRA_CFLAGS += -Idrivers/media/video/msm_legacy/csi
  EXTRA_CFLAGS += -Idrivers/media/video/msm_legacy/io
  EXTRA_CFLAGS += -Idrivers/media/video/msm_legacy/eeprom
  EXTRA_CFLAGS += -Idrivers/media/video/msm_legacy/sensors
  EXTRA_CFLAGS += -Idrivers/media/video/msm_legacy/actuators
  obj-$(CONFIG_MSM_CAMERA) += msm_isp.o msm.o msm_mem.o msm_mctl.o msm_mctl_buf.o msm_mctl_pp.o
  obj-$(CONFIG_MSM_CAMERA) += io/ eeprom/ sensors/ actuators/ csi/
  obj-$(CONFIG_MSM_CAMERA) += msm_gesture.o
else
  obj-$(CONFIG_MSM_CAMERA) += msm_camera.o
endif
obj-$(CONFIG_MSM_CAMERA) += msm_axi_qos.o gemini/
obj-$(CONFIG_MSM_CAMERA_FLASH) += flash.o
obj-$(CONFIG_ARCH_MSM_ARM11) += msm_vfe7x.o
ifeq ($(CONFIG_MSM_CAMERA_V4L2),y)
  obj-$(CONFIG_ARCH_MSM7X27A) += msm_vfe7x27a_v4l2.o
else
  obj-$(CONFIG_ARCH_MSM7X27A) += msm_vfe7x27a.o
endif
obj-$(CONFIG_ARCH_QSD8X50) += msm_vfe8x.o msm_vfe8x_proc.o
ifeq ($(CONFIG_MSM_CAMERA_V4L2),y)
  obj-$(CONFIG_ARCH_MSM8X60) += msm_vfe31_v4l2.o msm_vpe.o
  obj-$(CONFIG_ARCH_MSM7X30) += msm_vfe31_v4l2.o msm_vpe.o msm_axi_qos.o
else
  obj-$(CONFIG_ARCH_MSM8X60) += msm_vfe31.o msm_vpe1.o
  obj-$(CONFIG_ARCH_MSM7X30) += msm_vfe31.o msm_vpe1.o
endif
obj-$(CONFIG_ARCH_MSM8960) += msm_vfe32.o msm_vpe.o
obj-$(CONFIG_MSM_V4L2_VIDEO_OVERLAY_DEVICE) += msm_v4l2_video.o
obj-$(CONFIG_SEMC_VPE) += semc_vpe.o

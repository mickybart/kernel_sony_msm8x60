ifeq ($(BOARD_KERNEL_MSM),true)

LOCAL_PATH := $(call my-dir)

##########
# Kernel #
##########

include $(CLEAR_VARS)

KERNEL_DIR := $(ANDROID_BUILD_TOP)/kernel/sony/msm8x60
KERNEL_TOOLS_PREFIX := $(ANDROID_BUILD_TOP)/prebuilts/gcc/$(HOST_PREBUILT_TAG)/arm/arm-eabi-4.7/bin/arm-eabi-
-include $(KERNEL_DIR)/AndroidKernel.mk

LOCAL_PREBUILT_MODULE_FILE := $(TARGET_PREBUILT_KERNEL)

LOCAL_MODULE       := kernel
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := KERNEL_OBJ
LOCAL_MODULE_PATH  := $(PRODUCT_OUT)
include $(BUILD_PREBUILT)

########################
# Multi kernel support #
########################
ifeq ($(TARGET_NO_MULTIKERNEL),false)

###################
# Kernel OC Ultra #
###################
ifeq ($(BOARD_KERNEL_MSM_OC_ULTRA),true)

include $(CLEAR_VARS)

KERNEL_DIR := $(ANDROID_BUILD_TOP)/kernel/sony/msm8x60
KERNEL_TOOLS_PREFIX := $(ANDROID_BUILD_TOP)/prebuilts/gcc/$(HOST_PREBUILT_TAG)/arm/arm-eabi-4.7/bin/arm-eabi-
-include $(KERNEL_DIR)/AndroidKernel.mk

LOCAL_PREBUILT_MODULE_FILE := $(TARGET_PREBUILT_KERNEL_OC_ULTRA)

LOCAL_MODULE       := kernel-oc_ultra
LOCAL_MODULE_TAGS  := optional
LOCAL_MODULE_CLASS := KERNEL_OC_ULTRA_OBJ
LOCAL_MODULE_PATH  := $(PRODUCT_OUT)
include $(BUILD_PREBUILT)

endif #BOARD_KERNEL_MSM_OC_ULTRA
endif #TARGET_NO_MULTIKERNEL

endif #BOARD_KERNEL_MSM


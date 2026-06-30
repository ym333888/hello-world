LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_SRC_FILES:= agn_spi_test_app.c

LOCAL_MODULE:= agn_spi_test_app

LOCAL_MODULE_TAGS := optional

LOCAL_SHARED_LIBRARIES := libcutils libc libutils liblog libdl libext4_utils

include $(BUILD_EXECUTABLE)


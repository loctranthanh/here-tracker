#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#


PROJECT_NAME := tracker-here
CURRENT_PATH := $(shell pwd)
IDF_PATH := $(CURRENT_PATH)/esp-idf
# EXTRA_COMPONENT_DIRS := $(CURRENT_PATH)/components
include $(IDF_PATH)/make/project.mk

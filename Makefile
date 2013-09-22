# Target specific macros
TARGET = nxttemplate
TARGET_CPP_SOURCES = nxttemplate.cpp
TOPPERS_OSEK_OIL_SOURCE = ./nxttemplate.oil

O_PATH ?= build

include ../../../ecrobot/ecrobot++.mak

# Target specific macros
TARGET = nxtdemo
TARGET_CPP_SOURCES = nxtdemo.cpp
TOPPERS_OSEK_OIL_SOURCE = ./nxtdemo.oil

O_PATH ?= build

include ../ecrobot/ecrobot++.mak

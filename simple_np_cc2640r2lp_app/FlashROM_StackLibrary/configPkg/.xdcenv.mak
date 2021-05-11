#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = D:/ProgramFiles/ti/ccsv7/ccs_base;D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source;D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/kernel/tirtos/packages;D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack
override XDCROOT = D:/ProgramFiles/ti/xdctools_3_50_03_33_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = D:/ProgramFiles/ti/ccsv7/ccs_base;D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source;D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/kernel/tirtos/packages;D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack;D:/ProgramFiles/ti/xdctools_3_50_03_33_core/packages;..
HOSTOS = Windows
endif

TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

DBD += ETH32AsynPortDriver.dbd

LIBRARY_IOC += ETH32AsynPortDriverSupport
ETH32AsynPortDriverSupport_SRCS += ETH32AsynPortDriver.cpp
ETH32AsynPortDriverSupport_LIBS += asyn 
ETH32AsynPortDriverSupport_LIBS += $(EPICS_BASE_IOC_LIBS)



#=============================

PROD_IOC += ETH32AsynPortDriver

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
ETH32AsynPortDriver_SRCS_DEFAULT += ETH32AsynPortDriver_registerRecordDeviceDriver.cpp ETH32AsynPortDriverMain.cpp
ETH32AsynPortDriverVx_SRCS_vxWorks  += ETH32AsynPortDriver_registerRecordDeviceDriver.cpp
ETH32AsynPortDriver_LIBS += ETH32AsynPortDriverSupport asyn 
ETH32AsynPortDriver_LIBS += $(EPICS_BASE_IOC_LIBS)

ETH32AsynPortDriver_LIBS += eth32
eth32_DIR = /usr/lib/

#LIBS += eth32

ETH32AsynPortDriver_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

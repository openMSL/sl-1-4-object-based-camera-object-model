#******************************************************************************
#**  CarMaker - Version 5.1beta2
#**  Vehicle Dynamic Simulation Toolkit
#**
#**  Copyright (C)   IPG Automotive GmbH
#**                  Bannwaldallee 60             Phone  +49.721.98520.0
#**                  76185 Karlsruhe              Fax    +49.721.98520.99
#**                  Germany                      WWW    http://www.ipg.de
#******************************************************************************
#**
#**  Application Makefile
#**  Run with 'make V=1 ...' for a verbose build.
#**
#******************************************************************************

CMVersion = 7.0.3

# For a 64-bit application include MakeDefs64 instead.
CMPathMakeDefs = $(CMDir)/include/MakeDefs

$(info OSTYPE='$(OSTYPE)')
ifeq ($(OSTYPE),linux)
    CMDir = /fs/opt/ipg/hil/linux-$(CMVersion)
else
    CMDir = D:/ipg/hil/win32-$(CMVersion)
	ifeq ($(wildcard $(CMDir)),)
	    CMDir := $(subst C:,D:,$(CMDir))
	endif
endif

$(info Including '$(CMPathMakeDefs)')
include $(CMPathMakeDefs)


#APP_VER =		"Car_Generic <insert.your.version.no>"
#APP_NAME =		CarMaker.$(ARCH)$(EXE_EXT)

#OPT_CFLAGS =		-g -O1 -std=c++11
OPT_CFLAGS =		-std=c++11 


#LDFLAGS += -L../src_OSI_SharedProto
LDFLAGS += -L../lib/$(ARCH)

### PROTO Kommunikation
#PROTO_FILES += osi_common.proto osi_environment.proto osi_landmark.proto osi_lane.proto osi_object.proto #osi_occupant.proto osi_version.proto osi_groundtruth.proto osi_sensorspecific.proto
#PROTO_FILES += osi_detectedlandmark.proto osi_detectedlane.proto osi_detectedobject.proto osi_sensordata.proto #osi_modelinternal.proto osi_detectedoccupant.proto osi_lowleveldata.proto
#OBJS_PROTO	:=	$(PROTO_FILES:.proto=.pb.o)
#SRC_PROTO	:=	$(PROTO_FILES:.proto=.pb.cc) $(PROTO_FILES:.proto=.pb.h)
#OBJS	   =	$(OBJS_PROTO) 
#LDFLAGS	   +=	`pkg-config --cflags --lprotobuf`


#LD_LIBS =		$(CAR_LIB) \
#			$(CARMAKER_LIB) $(DRIVER_LIB) $(ROAD_LIB) $(TIRE_LIB) $(TAME_LIB)
LD_LIBS_OS += 	-lstdc++
#OBJS =			CM_Main.o CM_Vehicle.o User.o
#OBJS_xeno =		IO.o $(OBJS_CANIOGEN) $(OBJS_CANIOGEN_USER)
#OBJS_linux =
#OBJS_linux64 =
#OBJS_win32 =
#OBJS_win64 =


# Prepend local include/library directory to include path:
# PREINC_CFLAGS +=		-I../include -I../lib/$(ARCH) -I../lib

# Append local include/library directory to include path:
# INC_CFLAGS +=		-I../include -I../lib/$(ARCH) -I../lib -I../src_OSI_SharedProto
INC_CFLAGS +=		-I../include -I../lib/$(ARCH) -I../lib -I../include/osi_proto_h
INC_CFLAGS +=       -I../open-simulation-interface/build
INC_CFLAGS +=       -I../../protobuf-2.6.1/vsprojects/

#INC_CFLAGS +=	-I. -I../include -I../lib/$(ARCH)

# Append local include/library directory to include path:
#INC_CFLAGS +=	 -I/home/stw/protobuf_installation/include/


CFLAGS +=	-DFMI_COSIMULATION
#LDFLAGS +=	-lwinmm
OBJS +=		OSMPCameraSensor.o


### Linking with RTW-built Simulink models

#MATSUPP_MATVER =	R2010a
#LD_LIBS +=		$(MATSUPP_LIB)

#OBJS += libSimuModel_$(ARCH).a

# @@PLUGIN-BEGIN-LIBS@@ - Automatically generated code - don't edit!
# @@PLUGIN-END@@

### END (Linking with RTW-built Simulink models)


### FlexRay
#DEF_CFLAGS +=		-DWITH_FLEXRAY

### SOME/IP
#DEF_CFLAGS +=		-DWITH_SOMEIP

### XCP / CCP
#DEF_CFLAGS +=		-DWITH_XCP
#DEF_CFLAGS +=		-DWITH_CCP

### EtherCAT
#DEF_CFLAGS +=		-DWITH_ECAT

### I/O generator for CANdb files
#
# <---
#OBJS_CANIOGEN :=	IO_CAN.o
#OBJS_CANIOGEN_USER :=	$(OBJS_CANIOGEN:.o=_User.o)
#DEF_CFLAGS +=		-DWITH_IO_CAN
# <---
# I/O Module IO_CAN.o
#IO_CAN_DB =	../CANdb/CANiogenGeneric.dbc
#IO_CAN_FLAGS =	-srvECU ESP
# <---
#
### END (I/O generator for CANdb files)


#default:	$(APP_NAME)
#$(APP_NAME):	$(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS_MK) app_tmp.o 
#	$(QECHO) " LD     $@"
#	$Q $(CC) $(CFLAGS) $(LDFLAGS) -o $@ \
#		$(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS) $(LD_LIBS_OS) \
#		app_tmp.o
#	$(SET_EXE_PERMISSIONS) $@
	

#$(SRC_PROTO):	$(PROTO_FILES)
#	pkg-config --cflags protobuf  # fails if protobuf is not installed
#	../lib/$(ARCH)/protoc	--cpp_out=./ $(PROTO_FILES) 	
	


ifeq ($(OSTYPE),linux)
    FMU_EXT =		so
    LIBS_OSI = -lOSI_SharedProto -lprotobuf
else
    FMU_EXT =		dll
    LIBS_OSI = -lOSI_SharedProto -lprotobuf-13
endif




TARGET  = OSMPCameraSensor
default: $(TARGET).$(FMU_EXT)
$(TARGET).$(FMU_EXT):	OSMPCameraSensor.cpp
	$(QECHO) " CCC   $@"
	$Q $(CCC)  $(CFLAGS) $(INC_CFLAGS) -shared $(LDFLAGS) $(LD_LIBS) $(LIBS_OSI) -o $(TARGET).$(FMU_EXT) OSMPCameraSensor.cpp



#%.$(FMU_EXT):	$(SRC_PROTO)	%.cpp
#	$(QECHO) " CCC   $@"
#	$Q $(CCC) $(CFLAGS) -shared -o $@ $<



install: $(APP_NAME)
	$(INSTALL_APP) $(APP_NAME) $(ARCH)

clean:
	-rm -f 	*~ *% *.o *.so *.fmu  *.pb.cc *.pb.h core

app_tmp.c: Makefile $(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS_MK)
	$(QECHO) " MK     $@"
	$Q $(CREATE_INFO_CMD)

depend .depend: Makefile
	$(QECHO) " MK     $@"
#	$Q-$(CC)  $(CFLAGS)  $(DEPCFLAGS)  *.c > .depend 2>/dev/null
	$Q-$(CCC) $(CCFLAGS) $(DEPCCFLAGS) *.cc >> .depend 2>/dev/null
include .depend

%.depend_caniogen:	Makefile
	$(QECHO) " MK     $@"
	$Q $(CANIOGEN) -mkrules "$*.o" > $@
ifneq ($(OBJS_CANIOGEN),)
include $(OBJS_CANIOGEN:.o=.depend_caniogen)
endif

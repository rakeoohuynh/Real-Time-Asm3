#Build architecture/variant string, possible values: x86, armv7le, etc...
PLATFORM ?= x86_64

#Build profile, possible values: release, debug, profile, coverage
BUILD_PROFILE ?= debug

CONFIG_NAME ?= $(PLATFORM)-$(BUILD_PROFILE)
OUTPUT_DIR = build/$(CONFIG_NAME)

#Compiler definitions
CC = qcc -Vgcc_nto$(PLATFORM)
CXX = q++ -Vgcc_nto$(PLATFORM)_cxx
LD = $(CC)

#User defined include/preprocessor flags and libraries
#INCLUDES += -I/path/to/my/lib/include
#LIBS += -L/path/to/my/lib/$(PLATFORM)/usr/lib -lmylib

#Compiler flags for build profiles
CCFLAGS_release += -O2
CCFLAGS_debug += -g -O0 -fno-builtin
CCFLAGS_coverage += -g -O0 -ftest-coverage -fprofile-arcs
LDFLAGS_coverage += -ftest-coverage -fprofile-arcs
CCFLAGS_profile += -g -O0 -finstrument-functions
LIBS_profile += -lprofilingS

#Generic compiler flags (which include build type flags)
CCFLAGS_all += -Wall -fmessage-length=0
CCFLAGS_all += $(CCFLAGS_$(BUILD_PROFILE))
LDFLAGS_all += $(LDFLAGS_$(BUILD_PROFILE))
LIBS_all += $(LIBS_$(BUILD_PROFILE)) -lsocket
DEPS = -Wp,-MMD,$(@:%.o=%.d),-MT,$@

#---------------------------------------------------------------------
# TWO separate artifacts: LC and CC are separate programs (each has its
# own main()), so each gets its own object file and its own link step
# instead of both being linked together into one executable.
#---------------------------------------------------------------------
ARTIFACT_CC = central_controller
ARTIFACT_LC = local_controller

TARGET_CC = $(OUTPUT_DIR)/$(ARTIFACT_CC)
TARGET_LC = $(OUTPUT_DIR)/$(ARTIFACT_LC)

OBJ_CC = $(OUTPUT_DIR)/src/central_controller.o
OBJ_LC = $(OUTPUT_DIR)/src/local_controller.o

#Compiling rule
$(OUTPUT_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) -c $(DEPS) -o $@ $(INCLUDES) $(CCFLAGS_all) $(CCFLAGS) $

#Linking rules -- one per artifact
$(TARGET_CC): $(OBJ_CC)
	$(LD) -o $@ $(LDFLAGS_all) $(LDFLAGS) $^ $(LIBS_all) $(LIBS)

$(TARGET_LC): $(OBJ_LC)
	$(LD) -o $@ $(LDFLAGS_all) $(LDFLAGS) $^ $(LIBS_all) $(LIBS)

#Rules section for default compilation and linking
all: $(TARGET_CC) $(TARGET_LC)

clean:
	rm -fr $(OUTPUT_DIR)

rebuild: clean all

#Inclusion of dependencies (object files to source and includes)
-include $(OBJ_CC:%.o=%.d) $(OBJ_LC:%.o=%.d)

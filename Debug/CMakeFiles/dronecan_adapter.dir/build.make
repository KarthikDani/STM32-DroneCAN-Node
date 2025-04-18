# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/karthik/STM32CubeIDE/MySTM32Workpace/STM32H753-DroneCAN-Node

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/karthik/STM32CubeIDE/MySTM32Workpace/STM32H753-DroneCAN-Node/Debug

# Include any dependencies generated for this target.
include CMakeFiles/dronecan_adapter.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/dronecan_adapter.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/dronecan_adapter.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dronecan_adapter.dir/flags.make

CMakeFiles/dronecan_adapter.dir/Sources/dronecan_adapter.c.obj: CMakeFiles/dronecan_adapter.dir/flags.make
CMakeFiles/dronecan_adapter.dir/Sources/dronecan_adapter.c.obj: /home/karthik/STM32CubeIDE/MySTM32Workpace/STM32H753-DroneCAN-Node/Sources/dronecan_adapter.c
CMakeFiles/dronecan_adapter.dir/Sources/dronecan_adapter.c.obj: CMakeFiles/dronecan_adapter.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/karthik/STM32CubeIDE/MySTM32Workpace/STM32H753-DroneCAN-Node/Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/dronecan_adapter.dir/Sources/dronecan_adapter.c.obj"
	/opt/st/stm32cubeide_1.17.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.linux64_1.1.0.202410170702/tools/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/dronecan_adapter.dir/Sources/dronecan_adapter.c.obj -MF CMakeFiles/dronecan_adapter.dir/Sources/dronecan_adapter.c.obj.d -o CMakeFiles/dronecan_adapter.dir/Sources/dronecan_adapter.c.obj -c /home/karthik/STM32CubeIDE/MySTM32Workpace/STM32H753-DroneCAN-Node/Sources/dronecan_adapter.c

CMakeFiles/dronecan_adapter.dir/Sources/dronecan_adapter.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/dronecan_adapter.dir/Sources/dronecan_adapter.c.i"
	/opt/st/stm32cubeide_1.17.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.linux64_1.1.0.202410170702/tools/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/karthik/STM32CubeIDE/MySTM32Workpace/STM32H753-DroneCAN-Node/Sources/dronecan_adapter.c > CMakeFiles/dronecan_adapter.dir/Sources/dronecan_adapter.c.i

CMakeFiles/dronecan_adapter.dir/Sources/dronecan_adapter.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/dronecan_adapter.dir/Sources/dronecan_adapter.c.s"
	/opt/st/stm32cubeide_1.17.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.12.3.rel1.linux64_1.1.0.202410170702/tools/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/karthik/STM32CubeIDE/MySTM32Workpace/STM32H753-DroneCAN-Node/Sources/dronecan_adapter.c -o CMakeFiles/dronecan_adapter.dir/Sources/dronecan_adapter.c.s

# Object files for target dronecan_adapter
dronecan_adapter_OBJECTS = \
"CMakeFiles/dronecan_adapter.dir/Sources/dronecan_adapter.c.obj"

# External object files for target dronecan_adapter
dronecan_adapter_EXTERNAL_OBJECTS =

libdronecan_adapter.a: CMakeFiles/dronecan_adapter.dir/Sources/dronecan_adapter.c.obj
libdronecan_adapter.a: CMakeFiles/dronecan_adapter.dir/build.make
libdronecan_adapter.a: CMakeFiles/dronecan_adapter.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/karthik/STM32CubeIDE/MySTM32Workpace/STM32H753-DroneCAN-Node/Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libdronecan_adapter.a"
	$(CMAKE_COMMAND) -P CMakeFiles/dronecan_adapter.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dronecan_adapter.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/dronecan_adapter.dir/build: libdronecan_adapter.a
.PHONY : CMakeFiles/dronecan_adapter.dir/build

CMakeFiles/dronecan_adapter.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dronecan_adapter.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dronecan_adapter.dir/clean

CMakeFiles/dronecan_adapter.dir/depend:
	cd /home/karthik/STM32CubeIDE/MySTM32Workpace/STM32H753-DroneCAN-Node/Debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/karthik/STM32CubeIDE/MySTM32Workpace/STM32H753-DroneCAN-Node /home/karthik/STM32CubeIDE/MySTM32Workpace/STM32H753-DroneCAN-Node /home/karthik/STM32CubeIDE/MySTM32Workpace/STM32H753-DroneCAN-Node/Debug /home/karthik/STM32CubeIDE/MySTM32Workpace/STM32H753-DroneCAN-Node/Debug /home/karthik/STM32CubeIDE/MySTM32Workpace/STM32H753-DroneCAN-Node/Debug/CMakeFiles/dronecan_adapter.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/dronecan_adapter.dir/depend


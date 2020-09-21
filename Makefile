include ./proto/nanopb/extra/nanopb.mk

# CMAKE FLAGS
# VERBOSE_MAKEFILE=on to allow verbose mode for cmake makefiles 
ifeq ($(VERBOSE_MAKEFILE),on)
	CMAKE_FLAGS:=-v
endif

NPROCS:=$(shell grep -c ^processor /proc/cpuinfo)

.DEFAULT_GOAL := jenkins


# COMMON    ###################################################################################################################################
.PHONY: jenkins
jenkins:
	make -C highlevel jenkins

proto/gen/cpp/proto/outech.pb.c: proto/outech.proto
	$(PROTOC) $(PROTOC_OPTS) --nanopb_out=./proto/gen/cpp proto/outech.proto -I=./proto

proto/gen/python/outech_pb2.py: proto/outech.proto
	$(PROTOC) -I=./proto --python_out=./proto/gen/python proto/outech.proto

.PHONY: protoc
protoc: proto/gen/cpp/proto/outech.pb.c proto/gen/python/outech_pb2.py

idl/gen/python: idl/outech.fbs
	@which flatc >/dev/null || (echo '!!! Flatbuffers not installed, run `pacman -S flatbuffers` !!!'; exit 1)
	flatc --python idl/outech.fbs
	rm idl/__init__.py
	rm idl/gen/__init__.py

idl/gen/cpp: idl/outech.fbs
	@which flatc >/dev/null || (echo '!!! Flatbuffers not installed, run `pacman -S flatbuffers` !!!'; exit 1)
	flatc -o idl/gen/cpp --cpp idl/outech.fbs

.PHONY: idl
idl: idl/gen/python idl/gen/cpp

.PHONY: clean
clean:
	make -C highlevel clean
	rm -rf lowlevel/cmake-build-debug

.PHONY: candump
candump:
	@pipenv run python -m tool.script.read_bus $(dev) $(id_rx) $(id_tx)

# HIGH LEVEL ###################################################################################################################################


# LOW LEVEL ###################################################################################################################################
ll_cmake_setup:
	cmake -B lowlevel/cmake-build-debug lowlevel

.PHONY: ll_build_all
ll_build_all: ll_cmake_setup
	cmake --build lowlevel/cmake-build-debug --target build_motor build_motor_g4 build_servo build_servo_nucleo -j$(NPROCS) $(CMAKE_FLAGS)

.PHONY: ll_flash
ll_flash:
ifndef BOARD_NAME
	$(error BOARD_NAME is undefined. Should be either motor, motor_g4, servo or servo_nucleo.)
endif
ifeq ($(BOARD_NAME), $(filter $(BOARD_NAME),servo servo_nucleo sensor))
ifndef BOARD_ID
	$(error BOARD_ID is undefined. BOARD_NAME servo/servo_nucleo/sensor require it.)
endif
	@echo Trying to upload to board $(BOARD_NAME) with id $(BOARD_ID)...
	cmake -B lowlevel/cmake-build-debug lowlevel -DBOARD_ID=$(BOARD_ID)
	cmake --build lowlevel/cmake-build-debug --target flash_$(BOARD_NAME) $(CMAKE_FLAGS)
	@echo Upload done.
else
	@echo Trying to upload to board $(BOARD_NAME)...
	cmake --build lowlevel/cmake-build-debug --target flash_$(BOARD_NAME) $(CMAKE_FLAGS)
	@echo Upload done.
endif

.PHONY: ll_test
ll_test:
	@echo -e "\nBuilding tests..."
	cmake --build lowlevel/cmake-build-debug --target TestRunner
	@echo -e "\nStarting TestRunner..."
	lowlevel/cmake-build-debug/tests/TestRunner -c

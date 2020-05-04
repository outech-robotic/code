include ./proto/nanopb/extra/nanopb.mk

# CMAKE FLAGS
# VERBOSE_MAKEFILE=on to allow verbose mode for cmake makefiles 
ifeq ($(VERBOSE_MAKEFILE),on)
	CMAKE_FLAGS:=-v
endif

NPROCS:=$(shell grep -c ^processor /proc/cpuinfo)

.DEFAULT_GOAL := jenkins

.PHONY: jenkins
jenkins:
	make -C highlevel jenkins

proto/gen/cpp/proto/outech.pb.c: proto/outech.proto
	$(PROTOC) $(PROTOC_OPTS) --nanopb_out=./proto/gen/cpp proto/outech.proto -I=./proto

proto/gen/python/outech_pb2.py: proto/outech.proto
	$(PROTOC) -I=./proto --python_out=./proto/gen/python proto/outech.proto

.PHONY: protoc
protoc: proto/gen/cpp/proto/outech.pb.c proto/gen/python/outech_pb2.py

.PHONY: clean
clean:
	make -C highlevel clean
	cmake --build lowlevel/cmake-build-debug --target clean

.PHONY: ll_build_all
ll_build_all:
	cmake --build lowlevel/cmake-build-debug --target build_motor build_motor_g4 build_servo build_servo_nucleo -j$(NPROCS) $(CMAKE_FLAGS)

.PHONY: ll_flash
ll_flash:
ifndef BOARD_NAME
	$(error BOARD_NAME is undefined. Should be either motor, motor_g4, servo or servo_nucleo.)
endif
	@echo Trying to upload to board $(BOARD_NAME)...
	cmake --build lowlevel/cmake-build-debug --target flash_$(BOARD_NAME) $(CMAKE_FLAGS)
	@echo Upload done.

.PHONY: ll_test
ll_test:
	cmake --build lowlevel/cmake-build-debug --target TestRunner
	lowlevel/cmake-build-debug/tests/TestRunner -c 


.PHONY: candump
candump:
	@pipenv run python -m tool.script.read_bus $(port)


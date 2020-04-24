include ./proto/nanopb/extra/nanopb.mk

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

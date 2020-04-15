include ./proto/nanopb/extra/nanopb.mk

.DEFAULT_GOAL := jenkins

.PHONY: test
test:
	pytest highlevel -vv

.PHONY: lint
lint:
	pylint highlevel

.PHONY: format
format:
	yapf -e google -i -p -r highlevel

.PHONY: mypy
mypy:
	mypy --disallow-untyped-calls --ignore-missing-imports --disallow-incomplete-defs highlevel

.PHONY: jenkins
jenkins: test lint mypy format


.PHONY: clean
clean: 
	find . -type f -name "outech.log" -exec rm -f {} +
	find . -type d -name "__pycache__" -exec rm -rf {} +
	find . -type d -name ".pytest_cache" -exec rm -rf {} +
	find . -type d -name ".mypy_cache" -exec rm -rf {} +

proto/gen/cpp/proto/outech.pb.c: proto/outech.proto
	$(PROTOC) $(PROTOC_OPTS) --nanopb_out=./proto/gen/cpp proto/outech.proto

proto/gen/python/outech_pb2.py: proto/outech.proto
	$(PROTOC) -I=./proto --python_out=./proto/gen/python proto/outech.proto

.PHONY: protoc
protoc: proto/gen/cpp/proto/outech.pb.c proto/gen/python/outech_pb2.py

.PHONY: run-simulation
run-simulation:
	OUTECH_SIMULATION=true python -m highlevel.main

.PHONY: run
run:
	OUTECH_SIMULATION=false python -m highlevel.main

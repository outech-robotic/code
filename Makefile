.DEFAULT_GOAL := jenkins

.PHONY: test
test:
	pytest src -vv

.PHONY: lint
lint:
	pylint src

.PHONY: format
format:
	yapf -i -p -r src

.PHONY: mypy
mypy:
	mypy --disallow-untyped-calls --ignore-missing-imports --disallow-incomplete-defs src

.PHONY: jenkins
jenkins: test lint mypy format


.PHONY: clean
clean: 
	find . -type f -name "outech.log" -exec rm -f {} +
	find . -type d -name "__pycache__" -exec rm -rf {} +
	find . -type d -name ".pytest_cache" -exec rm -rf {} +
	find . -type d -name ".mypy_cache" -exec rm -rf {} +

.PHONY: protoc
protoc: 
	protoc -I=./proto --python_out=./proto/gen outech.proto

.PHONY: run-simulation
run-simulation:
	OUTECH_SIMULATION=true python -m src.main

.PHONY: run
run:
	OUTECH_SIMULATION=false python -m src.main

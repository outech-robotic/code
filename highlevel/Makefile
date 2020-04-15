.DEFAULT_GOAL := jenkins

.PHONY: test
test:
	pytest . -vv

.PHONY: lint
lint:
	pylint .

.PHONY: format
format:
	yapf -e google -i -p -r .

.PHONY: mypy
mypy:
	mypy --disallow-untyped-calls --ignore-missing-imports --disallow-incomplete-defs ../highlevel/

.PHONY: jenkins
jenkins: test lint mypy format


.PHONY: clean
clean: 
	find . -type f -name "outech.log" -exec rm -f {} +
	find . -type d -name "__pycache__" -exec rm -rf {} +
	find . -type d -name ".pytest_cache" -exec rm -rf {} +
	find . -type d -name ".mypy_cache" -exec rm -rf {} +

.PHONY: run-simulation
run-simulation:
	OUTECH_SIMULATION=true python -m highlevel.main

.PHONY: run
run:
	OUTECH_SIMULATION=false python -m highlevel.main

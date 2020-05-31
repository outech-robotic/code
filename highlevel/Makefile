.DEFAULT_GOAL := jenkins

.PHONY: test
test:
	pytest . -vv

.PHONY: lint
lint:
	pylint highlevel

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
	PYTHONPATH=${CURDIR}/.. OUTECH_SIMULATION=true python -m highlevel.main

.PHONY: run
run:
	PYTHONPATH=${CURDIR}/.. OUTECH_SIMULATION=false python -m highlevel.main

.PHONY: run-lidarless
run-stub:
	PYTHONPATH=${CURDIR}/.. STUB_LIDAR=true OUTECH_SIMULATION=false python -m highlevel.main


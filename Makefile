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


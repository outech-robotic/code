.PHONY: test
test:
	pytest src

.PHONY: lint
lint:
	pylint src

.PHONY: format
format:
	yapf -i -r src

.PHONY: mypy
mypy:
	mypy --disallow-untyped-calls --ignore-missing-imports --disallow-incomplete-defs src

.PHONY: jenkins
jenkins: test lint mypy format

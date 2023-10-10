.DEFAULT_GOAL := all
isort = python -m isort .
black = python -m black --target-version py39 .

.PHONY: format
format:
	$(isort)
	$(black)

.PHONY: lint
lint:
	$(black) --check --diff
	python -m flake8 .
	python -m pydocstyle . --count



.PHONY: mypy
mypy:
	mypy --config-file setup.cfg --package .
	mypy --config-file setup.cfg .

.PHONY: all
all: format lint

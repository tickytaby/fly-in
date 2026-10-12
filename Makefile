VENV    := .venv
PYTHON  := $(VENV)/bin/python
MAP     ?= maps/easy/01_linear_path.txt
ARGS    ?=

MYPY_FLAGS := --warn-return-any --warn-unused-ignores --ignore-missing-imports \
              --disallow-untyped-defs --check-untyped-defs

.PHONY: install run debug clean lint lint-strict test

install:
	@if command -v uv >/dev/null 2>&1; then \
		uv sync; \
	else \
		python3 -m venv $(VENV) && \
		$(VENV)/bin/pip install pydantic pytest flake8 mypy; \
	fi

run:
	$(PYTHON) src/main.py $(MAP) $(ARGS)

debug:
	$(PYTHON) -m pdb src/main.py $(MAP) $(ARGS)

clean:
	find . -path ./$(VENV) -prune -o -type d -name __pycache__ -exec rm -rf {} +
	rm -rf .mypy_cache .pytest_cache

lint:
	$(PYTHON) -m flake8 .
	$(PYTHON) -m mypy . $(MYPY_FLAGS)

lint-strict:
	$(PYTHON) -m flake8 .
	$(PYTHON) -m mypy . --strict

test:
	$(PYTHON) -m pytest

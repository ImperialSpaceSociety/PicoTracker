PYTHON ?= python3
SAMPLE_CAPTURE := tools/with_pips_data.txt
CAPTURE ?= $(SAMPLE_CAPTURE)
DECODE_ARGS ?=
SIM_ARGS ?=
DOCKER ?= docker
DEV_IMAGE ?= picotracker-dev
RUFF ?= ruff
CPPCHECK ?= cppcheck
PYTHON_PATHS := tools tests

.DEFAULT_GOAL := help

.PHONY: help demo simulate test stm8 decode quality format check clean container-build container-check c-quality python-quality whitespace

help:
	@printf '%s\n' \
	  'PicoTracker developer commands' \
	  '' \
	  '  make demo    Decode the included flight capture (no C compiler or hardware)' \
	  '  make simulate Run the hardware-independent tracker simulator' \
	  '  make test    Run the complete host regression suite' \
	  '  make decode  Decode CAPTURE (default: tools/with_pips_data.txt)' \
	  '  make stm8    Compile/link the structural STM8 check (requires SDCC)' \
	  '  make quality Run C/Python static analysis and whitespace checks' \
	  '  make format  Apply Ruff Python fixes and formatting' \
	  '  make check   Run quality, tests, decode, simulator, and STM8 verification' \
	  '  make clean   Remove generated host-test output' \
	  '  make container-build  Build the reproducible developer image' \
	  '  make container-check  Run all checks inside that image' \
	  '' \
	  'Examples:' \
	  '  make decode CAPTURE=path/to/capture.txt' \
	  '  make decode DECODE_ARGS=--print-frames'

demo:
	@echo 'PicoTracker demo: decoding the included flight capture'
	@$(PYTHON) tools/decode_data.py $(SAMPLE_CAPTURE)
	@echo 'Demo complete. Run `make test` next for the host regression suite.'

simulate:
	$(PYTHON) tools/simulate_tracker.py $(SIM_ARGS)

test:
	$(MAKE) -C tests clean test

stm8:
	$(MAKE) -C tests stm8

decode:
	$(PYTHON) tools/decode_data.py $(DECODE_ARGS) $(CAPTURE)

c-quality:
	$(CPPCHECK) --enable=warning,performance,portability --std=c99 --error-exitcode=1 --suppress=missingIncludeSystem --suppress=missingInclude --quiet firmware

python-quality:
	$(RUFF) check $(PYTHON_PATHS)
	$(RUFF) format --check $(PYTHON_PATHS)

whitespace:
	$(PYTHON) tools/check_whitespace.py

quality: c-quality python-quality whitespace
	@echo 'PicoTracker quality checks passed.'

format:
	$(RUFF) check --fix $(PYTHON_PATHS)
	$(RUFF) format $(PYTHON_PATHS)

check: quality test decode simulate stm8
	@echo 'All PicoTracker repository checks passed.'

clean:
	$(MAKE) -C tests clean

container-build:
	$(DOCKER) build -f .devcontainer/Dockerfile -t $(DEV_IMAGE) .

container-check: container-build
	$(DOCKER) run --rm --user "$$(id -u):$$(id -g)" -e HOME=/tmp -v "$(CURDIR):/workspace" -w /workspace $(DEV_IMAGE) make check

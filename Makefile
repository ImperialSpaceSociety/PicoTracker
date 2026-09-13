PYTHON ?= python3
SAMPLE_CAPTURE := tools/with_pips_data.txt
CAPTURE ?= $(SAMPLE_CAPTURE)
DECODE_ARGS ?=
DOCKER ?= docker
DEV_IMAGE ?= picotracker-dev

.DEFAULT_GOAL := help

.PHONY: help demo test stm8 decode check clean container-build container-check

help:
	@printf '%s\n' \
	  'PicoTracker developer commands' \
	  '' \
	  '  make demo    Decode the included flight capture (no C compiler or hardware)' \
	  '  make test    Run the complete host regression suite' \
	  '  make decode  Decode CAPTURE (default: tools/with_pips_data.txt)' \
	  '  make stm8    Compile/link the structural STM8 check (requires SDCC)' \
	  '  make check   Run test, decode, and stm8 verification' \
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

test:
	$(MAKE) -C tests clean test

stm8:
	$(MAKE) -C tests stm8

decode:
	$(PYTHON) tools/decode_data.py $(DECODE_ARGS) $(CAPTURE)

check: test decode stm8
	@echo 'All PicoTracker repository checks passed.'

clean:
	$(MAKE) -C tests clean

container-build:
	$(DOCKER) build -f .devcontainer/Dockerfile -t $(DEV_IMAGE) .

container-check: container-build
	$(DOCKER) run --rm --user "$$(id -u):$$(id -g)" -e HOME=/tmp -v "$(CURDIR):/workspace" -w /workspace $(DEV_IMAGE) make check

SHELL := /bin/bash

PROJECT_NAME := $(shell sed -n '/^[[:space:]]*[^#\[[:space:]]/p' PROJECT | head -1 | tr -d '[:space:]')
PROJECT_VERSION := $(shell sed -n '/^[[:space:]]*[^#\[[:space:]]/p' PROJECT | sed -n '2p' | tr -d '[:space:]')
ifeq ($(PROJECT_NAME),)
    $(error Error: PROJECT file not found or invalid)
endif

CARGO := cargo

$(info ------------------------------------------)
$(info Project: $(PROJECT_NAME) v$(PROJECT_VERSION))
$(info ------------------------------------------)

.PHONY: build b compile c run r test t check fmt clean help h

build:
	@$(CARGO) build --bin urdf2usd

b: build

compile:
	@$(CARGO) clean
	@$(MAKE) build

c: compile

# Usage: make run ARGS="path/to/robot.urdf path/to/out_dir"
run:
	@$(CARGO) run --bin urdf2usd -- $(ARGS)

r: run

test:
	@$(CARGO) test

t: test

check:
	@$(CARGO) check --bin urdf2usd

fmt:
	@$(CARGO) fmt --all

clean:
	@$(CARGO) clean

help:
	@echo
	@echo "Usage: make [target]"
	@echo
	@echo "Available targets:"
	@echo "  build        Build the urdf2usd binary"
	@echo "  compile      Clean and rebuild"
	@echo "  run          Run the converter: cargo run --bin urdf2usd -- \$$ARGS"
	@echo "  test         Run the test suite"
	@echo "  check        Run cargo check on the binary"
	@echo "  fmt          Format the workspace"
	@echo "  clean        Remove Cargo build artifacts"
	@echo
	@echo "Examples:"
	@echo "  make run ARGS=\"robot.urdf out_dir\""
	@echo

h: help

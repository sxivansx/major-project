# major-project: one entry point for the whole tool.
#
#   make setup    first time: SDK + bridge, python venv, web deps
#   make dev      server (:8000) + vite dev (:5173) side by side; Ctrl-C stops both
#   make sim      same, but with the simulated lidar + servo pre-connected
#   make build    production web bundle; then `make serve` runs everything on :8000
#   make test     python tests + web typecheck
#   make flash PORT=/dev/cu.usbmodem1101 [FQBN=esp32:esp32:esp32s3]

SHELL := /bin/bash
FQBN ?= esp32:esp32:esp32
PORT ?=

.PHONY: setup driver core web dev sim serve build test flash clean

setup: driver core web

driver:
	$(MAKE) -C driver

core:
	cd core && uv sync

web:
	cd web && bun install

dev:
	@trap 'kill 0' EXIT; \
	(cd core && uv run obslam-server) & \
	(cd web && bun run dev) & \
	wait

sim:
	@trap 'kill 0' EXIT; \
	(cd core && uv run obslam-server --sim) & \
	(cd web && bun run dev) & \
	wait

build:
	cd web && bun run build

serve: build
	cd core && uv run obslam-server

test:
	cd core && uv run pytest -q
	cd web && bun run typecheck

flash:
	@test -n "$(PORT)" || (echo "usage: make flash PORT=/dev/cu.usbmodemXXXX [FQBN=esp32:esp32:esp32s3]"; exit 1)
	arduino-cli compile --fqbn $(FQBN) firmware/servo_tilt
	arduino-cli upload --fqbn $(FQBN) -p $(PORT) firmware/servo_tilt

clean:
	$(MAKE) -C driver clean
	rm -rf web/dist

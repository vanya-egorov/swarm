start arm-check flight-check route-check: build
	@bash scripts/devstack.sh $@

swarm-group-route-check: build
	@DRONE_COUNT=3 bash scripts/devstack.sh group-route-check

swarm-flight-check: build
	@DRONE_COUNT=3 bash scripts/devstack.sh group-flight-check

swarm-route-check: build
	@DRONE_COUNT=3 CHECK_INSTANCE=0 bash scripts/devstack.sh route-check

swarm-arm-check: build
	@DRONE_COUNT=3 bash scripts/devstack.sh arm-check

swarm-start: build
	@DRONE_COUNT=3 bash scripts/devstack.sh start

sitl-build:
	@python3 scripts/build_sitl.py

VILLAGE_PYTHON ?= projectData/tools/village/bin/python
VILLAGE_SOURCE ?= проект_карта_деревни
VILLAGE_MAP_SIZE ?= 1500

village-deps:
	@python3 -m venv projectData/tools/village
	@$(VILLAGE_PYTHON) -m pip install -r scripts/requirements-village.txt

village-world:
	@$(VILLAGE_PYTHON) scripts/prepare_village.py --source "$(VILLAGE_SOURCE)" --map-size "$(VILLAGE_MAP_SIZE)"

village-sensors:
	@python3 sim/webots/configure_sensors.py

village-validate:
	@python3 sim/webots/configure_sensors.py --check

village-coverage-prepare:
	@$(VILLAGE_PYTHON) scripts/prepare_village_coverage.py

village-coverage-check: build
	@python3 scripts/coverage_check.py

village-coverage-failover: build
	@python3 scripts/coverage_check.py --failover

village-route-check: build
	@DRONE_COUNT=3 ARM_CHECK_PARAMS="$${RUN_DIR:-projectData/run}/webots/village-flight.parm" bash scripts/devstack.sh group-route-check

village-flight-check: village-route-check

test-village:
	@$(VILLAGE_PYTHON) -m unittest discover -s scripts/village/tests -v

swarm-world: village-world

test-sensors:
	@python3 -m unittest discover -s sim/webots/tests -v

stop status:
	@bash scripts/devstack.sh $@

console:
	@python3 scripts/console.py

build:
	@mkdir -p projectData/run/bin
	@go build -o projectData/run/bin/swarm ./cmd/swarm

test:
	go test ./...

test-race:
	go test -race ./...

lint:
	golangci-lint run
	@for script in scripts/devstack.sh scripts/devstack/*.sh; do bash -n "$$script" || exit; done

fmt:
	golangci-lint fmt

test-stack:
	python3 -m unittest discover -s scripts/tests -v

check: lint test-race test-stack test-sensors

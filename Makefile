start:
	@mkdir -p projectData/run/bin
	@go build -o projectData/run/bin/swarm ./cmd/swarm
	@bash scripts/devstack.sh start

stop:
	@bash scripts/devstack.sh stop

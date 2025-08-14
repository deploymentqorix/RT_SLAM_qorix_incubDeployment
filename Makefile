# The main Makefile for the entire project

up:
	docker compose up --build -d

down:
	docker compose down

clean:
	docker compose down -v --remove-orphans

logs:
	docker compose logs -f

shell:
	docker compose exec ros_nodes /bin/bash

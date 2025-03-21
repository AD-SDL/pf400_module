.DEFAULT_GOAL := init

.PHONY += init paths checks test clean

init: # Do the initial configuration of the project
	@test -e .env || cp example.env .env

.PHONY += init paths checks test clean
init: # Do the initial configuration of the project
	@test -e .env || cp example.env .env

.env: init

paths: .env # Create the necessary data directories
	@mkdir -p $(shell grep -E '^WEI_DATA_DIR=' .env | cut -d '=' -f 2)
	@mkdir -p $(shell grep -E '^REDIS_DIR=' .env | cut -d '=' -f 2)

checks: # Runs all the pre-commit checks
	@pre-commit install
	@pre-commit run --all-files || { echo "Checking fixes\n" ; pre-commit run --all-files; }

clean:
	@rm -f .env

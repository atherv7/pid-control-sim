BUILD_DEBUG_DIR := build/debug
BUILD_RELEASE_DIR := build/release
JOBS ?= $(shell nproc)

SRC_DIRS := src include tests
SOURCES := $(shell find $(SRC_DIRS) -name '*.cpp' -o -name '*.hpp')

.PHONY: all
all: release

.PHONY: configure
configure:
	cmake -S . -B $(BUILD_DEBUG_DIR) -DCMAKE_BUILD_TYPE=Debug
	ln -sf $(BUILD_DEBUG_DIR)/compile_commands.json compile_commands.json

.PHONY: debug
debug: configure
	cmake --build $(BUILD_DEBUG_DIR) -j$(JOBS)

.PHONY: release
release:
	cmake -S . -B $(BUILD_RELEASE_DIR) -DCMAKE_BUILD_TYPE=Release
	cmake --build $(BUILD_RELEASE_DIR) -j$(JOBS)

.PHONY: test
test: debug
	ctest --test-dir $(BUILD_DEBUG_DIR) --output-on-failure

.PHONY: tidy
tidy: configure
	clang-tidy -p $(BUILD_DEBUG_DIR) $(SOURCES)

.PHONY: fmt
fmt:
	clang-format -i $(SOURCES)

.PHONY: fmt-check
fmt-check:
	clang-format --dry-run --Werror $(SOURCES)

.PHONY: clean
clean:
	rm -rf $(BUILD_DEBUG_DIR) $(BUILD_RELEASE_DIR) compile_commands.json

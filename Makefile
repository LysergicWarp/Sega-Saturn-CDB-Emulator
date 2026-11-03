# Sega Saturn CD block emulator.
#
#   make              the static and shared library in build/
#   make test         the tests under AddressSanitizer and UBSan, then their run
#   make sanitized    the shared library under the same sanitizers, in build/sanitized/
#   make test SANITIZE=0   the tests without sanitizers, for toolchains that lack them (MinGW)
#   make firmware-test     the shared library, then tests/firmware on the dumps in dumps/, each
#                          test skipping what a missing dump needs (Python 3)
#   make firmware-test DUMPS=DIR TESTS="package decoder"   other dumps, some of the tests, their
#                          results in RESULTS (build/firmware-tests)
#   make format       the C and C++ sources through clang-format, in place
#
# Warnings are errors.

CXX ?= g++
AR ?= ar
CLANG_FORMAT ?= clang-format
OPT ?= -O2
LDFLAGS ?=
SANITIZE ?= 1
PYTHON ?= python3
DUMPS ?= dumps
RESULTS ?= build/firmware-tests
TESTS ?=

WARNINGS := -Wall -Wextra -Wpedantic -Werror -Wshadow -Wconversion -Wsign-conversion -Wcast-qual \
	-Wformat=2 -Wnull-dereference -Wundef -Wnon-virtual-dtor -Woverloaded-virtual \
	-Wmissing-declarations -Wimplicit-fallthrough
ifeq ($(findstring clang,$(shell $(CXX) --version 2>/dev/null)),)
WARNINGS += -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wuseless-cast
endif
SANITIZERS := -O1 -g -fno-omit-frame-pointer -fsanitize=address,undefined -fno-sanitize-recover=all \
	-D_GLIBCXX_ASSERTIONS
COMMON := -std=c++20 $(WARNINGS) -MMD -MP -Iinclude

ifeq ($(OS),Windows_NT)
SHARED_NAME := cdblock.dll
SHARED_FLAGS := -shared -static-libstdc++ -static-libgcc
EXE := .exe
else
COMMON += -fPIC -fvisibility=hidden
ifeq ($(shell uname -s),Darwin)
SHARED_NAME := libcdblock.dylib
SHARED_FLAGS := -dynamiclib
else
SHARED_NAME := libcdblock.so
SHARED_FLAGS := -shared
endif
EXE :=
endif

SOURCES := $(wildcard src/*.cpp src/*/*.cpp)
TEST_SOURCES := $(wildcard tests/emulator/*.cpp)
FORMATTED := $(SOURCES) $(TEST_SOURCES) $(wildcard include/*.h src/*.h src/*/*.h tests/emulator/*.h)
RELEASE := $(SOURCES:src/%.cpp=build/obj/%.o)
CHECKED := $(SOURCES:src/%.cpp=build/sanitized/obj/%.o)

ifeq ($(SANITIZE),1)
TEST_DIR := build/sanitized
TEST_FLAGS := $(SANITIZERS)
TEST_LIB_OBJECTS := $(CHECKED)
else
TEST_DIR := build
TEST_FLAGS := $(OPT)
TEST_LIB_OBJECTS := $(RELEASE)
endif
TEST_OBJECTS := $(TEST_SOURCES:tests/emulator/%.cpp=$(TEST_DIR)/obj/tests/%.o)

all: build/libcdblock.a build/$(SHARED_NAME)

sanitized: build/sanitized/$(SHARED_NAME)

build/obj/%.o: src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(COMMON) -DCDBLOCK_BUILD $(OPT) -c $< -o $@

build/sanitized/obj/%.o: src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(COMMON) -DCDBLOCK_BUILD $(SANITIZERS) -c $< -o $@

$(TEST_DIR)/obj/tests/%.o: tests/emulator/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(COMMON) $(TEST_FLAGS) -c $< -o $@

build/libcdblock.a: $(RELEASE)
	$(AR) rcs $@ $^

build/$(SHARED_NAME): $(RELEASE)
	$(CXX) $(SHARED_FLAGS) $(LDFLAGS) -o $@ $^

build/sanitized/$(SHARED_NAME): $(CHECKED)
	$(CXX) $(SHARED_FLAGS) -fsanitize=address,undefined $(LDFLAGS) -o $@ $^

$(TEST_DIR)/tests$(EXE): $(TEST_OBJECTS) $(TEST_LIB_OBJECTS)
	$(CXX) $(TEST_FLAGS) $(LDFLAGS) -o $@ $^

test: $(TEST_DIR)/tests$(EXE)
	ASAN_OPTIONS=detect_leaks=1:abort_on_error=1 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 ./$(TEST_DIR)/tests$(EXE)

firmware-test: build/$(SHARED_NAME)
	$(PYTHON) -m tests.firmware --dumps $(abspath $(DUMPS)) --out $(abspath $(RESULTS)) $(TESTS)

format:
	$(CLANG_FORMAT) -i $(FORMATTED)

clean:
	rm -rf build

.PHONY: all sanitized test firmware-test format clean

-include $(RELEASE:.o=.d) $(CHECKED:.o=.d) $(TEST_OBJECTS:.o=.d)

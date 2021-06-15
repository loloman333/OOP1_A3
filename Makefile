CXX           := clang++
CXXFLAGS      := -Wall -Wextra -pedantic -std=c++17 -g -c -o
ASSIGNMENT    := oop1

BUILDDIR      := build
SOURCES       := $(wildcard *.cpp)
OBJECTS       := $(patsubst %,$(BUILDDIR)/%,${SOURCES:.cpp=.o})

TESTRUNNER_EXECUTABLE := testrunner

.DEFAULT_GOAL := help
.PHONY: reset clean bin lib all run test help

prepare:
	mkdir -p $(BUILDDIR)

$(BUILDDIR)/%.o: %.cpp
	@echo -e "[\033[36mINFO\033[0m] Compiling object:" $<
	$(CXX) $(CXXFLAGS) $@ $< -MMD -MF ./$@.d

$(ASSIGNMENT) : $(OBJECTS)
	@echo -e "[\033[36mINFO\033[0m] Linking objects:" $@
	$(CXX) -pthread -o $@ $^

clean:						## cleans up project folder
	@echo -e "[\033[36mINFO\033[0m] Cleaning up folder..."
	rm -f $(ASSIGNMENT)
	rm -rf ./$(BUILDDIR)
	rm -rf testreport.html
	rm -rf ./valgrind_logs

bin: prepare $(ASSIGNMENT)	## compiles project to executable binary
	@echo -e "[\033[36mINFO\033[0m] Compiling binary..."
	chmod +x $(ASSIGNMENT)

all: clean bin				## all of the above

run: all					## runs the project
	@echo -e "[\033[36mINFO\033[0m] Executing binary..."
	./$(ASSIGNMENT)

test: clean					## runs public testcases on the project
	@echo -e "[\033[36mINFO\033[0m] Executing testrunner..."
	chmod +x testrunner
	./$(TESTRUNNER_EXECUTABLE) -c test.toml

help:						## prints the help text
	@echo -e "Usage: make \033[36m<TARGET>\033[0m"
	@echo -e "Available targets:"
	@awk -F':.*?##' '/^[a-zA-Z_-]+:.*?##.*$$/{printf "  \033[36m%-10s\033[0m%s\n", $$1, $$2}' $(MAKEFILE_LIST)

-include $(wildcard *.d)


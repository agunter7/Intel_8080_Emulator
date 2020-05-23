CC=gcc
# Miscellaneous compiler flags
GENERAL_FLAGS=-Wall
# Source code file names
SOURCES_TEST=src/cpuTest.c src/shell8080.c src/instructions.c src/helpers.c
# Final executable name
EXE_NAME_TEST=bin/cpu_test

test: $(SOURCES_TEST)
	$(CC) $(SOURCES_TEST) $(GENERAL_FLAGS) -o $(EXE_NAME_TEST)

clean:
	rm bin/cpu_test.exe

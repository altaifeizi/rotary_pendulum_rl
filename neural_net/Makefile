# Compiler used for building the project
CC = gcc

# Compiler flags:
# -Wall: Enable all warnings
# -Wextra: Enable additional warnings
# -g: Include debugging information
CFLAGS = -Wall -Wextra -g

# Source files to be compiled
SRC = src/main.c src/matrix_ops.c src/neural_net.c

# Include directory containing header files
INC = -Iinc

# Name of the generated executable
TARGET = neural_net_test

# Default target: build the executable
all: $(TARGET)

# Rule to generate the target executable from the source files.
# Compiler flags, source files, and include directory are passed to the compiler.
$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) $(INC) -o $(TARGET)

# "clean" target: removes the executable to allow a clean build.
clean:
	rm -f $(TARGET)

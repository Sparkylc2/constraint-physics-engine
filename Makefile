CXX := g++
CXXFLAGS := -Wall -Wextra -std=c++17
LDFLAGS := -lraylib -lm

# homebrew 
BREW_PREFIX := $(shell brew --prefix 2>/dev/null || echo /opt/homebrew)
CXXFLAGS += -I$(BREW_PREFIX)/include -Iinclude
LDFLAGS += -L$(BREW_PREFIX)/lib

# macos frameworks raylib depends on
LDFLAGS += -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo

SRC := $(wildcard src/*.cpp)
OBJ := $(SRC:src/%.cpp=build/%.o)
TARGET := game

.PHONY: all clean run

all: build $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(OBJ) -o $@ $(LDFLAGS)

build/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

build:
	mkdir -p build

run: all
	./$(TARGET)

clean:
	rm -rf build $(TARGET)

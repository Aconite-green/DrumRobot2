# Declare variables
CC = g++
CFLAGS = -Wall -O2 -g -std=c++17 -fPIC `pkg-config --cflags Qt5Widgets Qt5Charts`  # Qt 플래그 추가
INCLUDE = -I./include -I./lib `pkg-config --cflags Qt5Widgets Qt5Charts`
LDFLAGS = -lm -lpthread -lstdc++fs -L./lib -lUSBIO_64 `pkg-config --libs Qt5Widgets Qt5Charts`  # Qt 라이브러리 링크 추가
SRCDIR = ./src
BINDIR = ./src/main.out

# Automatically include all .cpp files from the src directory
SOURCES := $(wildcard $(SRCDIR)/*.cpp)
OBJFILES := $(patsubst %.cpp, %.o, $(SOURCES))

# Phony targets
.PHONY: all clean

# Qt MOC 처리
MOC = moc
UI_DIR = ./src/ui
MOC_SRC = $(wildcard $(UI_DIR)/*.h)
MOC_OBJ = $(patsubst %.h, %.moc.cpp, $(MOC_SRC))

# Build target
all: $(BINDIR)

$(BINDIR): $(OBJFILES)
	$(CC) $(CFLAGS) $^ -o $@ $(INCLUDE) $(LDFLAGS)

# Pattern rules
%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@ $(INCLUDE)

# MOC 규칙
%.moc.cpp: %.h
	$(MOC) $< -o $@

# Clean rule
clean:
	rm -f $(SRCDIR)/*.o $(BINDIR)

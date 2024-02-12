.PHONY: all generate build test run format clean

all: clean generate build test

generate:
	cmake -S . -B build

build:
	cmake --build build

test:
	ctest --test-dir build

run:
	./build/graphics_paper

format:
	clang-format -i --style=Google {src,lib}/**/*.[ch]pp

clean:
	rm -rf build .cache

.PHONY: all generate build test run format clean

all: clean generate build test

generate:
	cmake -DBUILD_TESTS=ON -S . -B build

build:
	cmake --build build

test:
	ctest --test-dir build

run:
	./build/graphics_paper

format:
	clang-format -i {src,lib}/**/*.[ch]pp

clean:
	rm -rf build .cache

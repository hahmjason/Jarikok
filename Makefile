# 자리콕 웹 빌드
#   make test  : C 로직 단위 테스트 (gcc)
#   make wasm  : 같은 C 로직을 WebAssembly 로 컴파일 -> assets/jarikok_wasm.js (웹에서 사용)
CC      ?= gcc
CLANG   ?= clang

all: test wasm report

report: assets/report.js

assets/report.js: report/03_공정방법.md report/07_결론.md scripts/build_report.py
	python3 scripts/build_report.py .

test: build/test_jarikok
	./build/test_jarikok

build/test_jarikok: c/jarikok.c c/jarikok.h tests/test_jarikok.c
	@mkdir -p build
	$(CC) -std=c99 -Wall -Wextra -O2 -o $@ c/jarikok.c tests/test_jarikok.c

wasm: assets/jarikok_wasm.js

build/jarikok.wasm: c/jarikok.c c/jarikok.h
	@mkdir -p build
	$(CLANG) --target=wasm32 -std=c99 -O2 -nostdlib -fvisibility=hidden \
		-Wl,--no-entry -Wl,--export-dynamic -o $@ c/jarikok.c

# wasm 바이너리를 base64 로 JS 파일에 내장 -> 서버 없이 파일을 더블클릭해도 동작
assets/jarikok_wasm.js: build/jarikok.wasm
	node scripts/embed_wasm.js $< $@

clean:
	rm -rf build

.PHONY: all test wasm report clean

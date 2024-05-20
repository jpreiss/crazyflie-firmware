EIGEN := $(shell python3 -c 'import eigenpip as e; print(e.get_include())')
MOD_INC := ../src/modules/interface

GENDIR := $(MOD_INC)/codegen
GEN := $(GENDIR)/ctrl.h
SRCS := benchmark.cpp $(MOD_INC)/gapsquad.h $(MOD_INC)/gapsquad.hpp $(GEN)
BUILD := setup.py pyproject.toml

all: benchmark bindings

$(GEN): ../gaps_codegen.py
	cd .. && python3 gaps_codegen.py

benchmark: $(SRCS)
	$(CXX) \
		$(CXXFLAGS) --std=c++14 -DEIGEN_NO_MALLOC -g -O3 -fstack-usage \
		-I$(EIGEN) -I$(EIGEN)/unsupported -I$(MOD_INC) \
		-o $@ $<

install: $(SRCS) $(BUILD)
	pip install -e .

bindings: $(SRCS) $(BUILD) $(GEN)
	mkdir -p gapsquad
	python3 setup.py build

.PHONY: stack

stack: benchmark
	@echo ctrl: `cat benchmark.su | grep "ctrl[^_]" | cut -f 2-`
	@echo dynamics: `cat benchmark.su | grep "dynamics[^_]" | cut -f 2-`

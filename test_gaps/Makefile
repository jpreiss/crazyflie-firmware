EIGEN := $(shell python3 -c 'import eigenpip as e; print(e.get_include())')
MOD_INC := ../src/modules/interface

SRCS := benchmark.cpp $(MOD_INC)/gapsquad.h $(MOD_INC)/gapsquad.hpp

all: benchmark bindings

benchmark: $(SRCS)
	$(CXX) \
		$(CXXFLAGS) --std=c++14 -DEIGEN_NO_MALLOC -g -O3 -fstack-usage \
		-I$(EIGEN) -I$(EIGEN)/unsupported -I$(MOD_INC) \
		-o $@ $<

install: $(SRCS) setup.py pyproject.toml
	pip install -e .

bindings: $(SRCS) setup.py pyproject.toml
	mkdir -p gapsquad
	python3 setup.py build

.PHONY: stack

stack: benchmark
	@echo ctrl: `cat benchmark.su | grep "ctrl[^_]" | cut -f 2-`
	@echo dynamics: `cat benchmark.su | grep "dynamics[^_]" | cut -f 2-`

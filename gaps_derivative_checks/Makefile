EIGEN := $(shell python -c 'import eigenpip as e; print(e.get_include())')
MOD_INC := ../src/modules/interface

SRCS := benchmark.cpp $(MOD_INC)/gapsquad.h $(MOD_INC)/gapsquad.hpp

benchmark: $(SRCS)
	$(CXX) \
		$(CXXFLAGS) --std=c++14 -DEIGEN_NO_MALLOC -g -O3 -fstack-usage \
		-I$(EIGEN) -I$(EIGEN)/unsupported -I$(MOD_INC) \
		-o $@ $<

bindings: $(SRCS) setup.py pyproject.toml
	python setup.py build

.PHONY: stack

stack: benchmark
	@echo ctrl: `cat benchmark.su | grep "ctrl[^_]" | cut -f 2-`
	@echo dynamics: `cat benchmark.su | grep "dynamics[^_]" | cut -f 2-`

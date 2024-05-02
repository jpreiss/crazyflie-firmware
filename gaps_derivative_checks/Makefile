EIGEN := $(shell python -c 'import eigenpip as e; print(e.get_include())')
MODULES := ../src/modules

benchmark: benchmark.cpp $(MODULES)/interface/gapsquad.h $(MODULES)/src/controller/gapsquad.hpp
	$(CXX) \
		$(CXXFLAGS) --std=c++14 -DEIGEN_NO_MALLOC -g -O3 -fstack-usage \
		-I$(EIGEN) -I$(EIGEN)/unsupported -I$(MODULES)/interface -I$(MODULES)/src/controller \
		-o $@ $<

.PHONY: stack

stack: benchmark
	@echo ctrl: `cat benchmark.su | grep "ctrl[^_]" | cut -f 2-`
	@echo dynamics: `cat benchmark.su | grep "dynamics[^_]" | cut -f 2-`

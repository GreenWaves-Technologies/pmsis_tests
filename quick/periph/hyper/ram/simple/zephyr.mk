ifdef runner_args
export RUNNER_ARGS=$(runner_args)
endif

BUILDDIR = build/zephyr$(build_dir_ext)

$(BUILDDIR):
	mkdir -p $(BUILDDIR)
	cd $(BUILDDIR) && cmake -DBOARD=gapuino ../../zephyr -DCMAKE_C_FLAGS="$(PULP_CFLAGS)"

clean:
	rm -rf $(BUILDDIR)

all: $(BUILDDIR)
	cd $(BUILDDIR) && make all

run:
	cd $(BUILDDIR) && make run

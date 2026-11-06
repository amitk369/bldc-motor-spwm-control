CC = clang
CFLAGS = -Wall -Wextra -O2 -I./firmware/include
SRC = firmware/src/hall_commutation.c firmware/src/digital_pid.c firmware/src/bldc_controller.c
TEST_SRC = tests/test_bldc.c

all: test

test:
	$(CC) $(CFLAGS) $(SRC) $(TEST_SRC) -o run_tests -lm
	./run_tests

sim:
	python3 simulation/simulate_bldc.py

clean:
	rm -f run_tests

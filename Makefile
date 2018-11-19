
build:
	newt build test_sensor

flash: build
	newt create-image test_sensor 1.0.0
	newt load test_sensor

debug: flash
	newt debug test_sensor

clean:
	newt clean all

.PHONY: build flash debug clean

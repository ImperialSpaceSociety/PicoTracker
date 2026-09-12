CC ?= cc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror
BUILD_DIR := build

.PHONY: test clean

test: $(BUILD_DIR)/test_number_format
	$(BUILD_DIR)/test_number_format

$(BUILD_DIR)/test_number_format: test_number_format.c ../firmware/string.c
	mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) test_number_format.c ../firmware/string.c -o $@

clean:
	rm -rf $(BUILD_DIR)

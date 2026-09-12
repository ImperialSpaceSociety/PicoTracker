CC ?= cc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror
BUILD_DIR := build

.PHONY: test clean

test: $(BUILD_DIR)/test_number_format $(BUILD_DIR)/test_ubx_protocol $(BUILD_DIR)/test_radio_measurements $(BUILD_DIR)/test_sleep_policy $(BUILD_DIR)/test_status_word $(BUILD_DIR)/test_telemetry_crc
	$(BUILD_DIR)/test_number_format
	$(BUILD_DIR)/test_ubx_protocol
	$(BUILD_DIR)/test_radio_measurements
	$(BUILD_DIR)/test_sleep_policy
	$(BUILD_DIR)/test_status_word
	$(BUILD_DIR)/test_telemetry_crc

$(BUILD_DIR)/test_number_format: test_number_format.c ../firmware/number_format.c
	mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) test_number_format.c ../firmware/number_format.c -o $@

$(BUILD_DIR)/test_ubx_protocol: test_ubx_protocol.c ../firmware/ubx_protocol.h
	mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) test_ubx_protocol.c -o $@

$(BUILD_DIR)/test_radio_measurements: test_radio_measurements.c ../firmware/radio_measurements.h
	mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) test_radio_measurements.c -o $@

$(BUILD_DIR)/test_sleep_policy: test_sleep_policy.c ../firmware/sleep_policy.h
	mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) test_sleep_policy.c -o $@

$(BUILD_DIR)/test_status_word: test_status_word.c ../firmware/status_word.h
	mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) test_status_word.c -o $@

$(BUILD_DIR)/test_telemetry_crc: test_telemetry_crc.c ../firmware/telemetry_crc.h
	mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) test_telemetry_crc.c -o $@

clean:
	rm -rf $(BUILD_DIR)

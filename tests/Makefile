CXX ?= g++
BUILD_DIR ?= /tmp/missioncontrol-tests
CXXFLAGS ?= -std=c++20 -O2 -g -Wall -Wextra -Werror -pthread

.PHONY: test
test:
	mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) ble_regression.cpp -o $(BUILD_DIR)/ble_regression
	$(BUILD_DIR)/ble_regression
	$(CXX) $(CXXFLAGS) -I../lib/libnx/nx/include advertising_regression.cpp -o $(BUILD_DIR)/advertising_regression
	$(BUILD_DIR)/advertising_regression

	mkdir -p $(BUILD_DIR)/gatt
	cp ../mc_mitm/source/ble/ble_gatt.cpp ../mc_mitm/source/ble/ble_gatt.hpp gatt_shim/* $(BUILD_DIR)/gatt/
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR)/gatt gatt_regression.cpp -o $(BUILD_DIR)/gatt_regression
	$(BUILD_DIR)/gatt_regression

	mkdir -p $(BUILD_DIR)/profile
	cp ../mc_mitm/source/ble/ble_profile.cpp ../mc_mitm/source/ble/ble_profile.hpp profile_shim/* $(BUILD_DIR)/profile/
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR)/profile profile_regression.cpp -o $(BUILD_DIR)/profile_regression
	$(BUILD_DIR)/profile_regression

	mkdir -p $(BUILD_DIR)/sleep
	cp ../mc_mitm/source/ble/ble_sleep.cpp ../mc_mitm/source/ble/ble_sleep.hpp profile_shim/* sleep_shim/* $(BUILD_DIR)/sleep/
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR)/sleep sleep_regression.cpp -o $(BUILD_DIR)/sleep_regression
	$(BUILD_DIR)/sleep_regression

	mkdir -p $(BUILD_DIR)/pairing
	cp ../mc_mitm/source/ble/ble_pairing.cpp ../mc_mitm/source/ble/ble_pairing.hpp gatt_shim/ble_log.hpp pairing_shim/* $(BUILD_DIR)/pairing/
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR)/pairing pairing_regression.cpp -o $(BUILD_DIR)/pairing_regression
	$(BUILD_DIR)/pairing_regression

	$(CXX) $(CXXFLAGS) stadia_regression.cpp -o $(BUILD_DIR)/stadia_regression
	$(BUILD_DIR)/stadia_regression

	mkdir -p $(BUILD_DIR)/sim
	cp ../mc_mitm/source/ble/ble_pairing.cpp ../mc_mitm/source/ble/ble_pairing.hpp ../mc_mitm/source/ble/ble_gatt.cpp ../mc_mitm/source/ble/ble_gatt.hpp sim_shim/* $(BUILD_DIR)/sim/
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR)/sim hos2250_sim_regression.cpp -o $(BUILD_DIR)/hos2250_sim_regression
	$(BUILD_DIR)/hos2250_sim_regression

	mkdir -p $(BUILD_DIR)/stadia_ctrl/controllers $(BUILD_DIR)/stadia_ctrl/ble
	cp ../mc_mitm/source/controllers/stadia_ble_controller.cpp ../mc_mitm/source/controllers/stadia_ble_controller.hpp ../mc_mitm/source/controllers/stadia_mapping.hpp stadia_ctrl_shim/*.hpp $(BUILD_DIR)/stadia_ctrl/controllers/
	cp stadia_ctrl_shim/ble/* $(BUILD_DIR)/stadia_ctrl/ble/
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR)/stadia_ctrl/controllers stadia_controller_regression.cpp -o $(BUILD_DIR)/stadia_controller_regression
	$(BUILD_DIR)/stadia_controller_regression
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR)/stadia_ctrl/controllers stadia_fuzz_regression.cpp -o $(BUILD_DIR)/stadia_fuzz_regression
	$(BUILD_DIR)/stadia_fuzz_regression

	mkdir -p $(BUILD_DIR)/events/ble $(BUILD_DIR)/events/bluetooth_mitm/bluetooth $(BUILD_DIR)/events/controllers
	cp ../mc_mitm/source/ble/ble_events.cpp ../mc_mitm/source/ble/ble_events.hpp ../mc_mitm/source/ble/ble_types.hpp ../mc_mitm/source/ble/ble_controller_routes.hpp $(BUILD_DIR)/events/ble/
	cp events_shim/switch.h events_shim/stratosphere.hpp events_shim/mcmitm_config.hpp $(BUILD_DIR)/events/
	cp events_shim/ble/* $(BUILD_DIR)/events/ble/
	cp events_shim/bluetooth_mitm/btdrv_ext.h $(BUILD_DIR)/events/bluetooth_mitm/
	cp events_shim/bluetooth_mitm/bluetooth/* $(BUILD_DIR)/events/bluetooth_mitm/bluetooth/
	cp events_shim/controllers/* $(BUILD_DIR)/events/controllers/
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR)/events -I$(BUILD_DIR)/events/ble events_regression.cpp -o $(BUILD_DIR)/events_regression
	$(BUILD_DIR)/events_regression

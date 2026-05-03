#pragma once

#include <memory>
#include <map>
#include <string>
#include <esphome/components/ble_client/ble_client.h>
#include <esphome/components/esp32_ble_tracker/esp32_ble_tracker.h>
#include <esphome/components/binary_sensor/binary_sensor.h>
#include <esphome/components/sensor/sensor.h>
#include <esphome/components/text_sensor/text_sensor.h>
#include <esphome/components/button/button.h>
#include <esphome/components/switch/switch.h>
#include <esphome/components/number/number.h>
#include <esphome/components/lock/lock.h>
#include <esphome/components/cover/cover.h>
#include <esphome/components/climate/climate.h>
#include <esphome/core/component.h>
#include <esphome/core/automation.h>

#include "ble_adapter_impl.h"
#include "storage_adapter_impl.h"
#include <vehicle.h>
#include "vehicle_state_manager.h"

namespace esphome {
namespace tesla_ble_vehicle {

namespace espbt = esphome::esp32_ble_tracker;

static const char *const TAG = "tesla_ble_vehicle";

static const char *const SERVICE_UUID = "00000211-b2d1-43f0-9b88-960cebf8b91e";
static const char *const READ_UUID = "00000213-b2d1-43f0-9b88-960cebf8b91e";
static const char *const WRITE_UUID = "00000212-b2d1-43f0-9b88-960cebf8b91e";

class TeslaBLEVehicle : public PollingComponent, public ble_client::BLEClientNode {
public:
    TeslaBLEVehicle();
    ~TeslaBLEVehicle() = default;

    void setup() override;
    void loop() override;
    void update() override;
    void dump_config() override;

    void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

    void set_vin(const char *vin);
    void set_role(const std::string &role);
    void set_charging_amps_max(int amps_max);
    
    void set_vcsec_poll_interval(uint32_t interval_ms);
    void set_infotainment_poll_interval_awake(uint32_t interval_ms);
    void set_infotainment_poll_interval_active(uint32_t interval_ms);
    void set_infotainment_sleep_timeout(uint32_t interval_ms);

    void set_binary_sensor(const std::string& id, binary_sensor::BinarySensor* sensor);
    void set_sensor(const std::string& id, sensor::Sensor* sensor);
    void set_text_sensor(const std::string& id, text_sensor::TextSensor* sensor);

    void set_charging_switch(switch_::Switch *sw);
    void set_steering_wheel_heat_switch(switch_::Switch *sw);
    void set_sentry_mode_switch(switch_::Switch *sw);
    void set_charging_amps_number(number::Number *number);
    void set_charging_limit_number(number::Number *number);

    void set_doors_lock(lock::Lock *lck);
    void set_charge_port_latch_lock(lock::Lock *lck);
    void set_trunk_cover(cover::Cover *cvr);
    void set_frunk_cover(cover::Cover *cvr);
    void set_windows_cover(cover::Cover *cvr);
    void set_charge_port_door_cover(cover::Cover *cvr);
    void set_climate(climate::Climate *clm);

    void set_wake_button(button::Button *button);
    void set_pair_button(button::Button *button);
    void set_regenerate_key_button(button::Button *button);
    void set_force_update_button(button::Button *button);
    void set_start_driving_button(button::Button *button);

    int wake_vehicle();
    int start_pairing();
    int regenerate_key();
    void force_update();
    void start_driving();

    int set_charging_state(bool charging);
    int set_charging_amps(int amps);
    int set_charging_limit(int limit);
    
    void lock_vehicle();
    void unlock_vehicle();
    void open_trunk();
    void close_trunk();
    void open_frunk();
    void open_charge_port();
    void close_charge_port();
    void unlock_charge_port();
    void unlatch_driver_door();
    
    void set_climate_on(bool enable);
    void set_climate_temp(float temp);
    void set_climate_keeper(int mode);
    void set_bioweapon_mode(bool enable);
    void set_preconditioning_max(bool enable);
    void set_steering_wheel_heat(bool enable);
    
    void flash_lights();
    void honk_horn();
    void set_sentry_mode(bool enable);
    void vent_windows();
    void close_windows();
    
    void update_charging_amps_max_value(int32_t new_max);

    VehicleStateManager* get_state_manager() const { return state_manager_.get(); }
    bool is_connected() const { return node_state == espbt::ClientState::ESTABLISHED; }
    uint16_t get_read_handle() const { return read_handle_; }
    uint16_t get_write_handle() const { return write_handle_; }

private:
    void initialize_managers();
    void initialize_ble_uuids();
    void configure_pending_sensors();
    void setup_button_callbacks();
    void handle_connection_established();
    void handle_connection_lost();

    std::shared_ptr<BleAdapterImpl> ble_adapter_;
    std::shared_ptr<StorageAdapterImpl> storage_adapter_;
    std::shared_ptr<::TeslaBLE::Vehicle> vehicle_;
    std::unique_ptr<VehicleStateManager> state_manager_;

    std::string vin_;
    std::string role_;
    
    uint32_t vcsec_poll_interval_{10000};
    uint32_t infotainment_poll_interval_awake_{30000};
    uint32_t infotainment_poll_interval_active_{10000};
    uint32_t infotainment_sleep_timeout_{660000};
    
    uint32_t last_vcsec_poll_{0};
    uint32_t last_infotainment_poll_{0};

    espbt::ESPBTUUID service_uuid_;
    espbt::ESPBTUUID read_uuid_;
    espbt::ESPBTUUID write_uuid_;
    uint16_t read_handle_{0};
    uint16_t write_handle_{0};

    std::map<std::string, binary_sensor::BinarySensor*> pending_binary_sensors_;
    std::map<std::string, sensor::Sensor*> pending_sensors_;
    std::map<std::string, text_sensor::TextSensor*> pending_text_sensors_;
    
    switch_::Switch *pending_charging_switch_{nullptr};
    switch_::Switch *pending_steering_wheel_heat_switch_{nullptr};
    switch_::Switch *pending_sentry_mode_switch_{nullptr};
    number::Number *pending_charging_amps_number_{nullptr};
    number::Number *pending_charging_limit_number_{nullptr};
    lock::Lock *pending_doors_lock_{nullptr};
    lock::Lock *pending_charge_port_latch_lock_{nullptr};
    cover::Cover *pending_trunk_cover_{nullptr};
    cover::Cover *pending_frunk_cover_{nullptr};
    cover::Cover *pending_windows_cover_{nullptr};
    cover::Cover *pending_charge_port_door_cover_{nullptr};
    climate::Climate *pending_climate_{nullptr};
    
    std::string last_rx_hex_;
    friend class VehicleStateManager;
};

class TeslaButtonBase : public button::Button {
public:
    void set_parent(TeslaBLEVehicle *parent) { parent_ = parent; }
protected:
    TeslaBLEVehicle *parent_{nullptr};
};

#define DEFINE_TESLA_BUTTON(ClassName, ParentMethod) \
    class ClassName : public TeslaButtonBase { \
    protected: \
        void press_action() override { if (parent_) parent_->ParentMethod(); } \
    };

DEFINE_TESLA_BUTTON(TeslaWakeButton, wake_vehicle)
DEFINE_TESLA_BUTTON(TeslaPairButton, start_pairing)
DEFINE_TESLA_BUTTON(TeslaRegenerateKeyButton, regenerate_key)
DEFINE_TESLA_BUTTON(TeslaForceUpdateButton, force_update)
DEFINE_TESLA_BUTTON(TeslaFlashLightsButton, flash_lights)
DEFINE_TESLA_BUTTON(TeslaHonkHornButton, honk_horn)
DEFINE_TESLA_BUTTON(TeslaUnlatchDriverDoorButton, unlatch_driver_door)
DEFINE_TESLA_BUTTON(TeslaStartDrivingButton, start_driving)

class TeslaSwitchBase : public switch_::Switch {
public:
    void set_parent(TeslaBLEVehicle *parent) { parent_ = parent; }
protected:
    TeslaBLEVehicle *parent_{nullptr};
};

#define DEFINE_TESLA_SWITCH(ClassName, ParentMethod) \
    class ClassName : public TeslaSwitchBase { \
    protected: \
        void write_state(bool state) override { \
            if (parent_) { parent_->ParentMethod(state); publish_state(state); } \
        } \
    };

DEFINE_TESLA_SWITCH(TeslaChargingSwitch, set_charging_state)
DEFINE_TESLA_SWITCH(TeslaSteeringWheelHeatSwitch, set_steering_wheel_heat)
DEFINE_TESLA_SWITCH(TeslaSentryModeSwitch, set_sentry_mode)

class TeslaChargingAmpsNumber : public number::Number {
public:
    void set_parent(TeslaBLEVehicle *parent) { parent_ = parent; }
    void update_max_value(int32_t new_max);
protected:
    void control(float value) override;
    TeslaBLEVehicle *parent_{nullptr};
};

class TeslaChargingLimitNumber : public number::Number {
public:
    void set_parent(TeslaBLEVehicle *parent) { parent_ = parent; }
protected:
    void control(float value) override;
    TeslaBLEVehicle *parent_{nullptr};
};

class TeslaLockBase : public lock::Lock {
public:
    void set_parent(TeslaBLEVehicle *parent) { parent_ = parent; }
protected:
    TeslaBLEVehicle *parent_{nullptr};
};

class TeslaDoorsLock : public TeslaLockBase { protected: void control(const lock::LockCall &call) override; };
class TeslaChargePortLatchLock : public TeslaLockBase { protected: void control(const lock::LockCall &call) override; };

class TeslaCoverBase : public cover::Cover {
public:
    void set_parent(TeslaBLEVehicle *parent) { parent_ = parent; }
    cover::CoverTraits get_traits() override;
protected:
    TeslaBLEVehicle *parent_{nullptr};
};

class TeslaTrunkCover : public TeslaCoverBase { protected: void control(const cover::CoverCall &call) override; };
class TeslaFrunkCover : public TeslaCoverBase { protected: void control(const cover::CoverCall &call) override; };
class TeslaWindowsCover : public TeslaCoverBase { protected: void control(const cover::CoverCall &call) override; };
class TeslaChargePortDoorCover : public TeslaCoverBase { protected: void control(const cover::CoverCall &call) override; };

class TeslaClimate : public climate::Climate {
public:
    void set_parent(TeslaBLEVehicle *parent) { parent_ = parent; }
    climate::ClimateTraits traits() override;
    void control(const climate::ClimateCall &call) override;
    void update_state(bool is_on, float current_temp, float target_temp);
protected:
    TeslaBLEVehicle *parent_{nullptr};
};

// Automation Actions
template<typename... Ts> class WakeAction : public Action<Ts...> {
public: WakeAction(TeslaBLEVehicle *parent) : parent_(parent) {} void play(Ts... x) override { parent_->wake_vehicle(); } protected: TeslaBLEVehicle *parent_;
};
template<typename... Ts> class PairAction : public Action<Ts...> {
public: PairAction(TeslaBLEVehicle *parent) : parent_(parent) {} void play(Ts... x) override { parent_->start_pairing(); } protected: TeslaBLEVehicle *parent_;
};
template<typename... Ts> class RegenerateKeyAction : public Action<Ts...> {
public: RegenerateKeyAction(TeslaBLEVehicle *parent) : parent_(parent) {} void play(Ts... x) override { parent_->regenerate_key(); } protected: TeslaBLEVehicle *parent_;
};
template<typename... Ts> class ForceUpdateAction : public Action<Ts...> {
public: ForceUpdateAction(TeslaBLEVehicle *parent) : parent_(parent) {} void play(Ts... x) override { parent_->force_update(); } protected: TeslaBLEVehicle *parent_;
};
template<typename... Ts> class SetChargingAction : public Action<Ts...> {
public: SetChargingAction(TeslaBLEVehicle *parent) : parent_(parent) {} void set_state(esphome::TemplatableValue<bool, Ts...> state) { state_ = state; } void play(Ts... x) override { parent_->set_charging_state(state_.value(x...)); } protected: TeslaBLEVehicle *parent_; esphome::TemplatableValue<bool, Ts...> state_;
};
template<typename... Ts> class SetChargingAmpsAction : public Action<Ts...> {
public: SetChargingAmpsAction(TeslaBLEVehicle *parent) : parent_(parent) {} void set_amps(esphome::TemplatableValue<int, Ts...> amps) { amps_ = amps; } void play(Ts... x) override { parent_->set_charging_amps(amps_.value(x...)); } protected: TeslaBLEVehicle *parent_; esphome::TemplatableValue<int, Ts...> amps_;
};
template<typename... Ts> class SetChargingLimitAction : public Action<Ts...> {
public: SetChargingLimitAction(TeslaBLEVehicle *parent) : parent_(parent) {} void set_limit(esphome::TemplatableValue<int, Ts...> limit) { limit_ = limit; } void play(Ts... x) override { parent_->set_charging_limit(limit_.value(x...)); } protected: TeslaBLEVehicle *parent_; esphome::TemplatableValue<int, Ts...> limit_;
};

template<typename... Ts> class StartDrivingAction : public Action<Ts...> {
public:
    explicit StartDrivingAction(TeslaBLEVehicle *parent) : parent_(parent) {}
    void play(Ts... x) override { parent_->start_driving(); }
protected:
    TeslaBLEVehicle *parent_;
};

} // namespace tesla_ble_vehicle
} // namespace esphome

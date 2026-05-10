#include "vehicle_state_manager.h"
#include "tesla_ble_vehicle.h"
#include <esphome/core/helpers.h>
#include <cmath>
#include <algorithm>

namespace esphome {
namespace tesla_ble_vehicle {

VehicleStateManager::VehicleStateManager(TeslaBLEVehicle* parent)
    : parent_(parent) {}

void VehicleStateManager::set_binary_sensor(const std::string& id, binary_sensor::BinarySensor* sensor) {
    if (sensor == nullptr) return;
    binary_sensors_[id] = sensor;
    ESP_LOGD(STATE_MANAGER_TAG, "Registered binary sensor: %s", id.c_str());
}

void VehicleStateManager::set_sensor(const std::string& id, sensor::Sensor* sensor) {
    if (sensor == nullptr) return;
    sensors_[id] = sensor;
    ESP_LOGD(STATE_MANAGER_TAG, "Registered sensor: %s", id.c_str());
}

void VehicleStateManager::set_text_sensor(const std::string& id, text_sensor::TextSensor* sensor) {
    if (sensor == nullptr) return;
    text_sensors_[id] = sensor;
    ESP_LOGD(STATE_MANAGER_TAG, "Registered text sensor: %s", id.c_str());
}

binary_sensor::BinarySensor* VehicleStateManager::get_binary_sensor(const std::string& id) {
    auto it = binary_sensors_.find(id);
    return (it != binary_sensors_.end()) ? it->second : nullptr;
}

sensor::Sensor* VehicleStateManager::get_sensor(const std::string& id) {
    auto it = sensors_.find(id);
    return (it != sensors_.end()) ? it->second : nullptr;
}

text_sensor::TextSensor* VehicleStateManager::get_text_sensor(const std::string& id) {
    auto it = text_sensors_.find(id);
    return (it != text_sensors_.end()) ? it->second : nullptr;
}

const binary_sensor::BinarySensor* VehicleStateManager::get_binary_sensor(const std::string& id) const {
    auto it = binary_sensors_.find(id);
    return (it != binary_sensors_.end()) ? it->second : nullptr;
}

const sensor::Sensor* VehicleStateManager::get_sensor(const std::string& id) const {
    auto it = sensors_.find(id);
    return (it != sensors_.end()) ? it->second : nullptr;
}

const text_sensor::TextSensor* VehicleStateManager::get_text_sensor(const std::string& id) const {
    auto it = text_sensors_.find(id);
    return (it != text_sensors_.end()) ? it->second : nullptr;
}

bool VehicleStateManager::publish_binary_sensor(const std::string& id, bool state) {
    auto* sensor = get_binary_sensor(id);
    return sensor != nullptr && publish_sensor_state(sensor, state);
}

bool VehicleStateManager::publish_sensor(const std::string& id, float state) {
    auto* sensor = get_sensor(id);
    return sensor != nullptr && publish_sensor_state(sensor, state);
}

bool VehicleStateManager::publish_text_sensor(const std::string& id, const std::string& state) {
    auto* sensor = get_text_sensor(id);
    return sensor != nullptr && publish_sensor_state(sensor, state);
}

void VehicleStateManager::update_vehicle_status(const VCSEC_VehicleStatus& status) {
    update_sleep_status(status.vehicleSleepStatus);
    update_lock_status(status.vehicleLockState);
    update_user_presence(status.userPresence);
    if (status.has_closureStatuses) {
        bool flap_open = (status.closureStatuses.chargePort == VCSEC_ClosureState_E_CLOSURESTATE_OPEN);
        update_charge_flap_open(flap_open);
    }
}

void VehicleStateManager::update_sleep_status(VCSEC_VehicleSleepStatus_E status) {
    auto asleep = convert_sleep_status(status);
    if (asleep.has_value()) update_asleep(asleep.value());
    else set_sensor_available(get_binary_sensor("asleep"), false);
}

void VehicleStateManager::update_lock_status(VCSEC_VehicleLockState_E status) {
    auto unlocked = convert_lock_status(status);
    if (unlocked.has_value()) update_unlocked(unlocked.value());
}

void VehicleStateManager::update_user_presence(VCSEC_UserPresence_E presence) {
    auto present = convert_user_presence(presence);
    if (present.has_value()) update_user_present(present.value());
    else set_sensor_available(get_binary_sensor("user_present"), false);
}

void VehicleStateManager::update_charge_state(const CarServer_ChargeState& charge_state) {
    // ... 保持原有逻辑不变 ...
    ESP_LOGD(STATE_MANAGER_TAG, "Updating charge state");
    if (charge_state.has_charging_state) {
        const bool was_charging = is_charging_;
        const bool new_charging_state = (
            charge_state.charging_state.which_type == CarServer_ChargeState_ChargingState_Charging_tag ||
            charge_state.charging_state.which_type == CarServer_ChargeState_ChargingState_Starting_tag
        );
        is_charging_ = new_charging_state;
        if (charging_switch_ && (!charging_switch_->has_state() || charging_switch_->state != is_charging_)) {
            publish_sensor_state(charging_switch_, is_charging_);
        }
        publish_text_sensor("charging_state", get_charging_state_text(charge_state.charging_state));
        publish_text_sensor("iec61851_state", get_iec61851_state_text(charge_state.charging_state));
        publish_binary_sensor("charger", is_charger_connected_from_state(charge_state.charging_state));
    }
    if (charge_state.which_optional_battery_level) {
        float bl = static_cast<float>(charge_state.optional_battery_level.battery_level);
        if (bl >= 0 && bl <= 100 && std::isfinite(bl)) publish_sensor("battery_level", bl);
    }
    if (charge_state.which_optional_charger_power) {
        float pw = static_cast<float>(charge_state.optional_charger_power.charger_power);
        if (pw >= 0 && pw <= 500 && std::isfinite(pw)) publish_sensor("charger_power", pw);
    }
    if (charge_state.which_optional_battery_range) {
        float range = charge_state.optional_battery_range.battery_range;
        if (range >= 0 && range <= 500 && std::isfinite(range)) publish_sensor("range", range);
    }
    if (charge_state.which_optional_charge_energy_added) {
        float energy = charge_state.optional_charge_energy_added.charge_energy_added;
        if (energy >= 0 && std::isfinite(energy)) publish_sensor("energy_added", energy);
    }
    if (charge_state.which_optional_minutes_to_full_charge) {
        float min = static_cast<float>(charge_state.optional_minutes_to_full_charge.minutes_to_full_charge);
        if (min >= 0 && std::isfinite(min)) publish_sensor("time_to_full", min);
    }
    if (charge_state.which_optional_charger_voltage) {
        float volt = static_cast<float>(charge_state.optional_charger_voltage.charger_voltage);
        if (volt >= 0 && volt <= 600 && std::isfinite(volt)) publish_sensor("charger_voltage", volt);
    }
    if (charge_state.which_optional_charger_actual_current) {
        float cur = static_cast<float>(charge_state.optional_charger_actual_current.charger_actual_current);
        if (cur >= 0 && cur <= 100 && std::isfinite(cur)) publish_sensor("charger_current", cur);
    }
    if (charge_state.which_optional_charge_rate_mph) {
        float rate = static_cast<float>(charge_state.optional_charge_rate_mph.charge_rate_mph);
        publish_sensor("charging_rate", rate);
    }
    if (charge_state.which_optional_charger_actual_current && charging_amps_number_) {
        float amps = static_cast<float>(charge_state.optional_charger_actual_current.charger_actual_current);
        update_charging_amps(amps);
    }
    if (charge_state.which_optional_charge_limit_soc && charging_limit_number_) {
        float limit = static_cast<float>(charge_state.optional_charge_limit_soc.charge_limit_soc);
        publish_sensor_state(charging_limit_number_, limit);
    }
    if (charge_state.which_optional_charge_current_request_max) {
        int32_t new_max = charge_state.optional_charge_current_request_max.charge_current_request_max;
        if (new_max > 0 && new_max != charging_amps_max_) {
            update_charging_amps_max(new_max);
        }
    }
    if (charge_state.which_optional_charge_port_door_open) {
        bool door_open = charge_state.optional_charge_port_door_open.charge_port_door_open;
        if (charge_port_door_cover_) {
            charge_port_door_cover_->position = door_open ? cover::COVER_OPEN : cover::COVER_CLOSED;
            charge_port_door_cover_->publish_state();
        }
    }
    if (charge_state.has_charge_port_latch) {
        const bool latch_engaged = (charge_state.charge_port_latch.which_type == CarServer_ChargePortLatchState_Engaged_tag);
        const bool latch_disengaged = (charge_state.charge_port_latch.which_type == CarServer_ChargePortLatchState_Disengaged_tag);
        if (charge_port_latch_lock_ != nullptr && (latch_engaged || latch_disengaged)) {
            auto new_state = latch_engaged ? lock::LOCK_STATE_LOCKED : lock::LOCK_STATE_UNLOCKED;
            if (charge_port_latch_lock_->state != new_state) {
                charge_port_latch_lock_->publish_state(new_state);
            }
        }
    }
}

void VehicleStateManager::update_climate_state(const CarServer_ClimateState& climate_state) {
    ESP_LOGD(STATE_MANAGER_TAG, "Updating climate state");
    if (climate_state.which_optional_inside_temp_celsius) {
        current_inside_temp_ = climate_state.optional_inside_temp_celsius.inside_temp_celsius;
        if (current_inside_temp_ >= -40 && current_inside_temp_ <= 60 && std::isfinite(current_inside_temp_))
            publish_sensor("inside_temp", current_inside_temp_);
    }
    if (climate_state.which_optional_outside_temp_celsius) {
        float temp = climate_state.optional_outside_temp_celsius.outside_temp_celsius;
        if (temp >= -50 && temp <= 60 && std::isfinite(temp)) publish_sensor("outside_temp", temp);
    }
    if (climate_state.which_optional_driver_temp_setting) {
        target_temp_ = climate_state.optional_driver_temp_setting.driver_temp_setting;
    }
    if (climate_state.which_optional_is_climate_on) {
        climate_on_ = climate_state.optional_is_climate_on.is_climate_on;
        publish_binary_sensor("climate_on", climate_on_);
    }
    if (climate_state.which_optional_steering_wheel_heater && steering_wheel_heat_switch_) {
        bool heater = climate_state.optional_steering_wheel_heater.steering_wheel_heater;
        if (!steering_wheel_heat_switch_->has_state() || steering_wheel_heat_switch_->state != heater) {
            publish_sensor_state(steering_wheel_heat_switch_, heater);
        }
    }
    if (auto* tesla_climate = static_cast<TeslaClimate*>(climate_)) {
        tesla_climate->update_state(climate_on_, current_inside_temp_, target_temp_);
    }
}

// ★ 修改后的 update_drive_state（新增速度和功率）
void VehicleStateManager::update_drive_state(const CarServer_DriveState& drive_state) {
    ESP_LOGD(STATE_MANAGER_TAG, "Updating drive state");

    if (drive_state.has_shift_state) {
        publish_text_sensor("shift_state", get_shift_state_text(drive_state.shift_state));
        const bool parked = (drive_state.shift_state.which_type == CarServer_ShiftState_P_tag);
        publish_binary_sensor("parking_brake", parked);
    }

    if (drive_state.which_optional_odometer_in_hundredths_of_a_mile) {
        const float odometer = static_cast<float>(drive_state.optional_odometer_in_hundredths_of_a_mile.odometer_in_hundredths_of_a_mile) / 100.0f;
        if (odometer >= 0.0f && std::isfinite(odometer)) {
            publish_sensor("odometer", odometer);
        }
    }

    // 速度 (km/h)
    if (drive_state.has_speed_kmh) {
        float speed = drive_state.speed_kmh;
        if (speed >= 0 && std::isfinite(speed)) {
            publish_sensor("speed", speed);
        }
    }

    // 功率 (kW)
    if (drive_state.has_power_kw) {
        float power = drive_state.power_kw;
        if (power >= 0 && std::isfinite(power)) {
            publish_sensor("power", power);
        }
    }
}

void VehicleStateManager::update_tire_pressure_state(const CarServer_TirePressureState& tire_pressure_state) {
    if (tire_pressure_state.which_optional_tpms_pressure_fl) publish_sensor("tpms_front_left", tire_pressure_state.optional_tpms_pressure_fl.tpms_pressure_fl);
    if (tire_pressure_state.which_optional_tpms_pressure_fr) publish_sensor("tpms_front_right", tire_pressure_state.optional_tpms_pressure_fr.tpms_pressure_fr);
    if (tire_pressure_state.which_optional_tpms_pressure_rl) publish_sensor("tpms_rear_left", tire_pressure_state.optional_tpms_pressure_rl.tpms_pressure_rl);
    if (tire_pressure_state.which_optional_tpms_pressure_rr) publish_sensor("tpms_rear_right", tire_pressure_state.optional_tpms_pressure_rr.tpms_pressure_rr);
}

void VehicleStateManager::update_closures_state(const CarServer_ClosuresState& closures_state) {
    // ... 原有代码保持不变 ...
    ESP_LOGD(STATE_MANAGER_TAG, "Updating closures state");
    if (closures_state.which_optional_door_open_driver_front) publish_binary_sensor("door_driver_front", closures_state.optional_door_open_driver_front.door_open_driver_front);
    if (closures_state.which_optional_door_open_driver_rear) publish_binary_sensor("door_driver_rear", closures_state.optional_door_open_driver_rear.door_open_driver_rear);
    if (closures_state.which_optional_door_open_passenger_front) publish_binary_sensor("door_passenger_front", closures_state.optional_door_open_passenger_front.door_open_passenger_front);
    if (closures_state.which_optional_door_open_passenger_rear) publish_binary_sensor("door_passenger_rear", closures_state.optional_door_open_passenger_rear.door_open_passenger_rear);
    if (closures_state.which_optional_door_open_trunk_front) {
        bool frunk_open = closures_state.optional_door_open_trunk_front.door_open_trunk_front;
        if (frunk_cover_) { frunk_cover_->position = frunk_open ? cover::COVER_OPEN : cover::COVER_CLOSED; frunk_cover_->publish_state(); }
    }
    if (closures_state.which_optional_door_open_trunk_rear) {
        bool trunk_open = closures_state.optional_door_open_trunk_rear.door_open_trunk_rear;
        if (trunk_cover_) { trunk_cover_->position = trunk_open ? cover::COVER_OPEN : cover::COVER_CLOSED; trunk_cover_->publish_state(); }
    }
    bool window_df = false, window_dr = false, window_pf = false, window_pr = false;
    if (closures_state.which_optional_window_open_driver_front) { window_df = closures_state.optional_window_open_driver_front.window_open_driver_front; publish_binary_sensor("window_driver_front", window_df); }
    if (closures_state.which_optional_window_open_driver_rear) { window_dr = closures_state.optional_window_open_driver_rear.window_open_driver_rear; publish_binary_sensor("window_driver_rear", window_dr); }
    if (closures_state.which_optional_window_open_passenger_front) { window_pf = closures_state.optional_window_open_passenger_front.window_open_passenger_front; publish_binary_sensor("window_passenger_front", window_pf); }
    if (closures_state.which_optional_window_open_passenger_rear) { window_pr = closures_state.optional_window_open_passenger_rear.window_open_passenger_rear; publish_binary_sensor("window_passenger_rear", window_pr); }
    const bool any_window_open = window_df || window_dr || window_pf || window_pr;
    if (windows_cover_) { windows_cover_->position = any_window_open ? cover::COVER_OPEN : cover::COVER_CLOSED; windows_cover_->publish_state(); }
    if (closures_state.which_optional_sun_roof_percent_open) {
        bool sunroof_open = closures_state.optional_sun_roof_percent_open.sun_roof_percent_open > 0;
        publish_binary_sensor("sunroof", sunroof_open);
    }
    if (closures_state.has_sentry_mode_state && sentry_mode_switch_) {
        bool sentry_active = (closures_state.sentry_mode_state.which_type == CarServer_ClosuresState_SentryModeState_Armed_tag ||
                              closures_state.sentry_mode_state.which_type == CarServer_ClosuresState_SentryModeState_Aware_tag ||
                              closures_state.sentry_mode_state.which_type == CarServer_ClosuresState_SentryModeState_Panic_tag);
        if (!sentry_mode_switch_->has_state() || sentry_mode_switch_->state != sentry_active) publish_sensor_state(sentry_mode_switch_, sentry_active);
    }
    if (closures_state.which_optional_locked) update_unlocked(!closures_state.optional_locked.locked);
}

// ... 其余函数（update_asleep, update_unlocked, 辅助方法等）保持不变 ...
void VehicleStateManager::update_asleep(bool asleep) {
    if (publish_binary_sensor("asleep", asleep)) ESP_LOGI(STATE_MANAGER_TAG, "Vehicle sleep: %s", asleep ? "ASLEEP" : "AWAKE");
}
void VehicleStateManager::update_unlocked(bool unlocked) {
    if (doors_lock_) {
        auto new_state = unlocked ? lock::LOCK_STATE_UNLOCKED : lock::LOCK_STATE_LOCKED;
        if (doors_lock_->state != new_state) { doors_lock_->publish_state(new_state); ESP_LOGI(STATE_MANAGER_TAG, "Lock: %s", unlocked ? "UNLOCKED" : "LOCKED"); }
    }
}
void VehicleStateManager::update_user_present(bool present) {
    if (publish_binary_sensor("user_present", present)) ESP_LOGI(STATE_MANAGER_TAG, "User: %s", present ? "PRESENT" : "NOT");
    is_user_present_ = present;
}
void VehicleStateManager::update_charge_flap_open(bool open) {
    if (charge_port_door_cover_) { charge_port_door_cover_->position = open ? cover::COVER_OPEN : cover::COVER_CLOSED; charge_port_door_cover_->publish_state(); }
}
void VehicleStateManager::update_charging_amps(float amps) { publish_sensor_state(charging_amps_number_, amps); }
void VehicleStateManager::update_charger_connected(bool connected) { publish_binary_sensor("charger", connected); }
void VehicleStateManager::set_sensors_available(bool available) {
    set_sensor_available(get_binary_sensor("asleep"), available);
    set_sensor_available(get_binary_sensor("user_present"), available);
}
void VehicleStateManager::reset_all_states() { is_charging_ = false; set_sensors_available(false); }
bool VehicleStateManager::is_asleep() const { auto* s = get_binary_sensor("asleep"); return s ? s->state : true; }
bool VehicleStateManager::is_unlocked() const { return doors_lock_ ? doors_lock_->state == lock::LOCK_STATE_UNLOCKED : false; }
bool VehicleStateManager::is_user_present() const { auto* s = get_binary_sensor("user_present"); return s ? s->state : false; }
bool VehicleStateManager::is_charge_flap_open() const { return charge_port_door_cover_ ? charge_port_door_cover_->position == cover::COVER_OPEN : false; }
float VehicleStateManager::get_charging_amps() const { return charging_amps_number_ ? charging_amps_number_->state : 0.0f; }
void VehicleStateManager::update_charging_amps_max(int32_t new_max) {
    if (new_max <= 0) return;
    if (new_max == charging_amps_max_) return;
    charging_amps_max_ = new_max;
    if (charging_amps_number_ && parent_) parent_->update_charging_amps_max_value(new_max);
}
void VehicleStateManager::track_command_issued() {}
bool VehicleStateManager::publish_sensor_state(binary_sensor::BinarySensor* sensor, bool state) {
    if (sensor && (!sensor->has_state() || sensor->state != state)) { sensor->publish_state(state); return true; }
    return false;
}
bool VehicleStateManager::publish_sensor_state(sensor::Sensor* sensor, float state) {
    if (sensor && (!sensor->has_state() || std::abs(sensor->state - state) > 0.001f)) { sensor->publish_state(state); return true; }
    return false;
}
bool VehicleStateManager::publish_sensor_state(switch_::Switch* sw, bool state) {
    if (sw && (!sw->has_state() || sw->state != state)) { sw->publish_state(state); return true; }
    return false;
}
bool VehicleStateManager::publish_sensor_state(number::Number* num, float state) {
    if (num && (!num->has_state() || std::abs(num->state - state) > 0.001f)) { num->publish_state(state); return true; }
    return false;
}
bool VehicleStateManager::publish_sensor_state(text_sensor::TextSensor* sensor, const std::string& state) {
    if (sensor && (!sensor->has_state() || sensor->state != state)) { sensor->publish_state(state); return true; }
    return false;
}
void VehicleStateManager::set_sensor_available(binary_sensor::BinarySensor* sensor, bool available) { if (sensor) sensor->set_has_state(available); }
void VehicleStateManager::set_sensor_available(sensor::Sensor* sensor, bool available) { if (sensor) sensor->set_has_state(available); }

std::optional<bool> VehicleStateManager::convert_sleep_status(VCSEC_VehicleSleepStatus_E status) {
    switch (status) {
        case VCSEC_VehicleSleepStatus_E_VEHICLE_SLEEP_STATUS_AWAKE: return false;
        case VCSEC_VehicleSleepStatus_E_VEHICLE_SLEEP_STATUS_ASLEEP: return true;
        default: return std::nullopt;
    }
}
std::optional<bool> VehicleStateManager::convert_lock_status(VCSEC_VehicleLockState_E status) {
    switch (status) {
        case VCSEC_VehicleLockState_E_VEHICLELOCKSTATE_UNLOCKED:
        case VCSEC_VehicleLockState_E_VEHICLELOCKSTATE_SELECTIVE_UNLOCKED: return true;
        case VCSEC_VehicleLockState_E_VEHICLELOCKSTATE_LOCKED:
        case VCSEC_VehicleLockState_E_VEHICLELOCKSTATE_INTERNAL_LOCKED: return false;
        default: return std::nullopt;
    }
}
std::optional<bool> VehicleStateManager::convert_user_presence(VCSEC_UserPresence_E presence) {
    switch (presence) {
        case VCSEC_UserPresence_E_VEHICLE_USER_PRESENCE_PRESENT: return true;
        case VCSEC_UserPresence_E_VEHICLE_USER_PRESENCE_NOT_PRESENT: return false;
        default: return std::nullopt;
    }
}
std::string VehicleStateManager::get_charging_state_text(const CarServer_ChargeState_ChargingState& state) {
    switch (state.which_type) {
        case CarServer_ChargeState_ChargingState_Disconnected_tag: return "Disconnected";
        case CarServer_ChargeState_ChargingState_NoPower_tag: return "No Power";
        case CarServer_ChargeState_ChargingState_Starting_tag: return "Starting";
        case CarServer_ChargeState_ChargingState_Charging_tag: return "Charging";
        case CarServer_ChargeState_ChargingState_Complete_tag: return "Complete";
        case CarServer_ChargeState_ChargingState_Stopped_tag: return "Stopped";
        case CarServer_ChargeState_ChargingState_Calibrating_tag: return "Calibrating";
        default: return "Unknown";
    }
}
bool VehicleStateManager::is_charger_connected_from_state(const CarServer_ChargeState_ChargingState& state) {
    return state.which_type != CarServer_ChargeState_ChargingState_Disconnected_tag &&
           state.which_type != CarServer_ChargeState_ChargingState_Unknown_tag;
}
std::string VehicleStateManager::get_iec61851_state_text(const CarServer_ChargeState_ChargingState& state) {
    switch (state.which_type) {
        case CarServer_ChargeState_ChargingState_Disconnected_tag: return "A";
        case CarServer_ChargeState_ChargingState_NoPower_tag: return "E";
        case CarServer_ChargeState_ChargingState_Starting_tag: return "C";
        case CarServer_ChargeState_ChargingState_Charging_tag: return "C";
        case CarServer_ChargeState_ChargingState_Complete_tag: return "B";
        case CarServer_ChargeState_ChargingState_Stopped_tag: return "B";
        case CarServer_ChargeState_ChargingState_Calibrating_tag: return "C";
        default: return "F";
    }
}
std::string VehicleStateManager::get_shift_state_text(const CarServer_ShiftState& state) {
    switch (state.which_type) {
        case CarServer_ShiftState_P_tag: return "P";
        case CarServer_ShiftState_R_tag: return "R";
        case CarServer_ShiftState_N_tag: return "N";
        case CarServer_ShiftState_D_tag: return "D";
        case CarServer_ShiftState_SNA_tag: return "SNA";
        case CarServer_ShiftState_Invalid_tag: return "Invalid";
        default: return "Unknown";
    }
}

} // namespace tesla_ble_vehicle
} // namespace esphome

#include "vehicle_state_manager.h"
#include "tesla_ble_vehicle.h"
#include <esphome/core/helpers.h>
#include <cmath>
#include <algorithm>

namespace esphome {
namespace tesla_ble_vehicle {

VehicleStateManager::VehicleStateManager(TeslaBLEVehicle* parent)
    : parent_(parent) {}

// ... (set_binary_sensor, set_sensor, set_text_sensor, get_* 等函数保持不变，省略)

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

// =============================================================================
// VCSEC State Updates
// =============================================================================

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

// =============================================================================
// CarServer State Updates
// =============================================================================

void VehicleStateManager::update_charge_state(const CarServer_ChargeState& charge_state) {
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

    // ... 其余充电状态更新（amps number, limit number, max amps, door/latch）保持不变，省略
}

void VehicleStateManager::update_climate_state(const CarServer_ClimateState& climate_state) {
    ESP_LOGD(STATE_MANAGER_TAG, "Updating climate state");

    // 车内温度
    if (climate_state.which_optional_inside_temp_celsius) {
        current_inside_temp_ = climate_state.optional_inside_temp_celsius.inside_temp_celsius;
        if (current_inside_temp_ >= -40 && current_inside_temp_ <= 60 && std::isfinite(current_inside_temp_)) {
            publish_sensor("inside_temp", current_inside_temp_);
        }
    }
    // 车外温度
    if (climate_state.which_optional_outside_temp_celsius) {
        float temp = climate_state.optional_outside_temp_celsius.outside_temp_celsius;
        if (temp >= -50 && temp <= 60 && std::isfinite(temp)) publish_sensor("outside_temp", temp);
    }
    // 目标温度
    if (climate_state.which_optional_driver_temp_setting) {
        target_temp_ = climate_state.optional_driver_temp_setting.driver_temp_setting;
    }
    // 空调开关状态
    if (climate_state.which_optional_is_climate_on) {
        climate_on_ = climate_state.optional_is_climate_on.is_climate_on;
        publish_binary_sensor("climate_on", climate_on_);
    }
    // 方向盘加热（同步 switch）
    if (climate_state.which_optional_steering_wheel_heater && steering_wheel_heat_switch_) {
        bool heater = climate_state.optional_steering_wheel_heater.steering_wheel_heater;
        if (!steering_wheel_heat_switch_->has_state() || steering_wheel_heat_switch_->state != heater) {
            publish_sensor_state(steering_wheel_heat_switch_, heater);
        }
    }

    // 更新 climate 实体
    if (auto* tesla_climate = static_cast<TeslaClimate*>(climate_)) {
        tesla_climate->update_state(climate_on_, current_inside_temp_, target_temp_);
    }
}

void VehicleStateManager::update_drive_state(const CarServer_DriveState& drive_state) {
    ESP_LOGD(STATE_MANAGER_TAG, "Updating drive state");

    // 档位文本传感器
    if (drive_state.has_shift_state) {
        publish_text_sensor("shift_state", get_shift_state_text(drive_state.shift_state));
        bool parked = (drive_state.shift_state.which_type == CarServer_ShiftState_P_tag);
        publish_binary_sensor("parking_brake", parked);
    }

    // 里程
    if (drive_state.which_optional_odometer_in_hundredths_of_a_mile) {
        float odo = static_cast<float>(drive_state.optional_odometer_in_hundredths_of_a_mile.odometer_in_hundredths_of_a_mile) / 100.0f;
        if (odo >= 0 && std::isfinite(odo)) publish_sensor("odometer", odo);
    }

    // ★ 新增：速度（字段名可能需要根据实际 protobuf 调整）
    if (drive_state.which_optional_speed) {
        float speed = drive_state.optional_speed.speed;
        if (speed >= 0 && std::isfinite(speed)) publish_sensor("speed", speed);
    }

    // ★ 新增：功率（字段名可能需要根据实际 protobuf 调整）
    if (drive_state.which_optional_power) {
        float power = drive_state.optional_power.power;
        if (power >= 0 && std::isfinite(power)) publish_sensor("power", power);
    }
}

void VehicleStateManager::update_tire_pressure_state(const CarServer_TirePressureState& tire_state) {
    if (tire_state.which_optional_tpms_pressure_fl) publish_sensor("tpms_front_left", tire_state.optional_tpms_pressure_fl.tpms_pressure_fl);
    if (tire_state.which_optional_tpms_pressure_fr) publish_sensor("tpms_front_right", tire_state.optional_tpms_pressure_fr.tpms_pressure_fr);
    if (tire_state.which_optional_tpms_pressure_rl) publish_sensor("tpms_rear_left", tire_state.optional_tpms_pressure_rl.tpms_pressure_rl);
    if (tire_state.which_optional_tpms_pressure_rr) publish_sensor("tpms_rear_right", tire_state.optional_tpms_pressure_rr.tpms_pressure_rr);
}

void VehicleStateManager::update_closures_state(const CarServer_ClosuresState& closures_state) {
    // ... 保持原有逻辑不变，省略
}

// 其余方法（update_asleep, update_unlocked, convert_* 等）均保持不变，省略

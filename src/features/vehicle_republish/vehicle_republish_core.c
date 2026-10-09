#include "features/vehicle_republish/vehicle_republish_core.h"

#include <stddef.h>

static bool sample_usable(const vehicle_signal_sample_t* s, bool (*in_phys_range)(float))
{
    return (s != NULL) && (s->state == VEHICLE_SIGNAL_VALID) && in_phys_range(s->physical);
}

void vehicle_republish_speed_msg(const vehicle_signal_sample_t* speed,
                                 struct platform_vehicle_speed_t* msg)
{
    (void)platform_vehicle_speed_init(msg);
    if (sample_usable(speed, platform_vehicle_speed_vehicle_speed_is_in_phys_range)) {
        msg->vehicle_speed_valid = PLATFORM_VEHICLE_SPEED_VEHICLE_SPEED_VALID_VALID_CHOICE;
        msg->vehicle_speed = platform_vehicle_speed_vehicle_speed_encode(speed->physical);
        msg->vehicle_speed_age =
            platform_vehicle_speed_vehicle_speed_age_encode((float)speed->age_ms);
    } else {
        msg->vehicle_speed_valid = PLATFORM_VEHICLE_SPEED_VEHICLE_SPEED_VALID_INVALID_CHOICE;
        msg->vehicle_speed = 0u;
        msg->vehicle_speed_age =
            platform_vehicle_speed_vehicle_speed_age_encode((float)VEHICLE_REPUBLISH_AGE_SATURATED_MS);
    }
}

void vehicle_republish_engine_msg(const vehicle_signal_sample_t samples[VEHICLE_CL250_DID_COUNT],
                                  bool ecu_present, struct platform_vehicle_engine_t* msg)
{
    (void)platform_vehicle_engine_init(msg);
    const vehicle_signal_sample_t* rpm = &samples[VEHICLE_CL250_IDX_ENGINE_SPEED];
    const vehicle_signal_sample_t* batt = &samples[VEHICLE_CL250_IDX_BATTERY_VOLTAGE];
    const vehicle_signal_sample_t* cool = &samples[VEHICLE_CL250_IDX_COOLANT_TEMP];
    const vehicle_signal_sample_t* tps = &samples[VEHICLE_CL250_IDX_THROTTLE_POS];

    if (sample_usable(rpm, platform_vehicle_engine_engine_speed_is_in_phys_range)) {
        msg->engine_speed = platform_vehicle_engine_engine_speed_encode(rpm->physical);
        msg->engine_speed_valid = PLATFORM_VEHICLE_ENGINE_ENGINE_SPEED_VALID_VALID_CHOICE;
    } else {
        msg->engine_speed_valid = PLATFORM_VEHICLE_ENGINE_ENGINE_SPEED_VALID_INVALID_CHOICE;
    }
    if (sample_usable(batt, platform_vehicle_engine_battery_voltage_is_in_phys_range)) {
        msg->battery_voltage = platform_vehicle_engine_battery_voltage_encode(batt->physical);
        msg->battery_voltage_valid = PLATFORM_VEHICLE_ENGINE_BATTERY_VOLTAGE_VALID_VALID_CHOICE;
    } else {
        msg->battery_voltage_valid = PLATFORM_VEHICLE_ENGINE_BATTERY_VOLTAGE_VALID_INVALID_CHOICE;
    }
    if (sample_usable(cool, platform_vehicle_engine_coolant_temp_is_in_phys_range)) {
        msg->coolant_temp = platform_vehicle_engine_coolant_temp_encode(cool->physical);
        msg->coolant_temp_valid = PLATFORM_VEHICLE_ENGINE_COOLANT_TEMP_VALID_VALID_CHOICE;
    } else {
        msg->coolant_temp_valid = PLATFORM_VEHICLE_ENGINE_COOLANT_TEMP_VALID_INVALID_CHOICE;
    }
    if (sample_usable(tps, platform_vehicle_engine_throttle_pos_is_in_phys_range)) {
        msg->throttle_pos = platform_vehicle_engine_throttle_pos_encode(tps->physical);
        msg->throttle_pos_valid = PLATFORM_VEHICLE_ENGINE_THROTTLE_POS_VALID_VALID_CHOICE;
    } else {
        msg->throttle_pos_valid = PLATFORM_VEHICLE_ENGINE_THROTTLE_POS_VALID_INVALID_CHOICE;
    }
    msg->ecu_present = ecu_present ? PLATFORM_VEHICLE_ENGINE_ECU_PRESENT_PRESENT_CHOICE
                                   : PLATFORM_VEHICLE_ENGINE_ECU_PRESENT_ABSENT_CHOICE;
}

bool vehicle_republish_speed_frame(const struct platform_vehicle_speed_t* msg,
                                   const moto_e2e_tx_state_t* committed,
                                   moto_e2e_tx_state_t* next,
                                   uint8_t out[PLATFORM_VEHICLE_SPEED_LENGTH])
{
    if ((msg == NULL) || (committed == NULL) || (next == NULL) || (out == NULL)) {
        return false;
    }
    if (platform_vehicle_speed_pack(out, msg, PLATFORM_VEHICLE_SPEED_LENGTH) !=
        (int)PLATFORM_VEHICLE_SPEED_LENGTH) {
        return false;
    }
    *next = *committed;
    return platform_vehicle_speed_e2e_protect(out, PLATFORM_VEHICLE_SPEED_LENGTH, next) == MOTO_E2E_OK;
}

bool vehicle_republish_engine_frame(const struct platform_vehicle_engine_t* msg,
                                    uint8_t out[PLATFORM_VEHICLE_ENGINE_LENGTH])
{
    if ((msg == NULL) || (out == NULL)) {
        return false;
    }
    return platform_vehicle_engine_pack(out, msg, PLATFORM_VEHICLE_ENGINE_LENGTH) ==
           (int)PLATFORM_VEHICLE_ENGINE_LENGTH;
}

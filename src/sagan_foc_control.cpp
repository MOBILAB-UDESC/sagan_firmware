#include "sagan_foc_control.hpp"
#include <algorithm>

// MODIFICATION: The constructor now saves the sensor pointer
SaganFOCControl::SaganFOCControl(const Config& config, as5600_t& sensor_obj) 
    : config(config), sensor_obj(sensor_obj) { // <-- Save the sensor object here
    init_pwm();
    target_angle_rad = 0.0;
    continuous_raw_angle = 0;
    prev_raw_angle = -1;
}

// Method to initialize the PWM outputs for the 3 motor phases
void SaganFOCControl::init_pwm() {
    for (int i = 0; i < 3; i++) {
        gpio_set_function(config.pwm_pins[i], GPIO_FUNC_PWM);
        pwm_slice[i] = pwm_gpio_to_slice_num(config.pwm_pins[i]);
        pwm_channel[i] = pwm_gpio_to_channel(config.pwm_pins[i]);
        
        pwm_config pwm_cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&pwm_cfg, 4.f);
        pwm_config_set_wrap(&pwm_cfg, 1000);
        pwm_init(pwm_slice[i], &pwm_cfg, true);
    }

    gpio_init(config.enable_pin);
    gpio_set_dir(config.enable_pin, GPIO_OUT);
    gpio_put(config.enable_pin, true);
}

// MODIFICATION: Calibrate now uses the internal sensor pointer
void SaganFOCControl::calibrate() {
    printf("Initiating BLDC Motor Calibration...\n");
    sleep_ms(3000);

    for (int i = 0; i < config.pole_pairs; i++) {
        for (float electrical_angle = 0.0; electrical_angle <= 2 * M_PI; electrical_angle += M_PI / 1024) {
            space_vector_modulation(0.3 * cosf(electrical_angle), 0.3 * sinf(electrical_angle));
            sleep_ms(1);

            // Reads sensor using the stored pointer
            int raw_sensor_angle = as5600_read_raw_angl(&sensor_obj); // <-- MODIFIED
            printf("Angle Readed: %i rad ; Angle Aplied: %.2f rad ; \n", raw_sensor_angle, electrical_angle);
            
            if (raw_sensor_angle >= 0 && raw_sensor_angle < LUT_SIZE) {
                this->lut_electrical_angle[raw_sensor_angle] = electrical_angle;
            }
        }
    }
    
    space_vector_modulation(0.0f, 0.0f);
    printf("Calibration Finalized.\n");
}

// MODIFICATION: Update now uses the internal sensor pointer
void SaganFOCControl::update() {
    // Reads sensor using the stored pointer
    int current_raw_angle = as5600_read_raw_angl(&sensor_obj); // <-- MODIFIED

    if (prev_raw_angle == -1) prev_raw_angle = current_raw_angle;

    // --- The rest of the update logic is identical ---
    if (current_raw_angle > prev_raw_angle && (current_raw_angle - prev_raw_angle) > (LUT_SIZE / 2)) {
        continuous_raw_angle += (current_raw_angle - LUT_SIZE) - prev_raw_angle;
    } else if (current_raw_angle < prev_raw_angle && (prev_raw_angle - current_raw_angle) > (LUT_SIZE / 2)) {
        continuous_raw_angle += (current_raw_angle + LUT_SIZE) - prev_raw_angle;
    } else {
        continuous_raw_angle += current_raw_angle - prev_raw_angle;
    }
    prev_raw_angle = current_raw_angle;

    float current_angle_rad = static_cast<float>(continuous_raw_angle) * 2.0f * M_PI / LUT_SIZE;
    float angle_error = target_angle_rad - current_angle_rad;

    int control_offset = static_cast<int>(fabs(angle_error) * config.angle_p_gain);
    if (control_offset >= 200) control_offset = 200;

    float control_offset_field = fabs(angle_error) * config.field_p_gain;
    if (control_offset_field >= 1.0f) control_offset_field = 1.0f;
    else if (control_offset_field <= 0.3f) control_offset_field = 0.3f;

    int final_angle_lookup_index = (angle_error > 0.0) ? (current_raw_angle + control_offset) : (current_raw_angle - control_offset);
    int lut_index = (final_angle_lookup_index + LUT_SIZE) % LUT_SIZE;
    float electrical_angle_to_apply = this->lut_electrical_angle[lut_index];

    space_vector_modulation(control_offset_field * cosf(electrical_angle_to_apply), control_offset_field * sinf(electrical_angle_to_apply));
}

// NEW: Implementation for the position getter function
float SaganFOCControl::get_position_rad() const {
    // Converts the internal continuous angle state to radians and returns it
    return static_cast<float>(this->continuous_raw_angle) * 2.0f * M_PI / LUT_SIZE;
}

// Method to update the controller's target
void SaganFOCControl::set_target_angle(float target_rad) {
    this->target_angle_rad = target_rad;
}

// Low-level SVM function (copied from your old library)
void SaganFOCControl::space_vector_modulation(float v_alpha, float v_beta) {
    // Space Vector Modulation (SVM)
    float v_u = v_alpha;
    float v_v = -0.5f * v_alpha + 0.86602540378f * v_beta; // sqrt(3)/2
    float v_w = -0.5f * v_alpha - 0.86602540378f * v_beta;
    
    // Normalize and scale to PWM range
    float max_val = std::max(std::max(std::abs(v_u), std::abs(v_v)), std::abs(v_w));
    if (max_val > 1.0f) {
        v_u /= max_val;
        v_v /= max_val;
        v_w /= max_val;
    }
    
    // Convert to PWM duty cycles (0-1 range)
    float duty_u = 0.5f + 0.5f * v_u;
    float duty_v = 0.5f + 0.5f * v_v;
    float duty_w = 0.5f + 0.5f * v_w;
    
    // Set PWM outputs
    pwm_set_chan_level(pwm_slice[0], pwm_channel[0], static_cast<uint16_t>(duty_u * 1000));
    pwm_set_chan_level(pwm_slice[1], pwm_channel[1], static_cast<uint16_t>(duty_v * 1000));
    pwm_set_chan_level(pwm_slice[2], pwm_channel[2], static_cast<uint16_t>(duty_w * 1000));
}
idf_component_register(
    SRC_DIRS "../../common/zcl_utility/src"
    SRCS "servo_calibration.c" "esp_zb_sleepy_end_device.c" "servo_control.c" "servo_position_reader.c" "switch_driver.c"
    INCLUDE_DIRS "." "../../common/zcl_utility/include"
)

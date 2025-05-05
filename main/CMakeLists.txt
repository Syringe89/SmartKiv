idf_component_register(
    SRC_DIRS "../../common/zcl_utility/src"
    SRCS "esp_zb_sleepy_end_device.c" "servo_control.c" "servo_position_reader.c" "servo_position_example.c"
    INCLUDE_DIRS "." "../../common/zcl_utility/include"
)

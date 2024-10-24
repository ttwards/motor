# SPDX-License-Identifier: Apache-2.0

# FIXME: SHADOW_VARS: Remove this once we have enabled -Wshadow globally.
# add_compile_options($<TARGET_PROPERTY:compiler,warning_shadow_variables>)

# add_definitions(-D__ZEPHYR_SUPERVISOR__)

add_subdirectory_ifdef(CONFIG_MOTOR_M3508 motor)

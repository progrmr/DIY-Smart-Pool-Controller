File structure on the Home Assistant machine:

/config
└── /esphome
    ├── pool_controller.yaml  <-- This is your renamed esphome-web-9e4fb0.yaml
    ├── common_types.h
    └── /components
        ├── /solar_controller
        │   ├── __init__.py
        │   ├── component.py
        │   ├── solar_controller.cpp
        │   └── solar_controller.h
        ├── /pool_pump_rs485
        │   ├── __init__.py
        │   ├── component.py
        │   ├── pool_pump_rs485.cpp
        │   └── pool_pump_rs485.h
        ├── /pool_temp_estimator
        │   ├── __init__.py
        │   ├── component.py
        │   ├── temp_estimator.cpp
        │   └── temp_estimator.h
        └── /valve_position
            ├── __init__.py
            ├── component.py
            ├── valve_actuator.cpp
            └── valve_actuator.h

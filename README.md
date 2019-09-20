# system_monitor_ros
[![Build Status](https://travis-ci.com/Auterion/system_monitor_ros.svg?branch=master)](https://travis-ci.com/Auterion/system_monitor_ros)

`system_monitor_ros` is a ros package to publish onboard companion status messages to the flight controller through [mavros](https://github.com/mavlink/mavros)
This is a reference implementation of the mavlink message [ONBOARD_COMPUTER_STATUS](https://mavlink.io/en/messages/common.html#ONBOARD_COMPUTER_STATUS).

The node publishes the information on `/mavros/onboard_computer/status` topic which contains the following information.

```
# Mavros message: ONBOARDCOMPUTERSTATUS

 std_msgs/Header header

 uint8 component               # See enum MAV_COMPONENT

 uint32 uptime               # [ms] Time since system boot
uint8 type                  # Type of the onboard computer: 0: Mission computer primary, 1: Mission computer backup 1, 2: Mission computer backup 2, 3: Compute node, 4-5: Compute spares, 6-9: Payload computers.
uint8[8] cpu_cores          # CPU usage on the component in percent (100 - idle). A value of UINT8_MAX implies the field is unused.
uint8[10] cpu_combined      # Combined CPU usage as the last 10 slices of 100 MS (a histogram). This allows to identify spikes in load that max out the system, but only for a short amount of time. A value of UINT8_MAX implies the field is unused
uint8[4] gpu_cores          # GPU usage on the component in percent (100 - idle). A value of UINT8_MAX implies the field is unused
uint8[10] gpu_combined      # Combined GPU usage as the last 10 slices of 100 MS (a histogram). This allows to identify spikes in load that max out the system, but only for a short amount of time. A value of UINT8_MAX implies the field is unused.
int8 temperature_board      # [degC] Temperature of the board. A value of INT8_MAX implies the field is unused.
int8[8] temperature_core    # [degC] Temperature of the CPU core. A value of INT8_MAX implies the field is unused.
int16[4] fan_speed          # [rpm] Fan speeds. A value of INT16_MAX implies the field is unused.
uint32 ram_usage            # [MiB] Amount of used RAM on the component system. A value of UINT32_MAX implies the field is unused.
uint32 ram_total            # [MiB] Total amount of RAM on the component system. A value of UINT32_MAX implies the field is unused.
uint32[4] storage_type      # Storage type: 0: HDD, 1: SSD, 2: EMMC, 3: SD card (non-removable), 4: SD card (removable). A value of UINT32_MAX implies the field is unused.
uint32[4] storage_usage     # [MiB] Amount of used storage space on the component system. A value of UINT32_MAX implies the field is unused.
uint32[4] storage_total     # [MiB] Total amount of storage space on the component system. A value of UINT32_MAX implies the field is unused.
uint32[6] link_type         # Link type: 0-9: UART, 10-19: Wired network, 20-29: Wifi, 30-39: Point-to-point proprietary, 40-49: Mesh proprietary.
uint32[6] link_tx_rate      # [KiB/s] Network traffic from the component system. A value of UINT32_MAX implies the field is unused.
uint32[6] link_rx_rate      # [KiB/s] Network traffic to the component system. A value of UINT32_MAX implies the field is unused.
uint32[6] link_tx_max       # [KiB/s] Network capacity from the component system. A value of UINT32_MAX implies the field is unused.
uint32[6] link_rx_max       # [KiB/s] Network capacity to the component system. A value of UINT32_MAX implies the field is unused.
```

> This is still WIP and some of the information are not being published.


## Running the package
The package can be run by the following roslaunch command.
```
roslaunch system_monitor_ros system_monitor.launch
```
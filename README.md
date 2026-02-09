# Project Title #
EdgeAI - System Controller

### Overview ###
The EdgeAI - System Controller serves as the Supervisor Middleware for autonomous drone and robotic systems. It is designed to bridge the gap between high-level AI perception processor (Companion Computer) and low-level execution (Flight/Motion Controller). The architecture prioritizes real-time processing, deterministic failsafe handling, and efficient communication multiplexing to ensure system reliability in complex environments.

### Features ###


### Hardware Requirements ###
* MCU supported by Zephyr RTOS - STM32 NUCLEO-F767ZI (Cortex-M7 @ 216MHz).

Interfaces:
* Ethernet compliant with IEEE-802.3-2002
* USB OTG or full-speed device
* 3 user LEDs
* 2 user and reset push-buttons
* 32.768 kHz crystal oscillator

capability: mass storage, virtual COM port and debug port.
Optional: In-circuit debug/programmer interface


### Software Requirements ###
* Zephyr RTOS 4.x or newer
* C/C++ toolchain for target MCU (west)
* Python3 (for build tools)
* west tool (Zephyr’s meta-tool)
* Board support package matching your hardware

### Code Overview ###
The project is organized into modular components to ensure portability across different MCU targets:
* src/main.c: Initializes the kernel, system clock, and spawns the primary supervisor threads.
* prj.conf: The Kconfig file enabling the Zephyr Shell, Logging, PWM, and preemptive multi-threading features.


### Contact ###
* For any questions or inquiries, please contact [leejunhong@yahoo.com.sg].
# EdgeAI System Controller - AI Development Guide

## Quick Reference: Essential Commands

```bash
# Build (clean build recommended for configuration changes)
west build -p auto -b nucleo_f767zi

# Flash to hardware
west flash

# Debug session
west debug

# Check build configuration
west build -t menuconfig
```

## Project Overview
This is a **Zephyr RTOS 4.x** embedded project for an autonomous drone/robotics supervisor middleware running on **STM32 NUCLEO-F767ZI** (Cortex-M7 @ 216MHz). The system bridges high-level AI perception processors and low-level flight/motion controllers with real-time, deterministic failsafe handling.

**Current Status:** Early-stage project with basic LED blinking demo. Production features are planned but not yet implemented.

## Architecture & Key Concepts

### Project Structure
- `src/main.c` - Main entry point with GPIO LED demo and kernel initialization
- `src/device_sm.c/h` - Device state machine implementation
- `src/config.h` - System configuration structures
- `prj.conf` - Zephyr Kconfig enabling GPIO, logging, PWM, preemptive threading
- `CMakeLists.txt` - Build configuration with JLink flash runner
- `build/` - Generated build artifacts (do not edit directly)

### Target Hardware
- **Primary target**: STM32 NUCLEO-F767ZI (Cortex-M7 @ 216MHz)
- **Flash runner**: JLink (default), with STM32CubeProgrammer and OpenOCD alternatives
- **Debug runner**: OpenOCD

## Boundaries and Restrictions

### Files to NEVER Modify
- `build/` - Generated build artifacts
- Device tree files (when added) - Hardware-specific configurations
- Zephyr SDK files outside project directory

### Forbidden Actions
- DO NOT use standard C library sleep functions (use `k_msleep()` instead)
- DO NOT introduce non-deterministic timing in real-time critical paths
- DO NOT bypass device readiness checks
- DO NOT hardcode hardware addresses (use device tree macros)
- DO NOT commit secrets, credentials, or hardware-specific calibration data
- DO NOT use dynamic memory allocation in interrupt contexts
- DO NOT introduce unbounded loops in real-time threads

## Coding Standards and Conventions

### Code Style
- **Language**: C (C11 standard)
- **Indentation**: Use tabs consistent with existing codebase
- **Line length**: Aim for 80 characters where practical
- **Naming conventions**:
  - Functions: `snake_case` (e.g., `device_init()`)
  - Macros/Constants: `UPPER_SNAKE_CASE` (e.g., `LED0_NODE`)
  - Struct types: `snake_case` with `_t` suffix (e.g., `config_data_t`)
  - Global variables: Prefix with module name (e.g., `main_led_state`)

### Required Patterns and Examples

#### Device Tree Integration (ALWAYS use this pattern)
```c
// ✅ CORRECT: Using device tree macros
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// ❌ WRONG: Hardcoding hardware addresses
#define LED_PORT 0x40020000  // Never do this
```

#### Threading & Timing (Critical for Real-Time Systems)
```c
// ✅ CORRECT: Use Zephyr timing functions
k_msleep(1000);  // Sleep for 1000ms
k_busy_wait(100);  // Busy wait for 100us (use sparingly)

// ❌ WRONG: Standard C library functions
sleep(1);  // Not available in Zephyr
usleep(1000);  // Not available in Zephyr
```

#### GPIO & Hardware Access (Always Check Readiness)
```c
// ✅ CORRECT: Check device readiness before use
if (!gpio_is_ready_dt(&led)) {
    LOG_ERR("LED device not ready");
    return -ENODEV;
}
gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

// ❌ WRONG: Assuming device is ready
gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);  // May crash if device not ready
```

#### Logging (Use Zephyr Logging Subsystem)
```c
// ✅ CORRECT: Module-level logging
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(module_name, LOG_LEVEL_INF);

LOG_INF("System initialized");
LOG_WRN("Sensor timeout, retrying...");
LOG_ERR("Critical failure: %d", error_code);

// ❌ WRONG: printf for production code
printf("Debug message\n");  // Use only for temporary debugging
```

## Project-Specific Conventions

### Logging
- Use Zephyr logging subsystem: `LOG_MODULE_REGISTER(module_name)`
- Configure log levels per module via Kconfig
- Example: `#define LOG_LEVEL CONFIG_MAIN_LOG_LEVEL`

## Error Handling Guidelines

### Return Value Conventions
- Return `0` on success (Zephyr convention)
- Return negative errno values on error (e.g., `-EINVAL`, `-ENOMEM`, `-ENODEV`)
- For fatal errors in main initialization, return `0` to halt system

### Error Handling Pattern
```c
// ✅ CORRECT: Proper error handling
int ret = device_init();
if (ret < 0) {
    LOG_ERR("Device initialization failed: %d", ret);
    return ret;
}

// ✅ CORRECT: Fatal error in main
if (!gpio_is_ready_dt(&led)) {
    LOG_ERR("LED device not ready");
    return 0;  // Halt system on critical failure
}

// ❌ WRONG: Ignoring errors
device_init();  // Always check return values
```

### Deterministic Failsafe Requirements
- All error paths must be deterministic and bounded in execution time
- Failsafe routines must complete within defined time constraints
- Use watchdog timers for critical operations
- Implement graceful degradation where possible

## Configuration Management

### Hardware Configuration Hierarchy
1. **Device Tree**: Hardware-specific settings (pins, peripherals, clocks)
2. **Kconfig (`prj.conf`)**: Feature enables and compile-time configuration
3. **Runtime Config (`config.h`)**: Application-level parameters

### Adding New Features
```bash
# Interactive configuration menu
west build -t menuconfig

# Enable features in prj.conf
echo "CONFIG_FEATURE_NAME=y" >> prj.conf
```

### Example Configuration Patterns
```c
// In prj.conf
CONFIG_GPIO=y
CONFIG_LOG=y
CONFIG_LOG_MODE_IMMEDIATE=y

// In code - checking config
#ifdef CONFIG_FEATURE_NAME
    // Feature-specific code
#endif
```

## Workflow and Git Practices

### Branch Strategy
- Create feature branches from `main`
- Use descriptive branch names: `feature/sensor-integration`, `fix/timing-issue`
- Keep changes focused and atomic

### Commit Message Format (Conventional Commits)
```
<type>: <description>

[optional body]

[optional footer]
```

**Types:**
- `feat:` New feature
- `fix:` Bug fix
- `docs:` Documentation changes
- `refactor:` Code refactoring without behavior change
- `test:` Adding or updating tests
- `chore:` Build process or auxiliary tool changes

**Examples:**
```
feat: add UART communication driver for companion computer

Implements basic UART driver with DMA support for high-bandwidth
communication with AI perception processor.

fix: correct GPIO initialization timing issue

Device tree spec was accessed before device ready check.
Added proper initialization order.
```

### Testing Requirements
- Test on actual hardware whenever possible (embedded project limitations)
- Verify timing-critical code paths with logic analyzer or oscilloscope
- Document test results in commit messages or PR descriptions
- Run clean builds (`west build -p auto`) after configuration changes

## Integration Patterns

### Expected Communication Interfaces (Planned)
- **Companion Computer Interface**: High-bandwidth AI data exchange (UART/USB)
- **Flight/Motion Controller Interface**: Real-time control commands (SPI/I2C)
- **Sensor Multiplexing**: Multiple sensor data streams (I2C/SPI buses)
- **Failsafe Communication**: Emergency override protocols (dedicated GPIO)

### Multi-Threading Architecture (Future Implementation)
```c
// Example thread structure for future implementation
#define SENSOR_THREAD_PRIORITY 5
#define AI_COMM_THREAD_PRIORITY 6
#define FAILSAFE_THREAD_PRIORITY 4  // Highest priority

K_THREAD_DEFINE(sensor_thread, SENSOR_STACK_SIZE,
                sensor_thread_entry, NULL, NULL, NULL,
                SENSOR_THREAD_PRIORITY, 0, 0);
```

### Communication Protocol Guidelines
- Use Zephyr's native drivers where available
- Implement timeout mechanisms on all blocking operations
- Design protocols with health-check / heartbeat mechanisms
- Plan for graceful degradation on communication failures

## Security and Safety Considerations

### Embedded Systems Security
- **NO hardcoded secrets**: Use secure storage mechanisms when available
- **Input validation**: Sanitize all external inputs (sensor data, commands)
- **Bounds checking**: Always validate array indices and buffer sizes
- **Overflow protection**: Use safe string functions (e.g., `strncpy` not `strcpy`)

### Safety-Critical Guidelines
- **Deterministic behavior**: Avoid unbounded operations in real-time paths
- **Resource management**: Track and limit dynamic allocations
- **Failsafe design**: Implement safe states for all failure modes
- **Watchdog integration**: Use hardware watchdog for critical systems
- **Code reviews**: All safety-critical code requires peer review

### Example: Safe Buffer Handling
```c
// ✅ CORRECT: Bounds-checked buffer operations
char buffer[64];
size_t received = uart_read(uart_dev, buffer, sizeof(buffer) - 1);
buffer[received] = '\0';  // Ensure null termination

// ❌ WRONG: Unbounded operations
char buffer[64];
strcpy(buffer, untrusted_input);  // Buffer overflow risk
```

## Acceptance Criteria for Tasks

When completing a task, ensure:
1. **Code compiles** without warnings: `west build -p auto -b nucleo_f767zi`
2. **Device tree usage**: Hardware access uses DT macros, no hardcoded addresses
3. **Error handling**: All return values checked, errors logged appropriately
4. **Logging**: Appropriate log levels used (INF, WRN, ERR)
5. **Documentation**: Code comments explain *why*, not *what* (code is self-documenting)
6. **Conventions**: Code follows existing style and naming conventions
7. **Safety**: No new warnings about timing, overflow, or resource issues
8. **Commit message**: Follows conventional commits format
9. **Testing notes**: Include verification steps in commit/PR description

## Task Prioritization

### High-Priority Tasks (Focus Areas)
- Real-time communication implementation
- Failsafe state machine
- Sensor integration and data processing
- Testing and validation frameworks

### Low-Risk Automation Tasks (Good for AI)
- Documentation improvements
- Code formatting and style consistency
- Adding logging statements
- Simple refactoring (renaming, extracting functions)
- Configuration file updates

### Complex/Risky Tasks (Require Extra Care)
- Interrupt handlers and timing-critical code
- Memory management in resource-constrained environment
- Hardware-specific initialization sequences
- Real-time scheduling and thread priorities

## Important Notes

### Project Status
- **Current stage**: Early development with basic LED blinking demo
- **Production readiness**: NOT production-ready; requires extensive development
- **Hardware testing**: Always test on actual hardware for embedded changes

### Development Mindset
- **Determinism first**: Prioritize predictable, bounded execution times
- **Safety-critical awareness**: This is for autonomous systems - failures can be catastrophic
- **Resource constraints**: 512KB flash, 512KB RAM on STM32F767ZI
- **Real-time requirements**: Use Zephyr's RTOS features for threading and timing
- **Hardware access**: Always use device tree macros and check device readiness

### Getting Help
- **Zephyr documentation**: https://docs.zephyrproject.org/
- **STM32F767ZI reference**: Use device tree bindings for peripheral configuration
- **Project contact**: leejunhong@yahoo.com.sg

### Common Pitfalls to Avoid
1. Using standard C library sleep/delay functions instead of Zephyr's `k_msleep()`
2. Hardcoding hardware addresses instead of using device tree
3. Ignoring device readiness checks
4. Using dynamic memory allocation in interrupt contexts
5. Creating unbounded loops in real-time threads
6. Forgetting to enable required Kconfig options in `prj.conf`
7. Not testing configuration changes with clean builds (`-p auto`)

## Quick Troubleshooting

### Build Issues
```bash
# Clean everything and rebuild
rm -rf build
west build -p auto -b nucleo_f767zi

# Check Kconfig options
west build -t menuconfig
```

### Flash Issues
```bash
# Verify JLink connection
JLinkExe -device STM32F767ZI -if SWD -speed 4000

# Alternative flashers
west flash --runner=stm32cubeprogrammer
west flash --runner=openocd
```

### Runtime Issues
- Check log output: Enable `CONFIG_LOG=y` and appropriate log levels
- Verify device tree: Ensure peripherals are properly defined
- Check initialization order: Some devices depend on others
- Timing issues: Use logic analyzer to verify timing-critical operations
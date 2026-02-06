# EdgeAI System Controller - AI Development Guide

## Project Overview
This is a **Zephyr RTOS 4.x** embedded project for an autonomous drone/robotics supervisor middleware running on **STM32 NUCLEO-F767ZI** (Cortex-M7 @ 216MHz). The system bridges high-level AI perception processors and low-level flight/motion controllers with real-time, deterministic failsafe handling.

## Architecture & Key Concepts

### Project Structure
- `src/main.c` - Main entry point with GPIO LED demo and kernel initialization
- `src/config.h` - System configuration structures (minimal template currently)
- `prj.conf` - Zephyr Kconfig enabling GPIO, logging, PWM, preemptive threading
- `CMakeLists.txt` - Build configuration with JLink flash runner
- `build/` - Generated build artifacts (do not edit directly)

### Target Hardware
- **Primary target**: STM32 NUCLEO-F767ZI (Cortex-M7)
- **Flash runner**: JLink (default), with STM32CubeProgrammer and OpenOCD alternatives
- **Debug runner**: OpenOCD

## Development Workflow

### Build Commands
```bash
# Clean and build for NUCLEO-F767ZI
west build -p auto -b nucleo_f767zi

# Flash to hardware
west flash

# Debug session
west debug
```

### Key Zephyr Patterns Used

#### Device Tree Integration
```c
// LED control using device tree macros
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
```

#### Threading & Timing
- Use `k_msleep()` for delays (not standard sleep functions)
- Real-time priorities will be critical for supervisor tasks
- Plan for multiple threads: sensor input, AI communication, failsafe monitoring

#### GPIO & Hardware Access
```c
// Check device readiness before use
if (!gpio_is_ready_dt(&led)) return 0;

// Configure pins through device tree specs
gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
```

## Project-Specific Conventions

### Logging
- Use Zephyr logging subsystem: `LOG_MODULE_REGISTER(module_name)`
- Configure log levels per module via Kconfig
- Example: `#define LOG_LEVEL CONFIG_MAIN_LOG_LEVEL`

### Error Handling
- Return 0 on fatal errors (Zephyr convention)
- Check device readiness before hardware access
- Use deterministic error paths for failsafe scenarios

### Configuration Management
- Hardware features enabled via `prj.conf` (Kconfig)
- Runtime configuration through `config.h` structures
- Device-specific settings in device tree overlays (not present yet)

## Integration Patterns

### Expected Communication Interfaces
Based on README, plan for:
- **Companion Computer Interface**: High-bandwidth AI data exchange
- **Flight/Motion Controller Interface**: Real-time control commands
- **Sensor Multiplexing**: Multiple sensor data streams
- **Failsafe Communication**: Emergency override protocols

### Future Development Areas
- Multi-threading architecture for real-time supervisor tasks
- Communication protocol implementation (UART, SPI, I2C)
- Sensor driver integration
- AI perception data processing pipelines
- Failsafe state machine implementation

## Important Notes
- This is an early-stage project with basic LED blinking demo
- Production implementation requires significant real-time systems expertise
- Focus on deterministic behavior and bounded execution times
- Test thoroughly on actual hardware due to real-time constraints
- Consider Zephyr's safety features for autonomous systems applications
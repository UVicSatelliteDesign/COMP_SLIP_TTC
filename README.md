# COMP_SLIP_TTC

---

## Goal
Use STM32 MCU to control the CC1201 transceiver via SPI, with radio settings configured using SmartRF Studio.

---

## Setup Overview

### 1. Export Radio Settings
- Use [SmartRF Studio 7](https://www.ti.com/tool/SMARTRFTM-STUDIO)
- Configure radio (frequency, modulation, packet length, etc.)
- Export register settings from "Register View" as C code
- Output:
    - `smartrf_settings.c`
    - `smartrf_settings.h`

### 2. Implement CC1201 SPI Driver
Driver must support:
- Read/write of registers
- Sending command strobes
- Packet TX/RX via FIFO

---

## File Structure
```text
/Drivers/
├── cc1201.c            # SPI driver implementation
├── cc1201.h            # Header with declarations
├── smartrf_settings.c  # Exported SmartRF register settings
└── smartrf_settings.h  # Header for SmartRF settings
/Core/
└── main.c              # Application entry point
```

## Git Strategy
- **Evan**: `feature/cc1201-driver`
- **Cole**: `feature/integration-main`
- Use **GitHub Issues** to track progress
- Submit **Pull Requests** for review before merging into `main`

---

## Additional Requirements
- **Interrupt handling** using `GDO0` / `GDO2` pins
- **FIFO usage**:
  - `CC1201_TXFIFO`
  - `CC1201_RXFIFO`
- **Status/error checking**:
  - `MARCSTATE`
  - `RXBYTES`
  - `TXBYTES`

---

## 🔗 Resources
- [CC120x User Guide (SWRU346)](https://www.ti.com/lit/ug/swru346b/swru346b.pdf)
- [CC1201 Datasheet](https://www.ti.com/lit/ds/symlink/cc1201.pdf)
- [TI CC120x Driver Examples (GitHub)](https://github.com/LSandler-TI/CC120x-Driver)
- [TI e2e Support Forums](https://e2e.ti.com/)
- [SmartRF Studio 7](https://www.ti.com/tool/SMARTRFTM-STUDIO)
- [STM32 HAL & SPI Documentation](https://www.st.com/en/embedded-software/stm32cube-mcu-packages.html)

# STM32 Microcontroller and Driver Development 🚀

This project is built as part of the [STM32 Microcontroller and Driver Development](https://www.udemy.com/course/mastering-microcontroller-with-peripheral-driver-development/) course on Udemy. It focuses on low-level driver development using C for STM32 microcontrollers, exploring peripherals such as GPIO, SPI, I2C, and USART — all developed **from scratch without relying on HAL libraries**.

---

## 🛠️ Development Environment

- **IDE**: STM32CubeIDE  
- **Toolchain**: arm-none-eabi-gcc (GCC for ARM Cortex-M)
- **Debugger**: ST-Link  
- **Language**: C (bare-metal)  
- **Board**: Any STM32 MCU (e.g., STM32F103C8T6, STM32F407VG, etc.)

---

## 📦 Project Structure

```
/Core              - Application code (main.c, startup, system files)
/Drivers           - CMSIS and custom peripheral drivers
/STM32CubeIDE      - IDE-specific configurations
/Debug or /Release - Build outputs and intermediate files
```

---

## 🎯 What You Will Learn

- Develop **peripheral drivers** for GPIO, SPI, I2C, and USART **from scratch**
- Learn **how to write driver headers, API prototypes, and implementations**
- Understand **MCU peripheral registers**: control, status, config registers
- Configure and handle **interrupts** correctly using NVIC and vector tables
- Master **clock tree configuration**: HCLK, PCLK, PLL, serial clock setup
- Work with **MCU buses** like AHB, APB, and understand their structure
- Analyze and debug serial protocols using a **logic analyzer**
- Explore **reference manuals**, **datasheets**, and **startup code**
- Debug real-world issues with case studies and **bare-metal C programming techniques**
- Demystify how peripherals **really work behind the scenes**

---

## 📎 Requirements

- Basic knowledge of embedded C
- Familiarity with STM32 microcontrollers
- STM32CubeIDE installed and configured
- ST-Link debugger and supported STM32 board

---

## 📄 License

---


> **Note**: Nội dung mã nguồn hoặc tài liệu trong dự án này chỉ được chia sẻ với mục đích học tập và nghiên cứu. Các nội dung khóa học trên Udemy không được phép tái phân phối hoặc thay đổi mà không có sự cho phép của tác giả khóa học.

> **Note**: The source code or materials in this project are shared for educational and research purposes only. The contents of the Udemy course may not be redistributed
or modified without the permission of the course author.


---

*Happy coding with STM32 and bare-metal development! 💻⚡*

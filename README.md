# logolith

Logolith is the flight controller firmware of Aerolith, a custom cinelifter built to carry a mirrorless camera (A7S III class) for cinematic but dynamic aerial footage.

> Early stage. Work in Progress.

## Intentions

- The goal is not state-of-the-art quadcopter control but to achieve stable and reliable flight with minimal code.
- Like the rest of Aerolith (the frame, the ESCs, the flight controller), Logolith is written from scratch rather than derived from an existing flight stack.
- Made for LithCore (custom STM32 F405 flight controller) but still aiming for portability so it can be useful for other custom flight controllers.
- (WIP) FreeRTOS-based.
- (WIP) OneShot125 / DShot.

## Getting Started

The project is built with the STM32 extension for Visual Studio Code. It can be flashed over SWD with an ST-Link, or over USB-C in DFU mode.

## License

[MIT](LICENSE)

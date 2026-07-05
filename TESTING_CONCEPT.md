# Testing Concept - MaerklinMotorola Library

This document outlines the current testing state and the proposed strategy for improving the reliability and maintainability of the MaerklinMotorola library.

## Current State

Currently, the library uses GitHub Actions to perform **compilation tests**.

- **Workflow**: `.github/workflows/main.yml`
- **Tool**: `ArminJo/arduino-test-compile`
- **Scope**: Verifies that all example sketches (`GetBits`, `LocoFunction`, `LocoMotor`) compile successfully for the `arduino:avr:uno` platform.
- **Limitations**: This only ensures the code is syntactically correct and fits in memory. It does **not** verify the correctness of the decoding logic or the protocol implementation.

## Proposed Testing Strategy: Native Unit Testing

To ensure the decoding logic is correct and to prevent regressions, we propose implementing native unit tests. These tests will run on a host machine (e.g., Linux/macOS/Windows) rather than on an actual Arduino board.

### 1. Decoupling Logic from Hardware

The core of the library is the `Parse()` method in `MaerklinMotorola.cpp`. This method operates on a buffer of timings and does not directly interact with hardware, except for its dependency on Arduino-specific types (`byte`) and the `micros()` function.

### 2. Mocking Arduino Dependencies

To compile the code natively, we need to provide a mock `Arduino.h` that defines:
- `byte` (as `unsigned char`)
- `micros()` (can be mocked to return a controlled value or just a dummy)
- Any other necessary constants or types.

### 3. Recommended Tools

- **Unity**: A lightweight unit testing framework for C, often used in embedded projects.
- **GoogleTest**: A more feature-rich C++ testing framework.
- **ACustom Test Runner**: Given the library's simplicity, a simple C++ program that feeds raw timings into the `MaerklinMotorola` class and asserts the results may also suffice.

### 4. Test Case Examples

Test cases should focus on the `Parse()` logic by populating the `Timings` array in `MaerklinMotorolaData` and verifying the resulting state.

#### Scenario: Valid MM1 Locomotive Packet
- **Input**: A sequence of timings representing a valid MM1 packet.
- **Expected Output**: `Address`, `Speed`, and `Function` state match the expected values; `IsMM2` is `false`.

#### Scenario: Valid MM2 Locomotive Packet
- **Input**: A sequence of timings representing a valid MM2 packet.
- **Expected Output**: `IsMM2` is `true`; MM2-specific functions or direction changes are correctly identified.

#### Scenario: Invalid Signal (MFX or Noise)
- **Input**: Timings that fall outside the expected ranges for MM/MM2.
- **Expected Output**: `State` is set to `DataGramState_Error` or the packet is not validated.

## Implementation Plan for Tests

1.  **Setup Directory Structure**: Create a `/test` directory.
2.  **Create Mocks**: Implement a minimal `Arduino.h` for native compilation.
3.  **Write Test Runner**: Implement test cases using a chosen framework.
4.  **Integrate with CI**: Add a step to the GitHub Actions workflow to run these native tests on every push.

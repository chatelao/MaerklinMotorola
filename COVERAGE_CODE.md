# Code Coverage Report

This document summarizes the code coverage for the `MaerklinMotorola` library, achieved through native unit testing.

## Summary

- **File**: `MaerklinMotorola.cpp`
- **Lines Executed**: 100.00% (129 of 129 lines)
- **Functions Executed**: 100.00% (4 of 4 functions)

## Testing Environment

The coverage was measured using `g++` with `--coverage` and `gcov` on a host machine. Arduino dependencies were mocked to allow native compilation.

### Mocks used:
- `Arduino.h`: Defines `byte`, `micros()`, and basic types.
- `micros()`: A manual clock controlled by the test runner.

## Covered Scenarios

The following scenarios were implemented in `tests/test_main.cpp` to ensure full coverage of the decoding logic:

1.  **MM1 Locomotive Packets**: Verified decoding of address, function state, and speed.
2.  **Direction Change**: Specifically tested the direction change bit (s=1).
3.  **MM2 Locomotive Packets**: Verified all MM2-specific function indices (F1-F4) and direction states (Forward/Backward) across all 12 valid bit patterns.
4.  **MM2 Special Patterns**: Verified the handling of the `01` bit pattern, which is only valid in the latter half of an MM2 packet.
5.  **Magnet Telegrams**: Verified decoding of turnout/accessory commands, including sub-addresses, port addresses, and red/green states.
6.  **MFX Filtering**: Verified that signals with bit periods between 125µs and 175µs are correctly identified as invalid (filtered).
7.  **Protocol Synchronization**:
    - Gap detection for frame start.
    - Resynchronization on timeout (>500µs gap mid-frame).
8.  **Data Integrity**:
    - Validated that identical consecutive packets are required for the `DataGramState_Validated` state.
    - Verified error handling for invalid frame lengths.
9.  **Queue Management**: Verified circular buffer wraparound of the `DataQueue`.

## Remaining Gaps

None. All lines and branches in `MaerklinMotorola.cpp` are exercised by the current test suite.

## How to Run Tests Locally

```bash
g++ -I./ -I./tests --coverage MaerklinMotorola.cpp tests/test_main.cpp -o coverage_test
./coverage_test
gcov MaerklinMotorola.cpp
```

# Code Coverage Report

This document details the code coverage for the `MaerklinMotorola` library, as measured on Sun Jul  5 20:12:33 UTC 2026.

## Summary

The `MaerklinMotorola` library has been tested using a mock Arduino environment. The tests cover various protocol scenarios including MM1/MM2 locomotive commands, magnet commands, and error conditions.

| File | Line Coverage |
| --- | --- |
| `MaerklinMotorola.cpp` | 100.00% |

## Methodology

### Environment
The coverage was measured by compiling the library with `g++` and `--coverage` flags on a Linux environment. A mock `Arduino.h` was provided to simulate the Arduino API.

### Test Cases
The test suite in `tests/test_main.cpp` includes:
- **MM1 Locomotive Commands:** Testing address decoding, speed, and function toggles.
- **MM2 Locomotive Commands:** Testing all MM2-specific function indices (F1-F4) and direction states (Forward/Backward).
- **Magnet Commands:** Testing both Green and Red states for magnet decoders.
- **Error Handling:**
    - Detection of MFX signals (filtered out).
    - Handling of invalid trit patterns (01 in the first 5 trits).
    - Timeout and resynchronization logic in signal edge detection.
- **Queue Management:** Testing queue wrapping and data retrieval.

## Detailed Results

### MaerklinMotorola.cpp

```
Lines executed:100.00% of 127
```

All branches and lines in the core decoding logic and state machine have been exercised by the test suite.

## How to reproduce

To run the coverage tests yourself:

1. Ensure you have `g++` and `gcov` installed.
2. Run the following commands:

```bash
mkdir -p tests
# (Ensure tests/Arduino.h and tests/test_main.cpp are present)
g++ -I./ -I./tests --coverage MaerklinMotorola.cpp tests/test_main.cpp -o coverage_test
./coverage_test
gcov coverage_test-MaerklinMotorola.gcda
```

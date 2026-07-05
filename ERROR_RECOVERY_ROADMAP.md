# Error Recovery Roadmap

This document outlines the planned implementation steps to address the vulnerabilities identified in `ERROR_RECOVERY_AUDIT.md`.

## Phase 1: Concurrency Safety
*Goal: Ensure thread safety between the Interrupt Service Routine (ISR) and the main loop.*

- [x] **Variable Decoration:** Mark the following members as `volatile` in `MaerklinMotorola.h`:
    - `last_tm`
    - `sync`
    - `timings_pos`
    - `DataQueueWritePosition`
    - `MaerklinMotorolaData::State`
- [ ] **Atomic Guarding:** Implement `util/atomic.h` (for AVR) or equivalent platform-specific atomic blocks in `Parse()` and `GetData()` to protect state transitions.

## Phase 2: Buffer Management
*Goal: Prevent data corruption from queue overruns during high-traffic periods.*

- [ ] **Overrun Protection in ISR:** Update `PinChange()` to check the state of the next available `DataQueue` slot. If the state is `DataGramState_ReadyToParse` or `DataGramState_Validated`, the ISR should drop the incoming frame and wait for the next sync gap.
- [ ] **Diagnostic Counter:** Add a `volatile unsigned long overrun_count` to track how many frames were dropped due to buffer saturation.

## Phase 3: Signal Robustness
*Goal: Improve noise rejection and signal stability.*

- [ ] **Spike Filtering:** Implement a simple software low-pass filter in `PinChange()` to ignore pulses shorter than a minimum threshold (e.g., 20µs).
- [ ] **Adaptive Sync Threshold:** Instead of a hardcoded 500µs, investigate using a threshold relative to the measured bit period to improve performance on varied hardware.

## Verification Plan
1. **Unit Testing:** Update the native test suite to simulate concurrent access and verify that atomic blocks prevent race conditions.
2. **Stress Testing:** Use a signal generator (or an Arduino acting as one) to flood the library with packets at maximum speed to verify overrun protection.
3. **CI Integration:** Ensure all changes pass the existing GitHub Actions workflow.

---
Created based on `ERROR_RECOVERY_AUDIT.md`.

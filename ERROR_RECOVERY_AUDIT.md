# Error Handling and Recovery Audit: MaerklinMotorola Library

This document provides a technical audit of the error handling and recovery mechanisms implemented in the MaerklinMotorola Arduino library.

## 1. Signal Acquisition and Synchronization (`PinChange`)

The library handles signal acquisition via pin-change interrupts, focusing on pulse-width timing.

### Mechanisms
*   **Initial Sync:** The library establishes synchronization by looking for a "long" pause (default > 500µs) between pulses.
*   **Mid-Frame Recovery:** If a gap exceeding 500µs is detected *during* bit collection, the library immediately resets its internal bit counter (`timings_pos = 0`) and restarts synchronization. This prevents fragmented or noise-interrupted frames from being processed.
*   **Fixed Edge Counting:** The library expects exactly 35 edges (17.5 bits) per frame. A frame is only marked for parsing (`DataGramState_ReadyToParse`) once this count is reached.

### Identified Risks
*   **ISR/Main Loop Concurrency:** The `DataQueue` state transitions and the `DataQueueWritePosition` are handled across both the ISR and the main thread. The absence of `volatile` keywords on several shared variables and lack of atomic guards could lead to race conditions under high CPU load or high signal frequency.
*   **Queue Overrun:** The circular buffer (10 slots) does not check if the next slot is currently occupied by a "ReadyToParse" or "Validated" packet. High-speed protocols (like MM2 at 38.4k/76.8k baud) could potentially overwrite data if the `Parse()` function is not called frequently enough.

## 2. Frame Parsing and Bit Validation (`Parse`)

The `Parse()` function performs bit-level and trit-level sanity checks.

### Mechanisms
*   **Adaptive Thresholding:** The library calculates a bit's duration (period) from the first received bit and uses half of that duration as the threshold for subsequent bits in the frame.
*   **MFX Suppression:** It explicitly identifies and discards signals with periods between 125µs and 175µs. This is an essential recovery feature to ignore mfx/DCC crosstalk on the same rail.
*   **Trit Pattern Sanity:**
    *   The first 5 trits (Address/Function) are strictly validated. If a "01" bit pattern (illegal in the ternary address space) is encountered, the frame is marked as invalid.
    *   MM1 vs MM2 detection is automatic based on the presence of "10" or "01" patterns in the trailing 4 trits.
*   **Protocol Bounds:** Total package duration (`tm_package_delta`) must fall between 1300µs and 4200µs. Frames outside this window are rejected.

## 3. Protocol Redundancy (Double-Packet Rule)

The Märklin-Motorola protocol inherently relies on redundancy for error correction.

### Mechanisms
*   **Redundancy Check:** After successful parsing, the library compares the `Trits` array of the current frame with the previous frame in the queue.
*   **Validation State:** A packet only reaches `DataGramState_Validated` (the only state returned by `GetData()`) if it matches the previous packet exactly.
*   **Automatic Recovery:** If a burst of noise corrupts one frame, the validation fails. The library "recovers" as soon as two subsequent identical and valid frames are received, effectively filtering out transient errors.

## 4. Summary of Audit Findings

| Category | Status | Notes |
| :--- | :--- | :--- |
| **Signal Sync** | Robust | Excellent handling of mid-frame timeouts and resync. |
| **Noise Filtering** | Moderate | MFX suppression is effective; lacks a low-pass filter for very short spikes. |
| **Protocol Compliance** | High | Implements mandatory double-packet validation. |
| **Memory Safety** | Moderate | Circular buffer is simple but lacks overrun protection. |
| **Concurrency** | Low | Potential race conditions due to non-volatile shared state. |

## 5. Recommendations

1.  **Concurrency Safety:** Mark `last_tm`, `sync`, `timings_pos`, `DataQueueWritePosition`, and `State` as `volatile` to ensure the compiler handles ISR-modified variables correctly.
2.  **Overrun Protection:** Modify the `PinChange()` ISR to skip the current buffer slot if its state is still `DataGramState_ReadyToParse` or `DataGramState_Validated`.
3.  **Atomic State Transitions:** Use atomic blocks or simple disabling of interrupts when transitioning states in the `Parse()` and `GetData()` functions to prevent corruption.

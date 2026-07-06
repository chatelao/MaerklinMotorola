# Gaps between Code and Märklin-Motorola Specification

This document identifies discrepancies between the current implementation in `MaerklinMotorola.cpp` and the protocol specification found at `specification/MM_Protocol_Description.html`.

## 1. Address Mapping (Address 80)
*   **Specification:** The ternary address `0000` is defined as address **80**. Address `open open open open` (ternary `2222`) is used for the **Idle State**.
*   **Code:** Currently calculates `Address = Trits[3]*27 + Trits[2]*9 + Trits[1]*3 + Trits[0]`.
    *   Ternary `0000` results in `Address = 0`.
    *   Ternary `2222` results in `Address = 80`.
*   **Gap:** The code lacks the mapping where address 0 should be interpreted as 80, and address 80 should be identified as the Idle/Reset state.

## 2. Idle State Detection
*   **Specification:** The "Idle State" consists of continuous packets with address `open open open open` (ternary `2222`) and a data part of five `0` trits.
*   **Code:** No specific handling for the Idle State. These packets are parsed as regular locomotive packets with address 80 and speed 0.
*   **Gap:** The library does not expose a way to detect or filter the Idle State, which is important for distinguishing between "Stop" commands to a specific loco and the general system idle state.

## 3. MM2 Function Mapping Exceptions (Table 4b)
*   **Specification:** Table 4b defines exceptions for the `EFGH` bits to avoid collisions with the "old" Motorola format. For specific speed steps, the `EFG` bits change from their standard values.
    *   Example: Speed step 2 with `f1` OFF uses `EFGH = 1010` instead of the standard `1100`.
*   **Code:** Uses a static `switch(sMM2)` based on standard `EFGH` values from Table 4a.
*   **Gap:** The current implementation will misinterpret or ignore function commands when the locomotive is at specific speed steps listed in Table 4b (Steps 2, 3, 5, 6, 10, 11, 13, 14).

## 4. "Old" Function Decoders
*   **Specification:** Old function decoders (e.g., for crane or light functions in wagons) use the "double frequency" (like turnouts) but have Trit 4 set to `1` (bits `11`).
*   **Code:** In the magnet telegram section, it explicitly checks `if(DataQueue[QueuePos].Trits[4]==0)`.
*   **Gap:** Old function decoders (Trit 4 = 1) are ignored. The library does not decode these extra functions (f1-f4) for non-locomotive addresses at double frequency.

## 5. Packet Redundancy and Validation
*   **Specification:** Locomotives require two identical consecutive packets to react. The spec mentions that typically 4 or more identical packets are sent for reliability.
*   **Code:** Implements double-packet validation (`memcmp(DataQueue[QueuePos].Trits, DataQueue[previousDataGramPos].Trits, 9)`).
*   **Gap:** While it implements the "double-packet" rule, it might be overly sensitive to noise if it doesn't account for the longer sequences (8 double-packets) mentioned in the spec for speed/direction changes. However, for a decoder implementation, the double-packet rule is generally the minimum required.

## 6. MFX Signal Filtering
*   **Specification:** MFX/mfx+ signals can cause interference.
*   **Code:** Filters packets where the period is between 125µs and 175µs.
*   **Gap:** This seems aligned with common practices, though not explicitly detailed in the provided MM2-only HTML specification, it is a known requirement for robust MM2 decoders.

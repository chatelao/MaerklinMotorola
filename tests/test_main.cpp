#include <iostream>
#include <vector>
#include <assert.h>
#include <string.h>
#include "MaerklinMotorola.h"

unsigned long current_micros = 0;
unsigned long micros() {
    return current_micros;
}

void send_packet(MaerklinMotorola &mm, const std::vector<int>& timings) {
    // Gap to sync
    current_micros += 1000;
    mm.PinChange();

    for (int t : timings) {
        current_micros += t;
        mm.PinChange();
    }
}

std::vector<int> create_timings(const std::vector<int>& bits, int short_t = 50, int long_t = 150) {
    std::vector<int> timings;
    for (int b : bits) {
        if (b == 0) {
            timings.push_back(short_t);
            timings.push_back(long_t);
        } else {
            timings.push_back(long_t);
            timings.push_back(short_t);
        }
    }
    if (timings.size() > 35) timings.resize(35);
    return timings;
}

void test_mm1_loco() {
    MaerklinMotorola mm(2);
    // Ensure all trailing bits are 00 or 11 for MM1
    std::vector<int> bits(18, 0);
    bits[8] = 1; bits[9] = 1; // Trit 4 = 1 -> Function On
    bits[12] = 1; bits[13] = 1; // Trit 6 = 1 (11)
    std::vector<int> timings = create_timings(bits);

    send_packet(mm, timings); mm.Parse();
    send_packet(mm, timings); mm.Parse();

    MaerklinMotorolaData* data = mm.GetData();
    assert(data != nullptr);
    assert(data->Address == 0);
    assert(data->Function == true);
    assert(data->IsMM2 == false);
    std::cout << "test_mm1_loco passed" << std::endl;
}

void test_loco_changedir() {
    MaerklinMotorola mm(2);
    std::vector<int> bits(18, 0);
    // s = 1 (Bits[10]=1) -> ChangeDir = true
    bits[10] = 1; bits[11] = 1; // Trit 5 = 11 (keep it MM1)
    std::vector<int> timings = create_timings(bits);

    send_packet(mm, timings); mm.Parse();
    send_packet(mm, timings); mm.Parse();

    MaerklinMotorolaData* data = mm.GetData();
    assert(data != nullptr);
    assert(data->ChangeDir == true);
    std::cout << "test_loco_changedir passed" << std::endl;
}

void test_mm2_loco_functions() {
    int sMM2_values[] = {2, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15};
    for (int val : sMM2_values) {
        MaerklinMotorola mm(2);
        std::vector<int> bits(18, 0);
        // Trit 0 = 1 (11)
        bits[0] = 1; bits[1] = 1;
        // Trit 5 = 2 (10) -> MM2
        bits[10] = 1; bits[11] = 0;

        // val = Bits[17] + Bits[15]*2 + Bits[13]*4 + Bits[11]*8
        bits[17] = (val >> 0) & 1;
        bits[15] = (val >> 1) & 1;
        bits[13] = (val >> 2) & 1;
        bits[11] = (val >> 3) & 1;

        std::vector<int> timings = create_timings(bits);
        send_packet(mm, timings); mm.Parse();
        send_packet(mm, timings); mm.Parse();

        MaerklinMotorolaData* data = mm.GetData();
        assert(data != nullptr);
        assert(data->IsMM2 == true);
        if (val == 4 || val == 5) assert(data->MM2Direction == MM2DirectionState_Forward);
        if (val == 10 || val == 11) assert(data->MM2Direction == MM2DirectionState_Backward);
        if (val == 3) assert(data->MM2FunctionIndex == 2 && data->IsMM2FunctionOn == true);
        std::cout << "test_mm2_loco_functions val=" << val << " passed" << std::endl;
    }
}

void test_magnet_telegram() {
    {
        // Magnet Green
        MaerklinMotorola mm(2);
        std::vector<int> bits(18, 0);
        // IsMagnet if period < 150. Let's use 30, 70 -> period 100.
        bits[0] = 1; bits[1] = 1; // Trit 0 = 1
        bits[8] = 0; bits[9] = 0; // Trit 4 = 0
        bits[16] = 1; // MagnetState = true
        bits[10] = 1; // DecoderState = Green

        std::vector<int> timings = create_timings(bits, 30, 70);
        send_packet(mm, timings); mm.Parse();
        send_packet(mm, timings); mm.Parse();

        MaerklinMotorolaData* data = mm.GetData();
        assert(data != nullptr);
        assert(data->IsMagnet == true);
        assert(data->MagnetState == true);
        assert(data->DecoderState == MM2DecoderState_Green);
    }
    {
        // Magnet Red
        MaerklinMotorola mm(2);
        std::vector<int> bits(18, 0);
        bits[0] = 1; bits[1] = 1; // Trit 0 = 1
        bits[8] = 0; bits[9] = 0; // Trit 4 = 0
        bits[16] = 1; // MagnetState = true
        bits[10] = 0; // DecoderState = Red

        std::vector<int> timings = create_timings(bits, 30, 70);
        send_packet(mm, timings); mm.Parse();
        send_packet(mm, timings); mm.Parse();

        MaerklinMotorolaData* data = mm.GetData();
        assert(data != nullptr);
        assert(data->DecoderState == MM2DecoderState_Red);
    }
    std::cout << "test_magnet_telegram passed" << std::endl;
}

void test_mm2_01_pattern() {
    {
        // Trit 5 = 3 (01) -> Valid MM2
        MaerklinMotorola mm(2);
        std::vector<int> bits(18, 0);
        bits[10] = 0; bits[11] = 1; // Trit 5 = 01
        std::vector<int> timings = create_timings(bits);
        send_packet(mm, timings); mm.Parse();
        send_packet(mm, timings); mm.Parse();
        MaerklinMotorolaData* data = mm.GetData();
        assert(data != nullptr);
        assert(data->IsMM2 == true);
    }
    {
        // Trit 0 = 01 -> Invalid
        MaerklinMotorola mm(2);
        std::vector<int> bits(18, 0);
        bits[0] = 0; bits[1] = 1; // Trit 0 = 01
        std::vector<int> timings = create_timings(bits);
        send_packet(mm, timings); mm.Parse();
        MaerklinMotorolaData* data = mm.GetData();
        assert(data == nullptr);
    }
    std::cout << "test_mm2_01_pattern passed" << std::endl;
}

void test_mfx_filter() {
    MaerklinMotorola mm(2);
    std::vector<int> timings(35, 75); // period 150
    send_packet(mm, timings); mm.Parse();
    MaerklinMotorolaData* data = mm.GetData();
    assert(data == nullptr);
    std::cout << "test_mfx_filter passed" << std::endl;
}

void test_pinchange_resync() {
    MaerklinMotorola mm(2);
    // Start collecting bits
    current_micros += 1000; mm.PinChange(); // sync
    current_micros += 50; mm.PinChange(); // timings_pos = 1
    // Timeout
    current_micros += 600; mm.PinChange(); // tm_delta > 500, resync
    // Should have timings_pos = 0 now (internally)
    std::cout << "test_pinchange_resync passed" << std::endl;
}

void test_queue_wraparound() {
    MaerklinMotorola mm(2);
    std::vector<int> bits(18, 0);
    std::vector<int> timings = create_timings(bits);

    for (int i=0; i<12; i++) {
        send_packet(mm, timings);
    }
    // Should have wrapped around DataQueueWritePosition
    std::cout << "test_queue_wraparound passed" << std::endl;
}

void test_invalid_length() {
    MaerklinMotorola mm(2);
    std::vector<int> bits(18, 0);
    std::vector<int> timings;
    for(int i=0; i<35; i++) timings.push_back(10);

    send_packet(mm, timings);
    mm.Parse();
    MaerklinMotorolaData* data = mm.GetData();
    assert(data == nullptr);
    std::cout << "test_invalid_length passed" << std::endl;
}

void test_overrun_protection() {
    MaerklinMotorola mm(2);
    std::vector<int> bits(18, 0);
    std::vector<int> timings = create_timings(bits);

    // Fill the queue
    for (int i = 0; i < MM_QUEUE_LENGTH; i++) {
        send_packet(mm, timings);
        // We DON'T call Parse() yet, so they stay in ReadyToParse state
    }

    // Now the queue should be full. The next packet should trigger overrun.
    unsigned long initial_overrun = mm.GetOverrunCount();
    send_packet(mm, timings);
    assert(mm.GetOverrunCount() == initial_overrun + 1);

    // Parse one and get it to free up a slot
    mm.Parse();
    MaerklinMotorolaData* data = mm.GetData();
    assert(data != nullptr);

    // Now we should be able to send another packet without overrun
    initial_overrun = mm.GetOverrunCount();
    send_packet(mm, timings);
    assert(mm.GetOverrunCount() == initial_overrun);

    std::cout << "test_overrun_protection passed" << std::endl;
}

void test_spike_filtering() {
    MaerklinMotorola mm(2);
    std::vector<int> bits(18, 0);
    std::vector<int> timings = create_timings(bits);

    // Normal sync
    current_micros += 1000;
    mm.PinChange();

    // Send bits with a spike
    for (size_t i = 0; i < timings.size(); i++) {
        if (i == 2) { // Inject spike in the middle of a timing interval
            current_micros += 10; // Less than MM_MIN_PULSE_WIDTH (20)
            mm.PinChange();
            current_micros += (timings[i] - 10);
            mm.PinChange();
        } else {
            current_micros += timings[i];
            mm.PinChange();
        }
    }

    mm.Parse();
    // Packet should still be parsed correctly because the spike at 10us was ignored,
    // so the total duration for that timing interval remains timings[2].

    // We need a second packet for validation
    send_packet(mm, timings);
    mm.Parse();

    MaerklinMotorolaData* data = mm.GetData();
    assert(data != nullptr);
    assert(data->Address == 0);
    std::cout << "test_spike_filtering passed" << std::endl;
}

int main() {
    test_mm1_loco();
    test_loco_changedir();
    test_mm2_loco_functions();
    test_magnet_telegram();
    test_mm2_01_pattern();
    test_mfx_filter();
    test_pinchange_resync();
    test_queue_wraparound();
    test_invalid_length();
    test_overrun_protection();
    test_spike_filtering();
    return 0;
}

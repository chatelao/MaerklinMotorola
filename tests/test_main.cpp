#include "Arduino.h"
#include "../MaerklinMotorola.h"
#include <iostream>
#include <vector>
#include <cassert>
#include <cmath>

unsigned long current_micros = 0;

unsigned long micros() {
    return current_micros;
}

void advance_micros(unsigned long delta) {
    current_micros += delta;
}

void send_signal(MaerklinMotorola& mm, const std::vector<int>& timings) {
    // Start with a pause to sync
    advance_micros(1000);
    mm.PinChange();

    for (int t : timings) {
        advance_micros(t);
        mm.PinChange();
    }
}

void append_bit(std::vector<int>& timings, bool bit, int period) {
    if (bit) {
        timings.push_back(period * 3 / 4);
        timings.push_back(period * 1 / 4);
    } else {
        timings.push_back(period * 1 / 4);
        timings.push_back(period * 3 / 4);
    }
}

void append_trit(std::vector<int>& timings, int trit, int period) {
    switch (trit) {
        case 0: // 00
            append_bit(timings, 0, period);
            append_bit(timings, 0, period);
            break;
        case 1: // 11
            append_bit(timings, 1, period);
            append_bit(timings, 1, period);
            break;
        case 2: // 10
            append_bit(timings, 1, period);
            append_bit(timings, 0, period);
            break;
        case 3: // 01
            append_bit(timings, 0, period);
            append_bit(timings, 1, period);
            break;
    }
}

int main() {
    MaerklinMotorola mm(2);

    std::vector<int> timings;
    int period = 200; // > 150 for loco

    auto send_loco = [&](int address, bool function, int speed) {
        timings.clear();
        for (int i=0; i<4; ++i) {
            append_trit(timings, (address / (int)pow(3, i)) % 3, period);
        }
        append_trit(timings, function ? 1 : 0, period);

        append_bit(timings, speed & 1, period);
        append_bit(timings, 0, period);
        append_bit(timings, (speed >> 1) & 1, period);
        append_bit(timings, 0, period);
        append_bit(timings, (speed >> 2) & 1, period);
        append_bit(timings, 0, period);
        append_bit(timings, (speed >> 3) & 1, period);
        append_bit(timings, 0, period);

        if (timings.size() > 35) timings.resize(35);

        send_signal(mm, timings);
        mm.Parse();
        send_signal(mm, timings);
        mm.Parse();
    };

    auto send_loco_mm2 = [&](int address, bool function, int speed, int mm2_extra) {
        timings.clear();
        period = 200;
        for (int i=0; i<4; ++i) {
            append_trit(timings, (address / (int)pow(3, i)) % 3, period);
        }
        append_trit(timings, function ? 1 : 0, period);

        append_bit(timings, speed & 1, period);
        append_bit(timings, mm2_extra & 1, period);
        append_bit(timings, (speed >> 1) & 1, period);
        append_bit(timings, (mm2_extra >> 1) & 1, period);
        append_bit(timings, (speed >> 2) & 1, period);
        append_bit(timings, (mm2_extra >> 2) & 1, period);
        append_bit(timings, (speed >> 3) & 1, period);
        append_bit(timings, (mm2_extra >> 3) & 1, period);

        if (timings.size() > 35) timings.resize(35);

        send_signal(mm, timings);
        mm.Parse();
        send_signal(mm, timings);
        mm.Parse();
    };

    auto send_magnet = [&](int address, int subaddress, bool state, bool bit10) {
        timings.clear();
        period = 100; // < 150 for magnet
        for (int i=0; i<4; ++i) {
            append_trit(timings, (address / (int)pow(3, i)) % 3, period);
        }
        append_trit(timings, 0, period); // Trit 4 = 0 for magnet

        append_bit(timings, bit10, period); // Bit 10
        append_bit(timings, 0, period);               // Bit 11
        append_bit(timings, (subaddress >> 1) & 1, period); // Bit 12
        append_bit(timings, 0, period);               // Bit 13
        append_bit(timings, (subaddress >> 2) & 1, period); // Bit 14
        append_bit(timings, 0, period);               // Bit 15
        append_bit(timings, state ? 1 : 0, period);    // Bit 16
        append_bit(timings, 0, period);               // Bit 17

        if (timings.size() > 35) timings.resize(35);

        send_signal(mm, timings);
        mm.Parse();
        send_signal(mm, timings);
        mm.Parse();
    };

    std::cout << "Testing MM1 Loco..." << std::endl;
    send_loco(1, true, 8);
    MaerklinMotorolaData* data = mm.GetData();
    if (data) {
        std::cout << "Got data! Address: " << (int)data->Address << " Speed: " << (int)data->Speed << " Function: " << (int)data->Function << std::endl;
    }

    std::cout << "Testing MM1 Loco Speed 0..." << std::endl;
    send_loco(1, false, 0);
    data = mm.GetData();
    if (data) std::cout << "Speed: " << (int)data->Speed << " Stop: " << (int)data->Stop << std::endl;

    std::cout << "Testing MM2 extra cases..." << std::endl;
    for (int e=0; e<16; ++e) {
        send_loco_mm2(10, false, 5, e);
        data = mm.GetData();
    }

    std::cout << "Testing Magnet (Red)..." << std::endl;
    send_magnet(5, 1, true, false); // bit10 = false -> Red
    data = mm.GetData();
    if (data) std::cout << "DecoderState: " << (int)data->DecoderState << std::endl;

    std::cout << "Testing Error Case (Mfx)..." << std::endl;
    timings.clear();
    period = 150;
    for (int i=0; i<35; ++i) timings.push_back(75);
    send_signal(mm, timings);
    mm.Parse();
    data = mm.GetData();

    std::cout << "Testing Error Case (Invalid Trit 0-4)..." << std::endl;
    timings.clear();
    period = 200;
    append_bit(timings, 0, period); // Trit 0 first bit
    append_bit(timings, 1, period); // Trit 0 second bit -> 01 pattern
    for (int i=0; i<31; ++i) timings.push_back(100);
    send_signal(mm, timings);
    mm.Parse();
    data = mm.GetData();

    std::cout << "Testing Timeout/Resync in PinChange..." << std::endl;
    advance_micros(1000);
    mm.PinChange();
    advance_micros(100);
    mm.PinChange();
    advance_micros(600);
    mm.PinChange();

    std::cout << "Testing Queue Overflow..." << std::endl;
    for (int i=0; i<15; ++i) {
        send_loco(20, false, 1);
    }

    return 0;
}

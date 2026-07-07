This is a Arduino library for decoding the signals from the Märklin-Motorola-protocol.

You can control your Arduino (and additional actors) with signals from your central control.

See on Wikipedia: [Märklin Digital](https://en.wikipedia.org/wiki/M%C3%A4rklin_Digital)

<hr>

DCC may not be filtered correct at the moment.

Signal polarity from rail isn't relevant. Library is working with timings only.

Pin on Arduino must be Pinchange-interrupt-capable.

**Example-Circuit to get required signal to Arduino:**
<img width="100%" src="https://raw.githubusercontent.com/Laserlicht/MaerklinMotorola/master/circuit.svg">

## Minimal Example

```cpp
#include <MaerklinMotorola.h>

#define INPUT_PIN 2

MaerklinMotorola mm(INPUT_PIN);

void setup() {
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), isr, CHANGE);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  mm.Parse();
  MaerklinMotorolaData* Data = mm.GetData();
  if(Data) {
    // Control builtin LED with address 24 function
    if(!Data->IsMagnet && Data->Address == 24) {
      digitalWrite(LED_BUILTIN, Data->Function);
    }
  }
}

void isr() {
  mm.PinChange();
}
```

## High-level Interface

### Class `MaerklinMotorola`
- `MaerklinMotorola(int pin)`: Constructor.
- `void PinChange()`: Must be called from the ISR on pin change.
- `void Parse()`: Processes captured timings in the main loop.
- `MaerklinMotorolaData* GetData()`: Returns the next parsed telegram or `NULL` if none is available.
- `unsigned long GetOverrunCount() const`: Returns the number of dropped frames due to buffer saturation.

### Struct `MaerklinMotorolaData`
- `unsigned char Address`: Decoder address.
- `unsigned char Speed`: Speed level (0-14).
- `bool Function`: State of the function (f0/Light).
- `bool Stop`: True if the emergency stop bit is set.
- `bool ChangeDir`: True if the direction change bit is set.
- `bool IsMagnet`: True if it's a magnet (turnout/signal) telegram.
- `unsigned char SubAddress`: For magnet telegrams.
- `bool MagnetState`: State of the magnet output.
- `bool IsMM2`: True if decoded as MM2 protocol.
- `MM2DirectionState MM2Direction`: Direction state (`MM2DirectionState_Forward`, `MM2DirectionState_Backward`).
- `bool IsMM2FunctionOn`: State of additional functions (F1-F4).
- `unsigned char MM2FunctionIndex`: Index of the active MM2 function (1-4).

**Additional informations about the protocol**

Protocol:

- http://home.mnet-online.de/modelleisenbahn-digital/Dig-tutorial-start.html

- http://www.drkoenig.de/digital/motorola.htm

- http://www.skrauss.de/modellbahn/Schienenformat.pdf (MFX)

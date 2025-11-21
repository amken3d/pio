# pio

[![Build](https://github.com/tinygo-org/pio/actions/workflows/build.yml/badge.svg)](https://github.com/tinygo-org/pio/actions/workflows/build.yml)
[![Go Reference](https://pkg.go.dev/badge/github.com/tinygo-org/pio.svg)](https://pkg.go.dev/github.com/tinygo-org/pio)

A TinyGo library for programming the **RP2040/RP2350 Programmable I/O (PIO)** block on Raspberry Pi Pico and Pico 2 microcontrollers.

## What is PIO?

The Programmable I/O (PIO) is a unique feature of the RP2040 and RP2350 microcontrollers. It provides a flexible hardware interface that can implement a wide variety of I/O protocols without CPU intervention. Each PIO block contains:

- **4 state machines** that execute PIO assembly programs independently
- **32 instruction slots** for storing programs
- **8 IRQ flags** for synchronization
- **TX/RX FIFOs** (4 words each, configurable) for data transfer

PIO can implement protocols including:
- SPI, DSPI, QSPI
- I2C
- I2S audio
- UART
- 8080/6800 parallel bus
- SDIO
- DPI/VGA (via resistor DAC)
- WS2812B (NeoPixel) LED control
- And many custom protocols

## Supported Platforms

| Platform | Chip | Status |
|----------|------|--------|
| Raspberry Pi Pico | RP2040 | Fully supported |
| Raspberry Pi Pico W | RP2040 | Fully supported |
| Raspberry Pi Pico 2 | RP2350 | Fully supported |

## Installation

```bash
go get github.com/tinygo-org/pio
```

## Quick Start

### Example 1: Blink an LED using PIO

This example demonstrates the basics of PIO programming using the assembler API:

```go
package main

import (
    "machine"
    pio "github.com/tinygo-org/pio/rp2-pio"
)

func main() {
    // Get PIO instance and state machine
    Pio := pio.PIO0
    sm := Pio.StateMachine(0)
    sm.TryClaim()

    // Create a simple blink program using the assembler
    asm := pio.AssemblerV0{SidesetBits: 0}
    program := []uint16{
        asm.Set(pio.SetDestPins, 1).Encode(), // Set pin high
        asm.Set(pio.SetDestPins, 0).Encode(), // Set pin low
    }

    // Load program into PIO memory
    offset, err := Pio.AddProgram(program, -1) // -1 = auto-place
    if err != nil {
        panic(err)
    }

    // Configure state machine
    cfg := pio.DefaultStateMachineConfig()
    cfg.SetWrap(offset, offset+uint8(len(program))-1)
    cfg.SetSetPins(machine.LED, 1)
    cfg.SetClkDivIntFrac(65535, 0) // Slow clock for visible blink

    // Configure pin for PIO
    machine.LED.Configure(machine.PinConfig{Mode: Pio.PinMode()})

    // Initialize and start
    sm.SetPindirsConsecutive(machine.LED, 1, true)
    sm.Init(offset, cfg)
    sm.SetEnabled(true)

    // LED now blinks forever without CPU intervention!
    select {}
}
```

### Example 2: Drive WS2812B (NeoPixel) LEDs

Using the high-level `piolib` drivers:

```go
package main

import (
    "machine"
    "time"

    pio "github.com/tinygo-org/pio/rp2-pio"
    "github.com/tinygo-org/pio/rp2-pio/piolib"
)

func main() {
    // Claim a state machine
    sm, _ := pio.PIO0.ClaimStateMachine()

    // Create WS2812B driver on GPIO16
    ws, err := piolib.NewWS2812B(sm, machine.GPIO16)
    if err != nil {
        panic(err)
    }

    // Enable DMA for efficient transfers
    ws.EnableDMA(true)

    // Cycle through colors
    for {
        ws.PutRGB(255, 0, 0)   // Red
        time.Sleep(time.Second)
        ws.PutRGB(0, 255, 0)   // Green
        time.Sleep(time.Second)
        ws.PutRGB(0, 0, 255)   // Blue
        time.Sleep(time.Second)
    }
}
```

### Example 3: SPI Communication

```go
package main

import (
    "machine"
    pio "github.com/tinygo-org/pio/rp2-pio"
    "github.com/tinygo-org/pio/rp2-pio/piolib"
)

func main() {
    sm, _ := pio.PIO0.ClaimStateMachine()

    spi, err := piolib.NewSPI(sm, machine.SPIConfig{
        Frequency: 1_000_000, // 1 MHz
        SCK:       machine.GPIO2,
        SDO:       machine.GPIO3,
        SDI:       machine.GPIO4,
        Mode:      0,
    })
    if err != nil {
        panic(err)
    }

    // Transfer data
    tx := []byte{0x01, 0x02, 0x03}
    rx := make([]byte, len(tx))
    spi.Tx(tx, rx)
}
```

## Architecture

```
github.com/tinygo-org/pio
├── rp2-pio/              # Core PIO library
│   ├── pio.go            # PIO peripheral management
│   ├── statemachine.go   # State machine control
│   ├── config.go         # Configuration structures
│   ├── instr.go          # AssemblerV0 (RP2040)
│   ├── instrv1.go        # AssemblerV1 (RP2350 extensions)
│   ├── piolib/           # High-level drivers
│   │   ├── ws2812b.go    # NeoPixel LED driver
│   │   ├── spi.go        # SPI protocol driver
│   │   ├── i2s.go        # I2S audio driver
│   │   ├── parallel.go   # Parallel bus driver
│   │   ├── pulsar.go     # Square wave generator
│   │   └── dma.go        # DMA support
│   └── examples/         # Example programs
```

## API Reference

### PIO Peripheral

```go
// Access PIO blocks
pio.PIO0  // First PIO peripheral
pio.PIO1  // Second PIO peripheral

// Load a program into PIO instruction memory
offset, err := Pio.AddProgram(instructions, origin)

// Get a state machine (0-3)
sm := Pio.StateMachine(0)

// Or claim an available state machine
sm, err := Pio.ClaimStateMachine()
```

### State Machine

```go
// Claim ownership
sm.TryClaim()      // Returns true if claimed
sm.IsClaimed()     // Check if claimed
sm.Unclaim()       // Release ownership

// Control
sm.Init(offset, cfg)      // Initialize with program offset and config
sm.SetEnabled(true)       // Start/stop execution
sm.Restart()              // Reset state machine
sm.Exec(instr)            // Execute single instruction immediately

// FIFO Operations
sm.TxPut(data)            // Write to TX FIFO
data := sm.RxGet()        // Read from RX FIFO
sm.IsTxFIFOFull()         // Check TX FIFO status
sm.IsRxFIFOEmpty()        // Check RX FIFO status
sm.ClearFIFOs()           // Clear both FIFOs

// Pin Configuration
sm.SetPindirsConsecutive(pin, count, isOutput)
sm.SetPinsConsecutive(pin, count, level)

// Register Access
sm.SetX(value)            // Set X scratch register
sm.SetY(value)            // Set Y scratch register
sm.GetX()                 // Get X scratch register
sm.GetY()                 // Get Y scratch register
```

### State Machine Configuration

```go
cfg := pio.DefaultStateMachineConfig()

// Clock divider (frequency = CPU_freq / (whole + frac/256))
cfg.SetClkDivIntFrac(whole, frac)

// Program wrapping
cfg.SetWrap(wrapTarget, wrap)

// Pin mapping
cfg.SetOutPins(base, count)      // OUT instruction pins
cfg.SetSetPins(base, count)      // SET instruction pins
cfg.SetInPins(base, count)       // IN instruction pins
cfg.SetSidesetPins(firstPin)     // Sideset pins
cfg.SetJmpPin(pin)               // JMP PIN condition pin

// Shift register configuration
cfg.SetOutShift(shiftRight, autoPull, threshold)
cfg.SetInShift(shiftRight, autoPush, threshold)

// FIFO configuration
cfg.SetFIFOJoin(pio.FifoJoinNone)  // Separate RX/TX (default)
cfg.SetFIFOJoin(pio.FifoJoinTx)    // 8-deep TX FIFO
cfg.SetFIFOJoin(pio.FifoJoinRx)    // 8-deep RX FIFO

// Sideset configuration
cfg.SetSidesetParams(bitCount, optional, pindirs)
```

### Assembler API

Build PIO programs in Go without separate `.pio` files:

```go
asm := pio.AssemblerV0{SidesetBits: 1}

// Instructions (all return instructionV0 for chaining)
asm.Jmp(addr, cond)              // Jump
asm.WaitPin(polarity, pin)       // Wait for pin
asm.WaitGPIO(polarity, pin)      // Wait for GPIO
asm.WaitIRQ(polarity, rel, irq)  // Wait for IRQ
asm.In(src, bitCount)            // Shift in
asm.Out(dest, bitCount)          // Shift out
asm.Push(ifFull, block)          // Push ISR to RX FIFO
asm.Pull(ifEmpty, block)         // Pull TX FIFO to OSR
asm.Mov(dest, src)               // Move/copy
asm.MovInvert(dest, src)         // Move with bit inversion
asm.MovReverse(dest, src)        // Move with bit reversal
asm.IRQSet(relative, irqIndex)   // Set IRQ flag
asm.IRQClear(relative, irqIndex) // Clear IRQ flag
asm.Set(dest, value)             // Set immediate (0-31)
asm.Nop()                        // No operation

// Instruction modifiers
instr.Side(value)                // Add sideset
instr.Delay(cycles)              // Add delay cycles
instr.Encode()                   // Get final uint16
```

### Jump Conditions

```go
pio.JmpAlways       // Unconditional
pio.JmpXZero        // X == 0
pio.JmpXNZeroDec    // X != 0, then X--
pio.JmpYZero        // Y == 0
pio.JmpYNZeroDec    // Y != 0, then Y--
pio.JmpXNotEqualY   // X != Y
pio.JmpPinInput     // Jump pin is high
pio.JmpOSRNotEmpty  // OSR has data
```

### Clock Divider Helpers

```go
// Calculate divider from desired frequency
whole, frac, err := pio.ClkDivFromFrequency(targetFreq, cpuFreq)

// Calculate divider from desired period (nanoseconds)
whole, frac, err := pio.ClkDivFromPeriod(periodNs, cpuFreq)
```

## piolib Drivers

The `piolib` package provides ready-to-use drivers:

### WS2812B (NeoPixel)

```go
ws, err := piolib.NewWS2812B(sm, pin)

ws.PutRGB(r, g, b)              // Queue single LED color
ws.PutColor(color.Color)        // Queue using color.Color
ws.WriteRaw([]uint32{...})      // Write multiple LEDs
ws.EnableDMA(true)              // Use DMA for transfers
ws.IsQueueFull()                // Check if FIFO full
```

### SPI

```go
spi, err := piolib.NewSPI(sm, machine.SPIConfig{
    Frequency: 1_000_000,
    SCK:       sckPin,
    SDO:       mosiPin,
    SDI:       misoPin,
    Mode:      0, // 0 or 1 supported
})

spi.Tx(txBuf, rxBuf)            // Full-duplex transfer
rx, err := spi.Transfer(byte)   // Single byte transfer
```

### I2S Audio

```go
i2s, err := piolib.NewI2S(sm, dataPin, clockBasePin)

i2s.SetSampleFrequency(44100)   // Set sample rate
i2s.WriteMono([]uint16{...})    // Write mono samples
i2s.WriteStereo([]uint32{...})  // Write stereo samples
i2s.Enable(bool)                // Enable/disable
```

### Parallel Bus

```go
par, err := piolib.NewParallel(sm, piolib.ParallelConfig{
    Baud:        10_000_000,     // Bus frequency
    Clock:       clockPin,       // Clock pin
    DataBase:    dataPin,        // First data pin
    BusWidth:    8,              // Number of data pins
    BitsPerPull: 32,             // Bits per FIFO pull
})

par.Tx8([]byte{...})            // Transmit bytes
par.Tx16([]uint16{...})         // Transmit 16-bit words
par.Tx32([]uint32{...})         // Transmit 32-bit words
par.EnableDMA(true)             // Use DMA
```

### Pulsar (Square Wave Generator)

```go
pulsar, err := piolib.NewPulsar(sm, pin)

pulsar.SetPeriod(time.Microsecond * 100)  // Set period
pulsar.TryQueue(1000)                      // Queue 1000 pulses
pulsar.Queued()                            // Check queue depth
pulsar.Pause(true)                         // Pause output
pulsar.Stop()                              // Stop and reset
```

## Developing PIO Programs

### Using the Assembler API

For simple programs, use the Go assembler directly:

```go
asm := pio.AssemblerV0{}
program := []uint16{
    asm.Pull(false, true).Encode(),         // Pull from FIFO, block
    asm.Out(pio.OutDestPins, 8).Encode(),   // Output 8 bits to pins
}
```

### Using pioasm

For complex programs, write `.pio` files and use the `pioasm` tool:

1. **Install pioasm:**
   ```bash
   git clone https://github.com/raspberrypi/pico-sdk.git
   cd pico-sdk/tools/pioasm
   cmake .
   make
   sudo make install
   ```

2. **Create a `.pio` file:**
   ```pio
   ; myprogram.pio
   .program myprogram
   .wrap_target
       pull block
       out pins, 8
   .wrap
   ```

3. **Generate Go code:**
   ```go
   //go:generate pioasm -o go myprogram.pio myprogram_pio.go
   ```

4. **Run code generation:**
   ```bash
   go generate ./...
   ```

5. **Use the generated code:**
   ```go
   offset, _ := Pio.AddProgram(myprogramInstructions, myprogramOrigin)
   cfg := myprogramProgramDefaultConfig(offset)
   ```

## PIO Instruction Reference

| Instruction | Description |
|-------------|-------------|
| `JMP` | Jump to address (conditional or unconditional) |
| `WAIT` | Stall until condition is met (pin, GPIO, or IRQ) |
| `IN` | Shift bits into Input Shift Register (ISR) |
| `OUT` | Shift bits from Output Shift Register (OSR) |
| `PUSH` | Push ISR contents to RX FIFO |
| `PULL` | Pull data from TX FIFO to OSR |
| `MOV` | Copy data between registers/pins |
| `IRQ` | Set, clear, or wait on IRQ flags |
| `SET` | Write immediate value (0-31) to destination |

Each instruction executes in one clock cycle (plus optional delay cycles).

## Tips and Common Pitfalls

1. **SetOutPins vs SetSetPins**: OUT pins are for shifting data (multiple bits), SET pins are for immediate values (0-31). Choose based on your use case.

2. **Pin Configuration**: Always configure pins with `Pio.PinMode()` and set pin directions before enabling the state machine.

3. **Clock Divider**: The minimum divider is 1.0 (no division). Use `ClkDivFromFrequency` for accurate timing calculations.

4. **FIFO Joining**: Join TX FIFOs (`FifoJoinTx`) for output-only applications to get an 8-deep FIFO.

5. **DMA**: Enable DMA for bulk transfers to reduce CPU overhead.

6. **Sideset**: Sideset allows changing pins simultaneously with other instructions. Useful for clock signals.

## Examples

The `rp2-pio/examples/` directory contains complete examples:

- `blinky/` - Basic LED blinking with PIO
- `ws2812b/` - NeoPixel LED control
- `i2s/` - I2S audio output
- `pulsar/` - Square wave generation
- `parallel/` - Parallel bus implementations
- `rxfifoput/` - FIFO direct access (RP2350)

## Resources

- [RP2040 Datasheet - Chapter 3 (PIO)](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf#page=310)
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [Raspberry Pi Pico SDK PIO Examples](https://github.com/raspberrypi/pico-examples/tree/master/pio)
- [TinyGo Documentation](https://tinygo.org/docs/)

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

## License

This project is licensed under the BSD 3-Clause License - see the [LICENSE](LICENSE) file for details.

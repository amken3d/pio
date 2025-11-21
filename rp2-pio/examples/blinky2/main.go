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

	// Create blink program using the assembler
	// This replicates the following PIO assembly:
	//   .program blink
	//   pull block          ; 0: Get delay count from TX FIFO
	//   out y, 32           ; 1: Store in Y register
	//   .wrap_target
	//   mov x, y            ; 2: Copy Y to X for countdown
	//   set pins, 1         ; 3: Turn LED on
	//   lp1: jmp x-- lp1    ; 4: Delay loop (x+1 cycles)
	//   mov x, y            ; 5: Reload X from Y
	//   set pins, 0         ; 6: Turn LED off
	//   lp2: jmp x-- lp2    ; 7: Delay loop (x+1 cycles)
	//   .wrap
	asm := pio.AssemblerV1{SidesetBits: 0}
	program := []uint16{
		asm.Pull(false, true).Encode(),              // 0: pull block
		asm.Out(pio.OutDestY, 32).Encode(),          // 1: out y, 32
		asm.Mov(pio.MovDestX, pio.MovSrcY).Encode(), // 2: mov x, y
		asm.Set(pio.SetDestPins, 1).Encode(),        // 3: set pins, 1
		asm.Jmp(4, pio.JmpXNZeroDec).Encode(),       // 4: jmp x--, 4
		asm.Mov(pio.MovDestX, pio.MovSrcY).Encode(), // 5: mov x, y
		asm.Set(pio.SetDestPins, 0).Encode(),        // 6: set pins, 0
		asm.Jmp(7, pio.JmpXNZeroDec).Encode(),       // 7: jmp x--, 7
	}

	// Load program at offset 0 (jmp addresses are hardcoded)
	offset, err := Pio.AddProgram(program, 0)
	if err != nil {
		panic(err)
	}

	// Configure state machine
	cfg := pio.DefaultStateMachineConfig()
	cfg.SetWrap(offset+2, offset+7) // wrap_target=2, wrap=7
	cfg.SetSetPins(machine.LED, 1)

	// Configure pin for PIO
	machine.LED.Configure(machine.PinConfig{Mode: Pio.PinMode()})

	// Initialize and start
	sm.SetPindirsConsecutive(machine.LED, 1, true)
	sm.Init(offset, cfg)
	sm.SetEnabled(true)

	// Push delay value: cycles per half-blink period
	// For 3Hz blink: delay = CPU_freq / (2 * freq) - 3
	freq := uint32(3)
	sm.TxPut(machine.CPUFrequency()/(2*freq) - 3)

	// LED now blinks forever without CPU intervention!
	select {}
}

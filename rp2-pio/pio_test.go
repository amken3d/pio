package pio

import (
	"testing"
)

func TestAssemblerV1_rxfifo(t *testing.T) {
	// Test cases derived from pioasm-generated code in examples/rxfifoputget/rxfifoputget_pio.go
	// and examples/ws2812bfourpixels (piolib/ws2812bfourpixels_pio.go)
	asm := AssemblerV1{}

	tests := []struct {
		name     string
		got      uint16
		expected uint16
	}{
		// From rxfifoputget_pio.go:
		// 0x8018 = mov rxfifo[0], isr (write ISR to RX FIFO, immediate index 0)
		{"MovISRToRx immediate idx=0", asm.MovISRToRx(true, 0).Encode(), 0x8018},
		// 0x8098 = mov osr, rxfifo[0] (read from RX FIFO to OSR, immediate index 0)
		{"MovOSRFromRx immediate idx=0", asm.MovOSRFromRx(true, 0).Encode(), 0x8098},

		// From ws2812bfourpixels_pio.go:
		// 0x8090 = mov osr, rxfifo[y] (read from RX FIFO indexed by Y)
		{"MovOSRFromRx by Y register", asm.MovOSRFromRx(false, 0).Encode(), 0x8090},

		// Additional test cases for different indices
		{"MovISRToRx immediate idx=1", asm.MovISRToRx(true, 1).Encode(), 0x8019},
		{"MovISRToRx immediate idx=2", asm.MovISRToRx(true, 2).Encode(), 0x801a},
		{"MovISRToRx immediate idx=3", asm.MovISRToRx(true, 3).Encode(), 0x801b},
		{"MovOSRFromRx immediate idx=1", asm.MovOSRFromRx(true, 1).Encode(), 0x8099},
		{"MovOSRFromRx immediate idx=2", asm.MovOSRFromRx(true, 2).Encode(), 0x809a},
		{"MovOSRFromRx immediate idx=3", asm.MovOSRFromRx(true, 3).Encode(), 0x809b},

		// Index by Y register (IdxI=0) - index bits should be 0
		{"MovISRToRx by Y register", asm.MovISRToRx(false, 0).Encode(), 0x8010},
	}

	for _, tt := range tests {
		if tt.got != tt.expected {
			t.Errorf("%s: got %#04x, expected %#04x", tt.name, tt.got, tt.expected)
		}
	}
}

func TestAssemblerV1_irq(t *testing.T) {
	asm := AssemblerV1{}

	tests := []struct {
		name     string
		got      uint16
		expected uint16
	}{
		// IRQ set (clear=0, wait=0)
		{"IRQSet direct idx=0", asm.IRQSet(0, IRQDirect).Encode(), 0xc000},
		{"IRQSet direct idx=1", asm.IRQSet(1, IRQDirect).Encode(), 0xc001},
		{"IRQSet rel idx=0", asm.IRQSet(0, IRQRel).Encode(), 0xc010},
		{"IRQSet prev idx=0", asm.IRQSet(0, IRQPrev).Encode(), 0xc008},
		{"IRQSet next idx=0", asm.IRQSet(0, IRQNext).Encode(), 0xc018},

		// IRQ clear (clear=1, wait=0)
		{"IRQClear direct idx=0", asm.IRQClear(0, IRQDirect).Encode(), 0xc040},
		{"IRQClear rel idx=0", asm.IRQClear(0, IRQRel).Encode(), 0xc050},

		// IRQ wait (clear=0, wait=1)
		{"IRQWait direct idx=0", asm.IRQWait(0, IRQDirect).Encode(), 0xc020},
		{"IRQWait rel idx=0", asm.IRQWait(0, IRQRel).Encode(), 0xc030},
	}

	for _, tt := range tests {
		if tt.got != tt.expected {
			t.Errorf("%s: got %#04x, expected %#04x", tt.name, tt.got, tt.expected)
		}
	}
}

func TestAssemblerV1_waitJmpPin(t *testing.T) {
	asm := AssemblerV1{}

	tests := []struct {
		name     string
		got      uint16
		expected uint16
	}{
		// WaitJmpPin: WAIT instruction (0x2000) with source=0b11 (JMP_PIN)
		// Polarity at bit 7, source at bits 6-5
		{"WaitJmpPin pol=1 idx=0", asm.WaitJmpPin(true, 0).Encode(), 0x20e0},
		{"WaitJmpPin pol=0 idx=0", asm.WaitJmpPin(false, 0).Encode(), 0x2060},
		{"WaitJmpPin pol=1 idx=1", asm.WaitJmpPin(true, 1).Encode(), 0x20e1},
		{"WaitJmpPin pol=1 idx=2", asm.WaitJmpPin(true, 2).Encode(), 0x20e2},
		{"WaitJmpPin pol=1 idx=3", asm.WaitJmpPin(true, 3).Encode(), 0x20e3},
	}

	for _, tt := range tests {
		if tt.got != tt.expected {
			t.Errorf("%s: got %#04x, expected %#04x", tt.name, tt.got, tt.expected)
		}
	}
}

func TestAssemblerV1_waitIRQWithMode(t *testing.T) {
	asm := AssemblerV1{}

	tests := []struct {
		name     string
		got      uint16
		expected uint16
	}{
		// WaitIRQWithMode: WAIT instruction (0x2000) with source=0b10 (IRQ)
		// Polarity at bit 7, source at bits 6-5, idxMode at bits 4-3, irqIndex at bits 2-0
		{"WaitIRQ pol=1 direct idx=0", asm.WaitIRQWithMode(true, 0, IRQDirect).Encode(), 0x20c0},
		{"WaitIRQ pol=0 direct idx=0", asm.WaitIRQWithMode(false, 0, IRQDirect).Encode(), 0x2040},
		{"WaitIRQ pol=1 rel idx=0", asm.WaitIRQWithMode(true, 0, IRQRel).Encode(), 0x20d0},
		{"WaitIRQ pol=1 prev idx=0", asm.WaitIRQWithMode(true, 0, IRQPrev).Encode(), 0x20c8},
		{"WaitIRQ pol=1 next idx=0", asm.WaitIRQWithMode(true, 0, IRQNext).Encode(), 0x20d8},
		{"WaitIRQ pol=1 direct idx=3", asm.WaitIRQWithMode(true, 3, IRQDirect).Encode(), 0x20c3},
	}

	for _, tt := range tests {
		if tt.got != tt.expected {
			t.Errorf("%s: got %#04x, expected %#04x", tt.name, tt.got, tt.expected)
		}
	}
}

func TestAssemblerV1_inheritedFromV0(t *testing.T) {
	// Verify that V1 produces identical output to V0 for inherited instructions
	v0 := AssemblerV0{SidesetBits: 1}
	v1 := AssemblerV1{SidesetBits: 1}

	tests := []struct {
		name string
		v0   uint16
		v1   uint16
	}{
		{"Jmp always", v0.Jmp(5, JmpAlways).Encode(), v1.Jmp(5, JmpAlways).Encode()},
		{"Jmp x--", v0.Jmp(0, JmpXNZeroDec).Encode(), v1.Jmp(0, JmpXNZeroDec).Encode()},
		{"Out pins", v0.Out(OutDestPins, 8).Encode(), v1.Out(OutDestPins, 8).Encode()},
		{"In pins", v0.In(InSrcPins, 1).Encode(), v1.In(InSrcPins, 1).Encode()},
		{"Set pindirs", v0.Set(SetDestPindirs, 1).Encode(), v1.Set(SetDestPindirs, 1).Encode()},
		{"Push", v0.Push(false, true).Encode(), v1.Push(false, true).Encode()},
		{"Pull", v0.Pull(false, true).Encode(), v1.Pull(false, true).Encode()},
		{"Mov", v0.Mov(MovDestX, MovSrcY).Encode(), v1.Mov(MovDestX, MovSrcY).Encode()},
		{"MovInvert", v0.MovInvert(MovDestX, MovSrcNull).Encode(), v1.MovInvert(MovDestX, MovSrcNull).Encode()},
		{"MovReverse", v0.MovReverse(MovDestY, MovSrcX).Encode(), v1.MovReverse(MovDestY, MovSrcX).Encode()},
		{"Nop", v0.Nop().Encode(), v1.Nop().Encode()},
		{"WaitPin", v0.WaitPin(true, 0).Encode(), v1.WaitPin(true, 0).Encode()},
		{"WaitGPIO", v0.WaitGPIO(false, 5).Encode(), v1.WaitGPIO(false, 5).Encode()},
		{"WaitIRQ", v0.WaitIRQ(true, false, 0).Encode(), v1.WaitIRQ(true, false, 0).Encode()},
		// Test Side and Delay work correctly
		{"Out with side", v0.Out(OutDestPins, 1).Side(1).Encode(), v1.Out(OutDestPins, 1).Side(1).Encode()},
		{"Nop with delay", v0.Nop().Delay(5).Encode(), v1.Nop().Delay(5).Encode()},
	}

	for _, tt := range tests {
		if tt.v0 != tt.v1 {
			t.Errorf("%s: V0 (%#04x) != V1 (%#04x)", tt.name, tt.v0, tt.v1)
		}
	}
}

func TestAssemblerV0_spi3w(t *testing.T) {
	assm := AssemblerV0{
		SidesetBits: 1,
	}
	const (
		wloopOff = 0
		rloopOff = 5
		endOff   = 7
	)

	var program = []uint16{
		//     .wrap_target
		// write out x-1 bits.
		wloopOff:// Write/Output loop.
		assm.Out(OutDestPins, 1).Side(0).Encode(), //  0: out    pins, 1         side 0
		assm.Jmp(wloopOff, JmpXNZeroDec).Side(1).Encode(), //  1: jmp    x--, 0          side 1
		assm.Jmp(endOff, JmpYZero).Side(0).Encode(),       //  2: jmp    !y, 7           side 0
		assm.Set(SetDestPindirs, 0).Side(0).Encode(),      //  3: set    pindirs, 0      side 0
		assm.Nop().Side(0).Encode(),                       //  4: nop                    side 0
		// read in y-1 bits.
		rloopOff:// Read/input loop
		assm.In(InSrcPins, 1).Side(1).Encode(), // 5: in     pins, 1         side 1
		assm.Jmp(rloopOff, JmpYNZeroDec).Side(0).Encode(), //  6: jmp    y--, 5          side 0
		// Wait for SPI packet on IRQ.
		endOff:// Wait on input pin.
		assm.WaitPin(true, 0).Side(0).Encode(), //  7: wait   1 pin, 0        side 0
		assm.IRQSet(false, 0).Side(0).Encode(), //  8: irq    nowait 0        side 0
	}
	var expectedProgram = []uint16{
		//     .wrap_target
		0x6001, //  0: out    pins, 1         side 0
		0x1040, //  1: jmp    x--, 0          side 1
		0x0067, //  2: jmp    !y, 7           side 0
		0xe080, //  3: set    pindirs, 0      side 0
		0xa042, //  4: nop                    side 0
		0x5001, //  5: in     pins, 1         side 1
		0x0085, //  6: jmp    y--, 5          side 0
		0x20a0, //  7: wait   1 pin, 0        side 0
		0xc000, //  8: irq    nowait 0        side 0
		//     .wrap
	}
	if len(program) != len(expectedProgram) {
		t.Fatal("program length mismatch, this should not be tested")
	}
	for i := range program {
		if program[i] != expectedProgram[i] {
			t.Errorf("instr %d mismatch got!=expected: %#x != %#x", i, program[i], expectedProgram[i])
		}
	}
}

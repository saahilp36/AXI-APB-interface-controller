# AXI4-Lite to APB Bridge

A fully synthesisable AXI4-Lite → APB bridge written in SystemVerilog, targeting the ARM AMBA ecosystem. Implements the complete protocol stack from AXI manager-facing channels down to APB peripheral select, including a self-checking testbench suite and an APB register-file stub peripheral.

This is a common building block in real SoC designs — it sits between a high-performance AXI interconnect (CPU, DMA, NoC) and slower APB peripherals (UART, GPIO, SPI, I²C, timers).

---

## Architecture

```
AXI Manager (CPU / DMA)
        │
        │  5 independent channels
        │  AW  W  B  AR  R
        ▼
┌──────────────────────────────────────────────────┐
│            axi4lite_subordinate                  │
│  Captures AW+W (any order), drives B and R       │
│  wr_req_* / rd_req_* downstream interface        │
└────────────────┬─────────────────────────────────┘
                 │
                 ▼
┌──────────────────────────────────────────────────┐
│              bridge_arbiter                      │
│  Serialises concurrent write + read onto one     │
│  APB port.  Write-priority policy.               │
└────────────────┬─────────────────────────────────┘
                 │
                 ▼
┌──────────────────────────────────────────────────┐
│               apb_manager                        │
│  3-state FSM: IDLE → SETUP → ENABLE              │
│  Handles PREADY wait states, PSLVERR             │
└────────────────┬─────────────────────────────────┘
                 │
                 ▼
┌──────────────────────────────────────────────────┐
│               apb_decoder                        │
│  Address → one-hot PSEL[N]                       │
└────┬──────────┬──────────┬───────────────────────┘
     │          │          │
  Slave 0    Slave 1    Slave N
  (UART)     (GPIO)     (SPI …)
```

---

## File Structure

```
rtl/
  apb_manager.sv            APB manager FSM (IDLE/SETUP/ENABLE)
  axi4lite_subordinate.sv   AXI4-Lite subordinate, all 5 channels
  axi_apb_bridge.sv         Top-level bridge + arbiter + decoder

tb/
  apb_manager_tb.sv         Unit test — APB manager (6 tests, 6 SVA)
  axi4lite_subordinate_tb.sv  Unit test — AXI subordinate (10 tests, 7 SVA)
  apb_regfile.sv            Stub peripheral — 8-register APB register file
  bridge_integration_tb.sv  Integration test — end-to-end (10 tests, 5 SVA)
```

---

## What Each Module Does

### `apb_manager`

Implements the APB3 manager state machine. Accepts a simple `req_valid / req_ready / req_write / req_addr / req_wdata` interface and drives the APB bus signals. Supports unlimited PREADY wait states and PSLVERR error detection.

```
State machine:  IDLE ──► SETUP ──► ENABLE
                  ▲                  │
                  └──────────────────┘  (on PREADY)
```

### `axi4lite_subordinate`

Accepts all five AXI4-Lite channels and exposes a simple downstream request/response interface. Key design points:

- **AW and W channels are fully independent** — either can arrive first; both are captured before the downstream is invoked.
- Write and read state machines run concurrently — a read can be in flight while the arbiter processes a write.
- PSLVERR propagates back as `BRESP = SLVERR` (2'b10) or `RRESP = SLVERR`.

### `axi_apb_bridge` (top-level)

Structural wrapper that instantiates the three submodules and wires them together. Also contains the `bridge_arbiter` (serialises write/read onto the single APB port) and `apb_decoder` (address → PSEL one-hot).

### `apb_regfile` (test peripheral)

Eight 32-bit read/write registers at `BASE + 0x00 … BASE + 0x1C`. Returns PSLVERR for addresses outside this window. Supports configurable APB wait states for testing the wait-state path.

---

## Simulation

### Prerequisites

| Tool | Version | Notes |
|------|---------|-------|
| Questa Intel FPGA Starter | 23.x or later | Free with Intel FPGA account |
| OR Icarus Verilog | 12+ | Fully free, no licence needed |
| GTKWave | any | Waveform viewer |

### Running with Icarus Verilog (quickest start)

```bash
# Unit test — APB manager
iverilog -g2012 -o sim_apb \
  rtl/apb_manager.sv \
  tb/apb_manager_tb.sv && vvp sim_apb

# Unit test — AXI4-Lite subordinate
iverilog -g2012 -o sim_axi \
  rtl/axi4lite_subordinate.sv \
  tb/axi4lite_subordinate_tb.sv && vvp sim_axi

# Integration test — full bridge
iverilog -g2012 -o sim_bridge \
  rtl/apb_manager.sv \
  rtl/axi4lite_subordinate.sv \
  rtl/axi_apb_bridge.sv \
  tb/apb_regfile.sv \
  tb/bridge_integration_tb.sv && vvp sim_bridge
```

### Running with Questa

```tcl
# From the Questa transcript (or put this in a .do file):
vlog -sv rtl/apb_manager.sv
vlog -sv rtl/axi4lite_subordinate.sv
vlog -sv rtl/axi_apb_bridge.sv
vlog -sv tb/apb_regfile.sv
vlog -sv tb/bridge_integration_tb.sv
vsim bridge_integration_tb -assertdebug
run -all
```

### Expected output

```
============================================================
 AXI-APB Bridge Integration Testbench
============================================================

[Test 1] Read reset values from all 8 registers
  PASS  REG0 reset value
  PASS  REG1 reset value
  ...

[Test 10] Back-to-back writes (4 consecutive)
  PASS  BB write 0 BRESP
  ...

============================================================
 Results: 34 passed, 0 failed
 ALL TESTS PASSED
============================================================
```

---

## Parameters

All modules share these top-level parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ADDR_WIDTH` | 32 | Address bus width (bits) |
| `DATA_WIDTH` | 32 | Data bus width — 32 or 64 per AXI4-Lite spec |
| `NUM_SLAVES` | 1 | Number of APB peripherals (width of PSEL) |

To change the peripheral memory map, edit `BASE_ADDR` and `SLAVE_SIZE` in `apb_decoder` inside `axi_apb_bridge.sv`.

---

## Test Coverage

| Testbench | Tests | SVA assertions | What's covered |
|-----------|-------|----------------|----------------|
| `apb_manager_tb` | 6 | 6 | Write/read, wait states, PSLVERR, back-to-back, protocol |
| `axi4lite_subordinate_tb` | 10 | 7 | All channel orderings, SLVERR, B/R stalls, latency |
| `bridge_integration_tb` | 10+ | 5 | End-to-end, reset values, partial write (WSTRB), bad address, wait states, AW/W ordering, back-to-back |

SVA assertions run continuously throughout simulation and catch protocol violations (VALID stability, PENABLE without PSEL, PADDR stability, one-hot PSEL) as simulation-time errors.

---

## Design Decisions

**Write priority in the arbiter.** When a write and read request arrive simultaneously, the arbiter dispatches the write first. This matches common SoC practice — posted writes have already left the manager's write buffer, while reads are typically on the critical path and can be retried.

**Shared clock.** AXI and APB run on the same clock (`ACLK`). In a design where the APB domain runs at a divided clock, a CDC FIFO would be inserted between the arbiter and the APB manager. That is the natural next extension of this project.

**AW/W decoupling.** The AXI spec allows the write address and write data to arrive in any order. The subordinate uses independent capture registers for each channel and only dispatches to the arbiter once both are valid, correctly handling all orderings without deadlock.
---

## References

- [ARM AMBA AXI4-Lite Protocol Specification](https://developer.arm.com/documentation/ihi0022/latest) — free PDF, ARM developer portal
- [ARM AMBA APB Protocol Specification](https://developer.arm.com/documentation/ihi0024/latest) — free PDF, ~20 pages, read this first
- [Quartus Prime Lite Edition](https://www.intel.com/content/www/us/en/products/details/fpga/development-tools/quartus-prime/resource.html) — free synthesis and simulation

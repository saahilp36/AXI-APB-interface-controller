# AXI4-Lite to APB Bridge

A synthesisable AXI4-Lite to APB bridge written in SystemVerilog, following the ARM AMBA specifications. It takes AXI4-Lite reads and writes, serialises them onto a single APB port, and maps the APB result (PREADY wait states, PSLVERR) back to an AXI response. The repo includes self-checking unit and integration testbenches and an APB register-file peripheral used as the test target.

This is a common building block in SoC designs: it sits between an AXI interconnect (CPU, DMA, NoC) and slower APB peripherals (UART, GPIO, SPI, I²C, timers, configuration registers).

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

Key properties:

| Property | Value |
|---|---|
| Clocking | One clock for AXI and APB (`ACLK` = `PCLK`), active-low synchronous reset |
| Concurrency | One outstanding write and one outstanding read on the AXI side; one APB transfer at a time |
| Arbitration | Fixed priority, write over read, decided only when the arbiter is idle, no preemption |
| Latency | 4 cycles from the AXI handshake to BVALID / RVALID with zero wait states |
| Responses | OKAY (2'b00) or SLVERR (2'b10). Addresses that decode to no slave return SLVERR |
| APB | PREADY wait states, PSLVERR and PSTRB (APB4). PPROT is not implemented |

---

## File Structure

```
rtl/
  apb_manager.sv            APB manager FSM (IDLE/SETUP/ENABLE)
  axi4lite_subordinate.sv   AXI4-Lite subordinate, all 5 channels
  axi_apb_bridge.sv         Top-level bridge + bridge_arbiter + apb_decoder
  apb_regfile.sv            Test peripheral: 8-register APB register file

tb/
  apb_manager_tb.sv            Unit test: APB manager        (7 tests, 7 SVA)
  axi4lite_subordinate_tb.sv   Unit test: AXI subordinate    (12 tests, 13 SVA)
  bridge_integration_tb.sv     Integration test: end to end  (12 tests, 12 SVA)
  tb_stress.sv                 A read is not starved by a stream of writes
  tb_latency.sv                Measures write and read latency

run_sim.sh                  Builds and runs every testbench with Verilator
```

---

## What Each Module Does

### `apb_manager`

Implements the APB manager state machine. It accepts a simple `req_valid / req_ready / req_write / req_addr / req_wdata / req_strb` request interface, drives the APB bus, and returns a one-cycle `rsp_valid / rsp_rdata / rsp_error` pulse when the transfer completes. Supports unlimited PREADY wait states and PSLVERR.

```
State machine:  IDLE ──► SETUP ──► ENABLE ──► IDLE
                                      │
                                      └──────► SETUP   (new request on the completing cycle)
```

A new request is accepted (`req_valid && req_ready`) on the completing ENABLE cycle, so back-to-back transfers skip IDLE.

### `axi4lite_subordinate`

Accepts all five AXI4-Lite channels and exposes a simple downstream request/response interface. Key design points:

- **AW and W channels are fully independent.** Either can arrive first, or both together. Each half is captured as it is handshaken, and one downstream write request is issued once both are held.
- The write and read FSMs run concurrently, so a read can be accepted while a write is in progress.
- One outstanding transaction per direction. AWREADY/WREADY (ARREADY) stay low from the request handshake until the B (R) response has been accepted.
- A downstream error (`wr_rsp_error` / `rd_rsp_error`) becomes `BRESP` / `RRESP` = SLVERR (2'b10).

### `axi_apb_bridge` (top level)

Structural wrapper that instantiates the subordinate, arbiter, manager and decoder, and muxes PREADY / PRDATA / PSLVERR back from the selected slave. The same file contains:

- **`bridge_arbiter`**: a 3-state FSM (`ARB_IDLE`, `ARB_WRITE`, `ARB_READ`) that serialises the write and read requests onto the single APB port. Writes win when both are pending. PSTRB is driven low for reads.
- **`apb_decoder`**: generates the one-hot `PSEL` from the address. Slave `i` owns a 4 KB window at `BASE_ADDR + i * 4096` (`BASE_ADDR` = `0xC000_0000`). If no window matches, no PSEL is raised, `M_PENABLE` stays low, and the response mux returns SLVERR.

### `apb_regfile` (test peripheral)

Eight 32-bit registers at offsets `0x00` to `0x1C`, each reset to its own index so a read after reset is meaningful. Honours PSTRB byte lanes. Returns PSLVERR (and read data `0xDEAD_DEAD`) for out-of-window or unaligned accesses. A `wait_states` input injects exactly N PREADY wait states so the testbenches can exercise that path.

It expects a **window-relative** address, so the integration testbench passes `M_PADDR - 32'hC000_0000`. The bridge itself forwards the full address on `M_PADDR`.

---

## Parameters

| Parameter | Default | Used by | Description |
|-----------|---------|---------|-------------|
| `ADDR_WIDTH` | 32 | all modules | Address bus width (bits) |
| `DATA_WIDTH` | 32 | all modules | Data bus width, 32 or 64 per the AXI4-Lite spec |
| `NUM_SLAVES` | 1 | top level, decoder | Number of APB peripherals (width of PSEL) |

To change the peripheral memory map, edit `BASE_ADDR` and `SLAVE_SIZE` in `apb_decoder` inside `axi_apb_bridge.sv`. They are `localparam`s, so they cannot be overridden from an instance.

---

## Simulation

The testbenches use SystemVerilog concurrent assertions (SVA), so they need a simulator that supports them. Icarus Verilog does not, so it cannot run these testbenches.

### Prerequisites

| Tool | Version | Notes |
|------|---------|-------|
| Verilator | 5.x (tested with 5.034) | Free. Needs `--timing` and `--assert`. This is what `run_sim.sh` uses |
| Questa / ModelSim | recent | Standard flow below. Not part of this repo's regression, so run it once before relying on it |

### Running the whole regression (Verilator)

```bash
./run_sim.sh                              # uses `verilator` from PATH
VERILATOR=/path/to/verilator ./run_sim.sh # or point at a specific build
```

Each testbench ends with `ALL TESTS PASSED` or `SOME TESTS FAILED`, and the script exits non-zero if any of them fails. Assertion failures are counted in that summary.

### Running one testbench by hand

```bash
verilator --binary --timing --assert -Wno-fatal -Wno-lint -Wno-style -Wno-WIDTH \
  -Wno-TIMESCALEMOD --top-module bridge_integration_tb -Mdir build/it -o sim \
  tb/bridge_integration_tb.sv \
  rtl/axi_apb_bridge.sv rtl/axi4lite_subordinate.sv rtl/apb_manager.sv rtl/apb_regfile.sv
./build/it/sim
```

By default Verilator stops at the first assertion error. Add `+verilator+error+limit+100000` when running the binary to see every failure.

### Running with Questa / ModelSim

```tcl
vlib work
vlog -sv rtl/apb_manager.sv rtl/axi4lite_subordinate.sv rtl/axi_apb_bridge.sv rtl/apb_regfile.sv
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
  ...

[Test 12] Constrained-random mix (300 operations)

[Coverage] random test: AW/W ordering simul=21 aw-first=51 w-first=44 | concurrent=44 ...
  PASS  coverage: every APB wait-state count 0..4 exercised
  ...

============================================================
 Results: 577 passed, 0 failed, 0 assertion failures
 ALL TESTS PASSED
============================================================
```

The exact counts differ between simulators because the random test depends on the simulator's random-number generator.

---

## Verification

| Testbench | Tests | SVA | What it covers |
|-----------|-------|-----|----------------|
| `apb_manager_tb` | 7 | 7 | Write/read, wait states, PSLVERR on write and read, PSEL/PENABLE sequencing, back-to-back requests. Checks exact cycle counts and the address/control/data on the APB bus |
| `axi4lite_subordinate_tb` | 12 | 13 | Reset state, AW/W in every order, SLVERR, B/R back-pressure, downstream latency, back-to-back writes, concurrent write and read. Checks the downstream request payload (addr, data, strobes, prot) |
| `bridge_integration_tb` | 12 | 12 | Reset values, write and read-back of all registers, WSTRB byte lanes, bad-register / unaligned / unmapped errors, wait states, write-priority arbitration, AW/W ordering, B/R back-pressure, back-to-back writes, constrained-random mix against a software model |
| `tb_stress` | 1 | 0 | A read issued during 40 back-to-back writes still completes promptly |
| `tb_latency` | 1 | 0 | 4-cycle write and read latency |

The integration testbench compares every read against a software model of the register file, and counts coverage bins (AW/W ordering, concurrency, error responses, each wait-state count, B/R back-pressure) that must all be hit. There are no SystemVerilog covergroups, so it also runs on simulators that do not support them.

The SVA run continuously and catch, as simulation errors: VALID and payload stability on every channel, single outstanding transaction per direction, no response before the downstream response, PSLVERR to SLVERR mapping, PENABLE without PSEL, SETUP followed by ENABLE, address/control stability during a transfer, one-hot PSEL, and PSTRB low on reads.

Testbench conventions: stimulus is driven at the falling clock edge and DUT outputs are sampled at the rising edge before the DUT's nonblocking updates. The B/R channel managers run concurrently with the AW/W/AR drivers.

---

## Design Decisions

**Write priority in the arbiter.** When a write and a read are pending in the same cycle, the arbiter dispatches the write first, and the read waits in the subordinate until the APB port is free. It is a fixed policy decided only while idle, so a transfer in flight is never interrupted. Reads cannot be starved: each AXI direction has one outstanding transaction, so after a write is accepted no further write request appears until its B response has been taken and a new AW/W pair arrives, and a waiting read wins that gap. `tb_stress` shows a read finishing within 8 cycles in the middle of 40 back-to-back writes. With multiple outstanding transactions this would need round-robin or a weighted scheme.

**Shared clock.** AXI and APB run on the same clock (`ACLK`). If the APB domain ran on a divided or separate clock, a CDC FIFO or handshake synchroniser would be inserted between the subordinate and the arbiter. That is the natural next extension.

**AW/W decoupling.** The AXI spec allows the write address and write data to arrive in any order. The subordinate uses independent capture registers for each channel, gives the handshake capture priority over clearing the flags, and only dispatches once both halves are held, so every ordering works without deadlock.

**Unmapped addresses.** An address that falls in no decoder window raises no PSEL and no PENABLE on the external bus. The response mux completes the transfer internally with SLVERR instead of silently returning OKAY.

---

## Known Limitations

- `PRDATA` is one shared bus. A design with `NUM_SLAVES > 1` needs a per-slave read-data mux.
- `AWPROT` / `ARPROT` are captured but not forwarded (there is no PPROT).
- Unmapped addresses return SLVERR; AXI DECERR (2'b11) would be the more precise code.
- Throughput is about one transfer per 5 cycles in steady state (4 to the response, 1 to return to idle).
- The PREADY-to-response path is combinational end to end. Registering the manager's response outputs would shorten it at the cost of one cycle of latency.

---

## Changelog

1. **Write capture (`axi4lite_subordinate`)**: the handshake capture now has priority over clearing the AW/W flags, and the flags clear in `W_WAIT_DS`. Previously the second half of a write was acknowledged but not stored.
2. **`psel_any` (`axi_apb_bridge`)**: removed a second continuous driver that conflicted with the manager's `PSEL` output.
3. **Unmapped addresses (`axi_apb_bridge`)**: no decoded PSEL now returns SLVERR instead of OKAY, and `M_PENABLE` is gated with the decoded PSEL bits.
4. **Manager back-to-back path (`apb_manager`)**: a request presented on the completing ENABLE cycle is accepted and captured; previously the next transfer reused the previous address and data.
5. **PSTRB on reads (`bridge_arbiter`)**: driven low for reads, as APB4 requires.
6. **Accept-gated arbitration (`bridge_arbiter`)**: the arbiter leaves `ARB_IDLE` only when the manager has accepted the request.

---

## References

- [ARM AMBA AXI4-Lite Protocol Specification](https://developer.arm.com/documentation/ihi0022/latest): free PDF, ARM developer portal
- [ARM AMBA APB Protocol Specification](https://developer.arm.com/documentation/ihi0024/latest): free PDF, about 20 pages, read this first
- [Quartus Prime Lite Edition](https://www.intel.com/content/www/us/en/products/details/fpga/development-tools/quartus-prime/resource.html): free synthesis and simulation

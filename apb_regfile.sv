// =============================================================================
// apb_regfile.sv
// APB Subordinate — 8-register read/write register file
//
// Memory map (relative to whatever base address the decoder gives it):
//   offset 0x00  REG0
//   offset 0x04  REG1
//   ...
//   offset 0x1C  REG7
//   offset >= 0x20  → PSLVERR (bad address)
//
// Behaviour
//   - Responds in a single APB cycle (PREADY always 1 when PENABLE=1),
//     except when wait_states input is non-zero (used by testbench to
//     exercise the wait-state path).
//   - Asserts PSLVERR for any address outside the 8-register window.
//   - Registers reset to their index value (REG0=0, REG1=1, …, REG7=7)
//     so read-after-reset is meaningful without a prior write.
//   - PSTRB (byte enables) are honoured on writes.
// =============================================================================

`timescale 1ns / 1ps

module apb_regfile #(
    parameter int ADDR_WIDTH  = 32,
    parameter int DATA_WIDTH  = 32,
    parameter int NUM_REGS    = 8       // must be power-of-2 for simple decode
) (
    input  logic                    PCLK,
    input  logic                    PRESETn,

    // APB subordinate interface
    input  logic                    PSEL,
    input  logic                    PENABLE,
    input  logic                    PWRITE,
    input  logic [ADDR_WIDTH-1:0]   PADDR,
    input  logic [DATA_WIDTH-1:0]   PWDATA,
    input  logic [DATA_WIDTH/8-1:0] PSTRB,

    output logic                    PREADY,
    output logic [DATA_WIDTH-1:0]   PRDATA,
    output logic                    PSLVERR,

    // Testbench wait-state injection (tie to 0 in normal use)
    input  logic [3:0]              wait_states
);

    // =========================================================================
    // Register file storage
    // =========================================================================
    localparam int REG_ADDR_BITS = $clog2(NUM_REGS);

    logic [DATA_WIDTH-1:0] regs [0:NUM_REGS-1];

    // Word index from byte address
    wire [REG_ADDR_BITS-1:0] reg_idx = PADDR[REG_ADDR_BITS+1:2];

    // Address is valid if within the NUM_REGS window
    // (upper bits of PADDR must be zero relative to this peripheral's window)
    wire addr_valid = (PADDR[ADDR_WIDTH-1 : REG_ADDR_BITS+2] == '0) &&
                      (PADDR[1:0] == 2'b00);   // must be word-aligned

    // =========================================================================
    // Wait-state counter
    // =========================================================================
    logic [3:0] ws_count;

    always_ff @(posedge PCLK) begin
        if (!PRESETn)
            ws_count <= '0;
        else if (PSEL && !PENABLE)          // SETUP phase: load counter
            ws_count <= wait_states;
        else if (PSEL && PENABLE && ws_count != '0)
            ws_count <= ws_count - 1;
    end

    assign PREADY = (PSEL && PENABLE) ? (ws_count == '0) : 1'b1;

    // =========================================================================
    // Write path
    // =========================================================================
    always_ff @(posedge PCLK) begin
        if (!PRESETn) begin
            for (int i = 0; i < NUM_REGS; i++)
                regs[i] <= DATA_WIDTH'(i);  // reset to index value
        end else if (PSEL && PENABLE && PWRITE && PREADY && addr_valid) begin
            // Byte-enable aware write
            for (int b = 0; b < DATA_WIDTH/8; b++) begin
                if (PSTRB[b])
                    regs[reg_idx][b*8 +: 8] <= PWDATA[b*8 +: 8];
            end
        end
    end

    // =========================================================================
    // Read path (combinational)
    // =========================================================================
    always_comb begin
        PRDATA  = '0;
        PSLVERR = 1'b0;
        if (PSEL && PENABLE && PREADY) begin
            if (!addr_valid) begin
                PSLVERR = 1'b1;
                PRDATA  = 32'hDEAD_DEAD;
            end else begin
                PRDATA = regs[reg_idx];
            end
        end
    end

endmodule : apb_regfile

// =============================================================================
// apb_manager.sv
// APB (AMBA Advanced Peripheral Bus) Manager
//
// Implements the APB3 manager (master) state machine:
//   IDLE → SETUP → ENABLE → (back to IDLE or SETUP)
//
// Supports:
//   - Single read and write transfers
//   - PREADY wait-state insertion (peripheral can stall indefinitely)
//   - PSLVERR slave-error detection and reporting
//   - Synchronous active-low reset
//
// Port naming follows the ARM AMBA APB3 specification.
// =============================================================================

`timescale 1ns / 1ps

module apb_manager #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32
) (
    // -------------------------------------------------------------------------
    // Global signals
    // -------------------------------------------------------------------------
    input  logic                  PCLK,       // APB clock
    input  logic                  PRESETn,    // Active-low synchronous reset

    // -------------------------------------------------------------------------
    // User-facing request interface
    // (drive these from your AXI bridge or testbench)
    // -------------------------------------------------------------------------
    input  logic                  req_valid,  // Pulse high to start a transfer
    input  logic                  req_write,  // 1 = write, 0 = read
    input  logic [ADDR_WIDTH-1:0] req_addr,   // Target address
    input  logic [DATA_WIDTH-1:0] req_wdata,  // Write data (ignored for reads)
    input  logic [DATA_WIDTH/8-1:0] req_strb, // Write strobes (byte enables)

    output logic                  req_ready,  // High when manager can accept a new request
    output logic                  rsp_valid,  // High for one cycle when transfer completes
    output logic [DATA_WIDTH-1:0] rsp_rdata,  // Read data (valid when rsp_valid & !rsp_error)
    output logic                  rsp_error,  // High when peripheral signalled PSLVERR

    // -------------------------------------------------------------------------
    // APB bus outputs → subordinate (peripheral)
    // -------------------------------------------------------------------------
    output logic                  PSEL,       // Peripheral select
    output logic                  PENABLE,    // Enable (asserted in ENABLE phase)
    output logic                  PWRITE,     // 1 = write, 0 = read
    output logic [ADDR_WIDTH-1:0] PADDR,      // Transfer address
    output logic [DATA_WIDTH-1:0] PWDATA,     // Write data
    output logic [DATA_WIDTH/8-1:0] PSTRB,    // Write strobes (APB4 extension, safe to tie 1s)

    // -------------------------------------------------------------------------
    // APB bus inputs ← subordinate
    // -------------------------------------------------------------------------
    input  logic                  PREADY,     // Peripheral ready (extend with wait states)
    input  logic [DATA_WIDTH-1:0] PRDATA,     // Read data from peripheral
    input  logic                  PSLVERR     // Slave error (APB3/4)
);

    // =========================================================================
    // State encoding
    // =========================================================================
    typedef enum logic [1:0] {
        ST_IDLE   = 2'b00,
        ST_SETUP  = 2'b01,
        ST_ENABLE = 2'b10
    } apb_state_t;

    apb_state_t state, next_state;

    // =========================================================================
    // Internal request capture registers
    // Latch the request on entry to SETUP so the user interface can
    // change inputs without corrupting an in-flight transfer.
    // =========================================================================
    logic                    cap_write;
    logic [ADDR_WIDTH-1:0]   cap_addr;
    logic [DATA_WIDTH-1:0]   cap_wdata;
    logic [DATA_WIDTH/8-1:0] cap_strb;

    // =========================================================================
    // State register
    // =========================================================================
    always_ff @(posedge PCLK) begin
        if (!PRESETn)
            state <= ST_IDLE;
        else
            state <= next_state;
    end

    // =========================================================================
    // Request capture register
    // Sampled on the same clock edge that we leave IDLE → SETUP, so the
    // captured values are stable throughout SETUP and ENABLE.
    // =========================================================================
    always_ff @(posedge PCLK) begin
        if (!PRESETn) begin
            cap_write <= '0;
            cap_addr  <= '0;
            cap_wdata <= '0;
            cap_strb  <= '0;
        end else if (state == ST_IDLE && req_valid) begin
            cap_write <= req_write;
            cap_addr  <= req_addr;
            cap_wdata <= req_wdata;
            cap_strb  <= req_strb;
        end
    end

    // =========================================================================
    // Next-state logic (combinational)
    // =========================================================================
    always_comb begin
        // Default: stay in current state
        next_state = state;

        unique case (state)

            ST_IDLE: begin
                // Accept a new transfer request
                if (req_valid)
                    next_state = ST_SETUP;
            end

            ST_SETUP: begin
                // SETUP is always exactly one cycle long (PSEL=1, PENABLE=0)
                next_state = ST_ENABLE;
            end

            ST_ENABLE: begin
                // Stay here until the peripheral asserts PREADY
                if (PREADY) begin
                    // Transfer complete — go back to SETUP immediately if
                    // another request is already waiting, otherwise IDLE.
                    next_state = req_valid ? ST_SETUP : ST_IDLE;
                end
            end

            default: next_state = ST_IDLE;

        endcase
    end

    // =========================================================================
    // APB output drive (combinational, registered in the peripheral)
    // =========================================================================

    // PSEL: asserted during SETUP and ENABLE, deasserted in IDLE
    assign PSEL    = (state == ST_SETUP) || (state == ST_ENABLE);

    // PENABLE: asserted only during ENABLE
    assign PENABLE = (state == ST_ENABLE);

    // Address/data/control: driven from captured registers once in SETUP
    // Use mux: in IDLE drive zeros to avoid spurious peripheral decodes
    assign PWRITE  = (state == ST_IDLE) ? '0 : cap_write;
    assign PADDR   = (state == ST_IDLE) ? '0 : cap_addr;
    assign PWDATA  = (state == ST_IDLE) ? '0 : cap_wdata;
    assign PSTRB   = (state == ST_IDLE) ? '0 : cap_strb;

    // =========================================================================
    // User-facing response outputs
    // =========================================================================

    // Ready to accept a new request only when idle
    // (or when we're about to transition back to IDLE/SETUP on this cycle)
    assign req_ready = (state == ST_IDLE) ||
                       (state == ST_ENABLE && PREADY && !req_valid);

    // Response fires for exactly one cycle when ENABLE completes
    assign rsp_valid  = (state == ST_ENABLE) && PREADY;
    assign rsp_rdata  = PRDATA;
    assign rsp_error  = (state == ST_ENABLE) && PREADY && PSLVERR;

endmodule : apb_manager

// =============================================================================
// axi_apb_bridge.sv
// AXI4-Lite to APB Bridge
//
// Structural top-level that connects:
//   axi4lite_subordinate  ←→  bridge_arbiter  ←→  apb_manager
//
// Architecture
// ─────────────────────────────────────────────────────────────────────────────
//
//   AXI manager                                        APB peripherals
//   (CPU/DMA)                                          (UART, GPIO, SPI …)
//       │                                                     │
//       │  AW/W/B/AR/R                                 PSEL/PENABLE/…
//       ▼                                                     ▼
//  ┌──────────────────────────────────────────────────────────────────┐
//  │  axi4lite_subordinate                                            │
//  │    wr_req_* ──────────────────────────────────────────────────►  │
//  │    rd_req_* ──────────────────────────┐                          │
//  │                                       ▼                          │
//  │                             bridge_arbiter                       │
//  │                             (write wins over read)               │
//  │                                       │                          │
//  │                                 req_* ▼                          │
//  │                             apb_manager                          │
//  │    wr_rsp_* ◄─────────────────────────┤                          │
//  │    rd_rsp_* ◄─────────────────────────┘                          │
//  └──────────────────────────────────────────────────────────────────┘
//
// Arbitration policy (bridge_arbiter)
// ─────────────────────────────────────────────────────────────────────────────
//   • Writes are given priority over reads when both are pending simultaneously.
//     This matches typical SoC practice and avoids write-starvation concerns
//     (reads can tolerate a cycle or two of latency; posted writes cannot).
//   • One APB transfer at a time — the APB bus is non-pipelined.
//   • Read and write paths in the AXI subordinate run concurrently; the
//     arbiter serialises them onto the single APB port.
//
// Clock / Reset
// ─────────────────────────────────────────────────────────────────────────────
//   AXI and APB share a single clock (ACLK / PCLK tied together at the top).
//   In a real async-clock design you would insert a CDC FIFO between the
//   subordinate and the arbiter; that is left as an extension exercise.
//
// Parameters
//   ADDR_WIDTH  — address bus width (default 32)
//   DATA_WIDTH  — data bus width, must be 32 or 64 (default 32)
//   NUM_SLAVES  — number of APB peripherals (drives PSEL width, default 1)
//
// =============================================================================

`timescale 1ns / 1ps

module axi_apb_bridge #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32,
    parameter int NUM_SLAVES = 1
) (
    // -------------------------------------------------------------------------
    // Global — shared AXI/APB clock and reset
    // -------------------------------------------------------------------------
    input  logic                    ACLK,
    input  logic                    ARESETn,    // active-low synchronous reset

    // -------------------------------------------------------------------------
    // AXI4-Lite subordinate port (facing the AXI manager / CPU)
    // -------------------------------------------------------------------------
    // AW
    input  logic                    S_AWVALID,
    output logic                    S_AWREADY,
    input  logic [ADDR_WIDTH-1:0]   S_AWADDR,
    input  logic [2:0]              S_AWPROT,
    // W
    input  logic                    S_WVALID,
    output logic                    S_WREADY,
    input  logic [DATA_WIDTH-1:0]   S_WDATA,
    input  logic [DATA_WIDTH/8-1:0] S_WSTRB,
    // B
    output logic                    S_BVALID,
    input  logic                    S_BREADY,
    output logic [1:0]              S_BRESP,
    // AR
    input  logic                    S_ARVALID,
    output logic                    S_ARREADY,
    input  logic [ADDR_WIDTH-1:0]   S_ARADDR,
    input  logic [2:0]              S_ARPROT,
    // R
    output logic                    S_RVALID,
    input  logic                    S_RREADY,
    output logic [DATA_WIDTH-1:0]   S_RDATA,
    output logic [1:0]              S_RRESP,

    // -------------------------------------------------------------------------
    // APB manager port (facing the peripheral bus)
    // -------------------------------------------------------------------------
    output logic [NUM_SLAVES-1:0]   M_PSEL,     // one bit per peripheral
    output logic                    M_PENABLE,
    output logic                    M_PWRITE,
    output logic [ADDR_WIDTH-1:0]   M_PADDR,
    output logic [DATA_WIDTH-1:0]   M_PWDATA,
    output logic [DATA_WIDTH/8-1:0] M_PSTRB,

    input  logic [NUM_SLAVES-1:0]   M_PREADY,   // muxed to selected slave below
    input  logic [DATA_WIDTH-1:0]   M_PRDATA,   // muxed read data
    input  logic [NUM_SLAVES-1:0]   M_PSLVERR   // muxed error
);

    // =========================================================================
    // Internal wires between subordinate and arbiter
    // =========================================================================

    // Write request  (subordinate → arbiter)
    logic                    wr_req_valid;
    logic                    wr_req_ready;
    logic [ADDR_WIDTH-1:0]   wr_req_addr;
    logic [DATA_WIDTH-1:0]   wr_req_data;
    logic [DATA_WIDTH/8-1:0] wr_req_strb;
    logic [2:0]              wr_req_prot;

    // Write response (arbiter → subordinate)
    logic                    wr_rsp_valid;
    logic                    wr_rsp_error;

    // Read request   (subordinate → arbiter)
    logic                    rd_req_valid;
    logic                    rd_req_ready;
    logic [ADDR_WIDTH-1:0]   rd_req_addr;
    logic [2:0]              rd_req_prot;

    // Read response  (arbiter → subordinate)
    logic                    rd_rsp_valid;
    logic [DATA_WIDTH-1:0]   rd_rsp_data;
    logic                    rd_rsp_error;

    // Internal wires between arbiter and APB manager
    logic                    apb_req_valid;
    logic                    apb_req_ready;
    logic                    apb_req_write;
    logic [ADDR_WIDTH-1:0]   apb_req_addr;
    logic [DATA_WIDTH-1:0]   apb_req_wdata;
    logic [DATA_WIDTH/8-1:0] apb_req_strb;

    logic                    apb_rsp_valid;
    logic [DATA_WIDTH-1:0]   apb_rsp_rdata;
    logic                    apb_rsp_error;

    // =========================================================================
    // APB slave-select mux
    //   PSEL is a one-hot vector; PREADY/PRDATA/PSLVERR are muxed back from
    //   whichever slave is selected.  The decoder lives in apb_decoder below.
    // =========================================================================
    logic                    psel_any;       // OR of all PSEL bits → single PSEL to manager
    logic                    pready_mux;
    logic [DATA_WIDTH-1:0]   prdata_mux;
    logic                    pslverr_mux;

    // PSEL to the manager is asserted when any slave is selected
    assign psel_any = |M_PSEL;

    // Mux PREADY/PRDATA/PSLVERR from the selected slave
    // (only one PSEL bit should be high at a time)
    always_comb begin
        pready_mux  = 1'b1;     // default: no slave selected → don't stall
        prdata_mux  = '0;
        pslverr_mux = 1'b0;
        for (int i = 0; i < NUM_SLAVES; i++) begin
            if (M_PSEL[i]) begin
                pready_mux  = M_PREADY[i];
                prdata_mux  = M_PRDATA;      // shared data bus
                pslverr_mux = M_PSLVERR[i];
            end
        end
    end

    // =========================================================================
    // AXI4-Lite subordinate instance
    // =========================================================================
    axi4lite_subordinate #(
        .ADDR_WIDTH (ADDR_WIDTH),
        .DATA_WIDTH (DATA_WIDTH)
    ) u_axi_sub (
        .ACLK           (ACLK),
        .ARESETn         (ARESETn),
        // AW
        .AWVALID        (S_AWVALID),
        .AWREADY        (S_AWREADY),
        .AWADDR         (S_AWADDR),
        .AWPROT         (S_AWPROT),
        // W
        .WVALID         (S_WVALID),
        .WREADY         (S_WREADY),
        .WDATA          (S_WDATA),
        .WSTRB          (S_WSTRB),
        // B
        .BVALID         (S_BVALID),
        .BREADY         (S_BREADY),
        .BRESP          (S_BRESP),
        // AR
        .ARVALID        (S_ARVALID),
        .ARREADY        (S_ARREADY),
        .ARADDR         (S_ARADDR),
        .ARPROT         (S_ARPROT),
        // R
        .RVALID         (S_RVALID),
        .RREADY         (S_RREADY),
        .RDATA          (S_RDATA),
        .RRESP          (S_RRESP),
        // downstream write
        .wr_req_valid   (wr_req_valid),
        .wr_req_ready   (wr_req_ready),
        .wr_req_addr    (wr_req_addr),
        .wr_req_data    (wr_req_data),
        .wr_req_strb    (wr_req_strb),
        .wr_req_prot    (wr_req_prot),
        .wr_rsp_valid   (wr_rsp_valid),
        .wr_rsp_error   (wr_rsp_error),
        // downstream read
        .rd_req_valid   (rd_req_valid),
        .rd_req_ready   (rd_req_ready),
        .rd_req_addr    (rd_req_addr),
        .rd_req_prot    (rd_req_prot),
        .rd_rsp_valid   (rd_rsp_valid),
        .rd_rsp_data    (rd_rsp_data),
        .rd_rsp_error   (rd_rsp_error)
    );

    // =========================================================================
    // Bridge arbiter
    //   Serialises concurrent AXI write and read requests onto the single
    //   APB manager port.  Write-priority policy.
    // =========================================================================
    bridge_arbiter #(
        .ADDR_WIDTH (ADDR_WIDTH),
        .DATA_WIDTH (DATA_WIDTH)
    ) u_arbiter (
        .clk            (ACLK),
        .resetn          (ARESETn),
        // from subordinate
        .wr_req_valid   (wr_req_valid),
        .wr_req_ready   (wr_req_ready),
        .wr_req_addr    (wr_req_addr),
        .wr_req_data    (wr_req_data),
        .wr_req_strb    (wr_req_strb),
        // to subordinate
        .wr_rsp_valid   (wr_rsp_valid),
        .wr_rsp_error   (wr_rsp_error),
        // from subordinate
        .rd_req_valid   (rd_req_valid),
        .rd_req_ready   (rd_req_ready),
        .rd_req_addr    (rd_req_addr),
        // to subordinate
        .rd_rsp_valid   (rd_rsp_valid),
        .rd_rsp_data    (rd_rsp_data),
        .rd_rsp_error   (rd_rsp_error),
        // to APB manager
        .apb_req_valid  (apb_req_valid),
        .apb_req_ready  (apb_req_ready),
        .apb_req_write  (apb_req_write),
        .apb_req_addr   (apb_req_addr),
        .apb_req_wdata  (apb_req_wdata),
        .apb_req_strb   (apb_req_strb),
        // from APB manager
        .apb_rsp_valid  (apb_rsp_valid),
        .apb_rsp_rdata  (apb_rsp_rdata),
        .apb_rsp_error  (apb_rsp_error)
    );

    // =========================================================================
    // APB manager instance
    // =========================================================================
    apb_manager #(
        .ADDR_WIDTH (ADDR_WIDTH),
        .DATA_WIDTH (DATA_WIDTH)
    ) u_apb_mgr (
        .PCLK           (ACLK),
        .PRESETn         (ARESETn),
        // user request
        .req_valid      (apb_req_valid),
        .req_write      (apb_req_write),
        .req_addr       (apb_req_addr),
        .req_wdata      (apb_req_wdata),
        .req_strb       (apb_req_strb),
        .req_ready      (apb_req_ready),
        .rsp_valid      (apb_rsp_valid),
        .rsp_rdata      (apb_rsp_rdata),
        .rsp_error      (apb_rsp_error),
        // APB bus
        .PSEL           (psel_any),         // driven to decoder, not bus directly
        .PENABLE        (M_PENABLE),
        .PWRITE         (M_PWRITE),
        .PADDR          (M_PADDR),
        .PWDATA         (M_PWDATA),
        .PSTRB          (M_PSTRB),
        .PREADY         (pready_mux),
        .PRDATA         (prdata_mux),
        .PSLVERR        (pslverr_mux)
    );

    // =========================================================================
    // APB address decoder
    //   Generates per-slave PSEL from M_PADDR.
    //   Each slave occupies a 4 KB region starting at BASE_ADDR + i*4096.
    //   Override this with your actual memory map.
    // =========================================================================
    apb_decoder #(
        .ADDR_WIDTH (ADDR_WIDTH),
        .NUM_SLAVES (NUM_SLAVES)
    ) u_decoder (
        .PADDR  (M_PADDR),
        .PSEL_i (psel_any),     // gated by manager's PSEL
        .PSEL_o (M_PSEL)
    );

endmodule : axi_apb_bridge


// =============================================================================
// bridge_arbiter
//   Sits between the AXI subordinate's downstream request ports and the
//   APB manager's request port.
//
//   States
//   ──────
//   ARB_IDLE   : no transfer in flight; accept next request (write wins)
//   ARB_WRITE  : write dispatched to APB manager; waiting for rsp_valid
//   ARB_READ   : read  dispatched to APB manager; waiting for rsp_valid
// =============================================================================

module bridge_arbiter #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32
) (
    input  logic                    clk,
    input  logic                    resetn,

    // ── From AXI subordinate ──────────────────────────────────────────────────
    input  logic                    wr_req_valid,
    output logic                    wr_req_ready,
    input  logic [ADDR_WIDTH-1:0]   wr_req_addr,
    input  logic [DATA_WIDTH-1:0]   wr_req_data,
    input  logic [DATA_WIDTH/8-1:0] wr_req_strb,

    output logic                    wr_rsp_valid,
    output logic                    wr_rsp_error,

    input  logic                    rd_req_valid,
    output logic                    rd_req_ready,
    input  logic [ADDR_WIDTH-1:0]   rd_req_addr,

    output logic                    rd_rsp_valid,
    output logic [DATA_WIDTH-1:0]   rd_rsp_data,
    output logic                    rd_rsp_error,

    // ── To / from APB manager ─────────────────────────────────────────────────
    output logic                    apb_req_valid,
    input  logic                    apb_req_ready,
    output logic                    apb_req_write,
    output logic [ADDR_WIDTH-1:0]   apb_req_addr,
    output logic [DATA_WIDTH-1:0]   apb_req_wdata,
    output logic [DATA_WIDTH/8-1:0] apb_req_strb,

    input  logic                    apb_rsp_valid,
    input  logic [DATA_WIDTH-1:0]   apb_rsp_rdata,
    input  logic                    apb_rsp_error
);

    typedef enum logic [1:0] {
        ARB_IDLE  = 2'b00,
        ARB_WRITE = 2'b01,
        ARB_READ  = 2'b10
    } arb_state_t;

    arb_state_t state, next_state;

    // ── State register ────────────────────────────────────────────────────────
    always_ff @(posedge clk) begin
        if (!resetn) state <= ARB_IDLE;
        else         state <= next_state;
    end

    // ── Next-state logic ──────────────────────────────────────────────────────
    always_comb begin
        next_state = state;
        unique case (state)

            ARB_IDLE: begin
                // Write takes priority over read
                if      (wr_req_valid) next_state = ARB_WRITE;
                else if (rd_req_valid) next_state = ARB_READ;
            end

            ARB_WRITE: begin
                // Stay until APB manager completes the transfer
                if (apb_rsp_valid) next_state = ARB_IDLE;
            end

            ARB_READ: begin
                if (apb_rsp_valid) next_state = ARB_IDLE;
            end

            default: next_state = ARB_IDLE;
        endcase
    end

    // ── APB manager request outputs ───────────────────────────────────────────
    // In IDLE we mux the winning request; in WRITE/READ we hold the captured
    // values steady (apb_manager latches them internally on acceptance).

    assign apb_req_valid = (state == ARB_IDLE)
                           ? (wr_req_valid | rd_req_valid)
                           : 1'b0;   // manager busy; don't present a new req

    assign apb_req_write = wr_req_valid;    // write wins in IDLE mux

    assign apb_req_addr  = (wr_req_valid || state == ARB_WRITE)
                           ? wr_req_addr
                           : rd_req_addr;

    assign apb_req_wdata = wr_req_data;
    assign apb_req_strb  = wr_req_strb;

    // ── Back-pressure to AXI subordinate ─────────────────────────────────────
    // Accept the winning request for exactly one cycle (when APB manager takes it)
    assign wr_req_ready = (state == ARB_IDLE) && wr_req_valid && apb_req_ready;
    assign rd_req_ready = (state == ARB_IDLE) && rd_req_valid && !wr_req_valid && apb_req_ready;

    // ── Response routing ──────────────────────────────────────────────────────
    // Route the APB response back to whichever AXI path dispatched the request
    assign wr_rsp_valid = (state == ARB_WRITE) && apb_rsp_valid;
    assign wr_rsp_error = apb_rsp_error;

    assign rd_rsp_valid = (state == ARB_READ) && apb_rsp_valid;
    assign rd_rsp_data  = apb_rsp_rdata;
    assign rd_rsp_error = apb_rsp_error;

endmodule : bridge_arbiter


// =============================================================================
// apb_decoder
//   Generates one-hot PSEL from PADDR.
//   Each slave owns a 4 KB window: slave[i] at BASE + i*4096.
//   Change the localparam BASE_ADDR and SLAVE_SIZE to match your memory map.
// =============================================================================

module apb_decoder #(
    parameter int ADDR_WIDTH = 32,
    parameter int NUM_SLAVES = 1
) (
    input  logic [ADDR_WIDTH-1:0]   PADDR,
    input  logic                    PSEL_i,     // gated by AXI manager's PSEL
    output logic [NUM_SLAVES-1:0]   PSEL_o
);
    // Base address of peripheral region; each slave gets 4 KB
    localparam logic [ADDR_WIDTH-1:0] BASE_ADDR  = 32'hC000_0000;
    localparam int                    SLAVE_SIZE  = 4096;   // bytes per slave

    always_comb begin
        PSEL_o = '0;
        if (PSEL_i) begin
            for (int i = 0; i < NUM_SLAVES; i++) begin
                if (PADDR >= BASE_ADDR + (i * SLAVE_SIZE) &&
                    PADDR <  BASE_ADDR + ((i + 1) * SLAVE_SIZE))
                    PSEL_o[i] = 1'b1;
            end
        end
    end

endmodule : apb_decoder

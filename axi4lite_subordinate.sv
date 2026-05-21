// =============================================================================
// axi4lite_subordinate.sv
// AXI4-Lite Subordinate (Slave) Interface
//
// Implements the AXI4-Lite subordinate channel handshakes:
//   Write path:  AW (address) + W (data) → internal logic → B (response)
//   Read  path:  AR (address)            → internal logic → R (data+response)
//
// Key AXI4-Lite constraints implemented:
//   - No burst support (AxLEN=0 implied, AxSIZE=full width)
//   - All 5 channels are independent with their own VALID/READY handshakes
//   - AW and W may arrive in any order; both must be captured before
//     the downstream logic is invoked
//   - One outstanding write transaction at a time (AXI4-Lite simplification)
//   - One outstanding read  transaction at a time
//   - RRESP / BRESP: OKAY (2'b00) or SLVERR (2'b10)
//
// Downstream interface (connect to your APB bridge or register file):
//   wr_req_*  — fires when both AW and W have been captured
//   rd_req_*  — fires when AR has been captured
//   *_rsp_*   — downstream returns result; subordinate drives B / R channels
//
// =============================================================================

`timescale 1ns / 1ps

module axi4lite_subordinate #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32   // must be 32 or 64 per AXI4-Lite spec
) (
    // -------------------------------------------------------------------------
    // Global signals
    // -------------------------------------------------------------------------
    input  logic                    ACLK,
    input  logic                    ARESETn,    // active-low synchronous reset

    // -------------------------------------------------------------------------
    // AW channel — Write Address
    // -------------------------------------------------------------------------
    input  logic                    AWVALID,
    output logic                    AWREADY,
    input  logic [ADDR_WIDTH-1:0]   AWADDR,
    input  logic [2:0]              AWPROT,     // captured but not decoded here

    // -------------------------------------------------------------------------
    // W channel — Write Data
    // -------------------------------------------------------------------------
    input  logic                    WVALID,
    output logic                    WREADY,
    input  logic [DATA_WIDTH-1:0]   WDATA,
    input  logic [DATA_WIDTH/8-1:0] WSTRB,

    // -------------------------------------------------------------------------
    // B channel — Write Response
    // -------------------------------------------------------------------------
    output logic                    BVALID,
    input  logic                    BREADY,
    output logic [1:0]              BRESP,      // 2'b00 OKAY, 2'b10 SLVERR

    // -------------------------------------------------------------------------
    // AR channel — Read Address
    // -------------------------------------------------------------------------
    input  logic                    ARVALID,
    output logic                    ARREADY,
    input  logic [ADDR_WIDTH-1:0]   ARADDR,
    input  logic [2:0]              ARPROT,

    // -------------------------------------------------------------------------
    // R channel — Read Data
    // -------------------------------------------------------------------------
    output logic                    RVALID,
    input  logic                    RREADY,
    output logic [DATA_WIDTH-1:0]   RDATA,
    output logic [1:0]              RRESP,

    // -------------------------------------------------------------------------
    // Downstream write request (to APB bridge / register file)
    // -------------------------------------------------------------------------
    output logic                    wr_req_valid,   // pulse: both AW+W captured
    input  logic                    wr_req_ready,   // downstream accepts request
    output logic [ADDR_WIDTH-1:0]   wr_req_addr,
    output logic [DATA_WIDTH-1:0]   wr_req_data,
    output logic [DATA_WIDTH/8-1:0] wr_req_strb,
    output logic [2:0]              wr_req_prot,

    input  logic                    wr_rsp_valid,   // downstream write complete
    input  logic                    wr_rsp_error,   // 1 → BRESP=SLVERR

    // -------------------------------------------------------------------------
    // Downstream read request (to APB bridge / register file)
    // -------------------------------------------------------------------------
    output logic                    rd_req_valid,
    input  logic                    rd_req_ready,
    output logic [ADDR_WIDTH-1:0]   rd_req_addr,
    output logic [2:0]              rd_req_prot,

    input  logic                    rd_rsp_valid,
    input  logic [DATA_WIDTH-1:0]   rd_rsp_data,
    input  logic                    rd_rsp_error
);

    // =========================================================================
    // Local parameters
    // =========================================================================
    localparam logic [1:0] RESP_OKAY   = 2'b00;
    localparam logic [1:0] RESP_SLVERR = 2'b10;

    // =========================================================================
    // Write path state machine
    //
    //  W_IDLE    : waiting for AW and/or W (both must arrive before proceeding)
    //  W_WAIT_DS : both captured, waiting for downstream to accept (wr_req_ready)
    //  W_WAIT_RSP: downstream accepted, waiting for wr_rsp_valid
    //  W_RESP    : driving B channel, waiting for BREADY
    // =========================================================================
    typedef enum logic [1:0] {
        W_IDLE     = 2'b00,
        W_WAIT_DS  = 2'b01,
        W_WAIT_RSP = 2'b10,
        W_RESP     = 2'b11
    } wstate_t;

    wstate_t wstate, wstate_next;

    // AW capture registers
    logic                    aw_captured;
    logic [ADDR_WIDTH-1:0]   cap_awaddr;
    logic [2:0]              cap_awprot;

    // W capture registers
    logic                    w_captured;
    logic [DATA_WIDTH-1:0]   cap_wdata;
    logic [DATA_WIDTH/8-1:0] cap_wstrb;

    // B channel response register
    logic [1:0]              cap_bresp;

    // Both halves of write transaction captured?
    wire both_captured = aw_captured && w_captured;

    // -------------------------------------------------------------------------
    // Write state register
    // -------------------------------------------------------------------------
    always_ff @(posedge ACLK) begin
        if (!ARESETn) wstate <= W_IDLE;
        else          wstate <= wstate_next;
    end

    // -------------------------------------------------------------------------
    // Write next-state logic
    // -------------------------------------------------------------------------
    always_comb begin
        wstate_next = wstate;
        unique case (wstate)

            W_IDLE: begin
                // Only leave IDLE once we have both AW and W.
                // They can arrive in any order on the same cycle or different.
                if (both_captured ||
                    // Arriving simultaneously this cycle:
                    (AWVALID && WVALID) ||
                    // AW arrived previously, W arriving now:
                    (aw_captured && WVALID) ||
                    // W arrived previously, AW arriving now:
                    (w_captured && AWVALID))
                    wstate_next = W_WAIT_DS;
            end

            W_WAIT_DS: begin
                // Wait for downstream to accept the request
                if (wr_req_ready)
                    wstate_next = W_WAIT_RSP;
            end

            W_WAIT_RSP: begin
                // Wait for downstream to complete the write
                if (wr_rsp_valid)
                    wstate_next = W_RESP;
            end

            W_RESP: begin
                // Drive B channel until manager acknowledges
                if (BREADY)
                    wstate_next = W_IDLE;
            end

        endcase
    end

    // -------------------------------------------------------------------------
    // AW capture — accept as soon as we're idle (or capturing W simultaneously)
    // AWREADY: we accept AW whenever we're idle and don't already have it
    // -------------------------------------------------------------------------
    assign AWREADY = (wstate == W_IDLE) && !aw_captured;

    always_ff @(posedge ACLK) begin
        if (!ARESETn) begin
            aw_captured <= 1'b0;
            cap_awaddr  <= '0;
            cap_awprot  <= '0;
        end else begin
            // Clear capture flag when we leave W_IDLE (both captured → dispatch)
            if (wstate == W_IDLE && wstate_next == W_WAIT_DS)
                aw_captured <= 1'b0;
            // Capture on handshake
            else if (AWVALID && AWREADY) begin
                aw_captured <= 1'b1;
                cap_awaddr  <= AWADDR;
                cap_awprot  <= AWPROT;
            end
        end
    end

    // -------------------------------------------------------------------------
    // W capture — accept as soon as we're idle and don't already have it
    // -------------------------------------------------------------------------
    assign WREADY = (wstate == W_IDLE) && !w_captured;

    always_ff @(posedge ACLK) begin
        if (!ARESETn) begin
            w_captured <= 1'b0;
            cap_wdata  <= '0;
            cap_wstrb  <= '0;
        end else begin
            if (wstate == W_IDLE && wstate_next == W_WAIT_DS)
                w_captured <= 1'b0;
            else if (WVALID && WREADY) begin
                w_captured <= 1'b1;
                cap_wdata  <= WDATA;
                cap_wstrb  <= WSTRB;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Downstream write request
    // -------------------------------------------------------------------------
    assign wr_req_valid = (wstate == W_WAIT_DS);
    assign wr_req_addr  = cap_awaddr;
    assign wr_req_data  = cap_wdata;
    assign wr_req_strb  = cap_wstrb;
    assign wr_req_prot  = cap_awprot;

    // -------------------------------------------------------------------------
    // B channel response — latch SLVERR on wr_rsp_valid
    // -------------------------------------------------------------------------
    always_ff @(posedge ACLK) begin
        if (!ARESETn)
            cap_bresp <= RESP_OKAY;
        else if (wstate == W_WAIT_RSP && wr_rsp_valid)
            cap_bresp <= wr_rsp_error ? RESP_SLVERR : RESP_OKAY;
    end

    assign BVALID = (wstate == W_RESP);
    assign BRESP  = cap_bresp;

    // =========================================================================
    // Read path state machine
    //
    //  R_IDLE    : waiting for AR
    //  R_WAIT_DS : AR captured, waiting for downstream to accept
    //  R_WAIT_RSP: downstream accepted, waiting for rd_rsp_valid
    //  R_RESP    : driving R channel, waiting for RREADY
    // =========================================================================
    typedef enum logic [1:0] {
        R_IDLE     = 2'b00,
        R_WAIT_DS  = 2'b01,
        R_WAIT_RSP = 2'b10,
        R_RESP     = 2'b11
    } rstate_t;

    rstate_t rstate, rstate_next;

    logic [ADDR_WIDTH-1:0]  cap_araddr;
    logic [2:0]             cap_arprot;
    logic [DATA_WIDTH-1:0]  cap_rdata;
    logic [1:0]             cap_rresp;

    // -------------------------------------------------------------------------
    // Read state register
    // -------------------------------------------------------------------------
    always_ff @(posedge ACLK) begin
        if (!ARESETn) rstate <= R_IDLE;
        else          rstate <= rstate_next;
    end

    // -------------------------------------------------------------------------
    // Read next-state logic
    // -------------------------------------------------------------------------
    always_comb begin
        rstate_next = rstate;
        unique case (rstate)
            R_IDLE:     if (ARVALID && ARREADY)  rstate_next = R_WAIT_DS;
            R_WAIT_DS:  if (rd_req_ready)         rstate_next = R_WAIT_RSP;
            R_WAIT_RSP: if (rd_rsp_valid)         rstate_next = R_RESP;
            R_RESP:     if (RREADY)               rstate_next = R_IDLE;
            default:                              rstate_next = R_IDLE;
        endcase
    end

    // -------------------------------------------------------------------------
    // AR capture
    // -------------------------------------------------------------------------
    assign ARREADY = (rstate == R_IDLE);

    always_ff @(posedge ACLK) begin
        if (!ARESETn) begin
            cap_araddr <= '0;
            cap_arprot <= '0;
        end else if (ARVALID && ARREADY) begin
            cap_araddr <= ARADDR;
            cap_arprot <= ARPROT;
        end
    end

    // -------------------------------------------------------------------------
    // Downstream read request
    // -------------------------------------------------------------------------
    assign rd_req_valid = (rstate == R_WAIT_DS);
    assign rd_req_addr  = cap_araddr;
    assign rd_req_prot  = cap_arprot;

    // -------------------------------------------------------------------------
    // R channel response — latch data and SLVERR on rd_rsp_valid
    // -------------------------------------------------------------------------
    always_ff @(posedge ACLK) begin
        if (!ARESETn) begin
            cap_rdata <= '0;
            cap_rresp <= RESP_OKAY;
        end else if (rstate == R_WAIT_RSP && rd_rsp_valid) begin
            cap_rdata <= rd_rsp_data;
            cap_rresp <= rd_rsp_error ? RESP_SLVERR : RESP_OKAY;
        end
    end

    assign RVALID = (rstate == R_RESP);
    assign RDATA  = cap_rdata;
    assign RRESP  = cap_rresp;

endmodule : axi4lite_subordinate
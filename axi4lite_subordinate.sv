`timescale 1ns / 1ps

// =============================================================================
// axi4lite_subordinate.sv
// AXI4-Lite subordinate. Independent write (AW/W/B) and read (AR/R) FSMs that
// present a simple valid/ready request and a response pulse to the logic behind.
//   - AW and W may arrive in any order or in the same cycle; both halves are
//     captured, then one downstream write request is issued.
//   - One outstanding write and one outstanding read.
//   - BRESP / RRESP: OKAY (2'b00) or SLVERR (2'b10).
// Fix log:
//   - AW/W capture: the handshake capture now has priority over the flag-clear,
//     and the flags clear in W_WAIT_DS. Previously the second half of a write was
//     acknowledged (AWREADY/WREADY high) but its address/data was never stored.
// =============================================================================
module axi4lite_subordinate #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32
) (
    input  logic                    ACLK,
    input  logic                    ARESETn,
    input  logic                    AWVALID,
    output logic                    AWREADY,
    input  logic [ADDR_WIDTH-1:0]   AWADDR,
    input  logic [2:0]              AWPROT,
    input  logic                    WVALID,
    output logic                    WREADY,
    input  logic [DATA_WIDTH-1:0]   WDATA,
    input  logic [DATA_WIDTH/8-1:0] WSTRB,
    output logic                    BVALID,
    input  logic                    BREADY,
    output logic [1:0]              BRESP,
    input  logic                    ARVALID,
    output logic                    ARREADY,
    input  logic [ADDR_WIDTH-1:0]   ARADDR,
    input  logic [2:0]              ARPROT,
    output logic                    RVALID,
    input  logic                    RREADY,
    output logic [DATA_WIDTH-1:0]   RDATA,
    output logic [1:0]              RRESP,
    output logic                    wr_req_valid,
    input  logic                    wr_req_ready,
    output logic [ADDR_WIDTH-1:0]   wr_req_addr,
    output logic [DATA_WIDTH-1:0]   wr_req_data,
    output logic [DATA_WIDTH/8-1:0] wr_req_strb,
    output logic [2:0]              wr_req_prot,
    input  logic                    wr_rsp_valid,
    input  logic                    wr_rsp_error,
    output logic                    rd_req_valid,
    input  logic                    rd_req_ready,
    output logic [ADDR_WIDTH-1:0]   rd_req_addr,
    output logic [2:0]              rd_req_prot,
    input  logic                    rd_rsp_valid,
    input  logic [DATA_WIDTH-1:0]   rd_rsp_data,
    input  logic                    rd_rsp_error
);
    localparam logic [1:0] RESP_OKAY   = 2'b00;
    localparam logic [1:0] RESP_SLVERR = 2'b10;

    typedef enum logic [1:0] {
        W_IDLE     = 2'b00,
        W_WAIT_DS  = 2'b01,
        W_WAIT_RSP = 2'b10,
        W_RESP     = 2'b11
    } wstate_t;
    wstate_t wstate, wstate_next;

    logic                    aw_captured;
    logic [ADDR_WIDTH-1:0]   cap_awaddr;
    logic [2:0]              cap_awprot;
    logic                    w_captured;
    logic [DATA_WIDTH-1:0]   cap_wdata;
    logic [DATA_WIDTH/8-1:0] cap_wstrb;
    logic [1:0]              cap_bresp;

    wire both_captured = aw_captured && w_captured;

    always_ff @(posedge ACLK) begin
        if (!ARESETn) wstate <= W_IDLE;
        else          wstate <= wstate_next;
    end

    always_comb begin
        wstate_next = wstate;
        unique case (wstate)
            W_IDLE: begin
                if (both_captured ||
                    (AWVALID && WVALID) ||
                    (aw_captured && WVALID) ||
                    (w_captured && AWVALID))
                    wstate_next = W_WAIT_DS;
            end
            W_WAIT_DS: begin
                if (wr_req_ready)
                    wstate_next = W_WAIT_RSP;
            end
            W_WAIT_RSP: begin
                if (wr_rsp_valid)
                    wstate_next = W_RESP;
            end
            W_RESP: begin
                if (BREADY)
                    wstate_next = W_IDLE;
            end
        endcase
    end

    assign AWREADY = (wstate == W_IDLE) && !aw_captured;

    always_ff @(posedge ACLK) begin
        if (!ARESETn) begin
            aw_captured <= 1'b0;
            cap_awaddr  <= '0;
            cap_awprot  <= '0;
        end else begin
            if (AWVALID && AWREADY) begin          // capture has priority
                aw_captured <= 1'b1;
                cap_awaddr  <= AWADDR;
                cap_awprot  <= AWPROT;
            end else if (wstate == W_WAIT_DS)      // clear once dispatched
                aw_captured <= 1'b0;
        end
    end

    assign WREADY = (wstate == W_IDLE) && !w_captured;

    always_ff @(posedge ACLK) begin
        if (!ARESETn) begin
            w_captured <= 1'b0;
            cap_wdata  <= '0;
            cap_wstrb  <= '0;
        end else begin
            if (WVALID && WREADY) begin            // capture has priority
                w_captured <= 1'b1;
                cap_wdata  <= WDATA;
                cap_wstrb  <= WSTRB;
            end else if (wstate == W_WAIT_DS)      // clear once dispatched
                w_captured <= 1'b0;
        end
    end

    assign wr_req_valid = (wstate == W_WAIT_DS);
    assign wr_req_addr  = cap_awaddr;
    assign wr_req_data  = cap_wdata;
    assign wr_req_strb  = cap_wstrb;
    assign wr_req_prot  = cap_awprot;

    always_ff @(posedge ACLK) begin
        if (!ARESETn)
            cap_bresp <= RESP_OKAY;
        else if (wstate == W_WAIT_RSP && wr_rsp_valid)
            cap_bresp <= wr_rsp_error ? RESP_SLVERR : RESP_OKAY;
    end

    assign BVALID = (wstate == W_RESP);
    assign BRESP  = cap_bresp;

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

    always_ff @(posedge ACLK) begin
        if (!ARESETn) rstate <= R_IDLE;
        else          rstate <= rstate_next;
    end

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

    assign rd_req_valid = (rstate == R_WAIT_DS);
    assign rd_req_addr  = cap_araddr;
    assign rd_req_prot  = cap_arprot;

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

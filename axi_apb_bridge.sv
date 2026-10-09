`timescale 1ns / 1ps

// =============================================================================
// axi_apb_bridge.sv
// AXI4-Lite to APB bridge. Contains three modules:
//   axi_apb_bridge  : structural top (subordinate -> arbiter -> manager + decoder)
//   bridge_arbiter  : serializes write and read requests onto the single APB port;
//                     fixed priority, write over read, no mid-transfer preemption
//   apb_decoder     : one-hot PSEL, 4 KB window per slave starting at 0xC000_0000
// Single clock/reset for AXI and APB. One outstanding transaction per direction.
// M_PADDR carries the full address; a peripheral that expects a window-relative
// address needs the base subtracted (done in tb_sys.sv for apb_regfile).
// Fix log:
//   - psel_any now has a single driver (the manager's PSEL output).
//   - Addresses that decode to no slave return SLVERR instead of OKAY.
//   - M_PENABLE is gated with the decoded PSEL bits, so an unmapped access no longer
//     shows PENABLE high with every PSEL low on the external bus.
//   - Arbiter drives PSTRB low on reads (APB4) and only leaves ARB_IDLE when the
//     manager has accepted the request (apb_req_ready).
// =============================================================================
module axi_apb_bridge #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32,
    parameter int NUM_SLAVES = 1
) (
    input  logic                    ACLK,
    input  logic                    ARESETn,
    input  logic                    S_AWVALID,
    output logic                    S_AWREADY,
    input  logic [ADDR_WIDTH-1:0]   S_AWADDR,
    input  logic [2:0]              S_AWPROT,
    input  logic                    S_WVALID,
    output logic                    S_WREADY,
    input  logic [DATA_WIDTH-1:0]   S_WDATA,
    input  logic [DATA_WIDTH/8-1:0] S_WSTRB,
    output logic                    S_BVALID,
    input  logic                    S_BREADY,
    output logic [1:0]              S_BRESP,
    input  logic                    S_ARVALID,
    output logic                    S_ARREADY,
    input  logic [ADDR_WIDTH-1:0]   S_ARADDR,
    input  logic [2:0]              S_ARPROT,
    output logic                    S_RVALID,
    input  logic                    S_RREADY,
    output logic [DATA_WIDTH-1:0]   S_RDATA,
    output logic [1:0]              S_RRESP,
    output logic [NUM_SLAVES-1:0]   M_PSEL,
    output logic                    M_PENABLE,
    output logic                    M_PWRITE,
    output logic [ADDR_WIDTH-1:0]   M_PADDR,
    output logic [DATA_WIDTH-1:0]   M_PWDATA,
    output logic [DATA_WIDTH/8-1:0] M_PSTRB,
    input  logic [NUM_SLAVES-1:0]   M_PREADY,
    input  logic [DATA_WIDTH-1:0]   M_PRDATA,
    input  logic [NUM_SLAVES-1:0]   M_PSLVERR
);
    logic                    wr_req_valid;
    logic                    wr_req_ready;
    logic [ADDR_WIDTH-1:0]   wr_req_addr;
    logic [DATA_WIDTH-1:0]   wr_req_data;
    logic [DATA_WIDTH/8-1:0] wr_req_strb;
    logic [2:0]              wr_req_prot;
    logic                    wr_rsp_valid;
    logic                    wr_rsp_error;
    logic                    rd_req_valid;
    logic                    rd_req_ready;
    logic [ADDR_WIDTH-1:0]   rd_req_addr;
    logic [2:0]              rd_req_prot;
    logic                    rd_rsp_valid;
    logic [DATA_WIDTH-1:0]   rd_rsp_data;
    logic                    rd_rsp_error;
    logic                    apb_req_valid;
    logic                    apb_req_ready;
    logic                    apb_req_write;
    logic [ADDR_WIDTH-1:0]   apb_req_addr;
    logic [DATA_WIDTH-1:0]   apb_req_wdata;
    logic [DATA_WIDTH/8-1:0] apb_req_strb;
    logic                    apb_rsp_valid;
    logic [DATA_WIDTH-1:0]   apb_rsp_rdata;
    logic                    apb_rsp_error;

    logic                    psel_any;
    logic                    penable_int;   // manager's PENABLE before gating with the decoded selects
    logic                    pready_mux;
    logic [DATA_WIDTH-1:0]   prdata_mux;
    logic                    pslverr_mux;

    // psel_any is driven only by apb_manager's PSEL output (u_apb_mgr below).
    // PENABLE is only presented on the external bus when a slave is actually selected;
    // for an unmapped address no PSEL is raised, so PENABLE is held low as well (the
    // transfer still completes internally with SLVERR via the response mux below).
    assign M_PENABLE = penable_int & (|M_PSEL);

    always_comb begin
        pready_mux  = 1'b1;
        prdata_mux  = '0;
        pslverr_mux = psel_any && (M_PSEL == '0);   // no slave decoded -> SLVERR
        for (int i = 0; i < NUM_SLAVES; i++) begin
            if (M_PSEL[i]) begin
                pready_mux  = M_PREADY[i];
                prdata_mux  = M_PRDATA;
                pslverr_mux = M_PSLVERR[i];
            end
        end
    end

    axi4lite_subordinate #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH)) u_axi_sub (
        .ACLK(ACLK), .ARESETn(ARESETn),
        .AWVALID(S_AWVALID), .AWREADY(S_AWREADY), .AWADDR(S_AWADDR), .AWPROT(S_AWPROT),
        .WVALID(S_WVALID), .WREADY(S_WREADY), .WDATA(S_WDATA), .WSTRB(S_WSTRB),
        .BVALID(S_BVALID), .BREADY(S_BREADY), .BRESP(S_BRESP),
        .ARVALID(S_ARVALID), .ARREADY(S_ARREADY), .ARADDR(S_ARADDR), .ARPROT(S_ARPROT),
        .RVALID(S_RVALID), .RREADY(S_RREADY), .RDATA(S_RDATA), .RRESP(S_RRESP),
        .wr_req_valid(wr_req_valid), .wr_req_ready(wr_req_ready), .wr_req_addr(wr_req_addr),
        .wr_req_data(wr_req_data), .wr_req_strb(wr_req_strb), .wr_req_prot(wr_req_prot),
        .wr_rsp_valid(wr_rsp_valid), .wr_rsp_error(wr_rsp_error),
        .rd_req_valid(rd_req_valid), .rd_req_ready(rd_req_ready), .rd_req_addr(rd_req_addr),
        .rd_req_prot(rd_req_prot),
        .rd_rsp_valid(rd_rsp_valid), .rd_rsp_data(rd_rsp_data), .rd_rsp_error(rd_rsp_error)
    );

    bridge_arbiter #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH)) u_arbiter (
        .clk(ACLK), .resetn(ARESETn),
        .wr_req_valid(wr_req_valid), .wr_req_ready(wr_req_ready), .wr_req_addr(wr_req_addr),
        .wr_req_data(wr_req_data), .wr_req_strb(wr_req_strb),
        .wr_rsp_valid(wr_rsp_valid), .wr_rsp_error(wr_rsp_error),
        .rd_req_valid(rd_req_valid), .rd_req_ready(rd_req_ready), .rd_req_addr(rd_req_addr),
        .rd_rsp_valid(rd_rsp_valid), .rd_rsp_data(rd_rsp_data), .rd_rsp_error(rd_rsp_error),
        .apb_req_valid(apb_req_valid), .apb_req_ready(apb_req_ready), .apb_req_write(apb_req_write),
        .apb_req_addr(apb_req_addr), .apb_req_wdata(apb_req_wdata), .apb_req_strb(apb_req_strb),
        .apb_rsp_valid(apb_rsp_valid), .apb_rsp_rdata(apb_rsp_rdata), .apb_rsp_error(apb_rsp_error)
    );

    apb_manager #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH)) u_apb_mgr (
        .PCLK(ACLK), .PRESETn(ARESETn),
        .req_valid(apb_req_valid), .req_write(apb_req_write), .req_addr(apb_req_addr),
        .req_wdata(apb_req_wdata), .req_strb(apb_req_strb), .req_ready(apb_req_ready),
        .rsp_valid(apb_rsp_valid), .rsp_rdata(apb_rsp_rdata), .rsp_error(apb_rsp_error),
        .PSEL(psel_any), .PENABLE(penable_int), .PWRITE(M_PWRITE), .PADDR(M_PADDR),
        .PWDATA(M_PWDATA), .PSTRB(M_PSTRB),
        .PREADY(pready_mux), .PRDATA(prdata_mux), .PSLVERR(pslverr_mux)
    );

    apb_decoder #(.ADDR_WIDTH(ADDR_WIDTH), .NUM_SLAVES(NUM_SLAVES)) u_decoder (
        .PADDR(M_PADDR), .PSEL_i(psel_any), .PSEL_o(M_PSEL)
    );
endmodule : axi_apb_bridge

module bridge_arbiter #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32
) (
    input  logic                    clk,
    input  logic                    resetn,
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

    always_ff @(posedge clk) begin
        if (!resetn) state <= ARB_IDLE;
        else         state <= next_state;
    end

    always_comb begin
        next_state = state;
        unique case (state)
            ARB_IDLE: begin
                if (apb_req_ready) begin
                    if      (wr_req_valid) next_state = ARB_WRITE;
                    else if (rd_req_valid) next_state = ARB_READ;
                end
            end
            ARB_WRITE: if (apb_rsp_valid) next_state = ARB_IDLE;
            ARB_READ:  if (apb_rsp_valid) next_state = ARB_IDLE;
            default: next_state = ARB_IDLE;
        endcase
    end

    assign apb_req_valid = (state == ARB_IDLE) ? (wr_req_valid | rd_req_valid) : 1'b0;
    assign apb_req_write = wr_req_valid;
    assign apb_req_addr  = (wr_req_valid || state == ARB_WRITE) ? wr_req_addr : rd_req_addr;
    assign apb_req_wdata = wr_req_data;
    assign apb_req_strb  = wr_req_valid ? wr_req_strb : '0;   // APB4: PSTRB low on reads

    assign wr_req_ready = (state == ARB_IDLE) && wr_req_valid && apb_req_ready;
    assign rd_req_ready = (state == ARB_IDLE) && rd_req_valid && !wr_req_valid && apb_req_ready;

    assign wr_rsp_valid = (state == ARB_WRITE) && apb_rsp_valid;
    assign wr_rsp_error = apb_rsp_error;
    assign rd_rsp_valid = (state == ARB_READ) && apb_rsp_valid;
    assign rd_rsp_data  = apb_rsp_rdata;
    assign rd_rsp_error = apb_rsp_error;
endmodule : bridge_arbiter

module apb_decoder #(
    parameter int ADDR_WIDTH = 32,
    parameter int NUM_SLAVES = 1
) (
    input  logic [ADDR_WIDTH-1:0]   PADDR,
    input  logic                    PSEL_i,
    output logic [NUM_SLAVES-1:0]   PSEL_o
);
    localparam logic [ADDR_WIDTH-1:0] BASE_ADDR  = 32'hC000_0000;
    localparam int                    SLAVE_SIZE  = 4096;

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

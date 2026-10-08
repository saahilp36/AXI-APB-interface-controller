`timescale 1ns / 1ps

// =============================================================================
// apb_manager.sv
// APB manager FSM: IDLE -> SETUP -> ENABLE (held until PREADY) -> IDLE/SETUP.
// Drives PSEL/PENABLE/PWRITE/PADDR/PWDATA/PSTRB; returns PRDATA and PSLVERR as
// a one-cycle rsp_valid/rsp_rdata/rsp_error pulse on the completing cycle.
// Note: PSTRB is an APB4 signal; there is no PPROT.
// Fix log:
//   - A new request is now accepted AND captured on the completing ENABLE cycle
//     (req_valid && req_ready). Previously the ENABLE->SETUP back-to-back path
//     re-ran the previous transfer's address/data and reported req_ready = 0.
// =============================================================================
module apb_manager #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 32
) (
    input  logic                  PCLK,
    input  logic                  PRESETn,
    input  logic                  req_valid,
    input  logic                  req_write,
    input  logic [ADDR_WIDTH-1:0] req_addr,
    input  logic [DATA_WIDTH-1:0] req_wdata,
    input  logic [DATA_WIDTH/8-1:0] req_strb,
    output logic                  req_ready,
    output logic                  rsp_valid,
    output logic [DATA_WIDTH-1:0] rsp_rdata,
    output logic                  rsp_error,
    output logic                  PSEL,
    output logic                  PENABLE,
    output logic                  PWRITE,
    output logic [ADDR_WIDTH-1:0] PADDR,
    output logic [DATA_WIDTH-1:0] PWDATA,
    output logic [DATA_WIDTH/8-1:0] PSTRB,
    input  logic                  PREADY,
    input  logic [DATA_WIDTH-1:0] PRDATA,
    input  logic                  PSLVERR
);
    typedef enum logic [1:0] {
        ST_IDLE   = 2'b00,
        ST_SETUP  = 2'b01,
        ST_ENABLE = 2'b10
    } apb_state_t;
    apb_state_t state, next_state;

    logic                    cap_write;
    logic [ADDR_WIDTH-1:0]   cap_addr;
    logic [DATA_WIDTH-1:0]   cap_wdata;
    logic [DATA_WIDTH/8-1:0] cap_strb;

    always_ff @(posedge PCLK) begin
        if (!PRESETn) state <= ST_IDLE;
        else          state <= next_state;
    end

    always_ff @(posedge PCLK) begin
        if (!PRESETn) begin
            cap_write <= '0;
            cap_addr  <= '0;
            cap_wdata <= '0;
            cap_strb  <= '0;
        end else if (req_valid && req_ready) begin
            cap_write <= req_write;
            cap_addr  <= req_addr;
            cap_wdata <= req_wdata;
            cap_strb  <= req_strb;
        end
    end

    always_comb begin
        next_state = state;
        unique case (state)
            ST_IDLE:   if (req_valid) next_state = ST_SETUP;
            ST_SETUP:  next_state = ST_ENABLE;
            ST_ENABLE: if (PREADY) begin if (req_valid) next_state = ST_SETUP; else next_state = ST_IDLE; end
            default:   next_state = ST_IDLE;
        endcase
    end

    assign PSEL    = (state == ST_SETUP) || (state == ST_ENABLE);
    assign PENABLE = (state == ST_ENABLE);
    assign PWRITE  = (state == ST_IDLE) ? '0 : cap_write;
    assign PADDR   = (state == ST_IDLE) ? '0 : cap_addr;
    assign PWDATA  = (state == ST_IDLE) ? '0 : cap_wdata;
    assign PSTRB   = (state == ST_IDLE) ? '0 : cap_strb;

    assign req_ready = (state == ST_IDLE) ||
                       (state == ST_ENABLE && PREADY);
    assign rsp_valid  = (state == ST_ENABLE) && PREADY;
    assign rsp_rdata  = PRDATA;
    assign rsp_error  = (state == ST_ENABLE) && PREADY && PSLVERR;
endmodule : apb_manager

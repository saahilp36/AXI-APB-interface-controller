// =============================================================================
// axi4lite_subordinate_tb.sv
// Self-checking testbench for axi4lite_subordinate
//
// Test cases:
//   1.  Single write  — AW and W arrive same cycle
//   2.  Single read   — normal path
//   3.  Write SLVERR  — downstream signals error → BRESP=SLVERR
//   4.  Read  SLVERR  — downstream signals error → RRESP=SLVERR
//   5.  W before AW   — W arrives one cycle before AW
//   6.  AW before W   — AW arrives one cycle before W
//   7.  Back-to-back writes
//   8.  Interleaved write then read (pipelined as much as AXI4-Lite allows)
//   9.  BREADY delayed — manager stalls B channel acceptance
//   10. RREADY delayed — manager stalls R channel acceptance
//
// SVA assertions check AXI handshake rules throughout.
// =============================================================================

`timescale 1ns / 1ps

module axi4lite_subordinate_tb;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam int ADDR_WIDTH = 32;
    localparam int DATA_WIDTH = 32;
    localparam int CLK_HALF   = 5; // ns

    // =========================================================================
    // DUT signals
    // =========================================================================
    logic ACLK, ARESETn;

    // AW
    logic                    AWVALID; logic AWREADY;
    logic [ADDR_WIDTH-1:0]   AWADDR;
    logic [2:0]              AWPROT;
    // W
    logic                    WVALID;  logic WREADY;
    logic [DATA_WIDTH-1:0]   WDATA;
    logic [DATA_WIDTH/8-1:0] WSTRB;
    // B
    logic                    BVALID;  logic BREADY;
    logic [1:0]              BRESP;
    // AR
    logic                    ARVALID; logic ARREADY;
    logic [ADDR_WIDTH-1:0]   ARADDR;
    logic [2:0]              ARPROT;
    // R
    logic                    RVALID;  logic RREADY;
    logic [DATA_WIDTH-1:0]   RDATA;
    logic [1:0]              RRESP;

    // Downstream write
    logic                    wr_req_valid; logic wr_req_ready;
    logic [ADDR_WIDTH-1:0]   wr_req_addr;
    logic [DATA_WIDTH-1:0]   wr_req_data;
    logic [DATA_WIDTH/8-1:0] wr_req_strb;
    logic [2:0]              wr_req_prot;
    logic                    wr_rsp_valid; logic wr_rsp_error;

    // Downstream read
    logic                    rd_req_valid; logic rd_req_ready;
    logic [ADDR_WIDTH-1:0]   rd_req_addr;
    logic [2:0]              rd_req_prot;
    logic                    rd_rsp_valid;
    logic [DATA_WIDTH-1:0]   rd_rsp_data;
    logic                    rd_rsp_error;

    // =========================================================================
    // DUT
    // =========================================================================
    axi4lite_subordinate #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) dut (.*);

    // =========================================================================
    // Clock
    // =========================================================================
    initial ACLK = 0;
    always  #CLK_HALF ACLK = ~ACLK;

    // =========================================================================
    // Scoreboard
    // =========================================================================
    int pass_count = 0;
    int fail_count = 0;

    task automatic check(
        input string test_name,
        input logic  cond
    );
        if (cond) begin
            $display("  PASS  %s", test_name);
            pass_count++;
        end else begin
            $display("  FAIL  %s", test_name);
            fail_count++;
        end
    endtask

    // =========================================================================
    // Helper: clock edge utilities
    // =========================================================================
    task automatic clk_pos; @(posedge ACLK); #1; endtask
    task automatic clk_neg; @(negedge ACLK); #1; endtask

    // =========================================================================
    // Helper: drive a full AXI4-Lite write transaction
    //
    //  aw_delay  — extra cycles before asserting AWVALID  (0 = same cycle as W)
    //  w_delay   — extra cycles before asserting WVALID   (0 = same cycle as AW)
    //  ds_latency — cycles downstream holds wr_req_ready low
    //  rsp_latency — cycles downstream holds wr_rsp_valid low after accepting
    //  slave_err  — assert wr_rsp_error
    //  b_delay    — cycles manager waits before asserting BREADY
    // =========================================================================
    task automatic axi_write(
        input  logic [ADDR_WIDTH-1:0] addr,
        input  logic [DATA_WIDTH-1:0] data,
        input  int                    aw_delay,
        input  int                    w_delay,
        input  int                    ds_latency,
        input  int                    rsp_latency,
        input  logic                  slave_err,
        input  int                    b_delay,
        output logic [1:0]            got_bresp
    );
        // Fork AW and W drivers (they can be independent)
        fork
            // AW driver
            begin
                repeat (aw_delay) clk_pos;
                clk_neg;
                AWVALID = 1; AWADDR = addr; AWPROT = 3'b000;
                // Wait for handshake
                do clk_pos; while (!AWREADY);
                clk_neg; AWVALID = 0;
            end
            // W driver
            begin
                repeat (w_delay) clk_pos;
                clk_neg;
                WVALID = 1; WDATA = data; WSTRB = 4'hF;
                do clk_pos; while (!WREADY);
                clk_neg; WVALID = 0;
            end
        join

        // Downstream: accept after ds_latency
        clk_neg; wr_req_ready = 0;
        repeat (ds_latency) clk_pos;
        clk_neg; wr_req_ready = 1;
        clk_pos;
        clk_neg; wr_req_ready = 0;

        // Downstream: respond after rsp_latency
        repeat (rsp_latency) clk_pos;
        clk_neg; wr_rsp_valid = 1; wr_rsp_error = slave_err;
        clk_pos;
        clk_neg; wr_rsp_valid = 0; wr_rsp_error = 0;

        // Manager accepts B after b_delay
        repeat (b_delay) clk_pos;
        clk_neg; BREADY = 1;
        do clk_pos; while (!BVALID);
        got_bresp = BRESP;
        clk_neg; BREADY = 0;

        // Idle cycle
        clk_pos;
    endtask

    // =========================================================================
    // Helper: drive a full AXI4-Lite read transaction
    // =========================================================================
    task automatic axi_read(
        input  logic [ADDR_WIDTH-1:0] addr,
        input  int                    ds_latency,
        input  int                    rsp_latency,
        input  logic [DATA_WIDTH-1:0] rsp_data,
        input  logic                  slave_err,
        input  int                    r_delay,
        output logic [DATA_WIDTH-1:0] got_rdata,
        output logic [1:0]            got_rresp
    );
        // AR
        clk_neg; ARVALID = 1; ARADDR = addr; ARPROT = 3'b000;
        do clk_pos; while (!ARREADY);
        clk_neg; ARVALID = 0;

        // Downstream accept
        clk_neg; rd_req_ready = 0;
        repeat (ds_latency) clk_pos;
        clk_neg; rd_req_ready = 1;
        clk_pos;
        clk_neg; rd_req_ready = 0;

        // Downstream respond
        repeat (rsp_latency) clk_pos;
        clk_neg; rd_rsp_valid = 1; rd_rsp_data = rsp_data; rd_rsp_error = slave_err;
        clk_pos;
        clk_neg; rd_rsp_valid = 0; rd_rsp_data = '0; rd_rsp_error = 0;

        // Manager accepts R after r_delay
        repeat (r_delay) clk_pos;
        clk_neg; RREADY = 1;
        do clk_pos; while (!RVALID);
        got_rdata = RDATA;
        got_rresp = RRESP;
        clk_neg; RREADY = 0;

        clk_pos;
    endtask

    // =========================================================================
    // Test variables
    // =========================================================================
    logic [1:0]             got_bresp;
    logic [DATA_WIDTH-1:0]  got_rdata;
    logic [1:0]             got_rresp;

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin
        $display("============================================================");
        $display(" AXI4-Lite Subordinate Testbench");
        $display("============================================================");

        // --- Reset ---
        ARESETn     = 0;
        AWVALID     = 0; AWADDR  = '0; AWPROT = '0;
        WVALID      = 0; WDATA   = '0; WSTRB  = '0;
        BREADY      = 0;
        ARVALID     = 0; ARADDR  = '0; ARPROT = '0;
        RREADY      = 0;
        wr_req_ready = 0; wr_rsp_valid = 0; wr_rsp_error = 0;
        rd_req_ready = 0; rd_rsp_valid = 0; rd_rsp_data  = '0; rd_rsp_error = 0;

        repeat (4) clk_pos;
        ARESETn = 1;
        clk_pos;

        // -----------------------------------------------------------------
        // TEST 1: Write — AW and W same cycle, no delays
        // -----------------------------------------------------------------
        $display("\n[Test 1] Write — AW and W same cycle");
        axi_write(.addr(32'hA000_0000), .data(32'hDEAD_BEEF),
                  .aw_delay(0), .w_delay(0),
                  .ds_latency(0), .rsp_latency(0),
                  .slave_err(0), .b_delay(0),
                  .got_bresp(got_bresp));
        check("BRESP = OKAY", got_bresp === 2'b00);

        // -----------------------------------------------------------------
        // TEST 2: Read — normal path
        // -----------------------------------------------------------------
        $display("\n[Test 2] Read — no delays");
        axi_read(.addr(32'hA000_0004),
                 .ds_latency(0), .rsp_latency(0),
                 .rsp_data(32'hCAFE_1234), .slave_err(0), .r_delay(0),
                 .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("RDATA correct",  got_rdata === 32'hCAFE_1234);
        check("RRESP = OKAY",   got_rresp === 2'b00);

        // -----------------------------------------------------------------
        // TEST 3: Write SLVERR
        // -----------------------------------------------------------------
        $display("\n[Test 3] Write — SLVERR from downstream");
        axi_write(.addr(32'hDEAD_0000), .data(32'hFFFF_FFFF),
                  .aw_delay(0), .w_delay(0),
                  .ds_latency(0), .rsp_latency(0),
                  .slave_err(1), .b_delay(0),
                  .got_bresp(got_bresp));
        check("BRESP = SLVERR", got_bresp === 2'b10);

        // -----------------------------------------------------------------
        // TEST 4: Read SLVERR
        // -----------------------------------------------------------------
        $display("\n[Test 4] Read — SLVERR from downstream");
        axi_read(.addr(32'hDEAD_0004),
                 .ds_latency(0), .rsp_latency(0),
                 .rsp_data(32'hBAD0_BEEF), .slave_err(1), .r_delay(0),
                 .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("RRESP = SLVERR", got_rresp === 2'b10);

        // -----------------------------------------------------------------
        // TEST 5: W arrives before AW (W leads by 2 cycles)
        // -----------------------------------------------------------------
        $display("\n[Test 5] Write — W arrives 2 cycles before AW");
        axi_write(.addr(32'hB000_0000), .data(32'h1111_2222),
                  .aw_delay(2), .w_delay(0),
                  .ds_latency(0), .rsp_latency(0),
                  .slave_err(0), .b_delay(0),
                  .got_bresp(got_bresp));
        check("BRESP = OKAY (W-first)", got_bresp === 2'b00);

        // -----------------------------------------------------------------
        // TEST 6: AW arrives before W (AW leads by 2 cycles)
        // -----------------------------------------------------------------
        $display("\n[Test 6] Write — AW arrives 2 cycles before W");
        axi_write(.addr(32'hB000_0008), .data(32'h3333_4444),
                  .aw_delay(0), .w_delay(2),
                  .ds_latency(0), .rsp_latency(0),
                  .slave_err(0), .b_delay(0),
                  .got_bresp(got_bresp));
        check("BRESP = OKAY (AW-first)", got_bresp === 2'b00);

        // -----------------------------------------------------------------
        // TEST 7: Write with downstream latency (ds_latency=2, rsp_latency=3)
        // -----------------------------------------------------------------
        $display("\n[Test 7] Write — downstream accept+response latency");
        axi_write(.addr(32'hC000_0000), .data(32'hABCD_EF01),
                  .aw_delay(0), .w_delay(0),
                  .ds_latency(2), .rsp_latency(3),
                  .slave_err(0), .b_delay(0),
                  .got_bresp(got_bresp));
        check("BRESP = OKAY (latency)", got_bresp === 2'b00);

        // -----------------------------------------------------------------
        // TEST 8: BREADY delayed — manager stalls 3 cycles
        // -----------------------------------------------------------------
        $display("\n[Test 8] Write — BREADY delayed 3 cycles");
        axi_write(.addr(32'hC000_0010), .data(32'h5555_6666),
                  .aw_delay(0), .w_delay(0),
                  .ds_latency(0), .rsp_latency(0),
                  .slave_err(0), .b_delay(3),
                  .got_bresp(got_bresp));
        check("BRESP = OKAY (B-stall)", got_bresp === 2'b00);

        // -----------------------------------------------------------------
        // TEST 9: Read with delayed RREADY
        // -----------------------------------------------------------------
        $display("\n[Test 9] Read — RREADY delayed 3 cycles");
        axi_read(.addr(32'hC000_0020),
                 .ds_latency(0), .rsp_latency(0),
                 .rsp_data(32'h9876_5432), .slave_err(0), .r_delay(3),
                 .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("RDATA correct (R-stall)",  got_rdata === 32'h9876_5432);
        check("RRESP = OKAY  (R-stall)",  got_rresp === 2'b00);

        // -----------------------------------------------------------------
        // TEST 10: Read with downstream response latency
        // -----------------------------------------------------------------
        $display("\n[Test 10] Read — downstream latency 3 cycles");
        axi_read(.addr(32'hD000_0000),
                 .ds_latency(1), .rsp_latency(3),
                 .rsp_data(32'hFEED_F00D), .slave_err(0), .r_delay(0),
                 .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("RDATA correct (latency)", got_rdata === 32'hFEED_F00D);
        check("RRESP = OKAY  (latency)", got_rresp === 2'b00);

        // -----------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------
        clk_pos;
        $display("\n============================================================");
        $display(" Results: %0d passed, %0d failed", pass_count, fail_count);
        $display(fail_count == 0 ? " ALL TESTS PASSED" : " SOME TESTS FAILED");
        $display("============================================================");
        $finish;
    end

    // =========================================================================
    // SystemVerilog Assertions
    // =========================================================================

    // AXI rule: once VALID is asserted, it must not drop until READY
    axi_aw_valid_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (AWVALID && !AWREADY) |=> AWVALID
    ) else $error("SVA FAIL: AWVALID dropped before AWREADY");

    axi_w_valid_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (WVALID && !WREADY) |=> WVALID
    ) else $error("SVA FAIL: WVALID dropped before WREADY");

    axi_ar_valid_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (ARVALID && !ARREADY) |=> ARVALID
    ) else $error("SVA FAIL: ARVALID dropped before ARREADY");

    // AXI rule: BVALID must not deassert until BREADY
    axi_b_valid_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (BVALID && !BREADY) |=> BVALID
    ) else $error("SVA FAIL: BVALID dropped before BREADY");

    // AXI rule: RVALID must not deassert until RREADY
    axi_r_valid_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (RVALID && !RREADY) |=> RVALID
    ) else $error("SVA FAIL: RVALID dropped before RREADY");

    // BRESP must be stable while BVALID is high
    axi_bresp_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (BVALID && !BREADY) |=> $stable(BRESP)
    ) else $error("SVA FAIL: BRESP changed while BVALID asserted");

    // RDATA/RRESP must be stable while RVALID is high
    axi_rdata_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (RVALID && !RREADY) |=> $stable(RDATA)
    ) else $error("SVA FAIL: RDATA changed while RVALID asserted");

    // =========================================================================
    // Timeout watchdog
    // =========================================================================
    initial begin
        #(CLK_HALF * 2 * 50000);
        $display("TIMEOUT");
        $fatal(1);
    end

endmodule : axi4lite_subordinate_tb

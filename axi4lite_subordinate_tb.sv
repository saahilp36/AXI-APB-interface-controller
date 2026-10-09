// =============================================================================
// axi4lite_subordinate_tb.sv
// Self-checking testbench for axi4lite_subordinate
//
// Test cases:
//   1.  Single write  - AW and W arrive same cycle
//   2.  Single read   - normal path
//   3.  Write SLVERR  - downstream signals error -> BRESP=SLVERR
//   4.  Read  SLVERR  - downstream signals error -> RRESP=SLVERR
//   5.  W before AW   - W arrives two cycles before AW
//   6.  AW before W   - AW arrives two cycles before W
//   7.  Downstream accept/response latency on a write
//   8.  BREADY delayed - manager stalls B channel acceptance
//   9.  RREADY delayed - manager stalls R channel acceptance
//   10. Downstream accept/response latency on a read
//   11. Back-to-back writes
//   12. Concurrent write and read (the two paths are independent FSMs)
//
// What every write / read transaction checks
//   - the downstream request carries EXACTLY what the master sent (address,
//     data, strobes, prot) at the cycle the downstream accepts it, even though the
//     master changes the AW/W/AR payload right after its handshake
//   - the downstream request stays valid until accepted
//   - BRESP / RRESP / RDATA match what the downstream returned
//
// SVA check the AXI handshake rules, payload stability, single outstanding
// transaction per direction, "no response before the downstream response" and the
// error mapping. Every failure is counted and reported in the summary.
//
// Conventions that keep the checks race-free
//   - Stimulus is driven at negedge ACLK.
//   - DUT outputs are sampled AT posedge ACLK (before the DUT's nonblocking
//     updates): a handshake happened at an edge when VALID && READY were both
//     high just before it. Sampling a delay after the edge sees the next cycle
//     and never sees a one-cycle READY.
//   - The B/R channel and the downstream model run concurrently with the AW/W/AR
//     drivers (fork/join). Serializing them deadlocks, because the DUT keeps
//     AWREADY/ARREADY low until the response has been accepted.
// =============================================================================

`timescale 1ns / 1ps

module axi4lite_subordinate_tb;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam int ADDR_WIDTH = 32;
    localparam int DATA_WIDTH = 32;
    localparam int CLK_HALF   = 5;  // ns
    localparam int TMO        = 60; // max cycles to wait for any single event

    localparam logic [2:0] WR_PROT = 3'b101;   // non-zero so prot forwarding is checked
    localparam logic [2:0] RD_PROT = 3'b010;

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
    int sva_fail   = 0;

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

    task automatic tmo_fail(input string what);
        $display("  FAIL  timeout waiting for %s", what);
        fail_count++;
    endtask

    // Called from every assertion's else-branch: counts the failure and reports it
    task automatic sva_hit(input string msg);
        sva_fail++;
        $error("%s", msg);
    endtask

    // =========================================================================
    // Helper: drive a full AXI4-Lite write transaction
    //
    //  aw_delay   - cycles before asserting AWVALID  (0 = same cycle as W)
    //  w_delay    - cycles before asserting WVALID   (0 = same cycle as AW)
    //  ds_latency - cycles the downstream keeps wr_req_ready low after it sees
    //               wr_req_valid
    //  rsp_latency- cycles between accepting the request and wr_rsp_valid
    //  slave_err  - assert wr_rsp_error
    //  b_delay    - cycles the manager waits after BVALID before asserting BREADY
    //
    // Four processes run concurrently: AW driver, W driver, downstream model,
    // B-channel manager.
    // =========================================================================
    task automatic axi_write(
        input  logic [ADDR_WIDTH-1:0]   addr,
        input  logic [DATA_WIDTH-1:0]   data,
        input  logic [DATA_WIDTH/8-1:0] strb,
        input  int                      aw_delay,
        input  int                      w_delay,
        input  int                      ds_latency,
        input  int                      rsp_latency,
        input  logic                    slave_err,
        input  int                      b_delay,
        output logic [1:0]              got_bresp
    );
        got_bresp = 2'bxx;
        fork
            // ---- AW driver ----
            begin : aw_drv
                int g;
                repeat (aw_delay) @(posedge ACLK);
                @(negedge ACLK);
                AWVALID = 1; AWADDR = addr; AWPROT = WR_PROT;
                g = 0;
                do begin @(posedge ACLK); g++; end while (!AWREADY && g < TMO);   // handshake edge
                if (g >= TMO) tmo_fail("AWREADY");
                @(negedge ACLK);
                AWVALID = 0; AWADDR = ~addr; AWPROT = ~WR_PROT;   // payload need not hold after the handshake
            end

            // ---- W driver ----
            begin : w_drv
                int g;
                repeat (w_delay) @(posedge ACLK);
                @(negedge ACLK);
                WVALID = 1; WDATA = data; WSTRB = strb;
                g = 0;
                do begin @(posedge ACLK); g++; end while (!WREADY && g < TMO);
                if (g >= TMO) tmo_fail("WREADY");
                @(negedge ACLK);
                WVALID = 0; WDATA = ~data; WSTRB = ~strb;
            end

            // ---- Downstream (APB side) model: reactive ----
            begin : ds_model
                int g;
                logic ok;
                g = 0;
                do begin @(posedge ACLK); g++; end while (!wr_req_valid && g < TMO);
                if (g >= TMO) tmo_fail("wr_req_valid");
                repeat (ds_latency) @(posedge ACLK);
                @(negedge ACLK);
                wr_req_ready = 1;
                @(posedge ACLK);                      // accept edge: sample the request
                ok = (wr_req_valid === 1'b1) &&
                     (wr_req_addr  === addr) && (wr_req_data === data) &&
                     (wr_req_strb  === strb) && (wr_req_prot === WR_PROT);
                if (!ok)
                    $display("         wr_req: valid=%0b addr=%08h data=%08h strb=%b prot=%b | expected addr=%08h data=%08h strb=%b prot=%b",
                             wr_req_valid, wr_req_addr, wr_req_data, wr_req_strb, wr_req_prot,
                             addr, data, strb, WR_PROT);
                check($sformatf("write request carries addr=%08h data=%08h strb=%b prot", addr, data, strb), ok);
                @(negedge ACLK);
                wr_req_ready = 0;
                repeat (rsp_latency) @(posedge ACLK);
                @(negedge ACLK);
                wr_rsp_valid = 1; wr_rsp_error = slave_err;
                @(posedge ACLK);                      // DUT consumes the one-cycle response
                @(negedge ACLK);
                wr_rsp_valid = 0; wr_rsp_error = 0;
            end

            // ---- B channel manager ----
            begin : b_ch
                int g;
                g = 0;
                do begin @(posedge ACLK); g++; end while (!BVALID && g < TMO);
                if (g >= TMO) tmo_fail("BVALID");
                repeat (b_delay) @(posedge ACLK);     // stall: BVALID must stay high (SVA)
                @(negedge ACLK);
                BREADY = 1;
                @(posedge ACLK);                      // handshake edge
                got_bresp = BRESP;
                @(negedge ACLK);
                BREADY = 0;
            end
        join

        repeat (2) @(posedge ACLK);                   // idle cycles between transactions
    endtask

    // =========================================================================
    // Helper: drive a full AXI4-Lite read transaction
    //  ds_latency / rsp_latency / slave_err / r_delay as for writes
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
        got_rdata = 'x;
        got_rresp = 2'bxx;
        fork
            // ---- AR driver ----
            begin : ar_drv
                int g;
                @(negedge ACLK);
                ARVALID = 1; ARADDR = addr; ARPROT = RD_PROT;
                g = 0;
                do begin @(posedge ACLK); g++; end while (!ARREADY && g < TMO);
                if (g >= TMO) tmo_fail("ARREADY");
                @(negedge ACLK);
                ARVALID = 0; ARADDR = ~addr; ARPROT = ~RD_PROT;
            end

            // ---- Downstream model: reactive ----
            begin : ds_model
                int g;
                logic ok;
                g = 0;
                do begin @(posedge ACLK); g++; end while (!rd_req_valid && g < TMO);
                if (g >= TMO) tmo_fail("rd_req_valid");
                repeat (ds_latency) @(posedge ACLK);
                @(negedge ACLK);
                rd_req_ready = 1;
                @(posedge ACLK);                      // accept edge
                ok = (rd_req_valid === 1'b1) && (rd_req_addr === addr) && (rd_req_prot === RD_PROT);
                if (!ok)
                    $display("         rd_req: valid=%0b addr=%08h prot=%b | expected addr=%08h prot=%b",
                             rd_req_valid, rd_req_addr, rd_req_prot, addr, RD_PROT);
                check($sformatf("read request carries addr=%08h prot", addr), ok);
                @(negedge ACLK);
                rd_req_ready = 0;
                repeat (rsp_latency) @(posedge ACLK);
                @(negedge ACLK);
                rd_rsp_valid = 1; rd_rsp_data = rsp_data; rd_rsp_error = slave_err;
                @(posedge ACLK);
                @(negedge ACLK);
                rd_rsp_valid = 0; rd_rsp_data = '0; rd_rsp_error = 0;
            end

            // ---- R channel manager ----
            begin : r_ch
                int g;
                g = 0;
                do begin @(posedge ACLK); g++; end while (!RVALID && g < TMO);
                if (g >= TMO) tmo_fail("RVALID");
                repeat (r_delay) @(posedge ACLK);
                @(negedge ACLK);
                RREADY = 1;
                @(posedge ACLK);                      // handshake edge
                got_rdata = RDATA;
                got_rresp = RRESP;
                @(negedge ACLK);
                RREADY = 0;
            end
        join

        repeat (2) @(posedge ACLK);
    endtask

    // =========================================================================
    // Test variables
    // =========================================================================
    logic [1:0]             got_bresp, got_bresp2;
    logic [DATA_WIDTH-1:0]  got_rdata, got_rdata2;
    logic [1:0]             got_rresp, got_rresp2;

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

        repeat (4) @(posedge ACLK);
        @(negedge ACLK);
        ARESETn = 1;
        @(posedge ACLK);

        $display("\n[Reset] Outputs idle after reset");
        check("no responses or downstream requests pending after reset",
              BVALID === 1'b0 && RVALID === 1'b0 && wr_req_valid === 1'b0 && rd_req_valid === 1'b0);
        check("AWREADY/WREADY/ARREADY high when idle",
              AWREADY === 1'b1 && WREADY === 1'b1 && ARREADY === 1'b1);

        // -----------------------------------------------------------------
        // TEST 1: Write - AW and W same cycle, no delays
        // -----------------------------------------------------------------
        $display("\n[Test 1] Write - AW and W same cycle");
        axi_write(.addr(32'hA000_0000), .data(32'hDEAD_BEEF), .strb(4'hF),
                  .aw_delay(0), .w_delay(0),
                  .ds_latency(0), .rsp_latency(0),
                  .slave_err(0), .b_delay(0),
                  .got_bresp(got_bresp));
        check("BRESP = OKAY", got_bresp === 2'b00);

        // -----------------------------------------------------------------
        // TEST 2: Read - normal path
        // -----------------------------------------------------------------
        $display("\n[Test 2] Read - no delays");
        axi_read(.addr(32'hA000_0004),
                 .ds_latency(0), .rsp_latency(0),
                 .rsp_data(32'hCAFE_1234), .slave_err(0), .r_delay(0),
                 .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("RDATA correct",  got_rdata === 32'hCAFE_1234);
        check("RRESP = OKAY",   got_rresp === 2'b00);

        // -----------------------------------------------------------------
        // TEST 3: Write SLVERR
        // -----------------------------------------------------------------
        $display("\n[Test 3] Write - SLVERR from downstream");
        axi_write(.addr(32'hDEAD_0000), .data(32'hFFFF_FFFF), .strb(4'hF),
                  .aw_delay(0), .w_delay(0),
                  .ds_latency(0), .rsp_latency(0),
                  .slave_err(1), .b_delay(0),
                  .got_bresp(got_bresp));
        check("BRESP = SLVERR", got_bresp === 2'b10);

        // -----------------------------------------------------------------
        // TEST 4: Read SLVERR
        // -----------------------------------------------------------------
        $display("\n[Test 4] Read - SLVERR from downstream");
        axi_read(.addr(32'hDEAD_0004),
                 .ds_latency(0), .rsp_latency(0),
                 .rsp_data(32'hBAD0_BEEF), .slave_err(1), .r_delay(0),
                 .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("RRESP = SLVERR", got_rresp === 2'b10);

        // -----------------------------------------------------------------
        // TEST 5: W arrives before AW (W leads by 2 cycles)
        // -----------------------------------------------------------------
        $display("\n[Test 5] Write - W arrives 2 cycles before AW");
        axi_write(.addr(32'hB000_0000), .data(32'h1111_2222), .strb(4'h3),
                  .aw_delay(2), .w_delay(0),
                  .ds_latency(0), .rsp_latency(0),
                  .slave_err(0), .b_delay(0),
                  .got_bresp(got_bresp));
        check("BRESP = OKAY (W-first)", got_bresp === 2'b00);

        // -----------------------------------------------------------------
        // TEST 6: AW arrives before W (AW leads by 2 cycles)
        // -----------------------------------------------------------------
        $display("\n[Test 6] Write - AW arrives 2 cycles before W");
        axi_write(.addr(32'hB000_0008), .data(32'h3333_4444), .strb(4'hC),
                  .aw_delay(0), .w_delay(2),
                  .ds_latency(0), .rsp_latency(0),
                  .slave_err(0), .b_delay(0),
                  .got_bresp(got_bresp));
        check("BRESP = OKAY (AW-first)", got_bresp === 2'b00);

        // -----------------------------------------------------------------
        // TEST 7: Write with downstream latency (ds_latency=2, rsp_latency=3)
        // -----------------------------------------------------------------
        $display("\n[Test 7] Write - downstream accept+response latency");
        axi_write(.addr(32'hC000_0000), .data(32'hABCD_EF01), .strb(4'h5),
                  .aw_delay(0), .w_delay(0),
                  .ds_latency(2), .rsp_latency(3),
                  .slave_err(0), .b_delay(0),
                  .got_bresp(got_bresp));
        check("BRESP = OKAY (latency)", got_bresp === 2'b00);

        // -----------------------------------------------------------------
        // TEST 8: BREADY delayed - manager stalls 3 cycles after BVALID
        // -----------------------------------------------------------------
        $display("\n[Test 8] Write - BREADY delayed 3 cycles");
        axi_write(.addr(32'hC000_0010), .data(32'h5555_6666), .strb(4'hA),
                  .aw_delay(0), .w_delay(0),
                  .ds_latency(0), .rsp_latency(0),
                  .slave_err(0), .b_delay(3),
                  .got_bresp(got_bresp));
        check("BRESP = OKAY (B-stall)", got_bresp === 2'b00);

        // -----------------------------------------------------------------
        // TEST 9: Read with delayed RREADY
        // -----------------------------------------------------------------
        $display("\n[Test 9] Read - RREADY delayed 3 cycles");
        axi_read(.addr(32'hC000_0020),
                 .ds_latency(0), .rsp_latency(0),
                 .rsp_data(32'h9876_5432), .slave_err(0), .r_delay(3),
                 .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("RDATA correct (R-stall)",  got_rdata === 32'h9876_5432);
        check("RRESP = OKAY  (R-stall)",  got_rresp === 2'b00);

        // -----------------------------------------------------------------
        // TEST 10: Read with downstream latency
        // -----------------------------------------------------------------
        $display("\n[Test 10] Read - downstream latency 3 cycles");
        axi_read(.addr(32'hD000_0000),
                 .ds_latency(1), .rsp_latency(3),
                 .rsp_data(32'hFEED_F00D), .slave_err(0), .r_delay(0),
                 .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("RDATA correct (latency)", got_rdata === 32'hFEED_F00D);
        check("RRESP = OKAY  (latency)", got_rresp === 2'b00);

        // -----------------------------------------------------------------
        // TEST 11: Back-to-back writes, new write started as soon as the last B completes
        // -----------------------------------------------------------------
        $display("\n[Test 11] Back-to-back writes (4 consecutive, mixed ordering and strobes)");
        for (int i = 0; i < 4; i++) begin
            axi_write(.addr(32'hE000_0000 + i*4), .data(32'h0B0B_0000 | i), .strb(4'(1 << i) | 4'h1),
                      .aw_delay(i % 3), .w_delay((i + 1) % 3),
                      .ds_latency(i % 2), .rsp_latency(i % 2),
                      .slave_err(i == 2), .b_delay(0),
                      .got_bresp(got_bresp));
            check($sformatf("back-to-back write %0d BRESP", i),
                  got_bresp === ((i == 2) ? 2'b10 : 2'b00));
        end

        // -----------------------------------------------------------------
        // TEST 12: Write and read at the same time - independent FSMs
        // -----------------------------------------------------------------
        $display("\n[Test 12] Concurrent write and read");
        fork
            axi_write(.addr(32'hF000_0000), .data(32'h1234_ABCD), .strb(4'hF),
                      .aw_delay(0), .w_delay(1),
                      .ds_latency(1), .rsp_latency(2),
                      .slave_err(0), .b_delay(1),
                      .got_bresp(got_bresp2));
            axi_read(.addr(32'hF000_0100),
                     .ds_latency(0), .rsp_latency(1),
                     .rsp_data(32'h0F0F_F0F0), .slave_err(0), .r_delay(2),
                     .got_rdata(got_rdata2), .got_rresp(got_rresp2));
        join
        check("concurrent: BRESP = OKAY",   got_bresp2 === 2'b00);
        check("concurrent: RDATA correct",  got_rdata2 === 32'h0F0F_F0F0);
        check("concurrent: RRESP = OKAY",   got_rresp2 === 2'b00);

        // -----------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------
        @(posedge ACLK);
        $display("\n============================================================");
        $display(" Results: %0d passed, %0d failed, %0d assertion failures",
                 pass_count, fail_count, sva_fail);
        if (fail_count == 0 && sva_fail == 0)
            $display(" ALL TESTS PASSED");
        else
            $display(" SOME TESTS FAILED");
        $display("============================================================");
        $finish;
    end

    // =========================================================================
    // SystemVerilog Assertions
    // =========================================================================

    // AXI rule: once VALID is asserted, it (and its payload) must not change
    // until READY.  AW / W / AR are driven by this testbench, so these also
    // prove the testbench itself follows the protocol.
    axi_aw_valid_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (AWVALID && !AWREADY) |=> (AWVALID && $stable(AWADDR) && $stable(AWPROT))
    ) else sva_hit("SVA FAIL: AWVALID dropped or AW payload changed before AWREADY");

    axi_w_valid_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (WVALID && !WREADY) |=> (WVALID && $stable(WDATA) && $stable(WSTRB))
    ) else sva_hit("SVA FAIL: WVALID dropped or W payload changed before WREADY");

    axi_ar_valid_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (ARVALID && !ARREADY) |=> (ARVALID && $stable(ARADDR) && $stable(ARPROT))
    ) else sva_hit("SVA FAIL: ARVALID dropped or AR payload changed before ARREADY");

    // AXI rule: BVALID / RVALID and their payload hold until BREADY / RREADY
    axi_b_valid_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (BVALID && !BREADY) |=> (BVALID && $stable(BRESP))
    ) else sva_hit("SVA FAIL: BVALID dropped or BRESP changed before BREADY");

    axi_r_valid_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (RVALID && !RREADY) |=> (RVALID && $stable(RDATA) && $stable(RRESP))
    ) else sva_hit("SVA FAIL: RVALID dropped or RDATA/RRESP changed before RREADY");

    // Downstream requests hold their payload until accepted
    ds_wr_req_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (wr_req_valid && !wr_req_ready) |=>
            (wr_req_valid && $stable(wr_req_addr) && $stable(wr_req_data) &&
             $stable(wr_req_strb) && $stable(wr_req_prot))
    ) else sva_hit("SVA FAIL: downstream write request dropped or changed before wr_req_ready");

    ds_rd_req_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (rd_req_valid && !rd_req_ready) |=>
            (rd_req_valid && $stable(rd_req_addr) && $stable(rd_req_prot))
    ) else sva_hit("SVA FAIL: downstream read request dropped or changed before rd_req_ready");

    // Single outstanding transaction per direction: no new AW/W/AR accepted
    // while a response is waiting
    single_outstanding_write: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        BVALID |-> (!AWREADY && !WREADY)
    ) else sva_hit("SVA FAIL: AW/W accepted while a write response is pending");

    single_outstanding_read: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        RVALID |-> !ARREADY
    ) else sva_hit("SVA FAIL: AR accepted while a read response is pending");

    // No response before the downstream response, and the error maps to SLVERR
    b_after_downstream_rsp: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        $rose(BVALID) |-> $past(wr_rsp_valid)
    ) else sva_hit("SVA FAIL: BVALID rose without a downstream write response");

    r_after_downstream_rsp: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        $rose(RVALID) |-> $past(rd_rsp_valid)
    ) else sva_hit("SVA FAIL: RVALID rose without a downstream read response");

    bresp_maps_error: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        $rose(BVALID) |-> (BRESP == ($past(wr_rsp_error) ? 2'b10 : 2'b00))
    ) else sva_hit("SVA FAIL: BRESP does not match the downstream error flag");

    rresp_maps_error: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        $rose(RVALID) |-> (RRESP == ($past(rd_rsp_error) ? 2'b10 : 2'b00))
    ) else sva_hit("SVA FAIL: RRESP does not match the downstream error flag");

    // =========================================================================
    // Timeout watchdog
    // =========================================================================
    initial begin
        #(CLK_HALF * 2 * 50000);
        $display("TIMEOUT");
        $fatal(1);
    end

endmodule : axi4lite_subordinate_tb

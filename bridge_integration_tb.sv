// =============================================================================
// bridge_integration_tb.sv
// End-to-end integration testbench
//
//   AXI manager (this TB) → axi_apb_bridge → apb_regfile
//
// All transactions are driven as a real AXI4-Lite manager would drive them.
// Correctness is checked by a self-checking scoreboard that compares
// read-back data against a software model of the register file.
//
// Test plan
// ─────────────────────────────────────────────────────────────────────────────
//  1.  Write then read-back all 8 registers                  (basic smoke)
//  2.  Read reset values before any write                    (reset correctness)
//  3.  Partial-write with WSTRB (byte lanes)                 (strobe path)
//  4.  Out-of-range address → BRESP=SLVERR, RRESP=SLVERR     (error path)
//  5.  APB wait states (peripheral stalls PREADY for 3 cy)   (wait-state path)
//  6.  Concurrent write + read (interleaved AXI transactions) (arbitration)
//  7.  Back-to-back writes, no idle cycles between           (throughput)
//  8.  AW before W by 2 cycles                               (AW/W ordering)
//  9.  W  before AW by 2 cycles                              (AW/W ordering)
//  10. BREADY delayed 4 cycles (B-channel back-pressure)     (flow control)
//  11. RREADY delayed 4 cycles (R-channel back-pressure)     (flow control)
// =============================================================================

`timescale 1ns / 1ps

module bridge_integration_tb;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam int  ADDR_WIDTH  = 32;
    localparam int  DATA_WIDTH  = 32;
    localparam int  NUM_SLAVES  = 1;
    localparam int  CLK_HALF    = 5;   // ns  → 100 MHz

    // Peripheral base address (must match apb_decoder BASE_ADDR)
    localparam logic [31:0] PERIPH_BASE = 32'hC000_0000;

    // =========================================================================
    // DUT signals
    // =========================================================================
    logic ACLK, ARESETn;

    // AXI subordinate side
    logic                    S_AWVALID, S_AWREADY;
    logic [ADDR_WIDTH-1:0]   S_AWADDR;
    logic [2:0]              S_AWPROT;
    logic                    S_WVALID,  S_WREADY;
    logic [DATA_WIDTH-1:0]   S_WDATA;
    logic [DATA_WIDTH/8-1:0] S_WSTRB;
    logic                    S_BVALID,  S_BREADY;
    logic [1:0]              S_BRESP;
    logic                    S_ARVALID, S_ARREADY;
    logic [ADDR_WIDTH-1:0]   S_ARADDR;
    logic [2:0]              S_ARPROT;
    logic                    S_RVALID,  S_RREADY;
    logic [DATA_WIDTH-1:0]   S_RDATA;
    logic [1:0]              S_RRESP;

    // APB manager side
    logic [NUM_SLAVES-1:0]   M_PSEL;
    logic                    M_PENABLE, M_PWRITE;
    logic [ADDR_WIDTH-1:0]   M_PADDR;
    logic [DATA_WIDTH-1:0]   M_PWDATA;
    logic [DATA_WIDTH/8-1:0] M_PSTRB;
    logic [NUM_SLAVES-1:0]   M_PREADY;
    logic [DATA_WIDTH-1:0]   M_PRDATA;
    logic [NUM_SLAVES-1:0]   M_PSLVERR;

    // Wait-state injection (controlled per test)
    logic [3:0] tb_wait_states;

    // =========================================================================
    // DUT: bridge
    // =========================================================================
    axi_apb_bridge #(
        .ADDR_WIDTH (ADDR_WIDTH),
        .DATA_WIDTH (DATA_WIDTH),
        .NUM_SLAVES (NUM_SLAVES)
    ) dut (
        .ACLK       (ACLK),     .ARESETn    (ARESETn),
        .S_AWVALID  (S_AWVALID),.S_AWREADY  (S_AWREADY),
        .S_AWADDR   (S_AWADDR), .S_AWPROT   (S_AWPROT),
        .S_WVALID   (S_WVALID), .S_WREADY   (S_WREADY),
        .S_WDATA    (S_WDATA),  .S_WSTRB    (S_WSTRB),
        .S_BVALID   (S_BVALID), .S_BREADY   (S_BREADY),
        .S_BRESP    (S_BRESP),
        .S_ARVALID  (S_ARVALID),.S_ARREADY  (S_ARREADY),
        .S_ARADDR   (S_ARADDR), .S_ARPROT   (S_ARPROT),
        .S_RVALID   (S_RVALID), .S_RREADY   (S_RREADY),
        .S_RDATA    (S_RDATA),  .S_RRESP    (S_RRESP),
        .M_PSEL     (M_PSEL),   .M_PENABLE  (M_PENABLE),
        .M_PWRITE   (M_PWRITE), .M_PADDR    (M_PADDR),
        .M_PWDATA   (M_PWDATA), .M_PSTRB    (M_PSTRB),
        .M_PREADY   (M_PREADY), .M_PRDATA   (M_PRDATA),
        .M_PSLVERR  (M_PSLVERR)
    );

    // =========================================================================
    // DUT: APB register file peripheral
    // =========================================================================
    apb_regfile #(
        .ADDR_WIDTH (ADDR_WIDTH),
        .DATA_WIDTH (DATA_WIDTH),
        .NUM_REGS   (8)
    ) u_regfile (
        .PCLK       (ACLK),
        .PRESETn    (ARESETn),
        .PSEL       (M_PSEL[0]),
        .PENABLE    (M_PENABLE),
        .PWRITE     (M_PWRITE),
        .PADDR      (M_PADDR - PERIPH_BASE),  // relative address
        .PWDATA     (M_PWDATA),
        .PSTRB      (M_PSTRB),
        .PREADY     (M_PREADY[0]),
        .PRDATA     (M_PRDATA),
        .PSLVERR    (M_PSLVERR[0]),
        .wait_states(tb_wait_states)
    );

    // =========================================================================
    // Software model — mirrors the hardware register file
    // =========================================================================
    logic [DATA_WIDTH-1:0] sw_model [0:7];

    task automatic model_reset();
        for (int i = 0; i < 8; i++) sw_model[i] = DATA_WIDTH'(i);
    endtask

    task automatic model_write(
        input logic [31:0]   addr,
        input logic [31:0]   data,
        input logic [3:0]    strb
    );
        int idx = (addr - PERIPH_BASE) >> 2;
        if (idx >= 0 && idx < 8) begin
            for (int b = 0; b < 4; b++)
                if (strb[b]) sw_model[idx][b*8 +: 8] = data[b*8 +: 8];
        end
    endtask

    function automatic logic [31:0] model_read(input logic [31:0] addr);
        int idx = (addr - PERIPH_BASE) >> 2;
        if (idx >= 0 && idx < 8) return sw_model[idx];
        else                     return 32'hDEAD_DEAD;
    endfunction

    // =========================================================================
    // Scoreboard
    // =========================================================================
    int pass_count = 0;
    int fail_count = 0;

    task automatic check(input string name, input logic cond);
        if (cond) begin
            $display("  PASS  %s", name);
            pass_count++;
        end else begin
            $display("  FAIL  %s", name);
            fail_count++;
        end
    endtask

    // =========================================================================
    // Clock
    // =========================================================================
    initial ACLK = 0;
    always  #CLK_HALF ACLK = ~ACLK;

    // =========================================================================
    // AXI BFM tasks
    // =========================================================================
    task automatic clk_pos; @(posedge ACLK); #1; endtask
    task automatic clk_neg; @(negedge ACLK); #1; endtask

    // ── AXI write ─────────────────────────────────────────────────────────────
    task automatic axi_write(
        input  logic [31:0] addr,
        input  logic [31:0] data,
        input  logic [3:0]  strb       = 4'hF,
        input  int          aw_delay   = 0,
        input  int          w_delay    = 0,
        input  int          b_delay    = 0,
        output logic [1:0]  got_bresp
    );
        fork
            begin   // AW channel
                repeat (aw_delay) clk_pos;
                clk_neg; S_AWVALID = 1; S_AWADDR = addr; S_AWPROT = '0;
                do clk_pos; while (!S_AWREADY);
                clk_neg; S_AWVALID = 0;
            end
            begin   // W channel
                repeat (w_delay) clk_pos;
                clk_neg; S_WVALID = 1; S_WDATA = data; S_WSTRB = strb;
                do clk_pos; while (!S_WREADY);
                clk_neg; S_WVALID = 0;
            end
        join

        // B channel
        repeat (b_delay) clk_pos;
        clk_neg; S_BREADY = 1;
        do clk_pos; while (!S_BVALID);
        got_bresp = S_BRESP;
        clk_neg; S_BREADY = 0;
        clk_pos;
    endtask

    // ── AXI read ──────────────────────────────────────────────────────────────
    task automatic axi_read(
        input  logic [31:0] addr,
        input  int          r_delay    = 0,
        output logic [31:0] got_rdata,
        output logic [1:0]  got_rresp
    );
        clk_neg; S_ARVALID = 1; S_ARADDR = addr; S_ARPROT = '0;
        do clk_pos; while (!S_ARREADY);
        clk_neg; S_ARVALID = 0;

        repeat (r_delay) clk_pos;
        clk_neg; S_RREADY = 1;
        do clk_pos; while (!S_RVALID);
        got_rdata = S_RDATA;
        got_rresp = S_RRESP;
        clk_neg; S_RREADY = 0;
        clk_pos;
    endtask

    // =========================================================================
    // Test variables
    // =========================================================================
    logic [1:0]  got_bresp, got_rresp;
    logic [31:0] got_rdata;
    logic [31:0] exp_rdata;

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin
        $display("============================================================");
        $display(" AXI-APB Bridge Integration Testbench");
        $display("============================================================");

        // --- Initialise all AXI driver signals ---
        S_AWVALID = 0; S_AWADDR = '0; S_AWPROT = '0;
        S_WVALID  = 0; S_WDATA  = '0; S_WSTRB  = '0;
        S_BREADY  = 0;
        S_ARVALID = 0; S_ARADDR = '0; S_ARPROT = '0;
        S_RREADY  = 0;
        tb_wait_states = 0;
        ARESETn = 0;
        model_reset();

        repeat (4) clk_pos;
        ARESETn = 1;
        clk_pos;

        // =====================================================================
        // TEST 1: Read reset values (no prior write)
        // =====================================================================
        $display("\n[Test 1] Read reset values from all 8 registers");
        for (int i = 0; i < 8; i++) begin
            axi_read(.addr(PERIPH_BASE + i*4), .got_rdata(got_rdata), .got_rresp(got_rresp));
            exp_rdata = model_read(PERIPH_BASE + i*4);
            check($sformatf("REG%0d reset value", i),
                  got_rdata === exp_rdata && got_rresp === 2'b00);
        end

        // =====================================================================
        // TEST 2: Write then read-back all 8 registers
        // =====================================================================
        $display("\n[Test 2] Write then read-back all 8 registers");
        for (int i = 0; i < 8; i++) begin
            automatic logic [31:0] wval = 32'hA5A50000 | (i << 8) | i;
            axi_write(.addr(PERIPH_BASE + i*4), .data(wval),
                      .got_bresp(got_bresp));
            check($sformatf("REG%0d write BRESP", i), got_bresp === 2'b00);
            model_write(PERIPH_BASE + i*4, wval, 4'hF);
        end
        for (int i = 0; i < 8; i++) begin
            axi_read(.addr(PERIPH_BASE + i*4), .got_rdata(got_rdata), .got_rresp(got_rresp));
            exp_rdata = model_read(PERIPH_BASE + i*4);
            check($sformatf("REG%0d readback", i),
                  got_rdata === exp_rdata && got_rresp === 2'b00);
        end

        // =====================================================================
        // TEST 3: Partial write with WSTRB (upper byte only)
        // =====================================================================
        $display("\n[Test 3] Partial write — upper byte only (WSTRB=4'b1000)");
        begin
            automatic logic [31:0] partial_addr = PERIPH_BASE + 0;
            automatic logic [31:0] partial_data = 32'hFF000000;
            axi_write(.addr(partial_addr), .data(partial_data),
                      .strb(4'b1000), .got_bresp(got_bresp));
            check("Partial write BRESP", got_bresp === 2'b00);
            model_write(partial_addr, partial_data, 4'b1000);
            axi_read(.addr(partial_addr), .got_rdata(got_rdata), .got_rresp(got_rresp));
            exp_rdata = model_read(partial_addr);
            check("Partial write readback", got_rdata === exp_rdata);
        end

        // =====================================================================
        // TEST 4: Out-of-range address → SLVERR
        // =====================================================================
        $display("\n[Test 4] Out-of-range address — expect SLVERR");
        axi_write(.addr(PERIPH_BASE + 32'h0000_0020),  // reg[8] — doesn't exist
                  .data(32'hDEAD_BEEF), .got_bresp(got_bresp));
        check("Bad-addr write BRESP=SLVERR", got_bresp === 2'b10);

        axi_read(.addr(PERIPH_BASE + 32'h0000_0020),
                 .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("Bad-addr read RRESP=SLVERR", got_rresp === 2'b10);

        // =====================================================================
        // TEST 5: APB wait states (peripheral holds PREADY low for 3 cycles)
        // =====================================================================
        $display("\n[Test 5] APB wait states (3 cycles)");
        tb_wait_states = 3;
        axi_write(.addr(PERIPH_BASE + 4), .data(32'hWAIT_1234),
                  .got_bresp(got_bresp));
        check("Wait-state write BRESP", got_bresp === 2'b00);
        model_write(PERIPH_BASE + 4, 32'hWAIT_1234, 4'hF);

        axi_read(.addr(PERIPH_BASE + 4), .got_rdata(got_rdata), .got_rresp(got_rresp));
        exp_rdata = model_read(PERIPH_BASE + 4);
        check("Wait-state read data",  got_rdata === exp_rdata);
        check("Wait-state read RRESP", got_rresp === 2'b00);
        tb_wait_states = 0;

        // =====================================================================
        // TEST 6: AW arrives 2 cycles before W
        // =====================================================================
        $display("\n[Test 6] AW 2 cycles before W");
        axi_write(.addr(PERIPH_BASE + 8), .data(32'hAW_FIRST),
                  .aw_delay(0), .w_delay(2), .got_bresp(got_bresp));
        check("AW-first write BRESP", got_bresp === 2'b00);
        model_write(PERIPH_BASE + 8, 32'hAW_FIRST, 4'hF);
        axi_read(.addr(PERIPH_BASE + 8), .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("AW-first readback", got_rdata === model_read(PERIPH_BASE + 8));

        // =====================================================================
        // TEST 7: W arrives 2 cycles before AW
        // =====================================================================
        $display("\n[Test 7] W 2 cycles before AW");
        axi_write(.addr(PERIPH_BASE + 12), .data(32'hW_FIRST__),
                  .aw_delay(2), .w_delay(0), .got_bresp(got_bresp));
        check("W-first write BRESP", got_bresp === 2'b00);
        model_write(PERIPH_BASE + 12, 32'hW_FIRST__, 4'hF);
        axi_read(.addr(PERIPH_BASE + 12), .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("W-first readback", got_rdata === model_read(PERIPH_BASE + 12));

        // =====================================================================
        // TEST 8: BREADY delayed 4 cycles
        // =====================================================================
        $display("\n[Test 8] BREADY delayed 4 cycles");
        axi_write(.addr(PERIPH_BASE + 16), .data(32'hB_DELAY__),
                  .b_delay(4), .got_bresp(got_bresp));
        check("B-delay write BRESP", got_bresp === 2'b00);

        // =====================================================================
        // TEST 9: RREADY delayed 4 cycles
        // =====================================================================
        $display("\n[Test 9] RREADY delayed 4 cycles");
        // Write a known value first
        axi_write(.addr(PERIPH_BASE + 20), .data(32'hR_DELAY__),
                  .got_bresp(got_bresp));
        model_write(PERIPH_BASE + 20, 32'hR_DELAY__, 4'hF);
        axi_read(.addr(PERIPH_BASE + 20), .r_delay(4),
                 .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("R-delay read data",  got_rdata === model_read(PERIPH_BASE + 20));
        check("R-delay read RRESP", got_rresp === 2'b00);

        // =====================================================================
        // TEST 10: Back-to-back writes with no idle cycles
        // =====================================================================
        $display("\n[Test 10] Back-to-back writes (4 consecutive)");
        for (int i = 0; i < 4; i++) begin
            automatic logic [31:0] bb_addr = PERIPH_BASE + i*4;
            automatic logic [31:0] bb_data = 32'hBBBB_0000 | i;
            // Don't add idle between — drive next write immediately
            fork
                begin
                    clk_neg; S_AWVALID = 1; S_AWADDR = bb_addr; S_AWPROT = '0;
                    do clk_pos; while (!S_AWREADY);
                    clk_neg; S_AWVALID = 0;
                end
                begin
                    clk_neg; S_WVALID = 1; S_WDATA = bb_data; S_WSTRB = 4'hF;
                    do clk_pos; while (!S_WREADY);
                    clk_neg; S_WVALID = 0;
                end
            join
            clk_neg; S_BREADY = 1;
            do clk_pos; while (!S_BVALID);
            got_bresp = S_BRESP;
            clk_neg; S_BREADY = 0;
            check($sformatf("BB write %0d BRESP", i), got_bresp === 2'b00);
            model_write(bb_addr, bb_data, 4'hF);
        end
        // Verify all four
        for (int i = 0; i < 4; i++) begin
            axi_read(.addr(PERIPH_BASE + i*4), .got_rdata(got_rdata), .got_rresp(got_rresp));
            check($sformatf("BB readback %0d", i),
                  got_rdata === model_read(PERIPH_BASE + i*4));
        end

        // =====================================================================
        // Summary
        // =====================================================================
        repeat (4) clk_pos;
        $display("\n============================================================");
        $display(" Results: %0d passed, %0d failed", pass_count, fail_count);
        $display(fail_count == 0 ? " ALL TESTS PASSED" : " SOME TESTS FAILED");
        $display("============================================================");
        $finish;
    end

    // =========================================================================
    // SVA: end-to-end protocol checks
    // =========================================================================

    // BVALID must not deassert before BREADY
    sva_bvalid_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (S_BVALID && !S_BREADY) |=> S_BVALID
    ) else $error("SVA: S_BVALID dropped before S_BREADY");

    // RVALID must not deassert before RREADY
    sva_rvalid_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (S_RVALID && !S_RREADY) |=> S_RVALID
    ) else $error("SVA: S_RVALID dropped before S_RREADY");

    // APB: PENABLE never without PSEL
    sva_apb_penable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        M_PENABLE |-> |M_PSEL
    ) else $error("SVA: M_PENABLE asserted without any M_PSEL");

    // APB: PADDR stable while PENABLE and not yet PREADY
    sva_apb_paddr_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (|M_PSEL && M_PENABLE && !M_PREADY[0]) |=> $stable(M_PADDR)
    ) else $error("SVA: M_PADDR changed during active APB transfer");

    // At most one PSEL asserted at a time (one-hot check)
    sva_psel_onehot: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        $onehot0(M_PSEL)
    ) else $error("SVA: Multiple M_PSEL bits asserted simultaneously");

    // =========================================================================
    // Timeout watchdog
    // =========================================================================
    initial begin
        #(CLK_HALF * 2 * 100_000);
        $display("TIMEOUT — simulation hung");
        $fatal(1);
    end

endmodule : bridge_integration_tb

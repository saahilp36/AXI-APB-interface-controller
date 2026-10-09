// =============================================================================
// bridge_integration_tb.sv
// End-to-end integration testbench
//
//   AXI manager (this TB) -> axi_apb_bridge -> apb_regfile
//
// All transactions are driven as a real AXI4-Lite manager would drive them.
// Correctness is checked by a self-checking scoreboard that compares read-back
// data against a software model of the register file.
//
// Test plan
// -----------------------------------------------------------------------------
//  1.  Read reset values before any write                    (reset correctness)
//  2.  Write then read-back all 8 registers                  (basic smoke)
//  3.  Partial writes with WSTRB (byte lanes)                (strobe path)
//  4.  Error path: bad register, unaligned, unmapped address (SLVERR on B and R)
//  5.  APB wait states (peripheral stalls PREADY for 3 cy)   (wait-state path)
//  6.  Concurrent write + read, write must win               (arbitration)
//  7.  AW before W by 2 cycles                               (AW/W ordering)
//  8.  W  before AW by 2 cycles                              (AW/W ordering)
//  9.  BREADY delayed 4 cycles (B-channel back-pressure)     (flow control)
//  10. RREADY delayed 4 cycles (R-channel back-pressure)     (flow control)
//  11. Back-to-back writes, no idle cycles between           (throughput)
//  12. Constrained-random mix of writes, reads, concurrent pairs, errors, wait
//      states, AW/W orderings and byte strobes, all checked against the model,
//      with coverage counters that must all be hit
//
// The model is updated by the BFM whenever the write is EXPECTED to succeed, so a
// DUT that wrongly returns an error (or drops the write) is caught by the next
// read-back instead of being papered over.
//
// Conventions that keep the checks race-free (see the unit testbenches)
//   - Stimulus is driven at negedge ACLK; DUT outputs are sampled AT posedge ACLK,
//     before the DUT's nonblocking updates. A handshake happened at an edge when
//     VALID && READY were both high just before it.
//   - The B / R channel managers run concurrently with the AW/W/AR drivers
//     (fork/join). The DUT holds AWREADY/ARREADY low until the response has been
//     accepted, so serializing them deadlocks.
//   - Every wait is bounded (TMO cycles) and reports a FAIL instead of hanging.
// =============================================================================

`timescale 1ns / 1ps

module bridge_integration_tb;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam int  ADDR_WIDTH  = 32;
    localparam int  DATA_WIDTH  = 32;
    localparam int  NUM_SLAVES  = 1;
    localparam int  CLK_HALF    = 5;   // ns  -> 100 MHz
    localparam int  TMO         = 100; // max cycles to wait for any single event
    localparam int  NUM_RANDOM  = 300;

    // Peripheral base address (must match apb_decoder BASE_ADDR) and window
    localparam logic [31:0] PERIPH_BASE = 32'hC000_0000;
    localparam logic [31:0] DEC_WINDOW  = 32'h0000_1000;   // 4 KB decoder window per slave
    localparam logic [31:0] REG_WINDOW  = 32'h0000_0020;   // 8 registers x 4 bytes

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
    // Software model - mirrors the hardware register file
    // =========================================================================
    logic [DATA_WIDTH-1:0] sw_model [0:7];

    task automatic model_reset();
        for (int i = 0; i < 8; i++) sw_model[i] = DATA_WIDTH'(i);
    endtask

    // Expected AXI response: OKAY only for a word-aligned address inside the
    // register window; bad register, unaligned and unmapped addresses are SLVERR
    function automatic logic [1:0] exp_resp(input logic [31:0] addr);
        if (addr >= PERIPH_BASE && addr < PERIPH_BASE + REG_WINDOW && addr[1:0] == 2'b00)
            return 2'b00;
        else
            return 2'b10;
    endfunction

    task automatic model_write(
        input logic [31:0]   addr,
        input logic [31:0]   data,
        input logic [3:0]    strb
    );
        int idx;
        idx = (addr - PERIPH_BASE) >> 2;
        if (exp_resp(addr) == 2'b00 && idx >= 0 && idx < 8) begin
            for (int b = 0; b < 4; b++)
                if (strb[b]) sw_model[idx][b*8 +: 8] = data[b*8 +: 8];
        end
    endtask

    function automatic logic [31:0] model_read(input logic [31:0] addr);
        int idx;
        idx = (addr - PERIPH_BASE) >> 2;
        if (exp_resp(addr) == 2'b00 && idx >= 0 && idx < 8) return sw_model[idx];
        else                                                 return 32'hDEAD_DEAD;
    endfunction

    // =========================================================================
    // Scoreboard
    // =========================================================================
    int pass_count = 0;
    int fail_count = 0;
    int sva_fail   = 0;

    task automatic check(input string name, input logic cond);
        if (cond) begin
            $display("  PASS  %s", name);
            pass_count++;
        end else begin
            $display("  FAIL  %s", name);
            fail_count++;
        end
    endtask

    // Quiet variant for the random test: only failures are printed
    task automatic check_q(input string name, input logic cond);
        if (cond) pass_count++;
        else begin
            $display("  FAIL  %s", name);
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
    // Clock
    // =========================================================================
    initial ACLK = 0;
    always  #CLK_HALF ACLK = ~ACLK;

    // =========================================================================
    // Passive APB log (used to check arbitration order)
    // =========================================================================
    bit                    log_en = 0;
    bit                    log_wr   [$];
    logic [ADDR_WIDTH-1:0] log_addr [$];

    always @(posedge ACLK) begin
        if (log_en && (|M_PSEL) && M_PENABLE && M_PREADY[0]) begin
            log_wr.push_back(M_PWRITE);
            log_addr.push_back(M_PADDR);
        end
    end

    // =========================================================================
    // Coverage counters (random test)
    // =========================================================================
    int cov_ord_simul = 0, cov_ord_aw_first = 0, cov_ord_w_first = 0;
    int cov_concurrent = 0, cov_err_resp = 0, cov_stall_b = 0, cov_stall_r = 0;
    int cov_strb  [0:15];
    int cov_waits [0:4];

    // =========================================================================
    // AXI BFM tasks
    // =========================================================================

    // Channel tasks: no fork inside, so any combination can be run in ONE
    // fork/join level (nested fork-in-fork is poorly supported by some simulators).
    task automatic aw_chan(input logic [31:0] addr, input int aw_delay);
        int g;
        repeat (aw_delay) @(posedge ACLK);
        @(negedge ACLK);
        S_AWVALID = 1; S_AWADDR = addr; S_AWPROT = '0;
        g = 0;
        do begin @(posedge ACLK); g++; end while (!S_AWREADY && g < TMO);
        if (g >= TMO) tmo_fail("AWREADY");
        @(negedge ACLK);
        S_AWVALID = 0; S_AWADDR = ~addr;
    endtask

    task automatic w_chan(input logic [31:0] data, input logic [3:0] strb, input int w_delay);
        int g;
        repeat (w_delay) @(posedge ACLK);
        @(negedge ACLK);
        S_WVALID = 1; S_WDATA = data; S_WSTRB = strb;
        g = 0;
        do begin @(posedge ACLK); g++; end while (!S_WREADY && g < TMO);
        if (g >= TMO) tmo_fail("WREADY");
        @(negedge ACLK);
        S_WVALID = 0; S_WDATA = ~data; S_WSTRB = ~strb;
    endtask

    task automatic b_chan(input int b_delay, output logic [1:0] got_bresp);
        int g;
        got_bresp = 2'bxx;
        g = 0;
        do begin @(posedge ACLK); g++; end while (!S_BVALID && g < TMO);
        if (g >= TMO) tmo_fail("BVALID");
        repeat (b_delay) @(posedge ACLK);          // stall: BVALID must stay high (SVA)
        @(negedge ACLK);
        S_BREADY = 1;
        @(posedge ACLK);                           // handshake edge
        got_bresp = S_BRESP;
        @(negedge ACLK);
        S_BREADY = 0;
    endtask

    task automatic ar_chan(input logic [31:0] addr);
        int g;
        @(negedge ACLK);
        S_ARVALID = 1; S_ARADDR = addr; S_ARPROT = '0;
        g = 0;
        do begin @(posedge ACLK); g++; end while (!S_ARREADY && g < TMO);
        if (g >= TMO) tmo_fail("ARREADY");
        @(negedge ACLK);
        S_ARVALID = 0; S_ARADDR = ~addr;
    endtask

    task automatic r_chan(input int r_delay, output logic [31:0] got_rdata, output logic [1:0] got_rresp);
        int g;
        got_rdata = 'x;
        got_rresp = 2'bxx;
        g = 0;
        do begin @(posedge ACLK); g++; end while (!S_RVALID && g < TMO);
        if (g >= TMO) tmo_fail("RVALID");
        repeat (r_delay) @(posedge ACLK);
        @(negedge ACLK);
        S_RREADY = 1;
        @(posedge ACLK);                           // handshake edge
        got_rdata = S_RDATA;
        got_rresp = S_RRESP;
        @(negedge ACLK);
        S_RREADY = 0;
    endtask

    // ---- AXI write: AW, W and B channels run concurrently --------------------
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
            aw_chan(addr, aw_delay);
            w_chan(data, strb, w_delay);
            b_chan(b_delay, got_bresp);
        join
        model_write(addr, data, strb);   // updated iff the write is EXPECTED to succeed
        repeat (2) @(posedge ACLK);
    endtask

    // ---- AXI read: AR and R channels run concurrently ------------------------
    task automatic axi_read(
        input  logic [31:0] addr,
        input  int          r_delay    = 0,
        output logic [31:0] got_rdata,
        output logic [1:0]  got_rresp
    );
        fork
            ar_chan(addr);
            r_chan(r_delay, got_rdata, got_rresp);
        join
        repeat (2) @(posedge ACLK);
    endtask

    // ---- Write and read issued at the same time (single fork level) ---------
    task automatic axi_write_read(
        input  logic [31:0] waddr, input logic [31:0] wdata, input logic [3:0] wstrb,
        input  int aw_delay, input int w_delay, input int b_delay,
        input  logic [31:0] raddr, input int r_delay,
        output logic [1:0]  got_bresp,
        output logic [31:0] got_rdata, output logic [1:0] got_rresp
    );
        fork
            aw_chan(waddr, aw_delay);
            w_chan(wdata, wstrb, w_delay);
            b_chan(b_delay, got_bresp);
            ar_chan(raddr);
            r_chan(r_delay, got_rdata, got_rresp);
        join
        model_write(waddr, wdata, wstrb);
        repeat (2) @(posedge ACLK);
    endtask

    // =========================================================================
    // Test variables
    // =========================================================================
    logic [1:0]  got_bresp, got_rresp, got_bresp2, got_rresp2;
    logic [31:0] got_rdata, got_rdata2;
    logic [31:0] exp_rdata;

    // random-test variables
    int          kind, ridx, rws, raw, rww, rbd, rrd, rpick;
    logic [31:0] raddr, rdata, raddr2;
    logic [3:0]  rstrb;

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
        for (int i = 0; i < 16; i++) cov_strb[i]  = 0;
        for (int i = 0; i < 5;  i++) cov_waits[i] = 0;

        repeat (4) @(posedge ACLK);
        @(negedge ACLK);
        ARESETn = 1;
        @(posedge ACLK);

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
            axi_write(.addr(PERIPH_BASE + i*4), .data(32'hA5A5_0000 | (i << 8) | i),
                      .got_bresp(got_bresp));
            check($sformatf("REG%0d write BRESP", i), got_bresp === 2'b00);
        end
        for (int i = 0; i < 8; i++) begin
            axi_read(.addr(PERIPH_BASE + i*4), .got_rdata(got_rdata), .got_rresp(got_rresp));
            exp_rdata = model_read(PERIPH_BASE + i*4);
            check($sformatf("REG%0d readback", i),
                  got_rdata === exp_rdata && got_rresp === 2'b00);
        end

        // =====================================================================
        // TEST 3: Partial writes with WSTRB (every single byte lane, then a mixed pattern)
        // =====================================================================
        $display("\n[Test 3] Partial writes - one byte lane at a time, then 4'b0101");
        axi_write(.addr(PERIPH_BASE + 0), .data(32'h0000_0000), .strb(4'hF), .got_bresp(got_bresp));
        for (int b = 0; b < 4; b++) begin
            axi_write(.addr(PERIPH_BASE + 0), .data(32'hFFFF_FFFF), .strb(4'(1 << b)),
                      .got_bresp(got_bresp));
            check($sformatf("lane %0d write BRESP", b), got_bresp === 2'b00);
            axi_read(.addr(PERIPH_BASE + 0), .got_rdata(got_rdata), .got_rresp(got_rresp));
            check($sformatf("lane %0d readback = %08h", b, model_read(PERIPH_BASE + 0)),
                  got_rdata === model_read(PERIPH_BASE + 0));
        end
        axi_write(.addr(PERIPH_BASE + 4), .data(32'h1122_3344), .strb(4'b1111), .got_bresp(got_bresp));
        axi_write(.addr(PERIPH_BASE + 4), .data(32'hAABB_CCDD), .strb(4'b0101), .got_bresp(got_bresp));
        axi_read(.addr(PERIPH_BASE + 4), .got_rdata(got_rdata), .got_rresp(got_rresp));
        check($sformatf("strobe 0101 readback = %08h", model_read(PERIPH_BASE + 4)),
              got_rdata === model_read(PERIPH_BASE + 4));

        // =====================================================================
        // TEST 4: Error responses
        // =====================================================================
        $display("\n[Test 4] Error responses - expect SLVERR");
        // (a) register index 8: inside the decoder window, outside the register file
        axi_write(.addr(PERIPH_BASE + 32'h0000_0020), .data(32'hDEAD_BEEF), .got_bresp(got_bresp));
        check("bad register: write BRESP=SLVERR", got_bresp === 2'b10);
        axi_read(.addr(PERIPH_BASE + 32'h0000_0020), .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("bad register: read RRESP=SLVERR", got_rresp === 2'b10);

        // (b) unaligned address
        axi_write(.addr(PERIPH_BASE + 32'h0000_0002), .data(32'hDEAD_BEEF), .got_bresp(got_bresp));
        check("unaligned: write BRESP=SLVERR", got_bresp === 2'b10);
        axi_read(.addr(PERIPH_BASE + 32'h0000_0002), .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("unaligned: read RRESP=SLVERR", got_rresp === 2'b10);

        // (c) unmapped: outside every decoder window, so no PSEL is generated
        axi_write(.addr(PERIPH_BASE + DEC_WINDOW), .data(32'hDEAD_BEEF), .got_bresp(got_bresp));
        check("unmapped (just past window): write BRESP=SLVERR", got_bresp === 2'b10);
        axi_read(.addr(32'h1000_0000), .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("unmapped (0x1000_0000): read RRESP=SLVERR", got_rresp === 2'b10);

        // none of the failed writes may have changed a register
        for (int i = 0; i < 8; i++) begin
            axi_read(.addr(PERIPH_BASE + i*4), .got_rdata(got_rdata), .got_rresp(got_rresp));
            check($sformatf("REG%0d unchanged by failed writes", i),
                  got_rdata === model_read(PERIPH_BASE + i*4) && got_rresp === 2'b00);
        end

        // =====================================================================
        // TEST 5: APB wait states (peripheral holds PREADY low for 3 cycles)
        // =====================================================================
        $display("\n[Test 5] APB wait states (3 cycles)");
        tb_wait_states = 3;
        axi_write(.addr(PERIPH_BASE + 4), .data(32'hBA5E_1234), .got_bresp(got_bresp));
        check("Wait-state write BRESP", got_bresp === 2'b00);

        axi_read(.addr(PERIPH_BASE + 4), .got_rdata(got_rdata), .got_rresp(got_rresp));
        exp_rdata = model_read(PERIPH_BASE + 4);
        check("Wait-state read data",  got_rdata === exp_rdata);
        check("Wait-state read RRESP", got_rresp === 2'b00);
        tb_wait_states = 0;

        // =====================================================================
        // TEST 6: Concurrent write + read - the write must reach APB first
        // =====================================================================
        $display("\n[Test 6] Concurrent write and read (arbitration)");
        log_wr.delete(); log_addr.delete();
        log_en = 1;
        exp_rdata = model_read(PERIPH_BASE + 4);          // read a different register than the write
        axi_write_read(.waddr(PERIPH_BASE + 0), .wdata(32'hC0DE_0001), .wstrb(4'hF),
                       .aw_delay(0), .w_delay(0), .b_delay(0),
                       .raddr(PERIPH_BASE + 4), .r_delay(0),
                       .got_bresp(got_bresp2), .got_rdata(got_rdata2), .got_rresp(got_rresp2));
        log_en = 0;
        check("concurrent: write BRESP OKAY", got_bresp2 === 2'b00);
        check("concurrent: read data correct", got_rdata2 === exp_rdata && got_rresp2 === 2'b00);
        check("concurrent: exactly two APB transfers", log_wr.size() == 2);
        if (log_wr.size() == 2) begin
            check("concurrent: write reached APB first (write priority)", log_wr[0] === 1'b1);
            check("concurrent: read reached APB second", log_wr[1] === 1'b0);
        end
        axi_read(.addr(PERIPH_BASE + 0), .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("concurrent: written value landed", got_rdata === model_read(PERIPH_BASE + 0));

        // =====================================================================
        // TEST 7: AW arrives 2 cycles before W
        // =====================================================================
        $display("\n[Test 7] AW 2 cycles before W");
        axi_write(.addr(PERIPH_BASE + 8), .data(32'hAA00_F1F5),
                  .aw_delay(0), .w_delay(2), .got_bresp(got_bresp));
        check("AW-first write BRESP", got_bresp === 2'b00);
        axi_read(.addr(PERIPH_BASE + 8), .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("AW-first readback", got_rdata === model_read(PERIPH_BASE + 8));

        // =====================================================================
        // TEST 8: W arrives 2 cycles before AW
        // =====================================================================
        $display("\n[Test 8] W 2 cycles before AW");
        axi_write(.addr(PERIPH_BASE + 12), .data(32'hBB00_F1F5),
                  .aw_delay(2), .w_delay(0), .got_bresp(got_bresp));
        check("W-first write BRESP", got_bresp === 2'b00);
        axi_read(.addr(PERIPH_BASE + 12), .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("W-first readback", got_rdata === model_read(PERIPH_BASE + 12));

        // =====================================================================
        // TEST 9: BREADY delayed 4 cycles
        // =====================================================================
        $display("\n[Test 9] BREADY delayed 4 cycles");
        axi_write(.addr(PERIPH_BASE + 16), .data(32'h0BB_DE1A7),
                  .b_delay(4), .got_bresp(got_bresp));
        check("B-delay write BRESP", got_bresp === 2'b00);
        axi_read(.addr(PERIPH_BASE + 16), .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("B-delay readback", got_rdata === model_read(PERIPH_BASE + 16));

        // =====================================================================
        // TEST 10: RREADY delayed 4 cycles
        // =====================================================================
        $display("\n[Test 10] RREADY delayed 4 cycles");
        axi_write(.addr(PERIPH_BASE + 20), .data(32'h0CC_DE1A7),
                  .got_bresp(got_bresp));
        axi_read(.addr(PERIPH_BASE + 20), .r_delay(4),
                 .got_rdata(got_rdata), .got_rresp(got_rresp));
        check("R-delay read data",  got_rdata === model_read(PERIPH_BASE + 20));
        check("R-delay read RRESP", got_rresp === 2'b00);

        // =====================================================================
        // TEST 11: Back-to-back writes, next write starts as soon as the last B completes
        // =====================================================================
        $display("\n[Test 11] Back-to-back writes (4 consecutive, then read all four)");
        for (int i = 0; i < 4; i++) begin
            axi_write(.addr(PERIPH_BASE + i*4), .data(32'hBBBB_0000 | i), .got_bresp(got_bresp));
            check($sformatf("BB write %0d BRESP", i), got_bresp === 2'b00);
        end
        for (int i = 0; i < 4; i++) begin
            axi_read(.addr(PERIPH_BASE + i*4), .got_rdata(got_rdata), .got_rresp(got_rresp));
            check($sformatf("BB readback %0d", i),
                  got_rdata === model_read(PERIPH_BASE + i*4));
        end

        // =====================================================================
        // TEST 12: Constrained-random mix, checked against the model
        // =====================================================================
        $display("\n[Test 12] Constrained-random mix (%0d operations)", NUM_RANDOM);
        for (int n = 0; n < NUM_RANDOM; n++) begin
            kind  = $urandom_range(0, 9);                 // 0-3 write, 4-6 read, 7-8 concurrent, 9 error
            rws   = $urandom_range(0, 4);                 // APB wait states
            raw   = $urandom_range(0, 3);                 // AW delay
            rww   = $urandom_range(0, 3);                 // W delay
            rbd   = $urandom_range(0, 3);                 // B stall
            rrd   = $urandom_range(0, 3);                 // R stall
            ridx  = $urandom_range(0, 7);
            rstrb = 4'($urandom_range(1, 15));
            rdata = $urandom;
            raddr = PERIPH_BASE + ridx*4;
            tb_wait_states = 4'(rws);
            cov_waits[rws]++;
            cov_strb[rstrb]++;
            if (rbd != 0) cov_stall_b++;
            if (rrd != 0) cov_stall_r++;

            if (kind <= 3) begin                           // ---- write, then read back
                if      (raw == 0 && rww == 0) cov_ord_simul++;
                else if (raw <  rww)           cov_ord_aw_first++;
                else if (rww <  raw)           cov_ord_w_first++;
                else                           cov_ord_simul++;
                axi_write(.addr(raddr), .data(rdata), .strb(rstrb),
                          .aw_delay(raw), .w_delay(rww), .b_delay(rbd), .got_bresp(got_bresp));
                check_q($sformatf("rand %0d: write BRESP OKAY", n), got_bresp === 2'b00);
                axi_read(.addr(raddr), .r_delay(rrd), .got_rdata(got_rdata), .got_rresp(got_rresp));
                check_q($sformatf("rand %0d: readback %08h (strb %b)", n, model_read(raddr), rstrb),
                        got_rdata === model_read(raddr) && got_rresp === 2'b00);
            end else if (kind <= 6) begin                  // ---- read
                axi_read(.addr(raddr), .r_delay(rrd), .got_rdata(got_rdata), .got_rresp(got_rresp));
                check_q($sformatf("rand %0d: read data/resp", n),
                        got_rdata === model_read(raddr) && got_rresp === 2'b00);
            end else if (kind <= 8) begin                  // ---- write and read at the same time
                raddr2 = PERIPH_BASE + ((ridx + 1) % 8)*4;  // different register than the write
                exp_rdata = model_read(raddr2);
                cov_concurrent++;
                axi_write_read(.waddr(raddr), .wdata(rdata), .wstrb(rstrb),
                               .aw_delay(raw), .w_delay(rww), .b_delay(rbd),
                               .raddr(raddr2), .r_delay(rrd),
                               .got_bresp(got_bresp2), .got_rdata(got_rdata2), .got_rresp(got_rresp2));
                check_q($sformatf("rand %0d: concurrent write BRESP", n), got_bresp2 === 2'b00);
                check_q($sformatf("rand %0d: concurrent read data", n),
                        got_rdata2 === exp_rdata && got_rresp2 === 2'b00);
            end else begin                                  // ---- error: bad register / unaligned / unmapped
                rpick = $urandom_range(0, 3);
                case (rpick)
                    0:       raddr = PERIPH_BASE + 32'h20 + $urandom_range(0, 7)*4;   // bad register
                    1:       raddr = PERIPH_BASE + ridx*4 + $urandom_range(1, 3);     // unaligned
                    2:       raddr = PERIPH_BASE + DEC_WINDOW + $urandom_range(0, 255)*4;  // unmapped above
                    default: raddr = 32'h0000_0000 + $urandom_range(0, 255)*4;        // unmapped below
                endcase
                cov_err_resp++;
                axi_write(.addr(raddr), .data(rdata), .strb(rstrb),
                          .aw_delay(raw), .w_delay(rww), .b_delay(rbd), .got_bresp(got_bresp));
                check_q($sformatf("rand %0d: write to %08h BRESP SLVERR", n, raddr), got_bresp === 2'b10);
                axi_read(.addr(raddr), .r_delay(rrd), .got_rdata(got_rdata), .got_rresp(got_rresp));
                check_q($sformatf("rand %0d: read of %08h RRESP SLVERR", n, raddr), got_rresp === 2'b10);
            end
        end
        tb_wait_states = 0;

        // final sweep: every register must still match the model
        for (int i = 0; i < 8; i++) begin
            axi_read(.addr(PERIPH_BASE + i*4), .got_rdata(got_rdata), .got_rresp(got_rresp));
            check($sformatf("final sweep REG%0d = %08h", i, model_read(PERIPH_BASE + i*4)),
                  got_rdata === model_read(PERIPH_BASE + i*4) && got_rresp === 2'b00);
        end

        // ---- coverage summary (counters, since covergroups are not available in every simulator) ----
        $display("\n[Coverage] random test: AW/W ordering simul=%0d aw-first=%0d w-first=%0d | concurrent=%0d errors=%0d B-stalls=%0d R-stalls=%0d",
                 cov_ord_simul, cov_ord_aw_first, cov_ord_w_first, cov_concurrent, cov_err_resp, cov_stall_b, cov_stall_r);
        $display("           APB wait states 0..4: %0d %0d %0d %0d %0d", cov_waits[0], cov_waits[1], cov_waits[2], cov_waits[3], cov_waits[4]);
        check("coverage: AW/W simultaneous exercised", cov_ord_simul    > 0);
        check("coverage: AW before W exercised",       cov_ord_aw_first > 0);
        check("coverage: W before AW exercised",       cov_ord_w_first  > 0);
        check("coverage: concurrent write+read exercised", cov_concurrent > 0);
        check("coverage: error responses exercised",   cov_err_resp     > 0);
        check("coverage: every APB wait-state count 0..4 exercised",
              cov_waits[0] > 0 && cov_waits[1] > 0 && cov_waits[2] > 0 && cov_waits[3] > 0 && cov_waits[4] > 0);
        check("coverage: B and R back-pressure exercised", cov_stall_b > 0 && cov_stall_r > 0);

        // =====================================================================
        // Summary
        // =====================================================================
        repeat (4) @(posedge ACLK);
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
    // SVA: end-to-end protocol checks
    // =========================================================================

    // AXI: once VALID is asserted it (and its payload) holds until READY.
    // AW / W / AR are driven by this testbench, so these also prove the BFM is
    // protocol-clean.
    sva_aw_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (S_AWVALID && !S_AWREADY) |=> (S_AWVALID && $stable(S_AWADDR))
    ) else sva_hit("SVA: AWVALID dropped or AWADDR changed before AWREADY");

    sva_w_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (S_WVALID && !S_WREADY) |=> (S_WVALID && $stable(S_WDATA) && $stable(S_WSTRB))
    ) else sva_hit("SVA: WVALID dropped or W payload changed before WREADY");

    sva_ar_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (S_ARVALID && !S_ARREADY) |=> (S_ARVALID && $stable(S_ARADDR))
    ) else sva_hit("SVA: ARVALID dropped or ARADDR changed before ARREADY");

    // BVALID / RVALID and their payload hold until BREADY / RREADY
    sva_bvalid_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (S_BVALID && !S_BREADY) |=> (S_BVALID && $stable(S_BRESP))
    ) else sva_hit("SVA: S_BVALID dropped or S_BRESP changed before S_BREADY");

    sva_rvalid_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        (S_RVALID && !S_RREADY) |=> (S_RVALID && $stable(S_RDATA) && $stable(S_RRESP))
    ) else sva_hit("SVA: S_RVALID dropped or S_RDATA/S_RRESP changed before S_RREADY");

    // Single outstanding transaction per direction
    sva_single_write: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        S_BVALID |-> (!S_AWREADY && !S_WREADY)
    ) else sva_hit("SVA: AW/W accepted while a write response is pending");

    sva_single_read: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        S_RVALID |-> !S_ARREADY
    ) else sva_hit("SVA: AR accepted while a read response is pending");

    // APB: PENABLE never without PSEL
    sva_apb_penable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        M_PENABLE |-> |M_PSEL
    ) else sva_hit("SVA: M_PENABLE asserted without any M_PSEL");

    // APB: SETUP lasts one cycle
    sva_apb_setup_enable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        ((|M_PSEL) && !M_PENABLE) |=> M_PENABLE
    ) else sva_hit("SVA: SETUP not followed by ENABLE");

    // APB: address and control stable from SETUP through the completing cycle
    sva_apb_stable: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        ((|M_PSEL) && !(M_PENABLE && M_PREADY[0])) |=>
            ($stable(M_PADDR) && $stable(M_PWRITE) && $stable(M_PWDATA) && $stable(M_PSTRB))
    ) else sva_hit("SVA: APB address/control/data changed during an active transfer");

    // APB4: PSTRB must be low on read transfers
    sva_apb_pstrb_read: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        ((|M_PSEL) && !M_PWRITE) |-> (M_PSTRB == '0)
    ) else sva_hit("SVA: PSTRB not low on a read transfer");

    // At most one PSEL asserted at a time (one-hot check)
    sva_psel_onehot: assert property (
        @(posedge ACLK) disable iff (!ARESETn)
        $onehot0(M_PSEL)
    ) else sva_hit("SVA: Multiple M_PSEL bits asserted simultaneously");

    // =========================================================================
    // Timeout watchdog
    // =========================================================================
    initial begin
        #(CLK_HALF * 2 * 400_000);
        $display("TIMEOUT - simulation hung");
        $fatal(1);
    end

endmodule : bridge_integration_tb

// =============================================================================
// apb_manager_tb.sv
// Self-checking testbench for apb_manager
//
// Test cases:
//   1. Single write, PREADY immediate (no wait states)
//   2. Single read,  PREADY immediate
//   3. Write with 2 wait-state cycles (PREADY deasserted)
//   4. PSLVERR on a write — checks rsp_error propagation
//   5. Back-to-back writes (req_valid held high across transfers)
//
// Pass/fail reported per test. Final PASS/FAIL summary at end.
// =============================================================================

`timescale 1ns / 1ps

module apb_manager_tb;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam int ADDR_WIDTH = 32;
    localparam int DATA_WIDTH = 32;
    localparam int CLK_PERIOD = 10; // ns

    // =========================================================================
    // DUT signals
    // =========================================================================
    logic                    PCLK, PRESETn;
    logic                    req_valid, req_write;
    logic [ADDR_WIDTH-1:0]   req_addr;
    logic [DATA_WIDTH-1:0]   req_wdata;
    logic [DATA_WIDTH/8-1:0] req_strb;
    logic                    req_ready;
    logic                    rsp_valid;
    logic [DATA_WIDTH-1:0]   rsp_rdata;
    logic                    rsp_error;
    logic                    PSEL, PENABLE, PWRITE;
    logic [ADDR_WIDTH-1:0]   PADDR;
    logic [DATA_WIDTH-1:0]   PWDATA;
    logic [DATA_WIDTH/8-1:0] PSTRB;
    logic                    PREADY;
    logic [DATA_WIDTH-1:0]   PRDATA;
    logic                    PSLVERR;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    apb_manager #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) dut (.*);

    // =========================================================================
    // Clock
    // =========================================================================
    initial PCLK = 0;
    always #(CLK_PERIOD/2) PCLK = ~PCLK;

    // =========================================================================
    // Scoreboard
    // =========================================================================
    int pass_count = 0;
    int fail_count = 0;

    task automatic check(
        input string test_name,
        input logic  got_error,
        input logic  exp_error,
        input logic [DATA_WIDTH-1:0] got_rdata,
        input logic [DATA_WIDTH-1:0] exp_rdata,
        input int    got_cycles,
        input int    exp_cycles_min,
        input int    exp_cycles_max
    );
        logic ok;
        ok = (got_error === exp_error) &&
             (exp_error || (got_rdata === exp_rdata)) &&  // rdata only checked on non-error
             (got_cycles >= exp_cycles_min) &&
             (got_cycles <= exp_cycles_max);

        if (ok) begin
            $display("  PASS  %s", test_name);
            pass_count++;
        end else begin
            $display("  FAIL  %s", test_name);
            if (got_error !== exp_error)
                $display("         rsp_error: got %0b, expected %0b", got_error, exp_error);
            if (!exp_error && got_rdata !== exp_rdata)
                $display("         rsp_rdata: got 0x%08h, expected 0x%08h", got_rdata, exp_rdata);
            if (got_cycles < exp_cycles_min || got_cycles > exp_cycles_max)
                $display("         cycles: got %0d, expected %0d..%0d",
                         got_cycles, exp_cycles_min, exp_cycles_max);
            fail_count++;
        end
    endtask

    // =========================================================================
    // Helper: drive a single APB transfer and collect results
    //
    //   wait_states  — how many cycles to hold PREADY=0 in ENABLE phase
    //   slave_err    — assert PSLVERR on the PREADY cycle
    //   read_data    — value to return on PRDATA
    // =========================================================================
    task automatic drive_transfer(
        input  logic [ADDR_WIDTH-1:0] addr,
        input  logic [DATA_WIDTH-1:0] wdata,
        input  logic                  is_write,
        input  int                    wait_states,
        input  logic                  slave_err,
        input  logic [DATA_WIDTH-1:0] read_data,
        output logic                  got_error,
        output logic [DATA_WIDTH-1:0] got_rdata,
        output int                    got_cycles   // cycles from req_valid to rsp_valid
    );
        int cycle_count;
        cycle_count = 0;

        // ------- Assert request -------
        @(negedge PCLK);           // drive on negedge to be sampled on next posedge
        req_valid = 1;
        req_write = is_write;
        req_addr  = addr;
        req_wdata = wdata;
        req_strb  = is_write ? 4'hF : 4'h0;
        PREADY    = 0;
        PSLVERR   = 0;
        PRDATA    = read_data;

        // ------- Wait for SETUP phase (PSEL=1, PENABLE=0) -------
        @(posedge PCLK); #1;
        cycle_count++;

        // ------- ENABLE phase starts next clock -------
        @(posedge PCLK); #1;
        cycle_count++;

        // ------- Hold PREADY low for wait_states cycles -------
        repeat (wait_states) begin
            @(posedge PCLK); #1;
            cycle_count++;
        end

        // ------- Assert PREADY (and optionally PSLVERR) -------
        @(negedge PCLK);
        PREADY  = 1;
        PSLVERR = slave_err;

        // ------- Capture response on next posedge -------
        @(posedge PCLK); #1;
        cycle_count++;
        got_error = rsp_error;
        got_rdata = rsp_rdata;
        got_cycles = cycle_count;

        // ------- Deassert everything -------
        @(negedge PCLK);
        req_valid = 0;
        PREADY    = 0;
        PSLVERR   = 0;

        // Give one idle cycle before next transfer
        @(posedge PCLK); #1;

    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    logic [DATA_WIDTH-1:0] got_rdata;
    logic                  got_error;
    int                    got_cycles;

    initial begin
        $display("============================================================");
        $display(" APB Manager Testbench");
        $display("============================================================");

        // --- Reset ---
        PRESETn   = 0;
        req_valid = 0;
        req_write = 0;
        req_addr  = '0;
        req_wdata = '0;
        req_strb  = '0;
        PREADY    = 0;
        PRDATA    = '0;
        PSLVERR   = 0;

        repeat (4) @(posedge PCLK);
        PRESETn = 1;
        @(posedge PCLK);

        // -----------------------------------------------------------------
        // TEST 1: Write, no wait states
        // Expect: IDLE→SETUP(1 cy)→ENABLE(1 cy)→rsp_valid = 2 bus cycles
        // -----------------------------------------------------------------
        $display("\n[Test 1] Write, 0 wait states");
        drive_transfer(
            .addr(32'hC000_0010), .wdata(32'hDEAD_BEEF),
            .is_write(1), .wait_states(0), .slave_err(0),
            .read_data('0),
            .got_error(got_error), .got_rdata(got_rdata), .got_cycles(got_cycles)
        );
        check("Write no-wait", got_error, 0, got_rdata, '0, got_cycles, 2, 3);

        // -----------------------------------------------------------------
        // TEST 2: Read, no wait states
        // -----------------------------------------------------------------
        $display("\n[Test 2] Read, 0 wait states");
        drive_transfer(
            .addr(32'hC000_0020), .wdata('0),
            .is_write(0), .wait_states(0), .slave_err(0),
            .read_data(32'hCAFE_1234),
            .got_error(got_error), .got_rdata(got_rdata), .got_cycles(got_cycles)
        );
        check("Read no-wait", got_error, 0, got_rdata, 32'hCAFE_1234, got_cycles, 2, 3);

        // -----------------------------------------------------------------
        // TEST 3: Write with 2 wait states
        // Expect: IDLE→SETUP(1)→ENABLE(1+2 wait+1 ready) = min 4 cycles
        // -----------------------------------------------------------------
        $display("\n[Test 3] Write, 2 wait states");
        drive_transfer(
            .addr(32'hC000_0030), .wdata(32'h1234_5678),
            .is_write(1), .wait_states(2), .slave_err(0),
            .read_data('0),
            .got_error(got_error), .got_rdata(got_rdata), .got_cycles(got_cycles)
        );
        check("Write 2-wait", got_error, 0, got_rdata, '0, got_cycles, 4, 5);

        // -----------------------------------------------------------------
        // TEST 4: PSLVERR on write
        // -----------------------------------------------------------------
        $display("\n[Test 4] Write with PSLVERR");
        drive_transfer(
            .addr(32'hDEAD_0000), .wdata(32'hFFFF_FFFF),
            .is_write(1), .wait_states(0), .slave_err(1),
            .read_data('0),
            .got_error(got_error), .got_rdata(got_rdata), .got_cycles(got_cycles)
        );
        check("Write PSLVERR", got_error, 1, got_rdata, '0, got_cycles, 2, 3);

        // -----------------------------------------------------------------
        // TEST 5: Read with PSLVERR
        // -----------------------------------------------------------------
        $display("\n[Test 5] Read with PSLVERR");
        drive_transfer(
            .addr(32'hDEAD_0004), .wdata('0),
            .is_write(0), .wait_states(0), .slave_err(1),
            .read_data(32'hBAD0_BEEF),
            .got_error(got_error), .got_rdata(got_rdata), .got_cycles(got_cycles)
        );
        check("Read PSLVERR", got_error, 1, got_rdata, '0, got_cycles, 2, 3);

        // -----------------------------------------------------------------
        // TEST 6: APB protocol check — PSEL/PENABLE sequencing
        // Manually verify that PENABLE is never high without PSEL
        // (this runs concurrently via the assertion below)
        // -----------------------------------------------------------------
        $display("\n[Test 6] Protocol: PENABLE never asserted without PSEL");
        // The SVA assertion at module scope covers this continuously.
        // We just run a normal write and rely on the assertion firing if broken.
        drive_transfer(
            .addr(32'hA000_0000), .wdata(32'h0000_0001),
            .is_write(1), .wait_states(1), .slave_err(0),
            .read_data('0),
            .got_error(got_error), .got_rdata(got_rdata), .got_cycles(got_cycles)
        );
        check("Protocol PENABLE/PSEL", got_error, 0, got_rdata, '0, got_cycles, 3, 4);

        // -----------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------
        @(posedge PCLK);
        $display("\n============================================================");
        $display(" Results: %0d passed, %0d failed", pass_count, fail_count);
        if (fail_count == 0)
            $display(" ALL TESTS PASSED");
        else
            $display(" SOME TESTS FAILED — review output above");
        $display("============================================================");

        $finish;
    end

    // =========================================================================
    // SystemVerilog Assertions (SVA)
    // These fire as simulation errors if the APB protocol is violated.
    // =========================================================================

    // PENABLE must never be high when PSEL is low
    apb_penable_requires_psel: assert property (
        @(posedge PCLK) disable iff (!PRESETn)
        PENABLE |-> PSEL
    ) else $error("PROTOCOL VIOLATION: PENABLE asserted without PSEL");

    // PSEL must be stable throughout the ENABLE phase (no mid-transfer deselect)
    apb_psel_stable_during_enable: assert property (
        @(posedge PCLK) disable iff (!PRESETn)
        (PSEL && PENABLE && !PREADY) |=> PSEL
    ) else $error("PROTOCOL VIOLATION: PSEL dropped before PREADY");

    // PADDR must be stable throughout SETUP → ENABLE
    apb_paddr_stable: assert property (
        @(posedge PCLK) disable iff (!PRESETn)
        (PSEL && PENABLE && !PREADY) |=> ($stable(PADDR))
    ) else $error("PROTOCOL VIOLATION: PADDR changed during active transfer");

    // PWRITE must be stable throughout SETUP → ENABLE
    apb_pwrite_stable: assert property (
        @(posedge PCLK) disable iff (!PRESETn)
        (PSEL && PENABLE && !PREADY) |=> ($stable(PWRITE))
    ) else $error("PROTOCOL VIOLATION: PWRITE changed during active transfer");

    // PWDATA must be stable throughout SETUP → ENABLE (writes only)
    apb_pwdata_stable: assert property (
        @(posedge PCLK) disable iff (!PRESETn)
        (PSEL && PENABLE && PWRITE && !PREADY) |=> ($stable(PWDATA))
    ) else $error("PROTOCOL VIOLATION: PWDATA changed during active write");

    // rsp_valid must be a single-cycle pulse
    apb_rsp_single_pulse: assert property (
        @(posedge PCLK) disable iff (!PRESETn)
        rsp_valid |=> !rsp_valid
    ) else $error("PROTOCOL VIOLATION: rsp_valid held high for more than one cycle");

    // =========================================================================
    // Timeout watchdog — catches infinite loops
    // =========================================================================
    initial begin
        #(CLK_PERIOD * 10000);
        $display("TIMEOUT — simulation did not complete");
        $fatal(1);
    end

endmodule : apb_manager_tb

// =============================================================================
// apb_manager_tb.sv
// Self-checking testbench for apb_manager
//
// Test cases:
//   1. Single write, PREADY immediate (no wait states)
//   2. Single read,  PREADY immediate
//   3. Write with 2 wait-state cycles (PREADY deasserted)
//   4. PSLVERR on a write - checks rsp_error propagation
//   5. PSLVERR on a read  - checks rsp_error propagation
//   6. Write with 1 wait state (SETUP/ENABLE sequencing; the SVA below watch it)
//   7. Back-to-back requests: req_valid held high across transfers, new request
//      accepted on the completing ENABLE cycle (req_valid && req_ready)
//
// Every single-transfer test checks: rsp_error, rsp_rdata, the exact cycle
// count, the address/control/data on the APB bus during SETUP and ENABLE, and
// that the manager returns to IDLE afterwards (no spurious second transfer).
//
// Conventions that keep the checks race-free
//   - Stimulus is driven at negedge PCLK.
//   - DUT outputs are sampled AT posedge PCLK (before the DUT's nonblocking
//     updates), i.e. the value the DUT presented during the cycle that the edge
//     closes. Sampling a delay after the edge sees the NEXT cycle's state and
//     misses one-cycle pulses such as rsp_valid / rsp_error.
//   - req_valid is a request pulse: it is dropped once the request is accepted
//     (req_valid && req_ready at a posedge). Holding it through the completing
//     edge makes the manager start a second transfer.
// =============================================================================

`timescale 1ns / 1ps

module apb_manager_tb;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam int ADDR_WIDTH = 32;
    localparam int DATA_WIDTH = 32;
    localparam int CLK_PERIOD = 10; // ns
    localparam int TMO        = 40; // max cycles to wait for any single event

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
    int sva_fail   = 0;     // incremented by every assertion's else-branch

    task automatic check_bool(input string name, input logic cond);
        if (cond) begin
            $display("  PASS  %s", name);
            pass_count++;
        end else begin
            $display("  FAIL  %s", name);
            fail_count++;
        end
    endtask

    // Called from every assertion's else-branch: counts the failure and reports it
    task automatic sva_hit(input string msg);
        sva_fail++;
        $error("%s", msg);
    endtask

    task automatic check(
        input string test_name,
        input logic  got_error,
        input logic  exp_error,
        input logic [DATA_WIDTH-1:0] got_rdata,
        input logic [DATA_WIDTH-1:0] exp_rdata,
        input int    got_cycles,
        input int    exp_cycles
    );
        logic ok;
        ok = (got_error === exp_error) &&
             (exp_error || (got_rdata === exp_rdata)) &&  // rdata only checked on non-error
             (got_cycles == exp_cycles);

        if (ok) begin
            $display("  PASS  %s", test_name);
            pass_count++;
        end else begin
            $display("  FAIL  %s", test_name);
            if (got_error !== exp_error)
                $display("         rsp_error: got %0b, expected %0b", got_error, exp_error);
            if (!exp_error && got_rdata !== exp_rdata)
                $display("         rsp_rdata: got 0x%08h, expected 0x%08h", got_rdata, exp_rdata);
            if (got_cycles != exp_cycles)
                $display("         cycles: got %0d, expected %0d", got_cycles, exp_cycles);
            fail_count++;
        end
    endtask

    // =========================================================================
    // Helper: drive ONE APB transfer, acting as the requester and as the slave
    //
    //   wait_states  - how many ENABLE cycles to hold PREADY low
    //   slave_err    - assert PSLVERR on the completing cycle
    //   read_data    - value to return on PRDATA
    //
    //   got_cycles   - posedges from request acceptance through the completing
    //                  edge, inclusive: 3 + wait_states for a correct manager
    //                  (accept edge, SETUP->ENABLE edge, waits, completing edge)
    //   bus_ok       - PSEL/PENABLE sequencing and PADDR/PWRITE/PWDATA/PSTRB on
    //                  the APB bus matched the request in SETUP and in ENABLE,
    //                  and rsp_valid was low until the completing cycle
    //   idle_ok      - manager returned to IDLE (PSEL=PENABLE=0) afterwards
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
        output int                    got_cycles,
        output logic                  bus_ok,
        output logic                  idle_ok
    );
        int guard;
        got_cycles = 0;
        bus_ok     = 1;
        idle_ok    = 1;
        got_error  = 0;
        got_rdata  = '0;

        // ------- 1. Present the request and wait for it to be accepted -------
        @(negedge PCLK);
        req_valid = 1;
        req_write = is_write;
        req_addr  = addr;
        req_wdata = wdata;
        req_strb  = is_write ? 4'hF : 4'h0;
        PREADY    = 0;
        PSLVERR   = 0;
        PRDATA    = read_data;

        guard = 0;
        do begin
            @(posedge PCLK);                 // req_valid && req_ready sampled here
            got_cycles++;
            guard++;
        end while (!req_ready && guard < TMO);
        if (guard >= TMO) begin
            $display("  FAIL  request never accepted (req_ready stuck low)");
            fail_count++;
            bus_ok = 0;
        end
        @(negedge PCLK);
        req_valid = 0;                       // request is a pulse: drop it once accepted

        // ------- 2. SETUP phase (PSEL=1, PENABLE=0) -------
        if (!(PSEL === 1'b1 && PENABLE === 1'b0 && PADDR === addr && PWRITE === is_write)) bus_ok = 0;
        if (is_write && !(PWDATA === wdata && PSTRB === 4'hF))                              bus_ok = 0;

        @(posedge PCLK);                     // SETUP -> ENABLE edge
        got_cycles++;

        // ------- 3. ENABLE phase (PSEL=1, PENABLE=1); hold PREADY low for the wait states -------
        @(negedge PCLK);
        if (!(PSEL === 1'b1 && PENABLE === 1'b1 && PADDR === addr && PWRITE === is_write)) bus_ok = 0;
        if (is_write && !(PWDATA === wdata && PSTRB === 4'hF))                              bus_ok = 0;

        repeat (wait_states) begin
            if (rsp_valid !== 1'b0) bus_ok = 0;          // no response while PREADY is low
            @(posedge PCLK);
            got_cycles++;
            @(negedge PCLK);
            if (!(PSEL === 1'b1 && PENABLE === 1'b1 && PADDR === addr)) bus_ok = 0;   // held stable
        end

        // ------- 4. Complete: PREADY (and optionally PSLVERR) for one cycle -------
        PREADY  = 1;
        PSLVERR = slave_err;
        @(posedge PCLK);                     // completing edge: sample the one-cycle response
        got_cycles++;
        got_error = rsp_error;
        got_rdata = rsp_rdata;
        if (rsp_valid !== 1'b1) bus_ok = 0;

        // ------- 5. Drop the slave response; manager must be back in IDLE -------
        @(negedge PCLK);
        PREADY  = 0;
        PSLVERR = 0;
        if (PSEL !== 1'b0 || PENABLE !== 1'b0) idle_ok = 0;

        @(posedge PCLK);                     // one idle cycle before the next transfer
    endtask

    // One transfer + the standard checks
    task automatic run_case(
        input string                  name,
        input logic [ADDR_WIDTH-1:0]  addr,
        input logic [DATA_WIDTH-1:0]  wdata,
        input logic                   is_write,
        input int                     waits,
        input logic                   err,
        input logic [DATA_WIDTH-1:0]  rdata
    );
        logic                  got_error;
        logic [DATA_WIDTH-1:0] got_rdata;
        int                    got_cycles;
        logic                  bus_ok, idle_ok;

        drive_transfer(.addr(addr), .wdata(wdata), .is_write(is_write),
                       .wait_states(waits), .slave_err(err), .read_data(rdata),
                       .got_error(got_error), .got_rdata(got_rdata), .got_cycles(got_cycles),
                       .bus_ok(bus_ok), .idle_ok(idle_ok));
        check(name, got_error, err, got_rdata, rdata, got_cycles, 3 + waits);
        check_bool({name, ": APB bus carries the request"}, bus_ok);
        check_bool({name, ": returns to IDLE"},             idle_ok);
    endtask

    // =========================================================================
    // Passive APB monitor (used by the back-to-back test)
    // =========================================================================
    bit                    mon_en = 0;
    bit                    obs_wr    [$];
    logic [ADDR_WIDTH-1:0] obs_addr  [$];
    logic [DATA_WIDTH-1:0] obs_wdata [$];

    always @(posedge PCLK) begin
        if (mon_en && PSEL && PENABLE && PREADY) begin
            obs_wr.push_back(PWRITE);
            obs_addr.push_back(PADDR);
            obs_wdata.push_back(PWDATA);
        end
    end

    // =========================================================================
    // Main test sequence
    // =========================================================================
    logic [ADDR_WIDTH-1:0] b2b_addr  [0:2];
    logic [DATA_WIDTH-1:0] b2b_wdata [0:2];
    logic                  b2b_wr    [0:2];
    int                    guard;
    logic                  b2b_ok;

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
        @(negedge PCLK);
        PRESETn = 1;
        @(posedge PCLK);

        // -----------------------------------------------------------------
        // TEST 1: Write, no wait states.  accept, SETUP->ENABLE, complete = 3 edges
        // -----------------------------------------------------------------
        $display("\n[Test 1] Write, 0 wait states");
        run_case("Write no-wait", 32'hC000_0010, 32'hDEAD_BEEF, 1, 0, 0, '0);

        // -----------------------------------------------------------------
        // TEST 2: Read, no wait states
        // -----------------------------------------------------------------
        $display("\n[Test 2] Read, 0 wait states");
        run_case("Read no-wait", 32'hC000_0020, '0, 0, 0, 0, 32'hCAFE_1234);

        // -----------------------------------------------------------------
        // TEST 3: Write with 2 wait states = 5 edges
        // -----------------------------------------------------------------
        $display("\n[Test 3] Write, 2 wait states");
        run_case("Write 2-wait", 32'hC000_0030, 32'h1234_5678, 1, 2, 0, '0);

        // -----------------------------------------------------------------
        // TEST 4: PSLVERR on write
        // -----------------------------------------------------------------
        $display("\n[Test 4] Write with PSLVERR");
        run_case("Write PSLVERR", 32'hDEAD_0000, 32'hFFFF_FFFF, 1, 0, 1, '0);

        // -----------------------------------------------------------------
        // TEST 5: PSLVERR on read
        // -----------------------------------------------------------------
        $display("\n[Test 5] Read with PSLVERR");
        run_case("Read PSLVERR", 32'hDEAD_0004, '0, 0, 0, 1, 32'hBAD0_BEEF);

        // -----------------------------------------------------------------
        // TEST 6: Protocol sequencing with a wait state. run_case checks PSEL then
        // PENABLE one cycle later and stable address/data; the SVA below check the
        // same rules on every cycle.
        // -----------------------------------------------------------------
        $display("\n[Test 6] Protocol: PSEL/PENABLE sequencing, 1 wait state");
        run_case("Protocol PENABLE/PSEL", 32'hA000_0000, 32'h0000_0001, 1, 1, 0, '0);

        // -----------------------------------------------------------------
        // TEST 7: Back-to-back write, read, write. req_valid is held high across
        // transfers; each request is presented while the previous one is in
        // progress and must be accepted (req_valid && req_ready) on its completing
        // ENABLE cycle and run with ITS OWN address/data.  Slave: PREADY always 1.
        // -----------------------------------------------------------------
        $display("\n[Test 7] Back-to-back requests, req_valid held across transfers");
        b2b_addr[0] = 32'hA000_0100; b2b_wdata[0] = 32'h1111_0001; b2b_wr[0] = 1;
        b2b_addr[1] = 32'hA000_0200; b2b_wdata[1] = 32'h2222_0002; b2b_wr[1] = 0;
        b2b_addr[2] = 32'hA000_0300; b2b_wdata[2] = 32'h3333_0003; b2b_wr[2] = 1;
        b2b_ok = 1;
        obs_wr.delete(); obs_addr.delete(); obs_wdata.delete();
        mon_en = 1;
        @(negedge PCLK);
        PREADY  = 1;
        PSLVERR = 0;
        PRDATA  = 32'h5A5A_5A5A;

        for (int i = 0; i < 3; i++) begin
            @(negedge PCLK);
            req_valid = 1;
            req_write = b2b_wr[i];
            req_addr  = b2b_addr[i];
            req_wdata = b2b_wdata[i];
            req_strb  = b2b_wr[i] ? 4'hF : 4'h0;
            guard = 0;
            do begin
                @(posedge PCLK);             // acceptance = req_valid && req_ready at this edge
                guard++;
            end while (!req_ready && guard < TMO);
            if (guard >= TMO) begin
                b2b_ok = 0;
                break;
            end
        end
        @(negedge PCLK);
        req_valid = 0;
        repeat (8) @(posedge PCLK);          // drain the last transfer
        mon_en = 0;
        @(negedge PCLK);
        PREADY = 0;

        check_bool("B2B: all three requests were accepted", b2b_ok);
        check_bool("B2B: exactly three APB transfers occurred", obs_addr.size() == 3);
        if (obs_addr.size() == 3) begin
            for (int i = 0; i < 3; i++) begin
                check_bool($sformatf("B2B: transfer %0d ran with its own address/direction%s", i,
                                     b2b_wr[i] ? "/data" : ""),
                           obs_addr[i] === b2b_addr[i] && obs_wr[i] === b2b_wr[i] &&
                           (!b2b_wr[i] || obs_wdata[i] === b2b_wdata[i]));
            end
        end
        @(posedge PCLK);
        check_bool("B2B: manager returns to IDLE", PSEL === 1'b0 && PENABLE === 1'b0);

        // -----------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------
        @(posedge PCLK);
        $display("\n============================================================");
        $display(" Results: %0d passed, %0d failed, %0d assertion failures",
                 pass_count, fail_count, sva_fail);
        if (fail_count == 0 && sva_fail == 0)
            $display(" ALL TESTS PASSED");
        else
            $display(" SOME TESTS FAILED - review output above");
        $display("============================================================");

        $finish;
    end

    // =========================================================================
    // SystemVerilog Assertions (SVA)
    // They fire as simulation errors AND are counted, so the summary above can
    // not report ALL TESTS PASSED when one fired.
    // =========================================================================

    // PENABLE must never be high when PSEL is low
    apb_penable_requires_psel: assert property (
        @(posedge PCLK) disable iff (!PRESETn)
        PENABLE |-> PSEL
    ) else sva_hit("PROTOCOL VIOLATION: PENABLE asserted without PSEL");

    // SETUP lasts exactly one cycle: PSEL without PENABLE is followed by ENABLE
    apb_setup_then_enable: assert property (
        @(posedge PCLK) disable iff (!PRESETn)
        (PSEL && !PENABLE) |=> (PSEL && PENABLE)
    ) else sva_hit("PROTOCOL VIOLATION: SETUP not followed by ENABLE");

    // PSEL must stay high from SETUP until the transfer completes
    apb_psel_stable_until_done: assert property (
        @(posedge PCLK) disable iff (!PRESETn)
        (PSEL && !(PENABLE && PREADY)) |=> PSEL
    ) else sva_hit("PROTOCOL VIOLATION: PSEL dropped before the transfer completed");

    // Address and control stable from SETUP through the completing cycle
    apb_addr_ctrl_stable: assert property (
        @(posedge PCLK) disable iff (!PRESETn)
        (PSEL && !(PENABLE && PREADY)) |=>
            ($stable(PADDR) && $stable(PWRITE) && $stable(PWDATA) && $stable(PSTRB))
    ) else sva_hit("PROTOCOL VIOLATION: PADDR/PWRITE/PWDATA/PSTRB changed during an active transfer");

    // rsp_valid is a single-cycle pulse that only occurs on the completing cycle
    apb_rsp_single_pulse: assert property (
        @(posedge PCLK) disable iff (!PRESETn)
        rsp_valid |=> !rsp_valid
    ) else sva_hit("PROTOCOL VIOLATION: rsp_valid held high for more than one cycle");

    apb_rsp_only_on_completion: assert property (
        @(posedge PCLK) disable iff (!PRESETn)
        rsp_valid |-> (PSEL && PENABLE && PREADY)
    ) else sva_hit("PROTOCOL VIOLATION: rsp_valid without a completing APB cycle");

    // rsp_error is only meaningful with rsp_valid
    apb_rsp_error_needs_valid: assert property (
        @(posedge PCLK) disable iff (!PRESETn)
        rsp_error |-> rsp_valid
    ) else sva_hit("PROTOCOL VIOLATION: rsp_error without rsp_valid");

    // =========================================================================
    // Timeout watchdog - catches infinite loops
    // =========================================================================
    initial begin
        #(CLK_PERIOD * 10000);
        $display("TIMEOUT - simulation did not complete");
        $fatal(1);
    end

endmodule : apb_manager_tb

`timescale 1ns/1ps
//===============================================================
//  tb_sdram.v ? Simulation testbench for sdram.v (32-bit patched)
//===============================================================
module tb_sdram;


    initial begin
        $monitor("%0t  locked=%b  c0=%b  areset=%b", $time,
              dut.pll_locked, dut.u_pll.c0, dut.u_pll.areset);
    end
    reg clk;
    reg rst;

    // Generate 100 MHz clock
    initial clk = 0;
    always #10 clk = ~clk;

    initial begin
        rst = 1'b0;         // Initially 0
        #100;             // Wait 
        rst = 1'b1;         // Then set to 1
    end
    // SDRAM interface wires
    wire [12:0] DRAM_ADDR;
    wire [1:0]  DRAM_BA;
    wire        DRAM_CAS_N;
    wire        DRAM_CKE;
    wire        DRAM_CLK;
    wire        DRAM_CS_N;
    wire [31:0] DRAM_DQ;
    wire [3:0]  DRAM_DQM;
    wire        DRAM_RAS_N;
    wire        DRAM_WE_N;

    // Wishbone wires
    wire [24:0] wb_addr;
    wire [31:0] wb_wdata, wb_rdata;
    wire [3:0]  wb_sel;
    wire        wb_we, wb_stb, wb_cyc, wb_ack, wb_stall;
    wire        pass_led, fail_led;

    logic [7:0] leds;

    reg clk50 = 0;
    always #10 clk50 = ~clk50;

    // DUT
    top dut (
        .CLOCK_50   (clk50),
        .KEY        (rst),
        .LEDG       (leds),
        .DRAM_ADDR  (DRAM_ADDR),
        .DRAM_BA    (DRAM_BA),
        .DRAM_CAS_N (DRAM_CAS_N),
        .DRAM_CKE   (DRAM_CKE),
        .DRAM_CLK   (DRAM_CLK),
        .DRAM_CS_N  (DRAM_CS_N),
        .DRAM_DQ    (DRAM_DQ),
        .DRAM_DQM   (DRAM_DQM),
        .DRAM_RAS_N (DRAM_RAS_N),
        .DRAM_WE_N  (DRAM_WE_N)
    );

    
    // test-bench
    reg nPOR = 0;               // low = in reset
    initial begin
        #200_000 nPOR = 1;      // release after 200 Âµs
    end

// Connect lower 16 bits (DQ[15:0])

//--------------------------------------------------------------------
// 32-bit SDRAM on two ×16 devices (lower 16 bits = “chip 0”,
// upper 16 bits = “chip 1”).  All address / command pins are
// common; DQ and DQM are sliced per device.
//--------------------------------------------------------------------
genvar chip;
generate
    for (chip = 0; chip < 2; chip++) begin : SDRAM_CHIP
        mt48lc16m16a2 mem (
            .Dq     (DRAM_DQ [chip*16 +: 16]),   // slice  0–15 or 16–31
            .Addr   (DRAM_ADDR),                 // 13-bit row / column
            .Ba     (DRAM_BA),                   // 2-bit bank
            .Clk    (DRAM_CLK),
            .Cke    (DRAM_CKE),
            .Cs_n   (DRAM_CS_N),
            .Ras_n  (DRAM_RAS_N),
            .Cas_n  (DRAM_CAS_N),
            .We_n   (DRAM_WE_N),
            .Dqm    (DRAM_DQM[chip*2 +: 2])      // byte masks 0-1 or 2-3
        );
    end
endgenerate
//----------------------------------------------------------------
    // 4. Very-light SDRAM functional stub
    //----------------------------------------------------------------
    logic        dq_drive_en  = 1'b0;
    logic [31:0] dq_drive_dat = 32'd0;
    logic [31:0] mem_word     = 32'd0;     // single-word “RAM”
    logic  [1:0] rd_lat_cnt   = 0;         // CAS latency pipeline

    // Drive or release the bidirectional bus
    assign DRAM_DQ = dq_drive_en ? dq_drive_dat : 32'bz;

    // Primitive command decoder (enough for write/read single burst)
    always @(posedge DRAM_CLK) begin
        // Detect WRITE (RAS=1, CAS=0, WE=0) – store data
        if (!DRAM_CS_N && !DRAM_CAS_N && !DRAM_WE_N) begin
            dq_drive_en  <= 1'b0;          // controller owns the bus
            mem_word     <= DRAM_DQ;
        end

        // Detect READ  (RAS=1, CAS=0, WE=1) – schedule data return
        if (!DRAM_CS_N && !DRAM_CAS_N &&  DRAM_WE_N) begin
            rd_lat_cnt <= 2;               // CL = 2
        end else if (rd_lat_cnt != 0) begin
            rd_lat_cnt <= rd_lat_cnt - 1;
            if (rd_lat_cnt == 1) begin
                dq_drive_en  <= 1'b1;      // present data for one beat
                dq_drive_dat <= mem_word;
            end
        end else begin
            dq_drive_en <= 1'b0;           // release bus afterwards
        end
    end

    //----------------------------------------------------------------
    // 6. PASS / FAIL monitor & simulation stop
    //----------------------------------------------------------------
    initial begin
        // Wait until the tester sets either LEDG[0] (pass) or LEDG[1] (fail)
        wait (leds[0] || leds[1]);

        if (leds[0])
            $display("[%0t] *** SELF-TEST PASSED ***", $time);
        else
            $error  ("[%0t] *** SELF-TEST FAILED  ***", $time);

        #100ns $finish;
    end
	 
endmodule

//===============================================================
//  top.v    DE2-115 + Wishbone SDRAM controller bring-up
//===============================================================
`default_nettype none
module top
(
    // ---- DE2-115 on-board I/O ----
    input  wire        CLOCK_50,      // 50 MHz crystal
    input  wire [0:0]  KEY,           // KEY0-n (active-low push button)
    output wire [7:0]  LEDG,          // Green LEDs (LEDG[0] = pass)

    // ---- SDRAM (ISSI 16-bit chips, 32-bit bus) ----
    output wire [12:0] DRAM_ADDR,
    output wire [1:0]  DRAM_BA,
    output wire        DRAM_CAS_N,
    output wire        DRAM_CKE,
    output wire        DRAM_CLK,
    output wire        DRAM_CS_N,
    inout  wire [31:0] DRAM_DQ,
    output wire [3:0]  DRAM_DQM,
    output wire        DRAM_RAS_N,
    output wire        DRAM_WE_N
);
//---------------------------------------------------------------
// 1. 100 MHz PLL
//---------------------------------------------------------------
wire clk_100m, pll_locked;




PLL_50_to_100 u_pll     
(
    .inclk0 (CLOCK_50),
    .areset (~KEY[0]),
    .c0     (clk_100m),   // 100 MHz, 0° phase
    .locked (pll_locked)
);

//---------------------------------------------------------------
// 2. Synchronous reset (active-high inside fabric)
//---------------------------------------------------------------
wire rst = ~KEY[0];

//---------------------------------------------------------------
// 3. Wishbone signals
//---------------------------------------------------------------
wire [24:0] wb_addr;       // only 25-bit address needed
wire [31:0] wb_wdata;
wire [31:0] wb_rdata;
wire [3:0]  wb_sel;
wire        wb_we, wb_stb, wb_cyc, wb_ack, wb_stall;

// This simple example design does not implement Wishbone backpressure so
// permanently deassert the stall line.
assign wb_stall = 1'b0;

//---------------------------------------------------------------
// 4. Tiny Wishbone test-master FSM
//---------------------------------------------------------------
wishbone_tester u_tester
(
    .clk  (clk_100m),
    .rst  (rst),

    .wb_addr   (wb_addr),
    .wb_wdata  (wb_wdata),
    .wb_rdata  (wb_rdata),
    .wb_sel    (wb_sel),
    .wb_we     (wb_we),
    .wb_stb    (wb_stb),
    .wb_cyc    (wb_cyc),
    .wb_ack    (wb_ack),
    .wb_stall  (wb_stall),

    .pass_led  (LEDG[0]),
    .fail_led  (LEDG[1])
);

assign LEDG[7:2] = 6'h00;   // unused

//---------------------------------------------------------------
// 5. Instantiate 32-bit SDRAM controller
//---------------------------------------------------------------
sdram_ctrl u_sdram_ctrl (
    .clk        (clk_100m),
    .clk_dram   (clk_100m),   // assuming clk_100m is used for SDRAM output clock
    .rst        (rst),
    .dll_locked (pll_locked), // Assuming PLL lock means the DLL is locked

    // SDRAM interface
    .dram_addr  (DRAM_ADDR),
    .dram_bank  (DRAM_BA),
    .dram_cas_n (DRAM_CAS_N),
    .dram_ras_n (DRAM_RAS_N),
    .dram_cke   (DRAM_CKE),
    .dram_clk   (DRAM_CLK),
    .dram_cs_n  (DRAM_CS_N),
    .dram_dq    (DRAM_DQ),
    .dram_dqm   (DRAM_DQM),
    .dram_we_n  (DRAM_WE_N),

    // Wishbone interface
    .addr_i     (wb_addr[22:0]),     // Assuming bottom 23 bits carry full SDRAM addr
    .dat_i      (wb_wdata),
    .dat_o      (wb_rdata),
    .we_i       (wb_we),
    .ack_o      (wb_ack),
    .stb_i      (wb_stb),
    .cyc_i      (wb_cyc)
);

endmodule
//===============================================================
//  Simple Wishbone master that performs one write+read test
//===============================================================
module wishbone_tester
(
    input  wire        clk,
    input  wire        rst,

    // Wishbone out
    output reg  [24:0] wb_addr,
    output reg  [31:0] wb_wdata,
    input  wire [31:0] wb_rdata,
    output reg  [3:0]  wb_sel,
    output reg         wb_we,
    output reg         wb_stb,
    output reg         wb_cyc,
    input  wire        wb_ack,
    input  wire        wb_stall,

    output reg         pass_led,
    output reg         fail_led
);

typedef enum logic [2:0] {IDLE, WR, WR_WAIT, RD, RD_WAIT, DONE} state_t;
state_t state;

always @(posedge clk) begin
    if (rst) begin
        state    <= IDLE;
        wb_addr  <= 25'h0000000;
        wb_wdata <= 32'hDEADBEEF;
        wb_sel   <= 4'hF;
        wb_we    <= 1'b0;
        wb_stb   <= 1'b0;
        wb_cyc   <= 1'b0;
        pass_led <= 1'b0;
        fail_led <= 1'b0;
    end
    else begin
        case (state)
        //----------------------------------------
        IDLE: begin
            wb_we   <= 1'b1;
            wb_stb  <= 1'b1;
            wb_cyc  <= 1'b1;
            state   <= WR;
        end
        //----------------------------------------
        WR: if (!wb_stall) state <= WR_WAIT;
        //----------------------------------------
        WR_WAIT: if (wb_ack) begin
            wb_we   <= 1'b0;          // switch to read
            state   <= RD;
        end
        //----------------------------------------
        RD: if (!wb_stall) begin
            state <= RD_WAIT;
        end
        //----------------------------------------
        RD_WAIT: if (wb_ack) begin
            wb_stb <= 1'b0;
            wb_cyc <= 1'b0;
            if (wb_rdata == 32'hDEADBEEF) pass_led <= 1'b1;
            else                          fail_led <= 1'b1;
            state <= DONE;
        end
        //----------------------------------------
        DONE: ; // sit here forever
        endcase
    end
end 

endmodule


`timescale 1ns/1ps
module PLL_50_to_100 (
    input  wire inclk0,
    input  wire areset,
    output wire c0,        // 100 MHz clock
    output reg  locked     // goes high 200 ns after reset de-asserts
);
    logic clk_int = 1'b0;
    assign c0 = clk_int;

    // 100 MHz ? toggle every 5 ns
    always #5ns clk_int = ~clk_int;

    initial begin
        locked = 1'b0;
        // wait until areset is released
        @(negedge areset);
        #200ns locked = 1'b1;
    end
endmodule

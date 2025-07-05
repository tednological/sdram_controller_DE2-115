module sdram_ctrl (
    input logic clk,// System clock 
    input logic clk_dram, // Dram clk
    input logic rst,    // Active-high reset
    input logic dll_locked,   // Indicates that the external DLL/PLL has locked, releasing dram_cs_n
    // SDRAM interface signals
    output logic [12:0] dram_addr, // Row / Column / mode-register address brought to the sdram
    output logic [1:0] dram_bank,  // bank address (BA1, BA0)
    output logic dram_cas_n, // column-address-strobe, active-low
    output logic dram_ras_n, // row-address-strobe, active-low
    output logic dram_cke, // clock enable
    output logic dram_clk, // forward copy of clk_dram
    output logic dram_cs_n, // chip select, held low whn dll_locked = 1
    inout tri [31:0] dram_dq, // bidirectional data bus
    output logic [3:0] dram_dqm,  // Data mask for byte lanes
    output logic dram_we_n, // write enable
    // wishbone interface signals
    input logic [22:0] addr_i, // Wishbone address {BA1 / BA0 / Row 11 bits / col 10 bits}
    input logic [31:0] dat_i, // wishbone write data
    output logic [31:0] dat_o, // wishbone read data
    input logic we_i, // write enable (1 = write 0 = read)
    output logic ack_o, // ack, goes high when transfer finishes
    input logic stb_i, // strobe, makes sure the request is valid this cycle
    input logic cyc_i // marks an ongoing bus transition
);
// burst-length = 2, sequential, CAS = 2, write-burst = program
localparam Mode = 13'b0000000100000; // A hard-coded pattern that the ctrl writes into the SDRAM's Mode Register

// SDRAM FSM Type definitions
typedef enum logic [5:0]{
    S_INIT,
    S_WAIT200,
    S_INIT_PRE,
    S_WAIT_INIT_PRE,
    S_INIT_REF,
    S_WAIT_INIT_REF,
    S_MODE_REG,
    S_WAIT_MODE_REG,
    S_DONE,
    S_IDLE,
    S_REFRESH,
    S_WAIT_REFRESH,
    S_ACT,
    S_WAIT_ACT,
    S_W0,
    S_W1,
    S_WPRE,
    S_R0,
    S_R1,
    S_R2,
    S_R3,
    S_RPRE,
    S_PRE,
    S_WAIT_PRE,
    S_ERROR
} state_t;

state_t        current_state, next_state;

localparam TRC_CNTR_C = 4'b0111; // reload value for tRC timer (ACT -> ACT, also activates when REF->REF)
localparam RFSH_CNTR_C = 16'b0000001111101000; // refresh every 1000 cycles =~ 8192 refreshes/64ms
localparam TRCD_CNTR_C = 3'b001; // tRCD timer (ACT->READ and ACT->WRITE)
localparam TRP_CNTR_C = 4'b0001; // tRP timer (PRECHARGE -> ACT)
localparam WAIT_200_CNTR_C = 16'b0110100101111000; // 200 us power-up wait-timer

logic [22:0] addr_r; // Latched copy of the incoming wishbone addy

logic [12:0] dram_addr_r = 13'd0; // registered version of dram_addr (value changes only on clock edge)
logic [1:0] dram_bank_r = 2'd0; // Registered bank address
logic [31:0] dram_dq_r = 32'd0; // outgoing data to drive dram_dq during write
logic dram_cas_n_r = 1'b0; // registered SDRAM command strobe (Column)
logic dram_ras_n_r = 1'b0; // registered SDRAM command strobe (row)
logic dram_we_n_r  = 1'b0; // registered SDRAM command strobe (write enable)

logic [31:0] dat_o_r; // captured read data to be forwarded to wishbone
logic ack_o_r; // registered ack_o
logic [31:0] dat_i_r; // Registered write data
logic we_i_r; // wishbone ctrl bits
logic stb_i_r; // wishbone ctrl bits
logic oe_r; // Output enable

logic [3:0] init_pre_cntr; // Counts up all 8 PRECHARGE ALL cycles required during init
logic [3:0] trc_cntr; // runtime down-counter enforcing tRC
logic [3:0] trp_cntr; // runtime down-counter enforcing tRP
logic [15:0] rfsh_int_cntr; // counter that periodically generates an auto-refresh request
logic [2:0] trcd_cntr; // runtime down-counter enforcing tRCD
logic [15:0] wait200_cntr; // counts down 200 us
logic do_refresh; // Tells FSM to enter refresh state

// feed inputs to their registered counterparts
always_ff @(posedge clk) begin
    if (rst) begin
        if(stb_i_r && current_state == S_ACT)
            stb_i_r <= 1'b0;
        else if (stb_i && cyc_i) begin
            addr_r <= addr_i;
            dat_i_r <= dat_i;
            we_i_r <= we_i;
            stb_i_r <= stb_i;
        end
    end
end

// Counter for waiting 200us
always_ff @(posedge clk) begin
    if (rst) 
        wait200_cntr <= 16'd0;
    else begin
        if(current_state == S_INIT)
            wait200_cntr <= WAIT_200_CNTR_C;
        else
            wait200_cntr <= wait200_cntr - 1;
    end
end

// interval between refreshes
always_ff @(posedge clk) begin
    if (rst) 
        rfsh_int_cntr <= 16'd0;
    else begin
        if (current_state == S_WAIT_REFRESH) begin
            do_refresh <= 1'b0;
			rfsh_int_cntr <= RFSH_CNTR_C;
        end else if (!rfsh_int_cntr) begin
            do_refresh <= 1'b1;
        end else begin
            rfsh_int_cntr <= rfsh_int_cntr - 1;
        end
    end
end

// tRP counter
always_ff @(posedge clk) begin
    if (rst) 
        trp_cntr <= 4'd0;
    else begin
        if (current_state == S_PRE || current_state == S_INIT_PRE) begin
            trp_cntr <= TRP_CNTR_C;
        end else begin
            trp_cntr <= trp_cntr - 1;
        end
    end
end

// tRC counter
always_ff @(posedge clk) begin
    if (rst) 
        trc_cntr <= 3'd0;
    else begin
        if ((current_state == S_ACT)|| (current_state == S_MODE_REG)) begin
            trc_cntr <= TRC_CNTR_C;
        end else begin
            trc_cntr <= trc_cntr - 1;
        end
    end
end

// tRCD counter
always_ff @(posedge clk) begin
    if (rst) 
        trcd_cntr <= 3'd0;
    else begin
        if ((current_state == S_ACT) || (current_state == S_MODE_REG)) begin
            trcd_cntr <= TRCD_CNTR_C;
        end else begin
            trcd_cntr <= trcd_cntr - 1;
        end
    end
end

// Timer for initialization
always_ff @(posedge clk) begin
    if (rst) 
        init_pre_cntr <= 4'd0;
    else begin
        if ((current_state == S_IDLE) || (current_state == S_INIT)) begin
            init_pre_cntr <= 4'd0;
        end else if ((current_state == S_INIT_REF) || (current_state == S_REFRESH)) begin
            init_pre_cntr <= init_pre_cntr + 1;
        end
    end
end

// Next state logic
always_ff @(posedge clk) begin
    if (rst) 
        current_state <= S_INIT; // if reset, return to initialzation
    else begin
        current_state <= next_state; // else, go to next state
    end
end

always_comb begin
    next_state = current_state; // default stay
    unique case (current_state)
        S_INIT            : next_state = S_WAIT200;

        S_WAIT200         : next_state = state_t'((wait200_cntr == 16'd0) ? S_INIT_PRE : S_WAIT200);

        S_INIT_PRE        : next_state = S_WAIT_INIT_PRE;

        S_WAIT_INIT_PRE   : next_state = state_t'((trp_cntr == 4'd0) ? S_INIT_REF : S_WAIT_INIT_PRE);

        S_INIT_REF        : next_state = S_WAIT_INIT_REF;

        S_WAIT_INIT_REF   : begin
                                if (trc_cntr == 4'd0) begin
                                    next_state = state_t'((init_pre_cntr == 4'd8) ? S_MODE_REG : S_INIT_REF);
                                end else begin
                                    next_state = S_WAIT_INIT_REF;
                                end
                            end

        S_MODE_REG        : next_state = S_WAIT_MODE_REG;

        S_WAIT_MODE_REG   : next_state = state_t'((trcd_cntr == 3'd0) ? S_DONE : S_WAIT_MODE_REG);

        S_DONE            : next_state = S_IDLE;

        S_IDLE            : begin
                                if (do_refresh)       next_state = S_REFRESH;
                                else if (stb_i_r)     next_state = S_ACT;
                                else                  next_state = S_IDLE;
                            end

        S_REFRESH         : next_state = S_WAIT_REFRESH;

        S_WAIT_REFRESH    : next_state = state_t'((trc_cntr == 4'd0) ? S_IDLE : S_WAIT_REFRESH);

        S_ACT             : next_state = S_WAIT_ACT;

        S_WAIT_ACT        : begin
                                if (trcd_cntr == 3'd0) begin
                                    next_state = state_t'(we_i_r ? S_W0 : S_R0);
                                end
                            end

        // ----------------- WRITE path ------------------
        S_W0              : next_state = S_WPRE;
        S_W1              : next_state = S_WPRE;
        S_WPRE            : next_state = S_PRE;

        // ----------------- READ path -------------------
        S_R0              : next_state = S_R1;
        S_R1              : next_state = S_R2;
        S_R2              : next_state = S_R3;
        S_R3              : next_state = S_RPRE;
        S_RPRE            : next_state = S_PRE;

        // ------------- common precharge ----------------
        S_PRE             : next_state = S_WAIT_PRE;
        S_WAIT_PRE        : next_state = state_t'((trp_cntr == 4'd0) ? S_IDLE : S_WAIT_PRE);

        default           : next_state = S_ERROR;
    endcase
end

// Ack_o signal
always_ff @(posedge clk) begin
    if (rst) 
        ack_o_r <= 1'b0;
    else begin
        if (current_state == S_WAIT_PRE) begin
            ack_o_r <= 1'b0;
        end else if ((current_state == S_RPRE)|| (current_state == S_WPRE)) begin
            ack_o_r <= 1'b1;
        end
    end
end

// Data processin, if current_state == W0 then we  drive 
always_ff @(posedge clk) begin
    if (rst) begin
        dat_o_r <= 32'd0;
        dram_dq_r <= 32'd0;
        oe_r <= 1'b0;
    end else begin
        if (current_state == S_W0) begin
            dram_dq_r <= dat_i_r; // Drive write data to SDRAM
            oe_r <= 1'b1; // enable dq bus output
        end else if (current_state == S_R2) begin
            dat_o_r <= dram_dq; // Latch read data from SDRAM
            dram_dq_r <= 32'd0; // clear output driver
            oe_r <= 1'b0; // Disable DQ output (we are reading)
        end else begin
            dram_dq_r <= 32'd0;
            oe_r <= 1'b0;
        end
    end
end

// Address processing 
always_ff @(posedge clk) begin
    if (current_state == S_MODE_REG) begin
        dram_addr_r <= Mode;
    end else if (current_state == S_INIT_PRE || current_state == S_PRE) begin
        dram_addr_r <= 13'b0010000000000; // initization addy|A10=1 (precharge all banks)
    end else if (current_state == S_ACT) begin
        dram_addr_r <= {2'b00, addr_r[20:10]}; // Row address
        dram_bank_r  <= addr_r[22:21];
    end else if (current_state == S_W0 || current_state == S_R0) begin
        dram_addr_r <= {3'b000, addr_r[9:0]}; // Column address
        dram_bank_r <= addr_r[22:21];
    end else begin
        dram_addr_r <= 13'd0;
        dram_bank_r <= 2'b00;
    end
end

// Commands
always_ff @(posedge clk) begin 
    if(current_state == S_INIT) begin
        dram_dqm <= 4'b1111; // disable writing to any byte during init
    end else if (current_state == S_DONE) begin
        dram_dqm <= 4'b0000; // clear data mask after init
    end 
    // row address strobe
    if(current_state == S_INIT_PRE || current_state == S_INIT_REF || current_state == S_PRE ||
	   current_state == S_MODE_REG || current_state == S_REFRESH || current_state == S_ACT) begin
        dram_ras_n_r <= 1'b0;
    end else begin
        dram_ras_n_r <= 1'b1;
    end  

    // Column address strobe
    if(current_state == S_R0 || current_state == S_W0 || current_state == S_INIT_REF || current_state == S_MODE_REG) begin
        dram_cas_n_r <= 1'b0;
    end else begin
        dram_cas_n_r <= 1'b1;
    end  

    // write enable
    if(current_state == S_INIT_PRE || current_state == S_PRE || current_state == S_W0 || current_state == S_MODE_REG) begin
        dram_we_n_r <= 1'b0;
    end else begin
        dram_we_n_r <= 1'b1;
    end
end

assign dram_addr   = dram_addr_r;
assign dram_bank   = dram_bank_r;
assign dram_cas_n  = dram_cas_n_r;
assign dram_ras_n  = dram_ras_n_r;
assign dram_we_n   = dram_we_n_r;
// Tri-state DQ bus
assign dram_dq     = oe_r ? dram_dq_r : 32'bz;

assign dat_o       = dat_o_r;
assign ack_o       = ack_o_r;

assign dram_cke    = 1'b1;
assign dram_cs_n   = ~dll_locked;
assign dram_clk    = clk_dram;

endmodule

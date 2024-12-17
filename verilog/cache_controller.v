module cache_controller(
   // Outputs
   toggle_victim_way, DataOut, Done, Stall, cache_rd, cache_wr, cache_comp, cache_addr, cache_data_in, force_enable, mem_addr, mem_data_in, mem_write, mem_read, controller_err,
   // Inputs
   Addr, DataIn, Rd, Wr, cache_en, real_hit, victimize, actual_tag, cache_data_out, mem_data_out, mem_stall, mem_busy, clk, rst,
   );


	// Module Interface
	input  [15:0]	Addr;
	input  [15:0]	DataIn;
	input			Rd;
	input			Wr;
	input			clk;
	input			rst;

	output reg [15:0]	DataOut;
	output reg 			Done;
	output reg			Stall;

	// Internal Control/Data
	input  [4:0]	cache_en;
	input			real_hit;
	input			victimize;	
	input  [4:0]	actual_tag;
	input  [15:0]	cache_data_out;
	input  [15:0]	mem_data_out;
	input  			mem_stall;
	input  [3:0]	mem_busy;


	output reg			cache_rd;
	output reg			cache_wr;
	output reg		  	cache_comp;
	output reg [15:0] 	cache_addr;
	output reg [15:0]	cache_data_in;
	output reg [1:0]	force_enable;
	output reg [15:0]	mem_addr;
	output reg [15:0]	mem_data_in;
	output reg			mem_write;
	output reg			mem_read;
	output reg			controller_err;

	output reg			toggle_victim_way;

	reg  [1:0] 			force_enable_hold; // Hold which cache to evict for use in later cycles
	wire [1:0]			force_enable_int;  // Decide which cache to evict

	reg					load_after_write;

	wire				dirty;
	assign dirty = cache_en[3] | cache_en[4];

	assign force_enable_int[0] = (cache_en[0] & cache_en[1] & ~cache_en[2] & ~dirty) | (~cache_en[0] & ~dirty) | cache_en[3];
	assign force_enable_int[1] = (cache_en[0] & cache_en[1] &  cache_en[2] & ~dirty) | (cache_en[0] & ~cache_en[1] & ~dirty) | (~cache_en[3] & cache_en[4]);


	localparam	Ready 			= 20'b00000000000000000000,
				rd_cache		= 20'b00000000000000000001,
				rd_evict 		= 20'b00000000000000000010,
				rd_evict_wait 	= 20'b00000000000000000100,
				ld_mem 			= 20'b00000000000000001000,
				wait_ld 		= 20'b00000000000000010000,
				return_data 	= 20'b00000000000000100000,
				wr_cache 		= 20'b00000000000001000000,
				wr_evict 		= 20'b00000000000010000000,
				wr_evict_wait 	= 20'b00000000000100000000,
				st_mem 			= 20'b00000000001000000000,
				wait_st 		= 20'b00000000010000000000,

				wr_evict_wait1 	= 20'b00000000100000000000, //11
				wr_evict_wait2 	= 20'b00000001000000000000, //12

				rd_evict_wait1 	= 20'b00000010000000000000, //13
				rd_evict_wait2 	= 20'b00000100000000000000, //14

				wait_ld1 		= 20'b00001000000000000000, //15
				wait_ld2 		= 20'b00010000000000000000, //16
				wait_ld3		= 20'b00100000000000000000, //17
				wait_ld4		= 20'b01000000000000000000, //18
				wait_extra		= 20'b10000000000000000000; //19

	wire [19:0]	state;
	wire [19:0] next_state;

	dff state_reg[19:0](.d(next_state), .q(state), .clk(clk), .rst(rst));


	// Read States
	assign next_state[0] = ~|state & Rd;
	assign next_state[1] = state[0] & ~real_hit & victimize;
	assign next_state[2] = state[1];
	assign next_state[3] = (state[19] & ~|mem_busy) | (state[0] & ~real_hit & ~victimize) | (state[10] & ~|mem_busy);
	assign next_state[4] = state[3];
	assign next_state[5] = state[0] & real_hit;

	// Write Statse
	assign next_state[6] = ~|state & Wr;
	assign next_state[7] = state[6] & ~real_hit & victimize;
	assign next_state[8] = state[7];
	assign next_state[9] = (state[12] & ~|mem_busy) | (state[6] & ~real_hit & ~victimize);
	assign next_state[10] = state[9] | (state[10] & |mem_busy);
	
	// wr_evict_wait
	assign next_state[11] = state[8];
	assign next_state[12] = state[11] | (state[12] & |mem_busy);

	// rd_evict_wait
	assign next_state[13] = state[2];
	assign next_state[14] = state[13];
	assign next_state[19] = state[14] | (state[19] & |mem_busy);

	// wait_ld
	assign next_state[15] = state[4];
	assign next_state[16] = state[15];
	assign next_state[17] = state[16];
	assign next_state[18] = state[17] | (state[18] & |mem_busy);




	

//Outputs
	always@(*)
	begin
		case(state)
			Ready:
				begin
				DataOut			  = 16'b0;
				Done			  = 1'b0;
				Stall			  = Rd | Wr;
				cache_rd		  = 1'b0;
				cache_wr		  = 1'b0;
				cache_comp		  = 1'b0;
				cache_addr		  = 16'b0;
				cache_data_in	  = 16'b0;
				force_enable	  = 2'b0;
				force_enable_hold = 2'b0; //
				mem_addr		  = 16'b0;
				mem_data_in		  = 16'b0;
				mem_write		  = 1'b0;
				mem_read		  = 1'b0;
				controller_err    = 1'b0;
				toggle_victim_way = 1'b1;
				load_after_write  = 1'b0;
				end

			rd_cache:
				begin
				DataOut			  = 16'b0;
				Done			  = 1'b0;
				Stall			  = 1'b1; //
				cache_rd		  = 1'b1; //
				cache_wr		  = 1'b0;
				cache_comp		  = 1'b1; //
				cache_addr		  = Addr; //
				cache_data_in	  = 16'b0;
				force_enable	  = 2'b0;
				force_enable_hold = force_enable_int; //
				mem_addr		  = 16'b0;
				mem_data_in		  = 16'b0;
				mem_write		  = 1'b0;
				mem_read		  = 1'b0;
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			rd_evict:
				begin
				DataOut			  = 16'b0;
				Done			  = 1'b0;
				Stall			  = 1'b1; //
				cache_rd		  = 1'b1;
				cache_wr		  = 1'b0;
				cache_comp		  = 1'b0;
				cache_addr		  = {Addr[15:3], 3'b000}; //
				cache_data_in	  = 16'b0;
				force_enable	  = force_enable_hold; //
				mem_addr		  = {actual_tag, Addr[10:3], 3'b000}; //
				mem_data_in		  = cache_data_out;
				mem_write		  = 1'b1;
				mem_read		  = 1'b0;
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			rd_evict_wait:
				begin
				DataOut			  = 16'b0;
				Done			  = 1'b0;
				Stall			  = 1'b1; //
				cache_rd		  = 1'b1;
				cache_wr		  = 1'b0;
				cache_comp		  = 1'b0;
				cache_addr		  = {Addr[15:3], 3'b010}; //
				cache_data_in	  = 16'b0;
				force_enable	  = force_enable_hold; //
				mem_addr		  = {actual_tag, Addr[10:3], 3'b010}; //
				mem_data_in		  = cache_data_out;
				mem_write		  = 1'b1;
				mem_read		  = 1'b0;
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			ld_mem:
				begin
				DataOut			  = 16'b0;
				Done			  = 1'b0;
				Stall			  = 1'b1;  
				cache_rd		  = 1'b0;
				cache_wr		  = 1'b0;
				cache_comp		  = 1'b0;
				cache_addr		  = 16'b0; 
				cache_data_in	  = 16'b0;
				force_enable	  = 2'b0;
				mem_addr		  = {Addr[15:3], 3'b000}; //
				mem_data_in		  = 16'b0;
				mem_write		  = 1'b0;
				mem_read		  = 1'b1; //
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			wait_ld:
				begin
				DataOut			  = 16'b0; 
				Done			  = 1'b0; 
				Stall			  = 1'b1; //
				cache_rd		  = 1'b0;
				cache_wr		  = 1'b0;
				cache_comp		  = 1'b0;
				cache_addr		  = 16'b0; 
				cache_data_in	  = 16'b0;
				force_enable	  = 2'b0;
				mem_addr		  = {Addr[15:3], 3'b010}; //
				mem_data_in		  = 16'b0;
				mem_write		  = 1'b0;
				mem_read		  = 1'b1; //
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			return_data:
				begin
				DataOut			  = cache_data_out; //
				Done			  = 1'b1; //
				Stall			  = 1'b0; //
				cache_rd		  = 1'b1; //
				cache_wr		  = 1'b0; //
				cache_comp		  = 1'b1; //
				cache_addr		  = Addr; //
				cache_data_in	  = 16'b0; //
				force_enable	  = 2'b0;
				mem_addr		  = 16'b0;
				mem_data_in		  = 16'b0;
				mem_write		  = 1'b0;
				mem_read		  = 1'b0;
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			wr_cache:
				begin
				DataOut			  = 16'b0;
				Done			  = real_hit; //
				Stall			  = ~real_hit; //
				cache_rd		  = 1'b0;
				cache_wr		  = 1'b1; //
				cache_comp		  = 1'b1; //
				cache_addr		  = Addr; //
				cache_data_in	  = DataIn; //
				force_enable	  = 2'b0;
				force_enable_hold = force_enable_int; //
				mem_addr		  = 16'b0;
				mem_data_in		  = 16'b0;
				mem_write		  = 1'b0;
				mem_read		  = 1'b0;
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			wr_evict:
				begin
				DataOut			  = 16'b0;
				Done			  = 1'b0;
				Stall			  = 1'b1; //
				cache_rd		  = 1'b1;
				cache_wr		  = 1'b0;
				cache_comp		  = 1'b0;
				cache_addr		  = {Addr[15:3], 3'b000}; //
				cache_data_in	  = 16'b0;
				force_enable	  = force_enable_hold; //
				mem_addr		  = {actual_tag, Addr[10:3], 3'b000}; //
				mem_data_in		  = cache_data_out;
				mem_write		  = 1'b1;
				mem_read		  = 1'b0;
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			wr_evict_wait:
				begin
				DataOut			  = 16'b0;
				Done			  = 1'b0;
				Stall			  = 1'b1; //
				cache_rd		  = 1'b1;
				cache_wr		  = 1'b0;
				cache_comp		  = 1'b0;
				cache_addr		  = {Addr[15:3], 3'b010}; //
				cache_data_in	  = 16'b0;
				force_enable	  = force_enable_hold; //
				mem_addr		  = {actual_tag, Addr[10:3], 3'b010}; //
				mem_data_in		  = cache_data_out;
				mem_write		  = 1'b1;
				mem_read		  = 1'b0;
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			st_mem:
				begin
				DataOut			  = 16'b0;
				Done			  = 1'b0;
				Stall			  = 1'b1; //
				cache_rd		  = 1'b0;
				cache_wr		  = 1'b0;
				cache_comp		  = 1'b0;
				cache_addr		  = Addr; //
				cache_data_in	  = 16'b0;
				force_enable	  = 2'b0;
				mem_addr		  = Addr; //
				mem_data_in		  = DataIn; //
				mem_write		  = 1'b1;
				mem_read		  = 1'b0;
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				load_after_write  = 1'b1;
				end

			wait_st:
				begin
				DataOut			  = 16'b0;
				Done			  = ~|mem_busy; //
				Stall			  = 1'b1; //
				cache_rd		  = 1'b0;
				cache_wr		  = 1'b1;
				cache_comp		  = 1'b0;
				cache_addr		  = Addr; //
				cache_data_in	  = DataIn; //
				force_enable	  = force_enable_hold; //
				mem_addr		  = 16'b0;
				mem_data_in		  = 16'b0;
				mem_write		  = 1'b0;
				mem_read		  = 1'b0;
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

		// STACKED CYCLES

			wr_evict_wait1:
				begin
				DataOut			  = 16'b0;
				Done			  = 1'b0;
				Stall			  = 1'b1; //
				cache_rd		  = 1'b1;
				cache_wr		  = 1'b0;
				cache_comp		  = 1'b0;
				cache_addr		  = {Addr[15:3], 3'b100}; //
				cache_data_in	  = 16'b0;
				force_enable	  = force_enable_hold; //
				mem_addr		  = {actual_tag, Addr[10:3], 3'b100}; //
				mem_data_in		  = cache_data_out;
				mem_write		  = 1'b1;
				mem_read		  = 1'b0;
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			wr_evict_wait2:
				begin
				DataOut			  = 16'b0;
				Done			  = 1'b0;
				Stall			  = 1'b1; //
				cache_rd		  = 1'b1;
				cache_wr		  = 1'b0;
				cache_comp		  = 1'b0;
				cache_addr		  = {Addr[15:3], 3'b110}; //
				cache_data_in	  = 16'b0;
				force_enable	  = force_enable_hold; //
				mem_addr		  = {actual_tag, Addr[10:3], 3'b110}; //
				mem_data_in		  = cache_data_out;
				mem_write		  = 1'b1;
				mem_read		  = 1'b0;
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			rd_evict_wait1:
				begin
				DataOut			  = 16'b0;
				Done			  = 1'b0;
				Stall			  = 1'b1; //
				cache_rd		  = 1'b1;
				cache_wr		  = 1'b0;
				cache_comp		  = 1'b0;
				cache_addr		  = {Addr[15:3], 3'b100}; //
				cache_data_in	  = 16'b0;
				force_enable	  = force_enable_hold; //
				mem_addr		  = {actual_tag, Addr[10:3], 3'b100}; //
				mem_data_in		  = cache_data_out;
				mem_write		  = 1'b1;
				mem_read		  = 1'b0;
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			rd_evict_wait2:
				begin
				DataOut			  = 16'b0;
				Done			  = 1'b0;
				Stall			  = 1'b1; //
				cache_rd		  = 1'b1;
				cache_wr		  = 1'b0;
				cache_comp		  = 1'b0;
				cache_addr		  = {Addr[15:3], 3'b110}; //
				cache_data_in	  = 16'b0;
				force_enable	  = force_enable_hold; //
				mem_addr		  = {actual_tag, Addr[10:3], 3'b110}; //
				mem_data_in		  = cache_data_out;
				mem_write		  = 1'b1;
				mem_read		  = 1'b0;
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			wait_ld1:
				begin
				DataOut			  = mem_data_out; //
				Done			  = (Addr[2:0] == 3'b000) & |mem_data_out & ~load_after_write; //
				Stall			  = 1'b1; //
				cache_rd		  = 1'b0;
				cache_wr		  = 1'b1 & |mem_data_out; //
				cache_comp		  = 1'b0;
				cache_addr		  = {Addr[15:3], 3'b000}; 
				cache_data_in	  = mem_data_out; //
				force_enable	  = force_enable_hold; //
				mem_addr		  = {Addr[15:3], 3'b100}; //
				mem_data_in		  = 16'b0;
				mem_write		  = 1'b0;
				mem_read		  = 1'b1; //
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			wait_ld2:
				begin
				DataOut			  = mem_data_out; //
				Done			  = (Addr[2:0] == 3'b010) & |mem_data_out & ~load_after_write; //
				Stall			  = 1'b1; //
				cache_rd		  = 1'b0;
				cache_wr		  = 1'b1 & |mem_data_out; //
				cache_comp		  = 1'b0;
				cache_addr		  = {Addr[15:3], 3'b010}; 
				cache_data_in	  = mem_data_out; //
				force_enable	  = force_enable_hold; //
				mem_addr		  = {Addr[15:3], 3'b110}; //
				mem_data_in		  = 16'b0;
				mem_write		  = 1'b0;
				mem_read		  = 1'b1; //
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			wait_ld3:
				begin
				DataOut			  = mem_data_out; //
				Done			  = (Addr[2:0] == 3'b100) & |mem_data_out & ~load_after_write; //
				Stall			  = 1'b1; //
				cache_rd		  = 1'b0;
				cache_wr		  = 1'b1 & |mem_data_out; //
				cache_comp		  = 1'b0;
				cache_addr		  = {Addr[15:3], 3'b100}; 
				cache_data_in	  = mem_data_out; //
				force_enable	  = force_enable_hold; //
				mem_addr		  = 16'b0;
				mem_data_in		  = 16'b0;
				mem_write		  = 1'b0;
				mem_read		  = 1'b0; 
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			wait_ld4:
				begin
				DataOut			  = mem_data_out; //
				Done			  = (Addr[2:0] == 3'b110) & |mem_data_out & ~load_after_write; //
				Stall			  = |mem_busy; //
				cache_rd		  = 1'b0;
				cache_wr		  = 1'b1  & |mem_data_out; //
				cache_comp		  = 1'b0;
				cache_addr		  = {Addr[15:3], 3'b110}; 
				cache_data_in	  = mem_data_out; //
				force_enable	  = force_enable_hold; //
				mem_addr		  = 16'b0;
				mem_data_in		  = 16'b0;
				mem_write		  = 1'b0;
				mem_read		  = 1'b0; 
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end

			wait_extra:
				begin
				DataOut			  = 16'b0;
				Done			  = 1'b0;
				Stall			  = 1'b1;
				cache_rd		  = 1'b0;
				cache_wr		  = 1'b0;
				cache_comp		  = 1'b0;
				cache_addr		  = 16'b0;
				cache_data_in	  = 16'b0;
				force_enable	  = 2'b0;
				mem_addr		  = 16'b0;
				mem_data_in		  = 16'b0;
				mem_write		  = 1'b0;
				mem_read		  = 1'b0;
				controller_err    = 1'b0;
				toggle_victim_way = 1'b0;
				end


		endcase
	end


endmodule
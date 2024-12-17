/* $Author: karu $ */
/* $LastChangedDate: 2009-04-24 09:28:13 -0500 (Fri, 24 Apr 2009) $ */
/* $Rev: 77 $ */

`default_nettype none
module mem_system(/*AUTOARG*/
   // Outputs
   DataOut, Done, Stall, CacheHit, err,
   // Inputs
   Addr, DataIn, Rd, Wr, createdump, clk, rst
   );
   
   input wire [15:0] Addr;
   input wire [15:0] DataIn;
   input wire        Rd;
   input wire        Wr;
   input wire        createdump;
   input wire        clk;
   input wire        rst;
   
   output reg [15:0] DataOut;
   output reg        Done;
   output reg        Stall;
   output reg        CacheHit;
   output reg        err;


	// Memory
	
	wire [15:0] mem_data_in;
	wire [15:0]	mem_addr;
	wire		mem_write;
	wire		mem_read;
	wire [15:0]	mem_data_out;
	wire		mem_stall;
	wire [3:0]  mem_busy;  
	wire		mem_err; 


   	// Basic Cache
	
	wire 		cache_comp;
	wire [15:0]	cache_addr;
	wire [15:0] cache_data_in;

	// Victim Logic

	wire 		toggle_victim_way;
	wire		victim_in;
	wire		victim_out;


	mux2_1	mux0(.a(victim_out), .b(~victim_out), .s(toggle_victim_way), .out(victim_in));
	dff 	victimway(.d(victim_in), .q(victim_out), .clk(clk), .rst(rst));


	// Cache Out Logic

	wire		cache0_hit;
	wire		cache1_hit;

	wire		cache0_valid;
	wire		cache1_valid;

	wire		cache0_dirty;
	wire		cache1_dirty;

	wire [4:0]	cache0_tag;
	wire [4:0]	cache1_tag;

	wire [15:0]	cache0_data;
	wire [15:0]	cache1_data;
	
	wire		cache_select;
	wire		real_hit;
	wire 		victimize;
	wire [15:0] cache_data_out;
	wire [4:0] 	actual_tag;
	wire [1:0]	force_enable;


	assign real_hit = (cache0_hit & cache0_valid) | (cache1_hit & cache1_valid);
	assign victimize = (~cache0_hit & cache0_dirty) | (~cache1_hit & cache1_dirty);
	assign cache_select = |force_enable ? force_enable[1] : ((cache1_tag == cache_addr[15:11]) ) & (~(cache0_tag == cache_addr[15:11])); // These should always complement

	mux2_1 datamux[15:0](.a(cache0_data), .b(cache1_data), .s(cache_select), .out(cache_data_out));
	mux2_1 tagmux [4:0] (.a(cache0_tag) , .b(cache1_tag),  .s(cache_select), .out(actual_tag));
	
	//Enable Logic
	
	wire		cache_rd;
	wire		cache_wr;


	wire 		cache_en;
	wire [4:0]	cache_en_controller;
	wire		cache0_enable;
	wire		cache1_enable;


	assign cache_en = (cache_rd | cache_wr);
	assign cache0_enable = force_enable[0] | (cache_comp & cache_en);
	assign cache1_enable = force_enable[1] | (cache_comp & cache_en);
	assign cache_en_controller[0] = cache0_valid;
	assign cache_en_controller[1] = cache1_valid;
	assign cache_en_controller[2] = victim_out;
	assign cache_en_controller[3] = cache0_dirty;
	assign cache_en_controller[4] = cache1_dirty;
	

	// Etc

	wire 		cache0_err;
	wire		cache1_err;

	wire 		controller_err;
	wire [15:0]	DataOutInt;
	wire		DoneInt;
	wire		StallInt;

	wire [7:0]	FORTESTINGINDEX;
	assign FORTESTINGINDEX = cache_addr[10:3];


   /* data_mem = 1, inst_mem = 0 *
    * needed for cache parameter */
   parameter memtype = 0;
   cache #(0 + memtype) c0(// Outputs
                          .tag_out              (cache0_tag),
                          .data_out             (cache0_data),
                          .hit                  (cache0_hit),
                          .dirty                (cache0_dirty),
                          .valid                (cache0_valid),
                          .err                  (cache0_err),
                          // Inputs
                          .enable               (cache0_enable),
                          .clk                  (clk),
                          .rst                  (rst),
                          .createdump           (createdump),
                          .tag_in               (cache_addr[15:11]),
                          .index                (cache_addr[10:3]),
                          .offset               (cache_addr[2:0]),
                          .data_in              (cache_data_in),
                          .comp                 (cache_comp),
                          .write                (cache_wr),
                          .valid_in             (1'b1));
   cache #(2 + memtype) c1(// Outputs
                          .tag_out              (cache1_tag),
                          .data_out             (cache1_data),
                          .hit                  (cache1_hit),
                          .dirty                (cache1_dirty),
                          .valid                (cache1_valid),
                          .err                  (cache1_err),
                          // Inputs
                          .enable               (cache1_enable),
                          .clk                  (clk),
                          .rst                  (rst),
                          .createdump           (createdump),
                          .tag_in               (cache_addr[15:11]),
                          .index                (cache_addr[10:3]),
                          .offset               (cache_addr[2:0]),
                          .data_in              (cache_data_in),
                          .comp                 (cache_comp),
                          .write                (cache_wr),
                          .valid_in             (1'b1));

   four_bank_mem mem(// Outputs
                     .data_out          (mem_data_out),
                     .stall             (mem_stall),
                     .busy              (mem_busy),
                     .err               (mem_err),
                     // Inputs
                     .clk               (clk),
                     .rst               (rst),
                     .createdump        (createdump),
                     .addr              (mem_addr),
                     .data_in           (mem_data_in),
                     .wr                (mem_write),
                     .rd                (mem_read));


	cache_controller controller(// Outputs
   								.toggle_victim_way(toggle_victim_way), 
								.DataOut(DataOutInt), 
								.Done(DoneInt), 
								.Stall(StallInt), 
								.cache_rd(cache_rd), 
								.cache_wr(cache_wr), 
								.cache_comp(cache_comp), 
								.cache_addr(cache_addr), 
								.cache_data_in(cache_data_in), 
								.force_enable(force_enable), 
								.mem_addr(mem_addr), 
								.mem_data_in(mem_data_in), 
								.mem_write(mem_write),
								.mem_read(mem_read), 
								.controller_err(controller_err),

   								// Inputs
   								.Addr(Addr), 
								.DataIn(DataIn), 
								.Rd(Rd), 
								.Wr(Wr), 
								.cache_en(cache_en_controller), 
								.real_hit(real_hit), 
								.victimize(victimize), 
								.actual_tag(actual_tag), 
								.cache_data_out(cache_data_out), 
								.mem_data_out(mem_data_out), 
								.mem_stall(mem_stall), 
								.mem_busy(mem_busy), 
								.clk(clk), 
								.rst(rst)
   );


	// Reg Outs
	always@(*)
	begin
		err = controller_err | mem_err;
		Done = DoneInt;
		DataOut = DataOutInt;
		Stall = StallInt;
		CacheHit = real_hit;
		case(rst)
			default: ;
		endcase
	end

   
endmodule // mem_system
`default_nettype wire
// DUMMY LINE FOR REV CONTROL :9:

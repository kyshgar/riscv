//-----------------------------------------------------------------
//                          Generated File
//-----------------------------------------------------------------
module dcache_core_tag_ram
(
    // Inputs
     input           clk0_i
    ,input           rst0_i
    ,input  [  7:0]  addr0_i
    ,input           clk1_i
    ,input           rst1_i
    ,input  [  7:0]  addr1_i
    ,input  [ 20:0]  data1_i
    ,input           wr1_i

    // Outputs
    ,output [ 20:0]  data0_o
);



//-----------------------------------------------------------------
// Tag RAM 0KB (256 x 21)
// Mode: Write First
//-----------------------------------------------------------------
/* verilator lint_off MULTIDRIVEN */
reg [20:0]   ram [255:0] /*verilator public*/;
/* verilator lint_on MULTIDRIVEN */

reg [20:0] ram_read0_q;

always @ (posedge clk1_i)
begin
    if (wr1_i)
        ram[addr1_i] = data1_i;

    ram_read0_q = ram[addr0_i];
end

assign data0_o = ram_read0_q;


endmodule

`define reset 3'b000
`define Sa 3'b001
`define Sb 3'b010
`define Sc 3'b011
`define COUNT 3'b100
`define DONE 3'b101
`define SUCCESS 3'b111

module top_module (
    input clk,
    input reset,      // Synchronous reset
    input data,
    output reg [3:0] count,
    output reg counting,
    output reg done,
    input ack );
    
    reg shift_ena, done_counting;
    reg[3:0] next_count;
    reg[9:0] next_counter, counter;
    
    signal_detector signal_detector(.clk(clk),
                                    .reset(reset),
                                    .data(data),
                                    .shift_ena(shift_ena),
                                    .counting(counting),
                                    .done_counting(done_counting),
                                    .done(done),
                                    .ack(ack));
    
    always_ff @(posedge clk) begin
        count = next_count;
        counter = next_counter;
    end
    
    assign done_counting = ((count == 4'b0000) & (counter == 999)) ? 1'b1 : 1'b0;
    
    always_comb begin
        casex({shift_ena, counting}) 
            6'b1x : begin
                next_count = {count[2:0], data};
                next_counter = {10{1'b0}};
            end
            4'b01 : begin
                if(counter == 999) begin
                    next_counter = {10{1'b0}};
                    next_count = count - 1'b1;
                end else begin
                    next_counter = counter + 1'b1;
                    next_count = count;
                end
            end
            default: begin
                next_count = count - 1'b1;
                next_counter = {10{1'b0}};
            end
        endcase
    end
endmodule


module signal_detector (
    input clk,
    input reset,      // Synchronous reset
    input data,
    output reg shift_ena,
    output reg counting,
    input done_counting,
    output reg done,
  	input ack );
    
    wire detected;
  	reg signal_reset;
    reg[2:0] next_state, state, next_state_rst;
    reg[1:0] counter, next_counter;
    
    assign next_state_rst = reset ? `reset : next_state;
    
    always_ff @(posedge clk) begin
        state <= next_state_rst;
        counter <= next_counter;
    end
    
    always_comb begin
        case(state)
            `reset : begin 
                counting = 1'b0;
                shift_ena = 1'b0;
                next_state = data ? `Sa : `reset;
                done = 1'b0;
            end
            `Sa : begin
                counting = 1'b0;
                shift_ena = 1'b0;
                next_state = data ? `Sb : `reset;
                done = 1'b0;
            end
            `Sb : begin
                counting = 1'b0;
                shift_ena = 1'b0;
                next_state = data ? `Sb : `Sc;
                done = 1'b0;
            end
            `Sc : begin
                counting = 1'b0;
                shift_ena = 1'b0;
                next_state = data ? `SUCCESS : `reset;
                done = 1'b0;
            end
            `SUCCESS: begin
                counting = 1'b0;
                shift_ena = 1'b1;
                next_state = (counter == 2'b11) ? `COUNT : `SUCCESS;
                done = 1'b0;
            end
            `COUNT : begin
                next_state = done_counting ? `DONE : `COUNT;
                shift_ena = 1'b0;
                counting = 1'b1;
                done = 1'b0;
            end
            `DONE : begin
                next_state = ack ? `reset : `DONE;
                shift_ena = 1'b0;
                counting = 1'b0;
                done = 1'b1;
            end
            default: begin
                {next_state, shift_ena} = 4'bxxxx;
                counting = 1'bx;
                done = 1'bx;
            end
        endcase
        
        if(shift_ena == 1'b1)
            next_counter = counter + 1'b1;
        else
            next_counter = 2'b00;
    end
endmodule

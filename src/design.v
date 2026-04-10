// ============================================================
//  Smart Parking Control System — FSM-based
//  TinyTapeout compatible (8-bit I/O)
//  Author : Claude (for Exploration Lab / TinyTapeout)
// ============================================================
//
//  I/O MAP (TinyTapeout ui_in / uo_out / uio_* conventions)
//
//  Inputs  (ui_in[7:0]):
//    [0] car_entry   — IR sensor: car detected at entry gate
//    [1] car_exit    — IR sensor: car detected at exit gate
//    [2] card_valid  — RFID/ticket reader: valid card presented
//    [3] rst_n       — active-low async reset (also mapped to rst_n port)
//    [7:4]           — reserved / unused
//
//  Outputs (uo_out[7:0]):
//    [0] entry_gate  — 1 = open entry barrier
//    [1] exit_gate   — 1 = open exit barrier
//    [2] lot_full    — 1 = parking lot is full (display red light)
//    [3] lot_empty   — 1 = parking lot is empty
//    [4] green_led   — entry allowed indicator
//    [5] red_led     — entry denied / full indicator
//    [6] buzzer      — beep on invalid card or overflow
//    [7] display_en  — enable 7-seg / LED matrix driver
//
//  Parameters:
//    MAX_SLOTS       — total parking slots (default 8 for TinyTapeout demo)
// ============================================================

`default_nettype none

module smart_parking_fsm #(
    parameter MAX_SLOTS = 8         // change to your actual lot capacity
)(
    input  wire       clk,
    input  wire       rst_n,        // active-low reset

    // Sensor inputs
    input  wire       car_entry,    // high for 1 cycle when car at entry
    input  wire       car_exit,     // high for 1 cycle when car at exit
    input  wire       card_valid,   // high when valid card/ticket detected

    // Status outputs
    output reg        entry_gate,   // open entry barrier
    output reg        exit_gate,    // open exit barrier
    output reg        lot_full,     // parking lot full flag
    output reg        lot_empty,    // parking lot empty flag
    output reg        green_led,    // entry OK
    output reg        red_led,      // entry denied
    output reg        buzzer,       // alert buzzer
    output reg        display_en,   // enable scoreboard display

    // Slot count (for display driver / debug)
    output reg [3:0]  slot_count    // number of occupied slots
);

    // ----------------------------------------------------------
    // FSM State Encoding (one-hot for reliability)
    // ----------------------------------------------------------
    localparam [4:0]
        S_IDLE          = 5'b00001,   // waiting for event
        S_ENTRY_CHECK   = 5'b00010,   // car at entry — check capacity & card
        S_ENTRY_OPEN    = 5'b00100,   // open entry gate, increment count
        S_EXIT_OPEN     = 5'b01000,   // open exit gate, decrement count
        S_FULL_ALERT    = 5'b10000;   // lot full, buzz & show red

    reg [4:0] state, next_state;

    // Internal slot counter width
    localparam CNT_W = 4;   // supports up to 15 slots; widen if needed
    reg [CNT_W-1:0] occupied;

    // Gate open timer (hold gate open for N clocks)
    localparam GATE_HOLD = 8;        // ~8 clock cycles (scale for real use)
    reg [3:0] gate_timer;

    // ----------------------------------------------------------
    // State Register (sequential)
    // ----------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            occupied   <= 4'd0;
            gate_timer <= 4'd0;
        end else begin
            state <= next_state;

            // Slot counter update
            case (state)
                S_ENTRY_OPEN: begin
                    if (gate_timer == 4'd0)
                        occupied <= occupied + 1;
                end
                S_EXIT_OPEN: begin
                    if (gate_timer == 4'd0 && occupied > 0)
                        occupied <= occupied - 1;
                end
            endcase

            // Gate hold-open timer
            if (state == S_ENTRY_OPEN || state == S_EXIT_OPEN) begin
                if (gate_timer < GATE_HOLD - 1)
                    gate_timer <= gate_timer + 1;
                else
                    gate_timer <= 4'd0;
            end else begin
                gate_timer <= 4'd0;
            end
        end
    end

    // ----------------------------------------------------------
    // Next-State Logic (combinational)
    // ----------------------------------------------------------
    always @(*) begin
        next_state = state;   // default: stay

        case (state)

            S_IDLE: begin
                if (car_exit)
                    next_state = S_EXIT_OPEN;
                else if (car_entry) begin
                    if (occupied >= MAX_SLOTS[CNT_W-1:0])
                        next_state = S_FULL_ALERT;
                    else
                        next_state = S_ENTRY_CHECK;
                end
            end

            S_ENTRY_CHECK: begin
                if (card_valid)
                    next_state = S_ENTRY_OPEN;
                else if (!car_entry)   // car left without valid card
                    next_state = S_IDLE;
                // else: keep waiting for card
            end

            S_ENTRY_OPEN: begin
                if (gate_timer == GATE_HOLD - 1)
                    next_state = S_IDLE;
            end

            S_EXIT_OPEN: begin
                if (gate_timer == GATE_HOLD - 1)
                    next_state = S_IDLE;
            end

            S_FULL_ALERT: begin
                // Stay until car leaves entry sensor
                if (!car_entry)
                    next_state = S_IDLE;
            end

            default: next_state = S_IDLE;
        endcase
    end

    // ----------------------------------------------------------
    // Output Logic (Moore outputs — depend only on state)
    // ----------------------------------------------------------
    always @(*) begin
        // Default all outputs off
        entry_gate  = 1'b0;
        exit_gate   = 1'b0;
        green_led   = 1'b0;
        red_led     = 1'b0;
        buzzer      = 1'b0;
        display_en  = 1'b1;   // always show count

        lot_full    = (occupied >= MAX_SLOTS[CNT_W-1:0]);
        lot_empty   = (occupied == 0);
        slot_count  = occupied;

        case (state)
            S_IDLE: begin
                green_led = !lot_full;
                red_led   = lot_full;
            end

            S_ENTRY_CHECK: begin
                green_led = 1'b1;       // blink / steady while waiting
            end

            S_ENTRY_OPEN: begin
                entry_gate = 1'b1;
                green_led  = 1'b1;
            end

            S_EXIT_OPEN: begin
                exit_gate = 1'b1;
                green_led = 1'b1;
            end

            S_FULL_ALERT: begin
                red_led  = 1'b1;
                buzzer   = 1'b1;
                lot_full = 1'b1;
            end
        endcase
    end

endmodule


// ============================================================
//  TinyTapeout top-level wrapper
//  Maps the generic ports to tt_um_* interface
// ============================================================
module tt_um_smart_parking (
    input  wire [7:0] ui_in,    // dedicated inputs
    output wire [7:0] uo_out,   // dedicated outputs
    input  wire [7:0] uio_in,   // IOs: input path
    output wire [7:0] uio_out,  // IOs: output path
    output wire [7:0] uio_oe,   // IOs: enable path (1=output)
    input  wire       ena,      // always 1 when design is powered
    input  wire       clk,
    input  wire       rst_n
);
    wire [3:0] slot_count_w;

    smart_parking_fsm #(.MAX_SLOTS(8)) u_parking (
        .clk        (clk),
        .rst_n      (rst_n & ui_in[3]),  // both hw reset and sw reset

        .car_entry  (ui_in[0]),
        .car_exit   (ui_in[1]),
        .card_valid (ui_in[2]),

        .entry_gate (uo_out[0]),
        .exit_gate  (uo_out[1]),
        .lot_full   (uo_out[2]),
        .lot_empty  (uo_out[3]),
        .green_led  (uo_out[4]),
        .red_led    (uo_out[5]),
        .buzzer     (uo_out[6]),
        .display_en (uo_out[7]),

        .slot_count (slot_count_w)
    );

    // Slot count on bidirectional IOs (lower nibble as output)
    assign uio_out = {4'b0000, slot_count_w};
    assign uio_oe  = 8'b00001111;   // lower 4 bits = output, upper = input

endmodule
`default_nettype wire

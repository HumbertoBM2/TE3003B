// Tang Nano 20K — Servo PWM controller para montacargas
// Jetson GPIO → pb1 / pb2 (activo alto, PULL_MODE=NONE)
//
// pb1=1 pb2=0 → SUBIR  (pulso ~2.48ms = 67000 cuentas a 27MHz)
// pb1=0 pb2=1 → BAJAR  (pulso ~0.56ms = 15000 cuentas a 27MHz)
// pb1=0 pb2=0 → NEUTRO (pulso  1.50ms = 40500 cuentas a 27MHz)
// pb1=1 pb2=1 → NEUTRO (seguro, no debería ocurrir)
//
// Periodo PWM: 20ms = 540000 cuentas a 27MHz

module servo_ctrl (
    input  clk,     // 27 MHz — FPGA pin 4
    input  pb1,     // Jetson GPIO168 → FPGA pin 15 (SUBIR)
    input  pb2,     // Jetson GPIO38  → FPGA pin 16 (BAJAR)
    output servo    // Señal PWM al servo — FPGA pin 17
);

    localparam PERIOD   = 540_000;  // 20ms @ 27MHz
    localparam NEUTRAL  =  40_500;  // 1.50ms
    localparam PULSE_UP =  67_000;  // 2.48ms  (subir)
    localparam PULSE_DN =  15_000;  // 0.56ms  (bajar)

    reg [19:0] cnt = 0;
    reg [16:0] pw  = NEUTRAL;

    always @(posedge clk) begin
        // Contador de periodo
        cnt <= (cnt == PERIOD - 1) ? 20'd0 : cnt + 20'd1;

        // Selección de ancho de pulso
        if (pb1 && !pb2)
            pw <= PULSE_UP;
        else if (!pb1 && pb2)
            pw <= PULSE_DN;
        else
            pw <= NEUTRAL;
    end

    assign servo = (cnt < pw) ? 1'b1 : 1'b0;

endmodule

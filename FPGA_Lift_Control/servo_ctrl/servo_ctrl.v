// Tang Nano 20K — Servo DS04-NFC continuo — Lifter montacargas
// Jetson GPIO → pb1 / pb2 (activo alto, PULL_MODE=NONE)
//
// Servo DS04-NFC continuo: el ancho de pulso controla VELOCIDAD y DIRECCIÓN.
//   1.5ms → STOP  (motor detenido)
//   > 1.5ms → gira sentido A
//   < 1.5ms → gira sentido B
//
// NOTA: pb1/pb2 asignados físicamente de forma que:
//   pb1=1, pb2=0 → PULSE_SUBIR (1.2ms) → fork SUBE   — medido: 5cm en 3.00s
//   pb1=0, pb2=1 → PULSE_BAJAR (1.8ms) → fork BAJA   — medido: 5cm en 2.92s
//   pb1=0, pb2=0 → PULSE_STP   (1.5ms) → STOP
//   pb1=1, pb2=1 → PULSE_STP   (1.5ms) → STOP (seguro)
//
// El sentido de 1.2ms sube el lifter por la disposición mecánica del eje.
// Si se invierte la mecánica: intercambiar PULSE_SUBIR y PULSE_BAJAR.
//
// Periodo PWM: 20ms = 540_000 cuentas a 27MHz

module servo_ctrl (
    input  clk,     // 27 MHz — FPGA pin 4
    input  pb1,     // Jetson GPIO168 → FPGA pin 15 (SUBIR)
    input  pb2,     // Jetson GPIO38  → FPGA pin 16 (BAJAR)
    output servo    // Señal PWM al servo — FPGA pin 17
);

    localparam PERIOD      = 540_000;  // 20ms @ 27MHz
    localparam PULSE_SUBIR =  32_400;  // 1.20ms → fork SUBE  (5cm en 3.00s)
    localparam PULSE_BAJAR =  48_600;  // 1.80ms → fork BAJA  (5cm en 2.92s)
    localparam PULSE_STP   =  40_500;  // 1.50ms → STOP

    reg [19:0] cnt = 0;
    reg [16:0] pw  = PULSE_STP;

    always @(posedge clk) begin
        cnt <= (cnt == PERIOD - 1) ? 20'd0 : cnt + 20'd1;

        if (pb1 && !pb2)
            pw <= PULSE_SUBIR;
        else if (!pb1 && pb2)
            pw <= PULSE_BAJAR;
        else
            pw <= PULSE_STP;
    end

    assign servo = (cnt < pw) ? 1'b1 : 1'b0;

endmodule

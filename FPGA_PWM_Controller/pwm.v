`timescale 1ns / 1ps
// TE3003B - Challenge Delivery 1
// Servo PWM controller using Tang Nano 20K (27MHz)
//
// Period = 20ms -> 540,000 counts @ 27MHz
// Pulse widths:
//   1.0ms (izquierda) -> 27,000 counts
//   1.5ms (neutral)   -> 40,500 counts
//   2.0ms (derecha)   -> 54,000 counts
//
// Pinout:
//   clk   -> pin 45 (LCD_CLK)
//   pb1   -> pin 15 (IOL47A, BANK6)
//   pb2   -> pin 16 (IOL47B, BANK6)
//   servo -> pin 17 (IOL49A, BANK6)
//
// Tabla de verdad:
//   pb1=0, pb2=0 -> neutral   (1.5ms)
//   pb1=1, pb2=0 -> derecha   (2.0ms)  +90°
//   pb1=0, pb2=1 -> izquierda (1.0ms)  -90°

module top(
    input  clk,    // 27 MHz — pin 45
    input  pb1,    // DIP SW1 — pin 15
    input  pb2,    // DIP SW2 — pin 16
    output servo   // PWM out — pin 17
);

    // --- Contador de período (20ms) ---
    // 27,000,000 Hz * 0.020 s = 540,000 counts
    reg [19:0] counter = 0;

    always @(posedge clk) begin
        if (counter < 540000 - 1)
            counter <= counter + 1;
        else
            counter <= 0;
    end

    // --- Selección de ancho de pulso ---
    // PULL_MODE=UP en CST: switch OFF -> pin = 1, switch ON -> pin = 0
    reg [19:0] pulse_width = 40500;

    always @(posedge clk) begin
        if      ( pb1 & ~pb2) pulse_width <= 67000;  // derecha   +90°  2.0ms
        else if (~pb1 &  pb2) pulse_width <= 15000;  // izquierda -90°  1.0ms
        else                  pulse_width <= 40500;  // neutral    0°   1.5ms
    end

    // --- Salida PWM ---
    assign servo = (counter < pulse_width) ? 1'b1 : 1'b0;

endmodule

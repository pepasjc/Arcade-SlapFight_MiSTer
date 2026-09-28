//============================================================================
//  Slap Fight / Tiger Heli for MiSTer FPGA - RetroAchievements RAM tap
//
//  Turns the memory writes of the two Z80s (and of the hiscore module) into
//  the single write port of slapfight_ra_mirror, laid out exactly as
//  FinalBurn Neo's "All Ram" block so the ARM region table is the identity.
//
//  RA arcade sets were written against FinalBurn Neo. Its Slap Fight driver
//  (src/burn/drv/toaplan/d_slapfght.cpp, MemIndex from AllRam; SlapfighInit
//  maps) exposes:
//      0x0000 DrvZ80RAM0   0x0800  main C000-C7FF (work RAM)
//      0x0800 DrvZ80RAM1   0x3000  audio D000-FFFF
//      0x3800 DrvVidRAM    0x0800  main D000-D7FF (bg tiles)
//      0x4000 DrvColRAM    0x0800  main D800-DFFF (bg attributes)
//      0x4800 DrvSprRAM    0x0800  main E000-E7FF
//      0x5000 DrvShareRAM  0x0800  main C800-CFFF, audio C800-CFFF
//      0x5800 DrvTxtRAM    0x0800  main F000-F7FF (fg chars)
//      0x6000 DrvTxcRAM    0x0800  main F800-FFFF (fg attributes)
//      0x6800 DrvMCURAM    0x0080  (MCU, not mirrored)
//      0x6880 DrvSprBuf    0x0800  (sprite buffer, not mirrored)
//  The shadow covers 0x0000-0x7FFF (AW=15). The Z80 is 8-bit: shadow byte
//  k is RA address k (no byte swap).
//
//  Writes are taken at each CPU's own bus (address as the CPU sees it),
//  which is what FBNeo models. On this board the audio CPU's only RAM is
//  the 2 kB chip it shares with the main CPU, so audio writes above CFFF
//  (FBNeo's DrvZ80RAM1) land here as FBNeo files them, not as the board
//  aliases them. Every source is a level write strobe (held for several
//  clocks, longer during wait states); each write is reported once, on the
//  clock after its strobe falls, with the address and data of its last
//  clock. The pulses then go through one-deep latches, so two sources
//  ending on the same clock are written on consecutive clocks.
//
//  SPDX-License-Identifier: GPL-2.0-or-later (same terms as the rest of the core)
//============================================================================

module slapfight_ra_tap (
    input             clk,

    input             m_wr,     // main CPU
    input      [15:0] m_ad,
    input      [ 7:0] m_do,
    input             s_wr,     // audio CPU
    input      [15:0] s_ad,
    input      [ 7:0] s_do,
    input             h_wr,     // hiscore module into main work RAM
    input      [10:0] h_ad,
    input      [ 7:0] h_do,

    // slapfight_ra_mirror write port
    output reg [14:0] ra_wr_word,
    output reg [15:0] ra_wr_din,
    output reg [ 1:0] ra_wr_be
);

// CPU address -> FBNeo All Ram offset, {valid, 15-bit offset}
function automatic [15:0] map_m(input [15:0] a);
    case( a[15:11] )
        5'b1100_0: map_m = { 1'b1, 15'h0000 + { 4'd0, a[10:0] } };  // C000-C7FF work RAM
        5'b1100_1: map_m = { 1'b1, 15'h5000 + { 4'd0, a[10:0] } };  // C800-CFFF shared
        5'b1101_0: map_m = { 1'b1, 15'h3800 + { 4'd0, a[10:0] } };  // D000-D7FF bg tiles
        5'b1101_1: map_m = { 1'b1, 15'h4000 + { 4'd0, a[10:0] } };  // D800-DFFF bg attrs
        5'b1110_0: map_m = { 1'b1, 15'h4800 + { 4'd0, a[10:0] } };  // E000-E7FF sprites
        5'b1111_0: map_m = { 1'b1, 15'h5800 + { 4'd0, a[10:0] } };  // F000-F7FF fg chars
        5'b1111_1: map_m = { 1'b1, 15'h6000 + { 4'd0, a[10:0] } };  // F800-FFFF fg attrs
        default:   map_m = 16'd0;
    endcase
endfunction

function automatic [15:0] map_s(input [15:0] a);
    reg [15:0] a_d000;
    a_d000 = a - 16'hD000;
    if( a[15:11] == 5'b1100_1 )  map_s = { 1'b1, 15'h5000 + { 4'd0, a[10:0] } };     // C800-CFFF shared
    else if( a[15:12] >= 4'hD )  map_s = { 1'b1, 15'h0800 + { 1'b0, a_d000[13:0] } }; // D000-FFFF
    else                         map_s = 16'd0;
endfunction

// Delayed strobes: capture each clock, report on the falling edge
reg        m_wr_l, s_wr_l, h_wr_l;
reg [15:0] m_ad_l, s_ad_l;
reg [10:0] h_ad_l;
reg  [7:0] m_do_l, s_do_l, h_do_l;

always_ff @(posedge clk) begin
    m_wr_l <= m_wr;
    s_wr_l <= s_wr;
    h_wr_l <= h_wr;
    if( m_wr ) begin m_ad_l <= m_ad; m_do_l <= m_do; end
    if( s_wr ) begin s_ad_l <= s_ad; s_do_l <= s_do; end
    if( h_wr ) begin h_ad_l <= h_ad; h_do_l <= h_do; end
end

wire [15:0] m_map = map_m(m_ad_l);
wire [15:0] s_map = map_s(s_ad_l);

// Source requests: {valid, byte offset, data}; order is the output priority
localparam N = 3;
wire [N-1:0] req;
wire [14:0]  r_addr[0:N-1];
wire [ 7:0]  r_din[0:N-1];

assign req[0]    = h_wr_l && !h_wr;
assign r_addr[0] = { 4'd0, h_ad_l };
assign r_din[0]  = h_do_l;

assign req[1]    = m_wr_l && !m_wr && m_map[15];
assign r_addr[1] = m_map[14:0];
assign r_din[1]  = m_do_l;

assign req[2]    = s_wr_l && !s_wr && s_map[15];
assign r_addr[2] = s_map[14:0];
assign r_din[2]  = s_do_l;

// One-deep latches
reg  [N-1:0] pend = 0;
reg  [14:0]  p_addr[0:N-1];
reg  [ 7:0]  p_din[0:N-1];

// first pending latch
reg [N-1:0] grant;
always_comb begin
    grant = 0;
    for (int i = N-1; i >= 0; i--) if (pend[i]) begin grant = 0; grant[i] = 1'b1; end
end

always_ff @(posedge clk) begin
    ra_wr_be <= 2'b00;
    for (int i = 0; i < N; i++) begin
        if (grant[i]) begin
            ra_wr_word <= { 1'b0, p_addr[i][14:1] };
            ra_wr_din  <= { 2{p_din[i]} };
            ra_wr_be   <= p_addr[i][0] ? 2'b10 : 2'b01;
            pend[i]    <= 0;
        end
        if (req[i]) begin
            pend[i]   <= 1;
            p_addr[i] <= r_addr[i];
            p_din[i]  <= r_din[i];
        end
    end
end

endmodule

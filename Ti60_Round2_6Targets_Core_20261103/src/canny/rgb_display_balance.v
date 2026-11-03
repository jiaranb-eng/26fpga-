`timescale 1ns / 1ps
// Fixed-gain display encoding, applied before RGB565 truncation.
// 固定红蓝补偿和标准显示曲线；组合逻辑，不增加缓存或流水延迟。
// Reference: W3C CSS Color 4, gam_sRGB (read 2026-09-25).
// Gains R/G/B = 1.5/1.0/1.5 are an initial visual correction, not calibration.
module rgb_display_balance (
    input wire [7:0] red_i,   // Bayer插值后的完整红色分量。
    input wire [7:0] green_i, // 插值绿色保持单位线性增益。
    input wire [7:0] blue_i,  // 插值蓝色使用与红色相同的补偿。
    output wire [15:0] pixel_o // 沿用高红、中绿、低蓝的RGB565布局。
);
    // Efinity synthesis guide: syn_romstyle="logic" preserves scarce block RAM.
    (* syn_romstyle = "logic" *) reg [4:0] red_code;   // 饱和、显示编码后的五位红色。
    (* syn_romstyle = "logic" *) reg [5:0] green_code; // 保留六位绿色显示精度。
    (* syn_romstyle = "logic" *) reg [4:0] blue_code;  // 不允许溢出回绕的五位蓝色。

    // red通道查找将增益、饱和及传递曲线合成一次组合映射。
    always @(*) begin
        case (red_i)
            8'd0: red_code = 5'd0;
            8'd1: red_code = 5'd2;
            8'd2: red_code = 5'd3;
            8'd3: red_code = 5'd4;
            8'd4: red_code = 5'd5;
            8'd5, 8'd6: red_code = 5'd6;
            8'd7, 8'd8: red_code = 5'd7;
            8'd9, 8'd10: red_code = 5'd8;
            8'd11, 8'd12, 8'd13: red_code = 5'd9;
            8'd14, 8'd15, 8'd16: red_code = 5'd10;
            8'd17, 8'd18, 8'd19: red_code = 5'd11;
            8'd20, 8'd21, 8'd22, 8'd23: red_code = 5'd12;
            8'd24, 8'd25, 8'd26, 8'd27: red_code = 5'd13;
            8'd28, 8'd29, 8'd30, 8'd31: red_code = 5'd14;
            8'd32, 8'd33, 8'd34, 8'd35, 8'd36: red_code = 5'd15;
            8'd37, 8'd38, 8'd39, 8'd40, 8'd41: red_code = 5'd16;
            8'd42, 8'd43, 8'd44, 8'd45, 8'd46, 8'd47: red_code = 5'd17;
            8'd48, 8'd49, 8'd50, 8'd51, 8'd52: red_code = 5'd18;
            8'd53, 8'd54, 8'd55, 8'd56, 8'd57, 8'd58, 8'd59: red_code = 5'd19;
            8'd60, 8'd61, 8'd62, 8'd63, 8'd64, 8'd65, 8'd66: red_code = 5'd20;
            8'd67, 8'd68, 8'd69, 8'd70, 8'd71, 8'd72, 8'd73: red_code = 5'd21;
            8'd74, 8'd75, 8'd76, 8'd77, 8'd78, 8'd79, 8'd80: red_code = 5'd22;
            8'd81, 8'd82, 8'd83, 8'd84, 8'd85, 8'd86, 8'd87, 8'd88,
            8'd89: red_code = 5'd23;
            8'd90, 8'd91, 8'd92, 8'd93, 8'd94, 8'd95, 8'd96, 8'd97: red_code = 5'd24;
            8'd98, 8'd99, 8'd100, 8'd101, 8'd102, 8'd103, 8'd104, 8'd105,
            8'd106: red_code = 5'd25;
            8'd107, 8'd108, 8'd109, 8'd110, 8'd111, 8'd112, 8'd113, 8'd114,
            8'd115, 8'd116: red_code = 5'd26;
            8'd117, 8'd118, 8'd119, 8'd120, 8'd121, 8'd122, 8'd123, 8'd124,
            8'd125, 8'd126: red_code = 5'd27;
            8'd127, 8'd128, 8'd129, 8'd130, 8'd131, 8'd132, 8'd133, 8'd134,
            8'd135, 8'd136: red_code = 5'd28;
            8'd137, 8'd138, 8'd139, 8'd140, 8'd141, 8'd142, 8'd143, 8'd144,
            8'd145, 8'd146, 8'd147: red_code = 5'd29;
            8'd148, 8'd149, 8'd150, 8'd151, 8'd152, 8'd153, 8'd154, 8'd155,
            8'd156, 8'd157, 8'd158: red_code = 5'd30;
            8'd159, 8'd160, 8'd161, 8'd162, 8'd163, 8'd164, 8'd165, 8'd166,
            8'd167, 8'd168, 8'd169, 8'd170, 8'd171, 8'd172, 8'd173, 8'd174,
            8'd175, 8'd176, 8'd177, 8'd178, 8'd179, 8'd180, 8'd181, 8'd182,
            8'd183, 8'd184, 8'd185, 8'd186, 8'd187, 8'd188, 8'd189, 8'd190,
            8'd191, 8'd192, 8'd193, 8'd194, 8'd195, 8'd196, 8'd197, 8'd198,
            8'd199, 8'd200, 8'd201, 8'd202, 8'd203, 8'd204, 8'd205, 8'd206,
            8'd207, 8'd208, 8'd209, 8'd210, 8'd211, 8'd212, 8'd213, 8'd214,
            8'd215, 8'd216, 8'd217, 8'd218, 8'd219, 8'd220, 8'd221, 8'd222,
            8'd223, 8'd224, 8'd225, 8'd226, 8'd227, 8'd228, 8'd229, 8'd230,
            8'd231, 8'd232, 8'd233, 8'd234, 8'd235, 8'd236, 8'd237, 8'd238,
            8'd239, 8'd240, 8'd241, 8'd242, 8'd243, 8'd244, 8'd245, 8'd246,
            8'd247, 8'd248, 8'd249, 8'd250, 8'd251, 8'd252, 8'd253, 8'd254,
            8'd255: red_code = 5'd31;
            default: red_code = 5'd0;
        endcase
    end

    // green通道查找将增益、饱和及传递曲线合成一次组合映射。
    always @(*) begin
        case (green_i)
            8'd0: green_code = 6'd0;
            8'd1: green_code = 6'd3;
            8'd2: green_code = 6'd5;
            8'd3: green_code = 6'd7;
            8'd4: green_code = 6'd8;
            8'd5: green_code = 6'd9;
            8'd6: green_code = 6'd10;
            8'd7: green_code = 6'd11;
            8'd8: green_code = 6'd12;
            8'd9: green_code = 6'd13;
            8'd10, 8'd11: green_code = 6'd14;
            8'd12: green_code = 6'd15;
            8'd13, 8'd14: green_code = 6'd16;
            8'd15, 8'd16: green_code = 6'd17;
            8'd17, 8'd18: green_code = 6'd18;
            8'd19, 8'd20: green_code = 6'd19;
            8'd21, 8'd22: green_code = 6'd20;
            8'd23, 8'd24: green_code = 6'd21;
            8'd25, 8'd26: green_code = 6'd22;
            8'd27, 8'd28, 8'd29: green_code = 6'd23;
            8'd30, 8'd31, 8'd32: green_code = 6'd24;
            8'd33, 8'd34: green_code = 6'd25;
            8'd35, 8'd36, 8'd37: green_code = 6'd26;
            8'd38, 8'd39, 8'd40: green_code = 6'd27;
            8'd41, 8'd42, 8'd43, 8'd44: green_code = 6'd28;
            8'd45, 8'd46, 8'd47: green_code = 6'd29;
            8'd48, 8'd49, 8'd50: green_code = 6'd30;
            8'd51, 8'd52, 8'd53, 8'd54: green_code = 6'd31;
            8'd55, 8'd56, 8'd57, 8'd58: green_code = 6'd32;
            8'd59, 8'd60, 8'd61, 8'd62: green_code = 6'd33;
            8'd63, 8'd64, 8'd65, 8'd66: green_code = 6'd34;
            8'd67, 8'd68, 8'd69, 8'd70: green_code = 6'd35;
            8'd71, 8'd72, 8'd73, 8'd74: green_code = 6'd36;
            8'd75, 8'd76, 8'd77, 8'd78, 8'd79: green_code = 6'd37;
            8'd80, 8'd81, 8'd82, 8'd83, 8'd84: green_code = 6'd38;
            8'd85, 8'd86, 8'd87, 8'd88, 8'd89: green_code = 6'd39;
            8'd90, 8'd91, 8'd92, 8'd93, 8'd94: green_code = 6'd40;
            8'd95, 8'd96, 8'd97, 8'd98, 8'd99: green_code = 6'd41;
            8'd100, 8'd101, 8'd102, 8'd103, 8'd104: green_code = 6'd42;
            8'd105, 8'd106, 8'd107, 8'd108, 8'd109, 8'd110: green_code = 6'd43;
            8'd111, 8'd112, 8'd113, 8'd114, 8'd115: green_code = 6'd44;
            8'd116, 8'd117, 8'd118, 8'd119, 8'd120, 8'd121: green_code = 6'd45;
            8'd122, 8'd123, 8'd124, 8'd125, 8'd126, 8'd127: green_code = 6'd46;
            8'd128, 8'd129, 8'd130, 8'd131, 8'd132, 8'd133: green_code = 6'd47;
            8'd134, 8'd135, 8'd136, 8'd137, 8'd138, 8'd139: green_code = 6'd48;
            8'd140, 8'd141, 8'd142, 8'd143, 8'd144, 8'd145, 8'd146: green_code = 6'd49;
            8'd147, 8'd148, 8'd149, 8'd150, 8'd151, 8'd152, 8'd153: green_code = 6'd50;
            8'd154, 8'd155, 8'd156, 8'd157, 8'd158, 8'd159: green_code = 6'd51;
            8'd160, 8'd161, 8'd162, 8'd163, 8'd164, 8'd165, 8'd166: green_code = 6'd52;
            8'd167, 8'd168, 8'd169, 8'd170, 8'd171, 8'd172, 8'd173, 8'd174: green_code = 6'd53;
            8'd175, 8'd176, 8'd177, 8'd178, 8'd179, 8'd180, 8'd181: green_code = 6'd54;
            8'd182, 8'd183, 8'd184, 8'd185, 8'd186, 8'd187, 8'd188, 8'd189: green_code = 6'd55;
            8'd190, 8'd191, 8'd192, 8'd193, 8'd194, 8'd195, 8'd196: green_code = 6'd56;
            8'd197, 8'd198, 8'd199, 8'd200, 8'd201, 8'd202, 8'd203, 8'd204: green_code = 6'd57;
            8'd205, 8'd206, 8'd207, 8'd208, 8'd209, 8'd210, 8'd211, 8'd212: green_code = 6'd58;
            8'd213, 8'd214, 8'd215, 8'd216, 8'd217, 8'd218, 8'd219, 8'd220,
            8'd221: green_code = 6'd59;
            8'd222, 8'd223, 8'd224, 8'd225, 8'd226, 8'd227, 8'd228, 8'd229: green_code = 6'd60;
            8'd230, 8'd231, 8'd232, 8'd233, 8'd234, 8'd235, 8'd236, 8'd237,
            8'd238: green_code = 6'd61;
            8'd239, 8'd240, 8'd241, 8'd242, 8'd243, 8'd244, 8'd245, 8'd246,
            8'd247: green_code = 6'd62;
            8'd248, 8'd249, 8'd250, 8'd251, 8'd252, 8'd253, 8'd254, 8'd255: green_code = 6'd63;
            default: green_code = 6'd0;
        endcase
    end

    // blue通道查找将增益、饱和及传递曲线合成一次组合映射。
    always @(*) begin
        case (blue_i)
            8'd0: blue_code = 5'd0;
            8'd1: blue_code = 5'd2;
            8'd2: blue_code = 5'd3;
            8'd3: blue_code = 5'd4;
            8'd4: blue_code = 5'd5;
            8'd5, 8'd6: blue_code = 5'd6;
            8'd7, 8'd8: blue_code = 5'd7;
            8'd9, 8'd10: blue_code = 5'd8;
            8'd11, 8'd12, 8'd13: blue_code = 5'd9;
            8'd14, 8'd15, 8'd16: blue_code = 5'd10;
            8'd17, 8'd18, 8'd19: blue_code = 5'd11;
            8'd20, 8'd21, 8'd22, 8'd23: blue_code = 5'd12;
            8'd24, 8'd25, 8'd26, 8'd27: blue_code = 5'd13;
            8'd28, 8'd29, 8'd30, 8'd31: blue_code = 5'd14;
            8'd32, 8'd33, 8'd34, 8'd35, 8'd36: blue_code = 5'd15;
            8'd37, 8'd38, 8'd39, 8'd40, 8'd41: blue_code = 5'd16;
            8'd42, 8'd43, 8'd44, 8'd45, 8'd46, 8'd47: blue_code = 5'd17;
            8'd48, 8'd49, 8'd50, 8'd51, 8'd52: blue_code = 5'd18;
            8'd53, 8'd54, 8'd55, 8'd56, 8'd57, 8'd58, 8'd59: blue_code = 5'd19;
            8'd60, 8'd61, 8'd62, 8'd63, 8'd64, 8'd65, 8'd66: blue_code = 5'd20;
            8'd67, 8'd68, 8'd69, 8'd70, 8'd71, 8'd72, 8'd73: blue_code = 5'd21;
            8'd74, 8'd75, 8'd76, 8'd77, 8'd78, 8'd79, 8'd80: blue_code = 5'd22;
            8'd81, 8'd82, 8'd83, 8'd84, 8'd85, 8'd86, 8'd87, 8'd88,
            8'd89: blue_code = 5'd23;
            8'd90, 8'd91, 8'd92, 8'd93, 8'd94, 8'd95, 8'd96, 8'd97: blue_code = 5'd24;
            8'd98, 8'd99, 8'd100, 8'd101, 8'd102, 8'd103, 8'd104, 8'd105,
            8'd106: blue_code = 5'd25;
            8'd107, 8'd108, 8'd109, 8'd110, 8'd111, 8'd112, 8'd113, 8'd114,
            8'd115, 8'd116: blue_code = 5'd26;
            8'd117, 8'd118, 8'd119, 8'd120, 8'd121, 8'd122, 8'd123, 8'd124,
            8'd125, 8'd126: blue_code = 5'd27;
            8'd127, 8'd128, 8'd129, 8'd130, 8'd131, 8'd132, 8'd133, 8'd134,
            8'd135, 8'd136: blue_code = 5'd28;
            8'd137, 8'd138, 8'd139, 8'd140, 8'd141, 8'd142, 8'd143, 8'd144,
            8'd145, 8'd146, 8'd147: blue_code = 5'd29;
            8'd148, 8'd149, 8'd150, 8'd151, 8'd152, 8'd153, 8'd154, 8'd155,
            8'd156, 8'd157, 8'd158: blue_code = 5'd30;
            8'd159, 8'd160, 8'd161, 8'd162, 8'd163, 8'd164, 8'd165, 8'd166,
            8'd167, 8'd168, 8'd169, 8'd170, 8'd171, 8'd172, 8'd173, 8'd174,
            8'd175, 8'd176, 8'd177, 8'd178, 8'd179, 8'd180, 8'd181, 8'd182,
            8'd183, 8'd184, 8'd185, 8'd186, 8'd187, 8'd188, 8'd189, 8'd190,
            8'd191, 8'd192, 8'd193, 8'd194, 8'd195, 8'd196, 8'd197, 8'd198,
            8'd199, 8'd200, 8'd201, 8'd202, 8'd203, 8'd204, 8'd205, 8'd206,
            8'd207, 8'd208, 8'd209, 8'd210, 8'd211, 8'd212, 8'd213, 8'd214,
            8'd215, 8'd216, 8'd217, 8'd218, 8'd219, 8'd220, 8'd221, 8'd222,
            8'd223, 8'd224, 8'd225, 8'd226, 8'd227, 8'd228, 8'd229, 8'd230,
            8'd231, 8'd232, 8'd233, 8'd234, 8'd235, 8'd236, 8'd237, 8'd238,
            8'd239, 8'd240, 8'd241, 8'd242, 8'd243, 8'd244, 8'd245, 8'd246,
            8'd247, 8'd248, 8'd249, 8'd250, 8'd251, 8'd252, 8'd253, 8'd254,
            8'd255: blue_code = 5'd31;
            default: blue_code = 5'd0;
        endcase
    end

    assign pixel_o = {red_code, green_code, blue_code}; // 独立通道拼接，不交叉混色。
endmodule

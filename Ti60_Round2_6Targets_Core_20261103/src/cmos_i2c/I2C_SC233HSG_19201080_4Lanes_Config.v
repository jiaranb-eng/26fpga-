/*-------------------------------------------------------------------------
This confidential and proprietary software may be only used as authorized
by a licensing agreement from CrazyBingo.www.cnblogs.com/crazybingo
(C) COPYRIGHT 2012 CrazyBingo. ALL RIGHTS RESERVED
Filename            :       I2C_SC130GS_12801024_Config.v
Author              :       CrazyBingo
Date                :       2019-08-03
Version             :       1.0
Description         :       I2C Configure Data of AR0135.
Modification History    :
Date            By          Version         Change Description
===========================================================================
19/08/03        CrazyBingo  1.0             Original
--------------------------------------------------------------------------*/

`timescale 1ns/1ns

module  I2C_SC233HGS_19201200_4Lanes_Config //1920*1200
(
    input       [8:0]   LUT_INDEX,
    output  reg [23:0]  LUT_DATA,
    output      [8:0]   LUT_SIZE
);
assign  LUT_SIZE = 193;

//-----------------------------------------------------------------
/////////////////////   Config Data LUT   //////////////////////////    
always@(*)
begin
    case(LUT_INDEX)
0:	LUT_DATA = {16'h2100,8'h00}; 
1:	LUT_DATA = {16'h36e9,8'h80}; 
2:	LUT_DATA = {16'h37f9,8'h80}; 
3:	LUT_DATA = {16'h300c,8'h24}; 
4:	LUT_DATA = {16'h301f,8'h01}; 
5:	LUT_DATA = {16'h3044,8'h10}; 
6:	LUT_DATA = {16'h3062,8'h00}; 
7:	LUT_DATA = {16'h320c,8'h04}; 
8:	LUT_DATA = {16'h320d,8'h65}; 
9:	LUT_DATA = {16'h320e,8'h06}; 
10:	LUT_DATA = {16'h320f,8'h40}; 
11:	LUT_DATA = {16'h321f,8'h0b}; 
12:	LUT_DATA = {16'h322f,8'h00}; 
13:	LUT_DATA = {16'h3231,8'h00}; 
14:	LUT_DATA = {16'h3250,8'h03}; 
15:	LUT_DATA = {16'h32e2,8'h00}; 
16:	LUT_DATA = {16'h3301,8'h18}; 
17:	LUT_DATA = {16'h3304,8'h68}; 
18:	LUT_DATA = {16'h3306,8'h48}; 
19:	LUT_DATA = {16'h3309,8'h88}; 
20:	LUT_DATA = {16'h330b,8'hf0}; 
21:	LUT_DATA = {16'h330d,8'h30}; 
22:	LUT_DATA = {16'h3314,8'hc0}; 
23:	LUT_DATA = {16'h3315,8'h30}; 
24:	LUT_DATA = {16'h3317,8'h30}; 
25:	LUT_DATA = {16'h3318,8'h1f}; 
26:	LUT_DATA = {16'h331d,8'h32}; 
27:	LUT_DATA = {16'h331f,8'h02}; 
28:	LUT_DATA = {16'h3320,8'hc1}; 
29:	LUT_DATA = {16'h333b,8'h30}; 
30:	LUT_DATA = {16'h3352,8'h1a}; 
31:	LUT_DATA = {16'h3356,8'h1c}; 
32:	LUT_DATA = {16'h3363,8'h0f}; 
33:	LUT_DATA = {16'h3385,8'h59}; 
34:	LUT_DATA = {16'h3387,8'h79}; 
35:	LUT_DATA = {16'h33b0,8'h00}; 
36:	LUT_DATA = {16'h33ef,8'h04}; 
37:	LUT_DATA = {16'h33f8,8'h02}; 
38:	LUT_DATA = {16'h33fa,8'h02}; 
39:	LUT_DATA = {16'h341c,8'h06}; 
40:	LUT_DATA = {16'h341e,8'h24}; 
41:	LUT_DATA = {16'h341f,8'h27}; 
42:	LUT_DATA = {16'h34ad,8'h05}; 
43:	LUT_DATA = {16'h34af,8'h02}; 
44:	LUT_DATA = {16'h34de,8'h02}; 
45:	LUT_DATA = {16'h34f2,8'h00}; 
46:	LUT_DATA = {16'h3619,8'h20}; 
47:	LUT_DATA = {16'h361a,8'h90}; 
48:	LUT_DATA = {16'h3630,8'h80}; 
49:	LUT_DATA = {16'h3633,8'h44}; 
50:	LUT_DATA = {16'h3637,8'h3e}; 
51:	LUT_DATA = {16'h363a,8'h00}; 
52:	LUT_DATA = {16'h363b,8'h17}; 
53:	LUT_DATA = {16'h363c,8'h0f}; 
54:	LUT_DATA = {16'h363d,8'h1f}; 
55:	LUT_DATA = {16'h363e,8'ha8}; 
56:	LUT_DATA = {16'h363f,8'h86}; 
57:	LUT_DATA = {16'h3648,8'h4a}; 
58:	LUT_DATA = {16'h364a,8'h22}; 
59:	LUT_DATA = {16'h3660,8'h02}; 
60:	LUT_DATA = {16'h3661,8'h05}; 
61:	LUT_DATA = {16'h3662,8'h0b}; 
62:	LUT_DATA = {16'h3663,8'h17}; 
63:	LUT_DATA = {16'h3664,8'h17}; 
64:	LUT_DATA = {16'h3665,8'h08}; 
65:	LUT_DATA = {16'h3666,8'h18}; 
66:	LUT_DATA = {16'h3667,8'h38}; 
67:	LUT_DATA = {16'h3668,8'h78}; 
68:	LUT_DATA = {16'h3670,8'hf0}; 
69:	LUT_DATA = {16'h3671,8'hc8}; 
70:	LUT_DATA = {16'h3672,8'ha8}; 
71:	LUT_DATA = {16'h367c,8'h43}; 
72:	LUT_DATA = {16'h367d,8'h43}; 
73:	LUT_DATA = {16'h367e,8'h43}; 
74:	LUT_DATA = {16'h36c0,8'h08}; 
75:	LUT_DATA = {16'h36c1,8'h38}; 
76:	LUT_DATA = {16'h36c6,8'h08}; 
77:	LUT_DATA = {16'h36c7,8'h38}; 
78:	LUT_DATA = {16'h36ea,8'h04}; 
79:	LUT_DATA = {16'h36eb,8'h0c}; 
80:	LUT_DATA = {16'h36ec,8'h53}; 
81:	LUT_DATA = {16'h36ed,8'h24}; 
82:	LUT_DATA = {16'h3718,8'h04}; 
83:	LUT_DATA = {16'h3719,8'h06}; 
84:	LUT_DATA = {16'h3722,8'h08}; 
85:	LUT_DATA = {16'h3728,8'ha0}; 
86:	LUT_DATA = {16'h372a,8'h10}; 
87:	LUT_DATA = {16'h3778,8'h08}; 
88:	LUT_DATA = {16'h3779,8'h18}; 
89:	LUT_DATA = {16'h377a,8'hd8}; 
90:	LUT_DATA = {16'h3794,8'h00}; 
91:	LUT_DATA = {16'h3795,8'h00}; 
92:	LUT_DATA = {16'h3796,8'h00}; 
93:	LUT_DATA = {16'h3798,8'h10}; 
94:	LUT_DATA = {16'h3799,8'h10}; 
95:	LUT_DATA = {16'h379a,8'h10}; 
96:	LUT_DATA = {16'h37c4,8'h08}; 
97:	LUT_DATA = {16'h37c5,8'h38}; 
98:	LUT_DATA = {16'h37d2,8'h08}; 
99:	LUT_DATA = {16'h37d3,8'h38}; 
100:	LUT_DATA = {16'h37d4,8'h08}; 
101:	LUT_DATA = {16'h37d5,8'h38}; 
102:	LUT_DATA = {16'h37fa,8'h04}; 
103:	LUT_DATA = {16'h37fb,8'he5}; 
104:	LUT_DATA = {16'h37fc,8'h01}; 
105:	LUT_DATA = {16'h37fd,8'h34}; 
106:	LUT_DATA = {16'h3900,8'h1d}; 
107:	LUT_DATA = {16'h3901,8'h06}; 
108:	LUT_DATA = {16'h3903,8'h40}; 
109:	LUT_DATA = {16'h3904,8'h0a}; 
110:	LUT_DATA = {16'h3905,8'h4d}; 
111:	LUT_DATA = {16'h391f,8'h44}; 
112:	LUT_DATA = {16'h3933,8'h00}; 
113:	LUT_DATA = {16'h3934,8'h20}; 
114:	LUT_DATA = {16'h3935,8'h00}; 
115:	LUT_DATA = {16'h3936,8'h3c}; 
116:	LUT_DATA = {16'h3937,8'h7e}; 
117:	LUT_DATA = {16'h3938,8'h64}; 
118:	LUT_DATA = {16'h3939,8'h1f}; 
119:	LUT_DATA = {16'h393a,8'haa}; 
120:	LUT_DATA = {16'h393b,8'h00}; 
121:	LUT_DATA = {16'h393c,8'h20}; 
122:	LUT_DATA = {16'h39c1,8'h6c}; 
123:	LUT_DATA = {16'h3c0a,8'h2a}; 
124:	LUT_DATA = {16'h3e00,8'h00}; 
125:	LUT_DATA = {16'h3e01,8'h40}; 
126:	LUT_DATA = {16'h3e02,8'h00}; 
127:	LUT_DATA = {16'h3e03,8'h0b}; 
128:	LUT_DATA = {16'h3e08,8'h00}; 
129:	LUT_DATA = {16'h3e09,8'h40}; 
130:	LUT_DATA = {16'h3e15,8'h00}; 
131:	LUT_DATA = {16'h3e1c,8'h00}; 
132:	LUT_DATA = {16'h3e8e,8'h00}; 
133:	LUT_DATA = {16'h4330,8'h50}; 
134:	LUT_DATA = {16'h4331,8'h20}; 
135:	LUT_DATA = {16'h4333,8'h02}; 
136:	LUT_DATA = {16'h4360,8'h07}; 
137:	LUT_DATA = {16'h4362,8'hf0}; 
138:	LUT_DATA = {16'h4364,8'hf0}; 
139:	LUT_DATA = {16'h4365,8'h18}; 
140:	LUT_DATA = {16'h4366,8'h38}; 
141:	LUT_DATA = {16'h4368,8'h58}; 
142:	LUT_DATA = {16'h436a,8'h78}; 
143:	LUT_DATA = {16'h436b,8'h18}; 
144:	LUT_DATA = {16'h436c,8'h38}; 
145:	LUT_DATA = {16'h4370,8'h20}; 
146:	LUT_DATA = {16'h4371,8'h10}; 
147:	LUT_DATA = {16'h4372,8'h18}; 
148:	LUT_DATA = {16'h4373,8'h38}; 
149:	LUT_DATA = {16'h4402,8'h03}; 
150:	LUT_DATA = {16'h4403,8'h0b}; 
151:	LUT_DATA = {16'h4404,8'h21}; 
152:	LUT_DATA = {16'h4405,8'h2b}; 
153:	LUT_DATA = {16'h440c,8'h36}; 
154:	LUT_DATA = {16'h440d,8'h36}; 
155:	LUT_DATA = {16'h440e,8'h29}; 
156:	LUT_DATA = {16'h440f,8'h44}; 
157:	LUT_DATA = {16'h4412,8'h01}; 
158:	LUT_DATA = {16'h450d,8'h10}; 
159:	LUT_DATA = {16'h4837,8'h1e}; 
160:	LUT_DATA = {16'h4b0a,8'h92}; 
161:	LUT_DATA = {16'h5000,8'h30}; 
162:	LUT_DATA = {16'h5002,8'h00}; 
163:	LUT_DATA = {16'h502e,8'h21}; 
164:	LUT_DATA = {16'h5034,8'h01}; 
165:	LUT_DATA = {16'h5104,8'h14}; 
166:	LUT_DATA = {16'h5105,8'h10}; 
167:	LUT_DATA = {16'h5106,8'h04}; 
168:	LUT_DATA = {16'h5107,8'h59}; 
169:	LUT_DATA = {16'h510d,8'h44}; 
170:	LUT_DATA = {16'h5302,8'h00}; 
171:	LUT_DATA = {16'h5787,8'h0a}; 
172:	LUT_DATA = {16'h5788,8'h0a}; 
173:	LUT_DATA = {16'h5789,8'h08}; 
174:	LUT_DATA = {16'h578a,8'h0a}; 
175:	LUT_DATA = {16'h578b,8'h0a}; 
176:	LUT_DATA = {16'h578c,8'h08}; 
177:	LUT_DATA = {16'h578d,8'h40}; 
178:	LUT_DATA = {16'h5790,8'h08}; 
179:	LUT_DATA = {16'h5791,8'h04}; 
180:	LUT_DATA = {16'h5792,8'h04}; 
181:	LUT_DATA = {16'h5793,8'h08}; 
182:	LUT_DATA = {16'h5794,8'h04}; 
183:	LUT_DATA = {16'h5795,8'h04}; 
184:	LUT_DATA = {16'h57aa,8'h2a}; 
185:	LUT_DATA = {16'h57ab,8'h7f}; 
186:	LUT_DATA = {16'h57ac,8'h00}; 
187:	LUT_DATA = {16'h57ad,8'h00}; 
188:	LUT_DATA = {16'h36e9,8'h04}; 
189:	LUT_DATA = {16'h37f9,8'h04}; 
190:	LUT_DATA = {16'h4424,8'h01}; 
191:	LUT_DATA = {16'h4407,8'ha0}; 
192:	LUT_DATA = {16'h2100,8'h01}; 

		default:LUT_DATA    =   {16'h0000, 8'h00};
    endcase
end

endmodule

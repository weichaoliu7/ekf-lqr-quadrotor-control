syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 x13 x14 x15
syms p q r ax ay az g
f1 = x4*cos(x8)*cos(x9) + x5 * (sin(x7)*sin(x8)*cos(x9) - cos(x7)*sin(x9)) + x6 * (cos(x7)*sin(x8)*cos(x9) + sin(x7)*sin(x9));
f2 = x4*cos(x8)*sin(x9) + x5 * (sin(x7)*sin(x8)*sin(x9) + cos(x7)*cos(x9)) + x6 * (cos(x7)*sin(x8)*sin(x9) - sin(x7)*cos(x9));
f3 = x4*(-sin(x8)) + x5 * (sin(x7)*cos(x8)) + x6 * cos(x7)*cos(x8);
f4 = r*x5 - q*x6 - g*sin(x8) + ax;
f5 = p*x6 - r*x4 + g*cos(x8)*sin(x7) + ay;
f6 = q*x4 - p*x5 + g*cos(x8)*cos(x7) + az;
f7 = p + q *sin(x7)*tan(x8) + r*cos(x7)*tan(x8);
f8 = q*cos(x7) - r*sin(x7);
f9 = q*sin(x7)/cos(x8) + r*cos(x7)/cos(x8);
f10 = 0;
f11 = 0;
f12 = 0;
f13 = 0;
f14 = 0;
f15 = 0;
J_f = jacobian([f1;f2;f3;f4;f5;f6;f7;f8;f9;f10;f11;f12;f13;f14;f15],[x1;x2;x3;x4;x5;x6;x7;x8;x9;x10;x11;x12;x13;x14;x15]);
J_f

h1 = p+x10;
h2 = q+x11;
h3 = r+x12;
h4 = ax+x13;
h5 = ay+x14;
h6 = az+x15;
h7 = x1;
h8 = x2;
h9 = x3;
h10 = x9;

J_h = jacobian([h1;h2;h3;h4;h5;h6;h7;h8;h9;h10],[x1;x2;x3;x4;x5;x6;x7;x8;x9;x10;x11;x12;x13;x14;x15]);
J_h

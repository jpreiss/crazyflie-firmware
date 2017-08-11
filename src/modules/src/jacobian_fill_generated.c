float t2 = 1.0f/inertia1;
float t3 = drag_constant*1.0E2f;
float t4 = thrust_constant*-3.983716857408418E1f;
float t5 = t3+t4;
float t7 = t2*t5*-2.5E-3f;
float t6 = -t7;
float t8 = 1.0f/inertia2;
float t9 = drag_constant*1.732050807568877E2f;
float t10 = thrust_constant*-6.9E1f;
float t11 = t9+t10;
float t13 = t8*t11*2.5E-3f;
float t12 = -t13;
float t14 = 1.0f/inertia3;
float t15 = thrust_constant*2.3E1f;
float t16 = t9+t15;
float t20 = t14*t16*-5.0E-3f;
float t19 = -t20;
float t18 = -t19;
float t17 = -t18;
J[3][0] = t6;
J[3][1] = t2*t5*5.0E-3f;
J[3][2] = t6;
J[3][3] = t7;
J[3][4] = t2*t5*-5.0E-3f;
J[3][5] = t7;
J[4][0] = t8*t11*2.5E-3f;
J[4][2] = t12;
J[4][3] = t12;
J[4][5] = t13;
J[5][0] = t17;
J[5][1] = t14*t16*-5.0E-3f;
J[5][2] = t17;
J[5][3] = t18;
J[5][4] = t19;
J[5][5] = t20;



t2 = 1.0f/mass;
t3 = t2*thrust_constant*2.5E-1f;
t4 = t2*thrust_constant*8.660254037844386E-1f;
F1_left.m[0][0] = t3;
F1_left.m[0][1] = t2*thrust_constant*-5.0E-1f;
F1_left.m[0][2] = t3;
F1_left.m[1][0] = t2*thrust_constant*4.330127018922193E-1f;
F1_left.m[1][2] = t2*thrust_constant*-4.330127018922193E-1f;
F1_left.m[2][0] = t4;
F1_left.m[2][1] = t4;
F1_left.m[2][2] = t4;



t2 = 1.0f/mass;
t3 = t2*thrust_constant*2.5E-1f;
t4 = t2*thrust_constant*8.660254037844386E-1f;
F1_right.m[0][0] = t3;
F1_right.m[0][1] = t2*thrust_constant*-5.0E-1f;
F1_right.m[0][2] = t3;
F1_right.m[1][0] = t2*thrust_constant*4.330127018922193E-1f;
F1_right.m[1][2] = t2*thrust_constant*-4.330127018922193E-1f;
F1_right.m[2][0] = t4;
F1_right.m[2][1] = t4;
F1_right.m[2][2] = t4;




t2 = 1.0f/mass;
t3 = t2*thrust_constant*-2.5E-1f;
t4 = t2*thrust_constant*8.660254037844386E-1f;
F1_left.m[0][1] = t2*thrust_constant*4.330127018922193E-1f;
F1_left.m[0][2] = t2*thrust_constant*-4.330127018922193E-1f;
F1_left.m[1][0] = t2*thrust_constant*5.0E-1f;
F1_left.m[1][1] = t3;
F1_left.m[1][2] = t3;
F1_left.m[2][0] = t4;
F1_left.m[2][1] = t4;
F1_left.m[2][2] = t4;

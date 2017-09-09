float t2 = 1.0f/inertia1;
float t3 = 1.0f/inertia2;
float t4 = 1.0f/inertia3;
float t5 = drag_constant*8.660254037844386E-1f;
J[3][0] = t2*(drag_constant*2.967094433018507E-1f-thrust_constant*1.18200641103385E-1f);
J[3][1] = t2*(drag_constant*4.968839595802982E-1f-thrust_constant*1.979445005955877E-1f);
J[3][2] = t2*(drag_constant*2.001745162784476E-1f-thrust_constant*7.974385949220274E-2f);
J[3][3] = t2*(drag_constant*2.001745162784473E-1f-thrust_constant*7.974385949220261E-2f)*-1.0f;
J[3][4] = t2*(drag_constant*4.968839595802982E-1f-thrust_constant*1.979445005955877E-1f)*-1.0f;
J[3][5] = t2*(drag_constant*2.96709443301851E-1f-thrust_constant*1.182006411033851E-1f)*-1.0f;
J[4][0] = t3*(drag_constant*4.024468986779571E-1f-thrust_constant*1.603234494475115E-1f);
J[4][1] = t3*(drag_constant*5.573446610316284E-2f-thrust_constant*2.220303321538279E-2f);
J[4][2] = t3*(drag_constant*4.581813647811198E-1f-thrust_constant*1.825264826628942E-1f)*-1.0f;
J[4][3] = t3*(drag_constant*4.581813647811199E-1f-thrust_constant*1.825264826628943E-1f)*-1.0f;
J[4][4] = t3*(drag_constant*5.573446610316228E-2f-thrust_constant*2.220303321538256E-2f);
J[4][5] = t3*(drag_constant*4.024468986779569E-1f-thrust_constant*1.603234494475114E-1f);
J[5][0] = t4*(t5+thrust_constant*1.15E-1f);
J[5][1] = t4*(t5+thrust_constant*1.15E-1f)*-1.0f;
J[5][2] = t4*(t5+thrust_constant*1.15E-1f);
J[5][3] = t4*(t5+thrust_constant*1.15E-1f)*-1.0f;
J[5][4] = t4*(t5+thrust_constant*1.15E-1f);
J[5][5] = t4*(t5+thrust_constant*1.15E-1f)*-1.0f;



t2 = 1.0f/mass;
t3 = t2*thrust_constant*8.660254037844386E-1f;
F1_left.m[0][0] = t2*thrust_constant*2.967094433018507E-1f;
F1_left.m[0][1] = t2*thrust_constant*-4.968839595802982E-1f;
F1_left.m[0][2] = t2*thrust_constant*2.001745162784476E-1f;
F1_left.m[1][0] = t2*thrust_constant*4.024468986779571E-1f;
F1_left.m[1][1] = t2*thrust_constant*-5.573446610316284E-2f;
F1_left.m[1][2] = t2*thrust_constant*-4.581813647811198E-1f;
F1_left.m[2][0] = t3;
F1_left.m[2][1] = t3;
F1_left.m[2][2] = t3;



t2 = 1.0f/mass;
t3 = t2*thrust_constant*8.660254037844386E-1f;
F1_right.m[0][0] = t2*thrust_constant*2.001745162784473E-1f;
F1_right.m[0][1] = t2*thrust_constant*-4.968839595802982E-1f;
F1_right.m[0][2] = t2*thrust_constant*2.96709443301851E-1f;
F1_right.m[1][0] = t2*thrust_constant*4.581813647811199E-1f;
F1_right.m[1][1] = t2*thrust_constant*5.573446610316228E-2f;
F1_right.m[1][2] = t2*thrust_constant*-4.024468986779569E-1f;
F1_right.m[2][0] = t3;
F1_right.m[2][1] = t3;
F1_right.m[2][2] = t3;




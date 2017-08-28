float t2 = 1.0f/inertia1;
float t3 = drag_constant*1.0E2f;
float t4 = thrust_constant*-3.983716857408418E1f;
float t5 = t3+t4;
float t6 = 1.0f/inertia2;
float t7 = 1.0f/inertia3;
float t8 = drag_constant*5.620826229351621E34f;
J[3][0] = t2*t5*2.967094433018507E-3f;
J[3][1] = t2*t5*4.968839595802982E-3f;
J[3][2] = t2*t5*2.001745162784476E-3f;
J[3][3] = t2*t5*-2.001745162784473E-3f;
J[3][4] = t2*t5*-4.968839595802982E-3f;
J[3][5] = t2*t5*-2.96709443301851E-3f;
J[4][0] = t5*t6*4.024468986779571E-3f;
J[4][1] = t5*t6*5.573446610316284E-4f;
J[4][2] = t5*t6*-4.581813647811198E-3f;
J[4][3] = t5*t6*-4.581813647811199E-3f;
J[4][4] = t5*t6*5.573446610316228E-4f;
J[4][5] = t5*t6*4.024468986779569E-3f;
J[5][0] = t7*(drag_constant*1.405206557337905E33f+thrust_constant*1.865981683535954E32f)*6.162975822039155E-34f;
J[5][1] = t7*(drag_constant*8.993321966962593E35f+thrust_constant*1.19422827746301E35f)*-9.629649721936179E-37f;
J[5][2] = t7*(t8+thrust_constant*7.463926734143815E33f)*1.540743955509789E-35f;
J[5][3] = t7*(t8+thrust_constant*7.463926734143814E33f)*-1.540743955509789E-35f;
J[5][4] = t7*(drag_constant*4.496660983481296E34f+thrust_constant*5.971141387315052E33f)*1.925929944387236E-35f;
J[5][5] = t7*(drag_constant*5.620826229351621E32f+thrust_constant*7.463926734143815E31f)*-1.540743955509789E-33f;



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




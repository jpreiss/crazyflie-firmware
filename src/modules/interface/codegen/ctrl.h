// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once


#include <Eigen/Dense>



namespace sym {


/**
* This function was autogenerated from a symbolic function. Do not modify by hand.
*
* Symbolic function: ctrl_symfn
*
* Args:
*     ierr: Matrix31
*     p: Matrix31
*     v: Matrix31
*     logR: Matrix31
*     w: Matrix31
*     p_d: Matrix31
*     v_d: Matrix31
*     a_d: Matrix31
*     w_d: Matrix31
*     theta_pos: Matrix61
*     theta_rot: Matrix41
*     dt: Scalar
*
* Outputs:
*     thrust_torque: Matrix41
*     jacobian: (4x25) jacobian of thrust_torque wrt args ierr (3), p (3), v (3), logR (3), w (3),
*               theta_pos (6), theta_rot (4)
*/
template <typename Scalar>
void Ctrl(const Eigen::Matrix<Scalar, 3, 1>& ierr, const Eigen::Matrix<Scalar, 3, 1>& p, const Eigen::Matrix<Scalar, 3, 1>& v, const Eigen::Matrix<Scalar, 3, 1>& logR, const Eigen::Matrix<Scalar, 3, 1>& w, const Eigen::Matrix<Scalar, 3, 1>& p_d, const Eigen::Matrix<Scalar, 3, 1>& v_d, const Eigen::Matrix<Scalar, 3, 1>& a_d, const Eigen::Matrix<Scalar, 3, 1>& w_d, const Eigen::Matrix<Scalar, 6, 1>& theta_pos, const Eigen::Matrix<Scalar, 4, 1>& theta_rot, const Scalar dt, Eigen::Matrix<Scalar, 4, 1>* const thrust_torque = nullptr, Eigen::Matrix<Scalar, 4, 25>* const jacobian = nullptr) {

    // Total ops: 1503

    // Unused inputs
    (void)dt;

    // Input arrays

    // Intermediate terms (332)
    const Scalar _tmp0 = a_d(1, 0) - ierr(1, 0)*theta_pos(0, 0) - theta_pos(2, 0)*(p(1, 0) - p_d(1, 0)) - theta_pos(4, 0)*(v(1, 0) - v_d(1, 0));
    const Scalar _tmp1 = std::pow(logR(0, 0), Scalar(2));
    const Scalar _tmp2 = std::pow(logR(2, 0), Scalar(2));
    const Scalar _tmp3 = std::pow(logR(1, 0), Scalar(2));
    const Scalar _tmp4 = _tmp1 + _tmp2 + _tmp3 + Scalar(9.9999999999999998e-13);
    const Scalar _tmp5 = std::sqrt(_tmp4);
    const Scalar _tmp6 = (Scalar(1)/Scalar(2))*_tmp5;
    const Scalar _tmp7 = std::sin(_tmp6);
    const Scalar _tmp8 = std::pow(_tmp7, Scalar(2));
    const Scalar _tmp9 = Scalar(1.0) / (_tmp4);
    const Scalar _tmp10 = _tmp9*logR(1, 0);
    const Scalar _tmp11 = _tmp10*logR(2, 0);
    const Scalar _tmp12 = _tmp11*_tmp8;
    const Scalar _tmp13 = _tmp7/_tmp5;
    const Scalar _tmp14 = _tmp13*logR(0, 0);
    const Scalar _tmp15 = std::cos(_tmp6);
    const Scalar _tmp16 = 2*_tmp15;
    const Scalar _tmp17 = 2*_tmp12 - _tmp14*_tmp16;
    const Scalar _tmp18 = a_d(0, 0) - ierr(0, 0)*theta_pos(0, 0) - theta_pos(2, 0)*(p(0, 0) - p_d(0, 0)) - theta_pos(4, 0)*(v(0, 0) - v_d(0, 0));
    const Scalar _tmp19 = _tmp8*logR(2, 0);
    const Scalar _tmp20 = _tmp9*logR(0, 0);
    const Scalar _tmp21 = _tmp19*_tmp20;
    const Scalar _tmp22 = _tmp13*_tmp16;
    const Scalar _tmp23 = 2*_tmp21 + _tmp22*logR(1, 0);
    const Scalar _tmp24 = _tmp3*_tmp9;
    const Scalar _tmp25 = _tmp24*_tmp8;
    const Scalar _tmp26 = _tmp1*_tmp9;
    const Scalar _tmp27 = _tmp26*_tmp8;
    const Scalar _tmp28 = -2*_tmp25 - 2*_tmp27 + 1;
    const Scalar _tmp29 = a_d(2, 0) - ierr(2, 0)*theta_pos(1, 0) - theta_pos(3, 0)*(p(2, 0) - p_d(2, 0)) - theta_pos(5, 0)*(v(2, 0) - v_d(2, 0)) + Scalar(9.8100000000000005);
    const Scalar _tmp30 = -theta_rot(2, 0)*(w(0, 0) - w_d(0, 0));
    const Scalar _tmp31 = _tmp13*logR(2, 0);
    const Scalar _tmp32 = std::pow(_tmp29, Scalar(2));
    const Scalar _tmp33 = std::pow(_tmp0, Scalar(2));
    const Scalar _tmp34 = std::pow(_tmp18, Scalar(2));
    const Scalar _tmp35 = _tmp32 + _tmp33 + _tmp34 + Scalar(9.9999999999999995e-7);
    const Scalar _tmp36 = std::pow(_tmp35, Scalar(Scalar(-1)/Scalar(2)));
    const Scalar _tmp37 = _tmp29*_tmp36;
    const Scalar _tmp38 = 2*_tmp37 + Scalar(2.0000010000000001);
    const Scalar _tmp39 = std::sqrt(_tmp38);
    const Scalar _tmp40 = Scalar(1.0) / (_tmp39);
    const Scalar _tmp41 = (((std::fabs(_tmp37 + 1) + Scalar(-9.9999999999999995e-7)) > 0) - ((std::fabs(_tmp37 + 1) + Scalar(-9.9999999999999995e-7)) < 0)) + 1;
    const Scalar _tmp42 = (Scalar(1)/Scalar(2))*_tmp41;
    const Scalar _tmp43 = _tmp40*_tmp42;
    const Scalar _tmp44 = _tmp36*_tmp43;
    const Scalar _tmp45 = _tmp18*_tmp44;
    const Scalar _tmp46 = (Scalar(1)/Scalar(4))*_tmp39;
    const Scalar _tmp47 = _tmp41*_tmp46;
    const Scalar _tmp48 = _tmp0*_tmp36;
    const Scalar _tmp49 = -_tmp42 - _tmp43*_tmp48 + 1;
    const Scalar _tmp50 = _tmp15*_tmp49;
    const Scalar _tmp51 = _tmp14*_tmp47 + _tmp31*_tmp45 - _tmp50;
    const Scalar _tmp52 = _tmp13*_tmp49;
    const Scalar _tmp53 = _tmp52*logR(0, 0);
    const Scalar _tmp54 = _tmp13*logR(1, 0);
    const Scalar _tmp55 = _tmp45*_tmp54;
    const Scalar _tmp56 = -_tmp53 - _tmp55;
    const Scalar _tmp57 = _tmp15*_tmp41;
    const Scalar _tmp58 = _tmp46*_tmp57;
    const Scalar _tmp59 = 2*std::min<Scalar>(0, (((-_tmp56 + _tmp58) > 0) - ((-_tmp56 + _tmp58) < 0))) + 1;
    const Scalar _tmp60 = 2*_tmp59;
    const Scalar _tmp61 = _tmp60*theta_rot(0, 0);
    const Scalar _tmp62 = std::min<Scalar>(Scalar(0.99999899999999997), std::fabs(_tmp56 - _tmp58));
    const Scalar _tmp63 = std::acos(_tmp62)/std::sqrt(Scalar(1 - std::pow(_tmp62, Scalar(2))));
    const Scalar _tmp64 = _tmp61*_tmp63;
    const Scalar _tmp65 = -theta_rot(2, 0)*(w(1, 0) - w_d(1, 0));
    const Scalar _tmp66 = _tmp15*_tmp44;
    const Scalar _tmp67 = _tmp52*logR(2, 0);
    const Scalar _tmp68 = -_tmp18*_tmp66 + _tmp47*_tmp54 - _tmp67;
    const Scalar _tmp69 = -theta_rot(3, 0)*(w(2, 0) - w_d(2, 0));
    const Scalar _tmp70 = _tmp52*logR(1, 0);
    const Scalar _tmp71 = -_tmp14*_tmp45 + _tmp31*_tmp47 + _tmp70;
    const Scalar _tmp72 = _tmp71*theta_rot(1, 0);
    const Scalar _tmp73 = std::pow(_tmp35, Scalar(Scalar(-3)/Scalar(2)));
    const Scalar _tmp74 = _tmp43*_tmp73;
    const Scalar _tmp75 = _tmp34*_tmp74;
    const Scalar _tmp76 = _tmp31*_tmp75;
    const Scalar _tmp77 = _tmp31*theta_pos(0, 0);
    const Scalar _tmp78 = std::pow(_tmp38, Scalar(Scalar(-3)/Scalar(2)));
    const Scalar _tmp79 = _tmp29/std::pow(_tmp35, Scalar(2));
    const Scalar _tmp80 = _tmp42*_tmp79;
    const Scalar _tmp81 = _tmp78*_tmp80;
    const Scalar _tmp82 = _tmp34*_tmp81;
    const Scalar _tmp83 = _tmp44*theta_pos(0, 0);
    const Scalar _tmp84 = _tmp18*_tmp29;
    const Scalar _tmp85 = (Scalar(1)/Scalar(4))*_tmp41;
    const Scalar _tmp86 = _tmp40*_tmp73;
    const Scalar _tmp87 = _tmp85*_tmp86;
    const Scalar _tmp88 = _tmp14*_tmp87;
    const Scalar _tmp89 = _tmp84*_tmp88;
    const Scalar _tmp90 = _tmp18*_tmp78;
    const Scalar _tmp91 = _tmp0*_tmp80*_tmp90;
    const Scalar _tmp92 = _tmp91*theta_pos(0, 0);
    const Scalar _tmp93 = _tmp0*_tmp74;
    const Scalar _tmp94 = _tmp18*_tmp93;
    const Scalar _tmp95 = _tmp94*theta_pos(0, 0);
    const Scalar _tmp96 = _tmp92 - _tmp95;
    const Scalar _tmp97 = _tmp53 + _tmp55 + _tmp58;
    const Scalar _tmp98 = std::fabs(_tmp97);
    const Scalar _tmp99 = std::min<Scalar>(Scalar(0.99999899999999997), _tmp98);
    const Scalar _tmp100 = 1 - std::pow(_tmp99, Scalar(2));
    const Scalar _tmp101 = std::acos(_tmp99);
    const Scalar _tmp102 = _tmp101/std::sqrt(_tmp100);
    const Scalar _tmp103 = _tmp102*_tmp61;
    const Scalar _tmp104 = _tmp54*_tmp75;
    const Scalar _tmp105 = _tmp54*_tmp82;
    const Scalar _tmp106 = _tmp13*_tmp96;
    const Scalar _tmp107 = (Scalar(1)/Scalar(4))*_tmp57;
    const Scalar _tmp108 = _tmp107*_tmp86;
    const Scalar _tmp109 = _tmp108*_tmp18;
    const Scalar _tmp110 = _tmp109*_tmp29;
    const Scalar _tmp111 = _tmp104*theta_pos(0, 0) - _tmp105*theta_pos(0, 0) + _tmp106*logR(0, 0) + _tmp110*theta_pos(0, 0) - _tmp54*_tmp83;
    const Scalar _tmp112 = _tmp59*((((Scalar(0.99999899999999997) - _tmp98) > 0) - ((Scalar(0.99999899999999997) - _tmp98) < 0)) + 1)*(((_tmp97) > 0) - ((_tmp97) < 0));
    const Scalar _tmp113 = _tmp112/_tmp100;
    const Scalar _tmp114 = _tmp113*theta_rot(0, 0);
    const Scalar _tmp115 = _tmp114*_tmp51;
    const Scalar _tmp116 = _tmp101*_tmp112*_tmp99/(_tmp100 * std::sqrt(_tmp100));
    const Scalar _tmp117 = _tmp116*theta_rot(0, 0);
    const Scalar _tmp118 = _tmp117*_tmp51;
    const Scalar _tmp119 = 1 - std::pow(Scalar(std::tanh((Scalar(1)/Scalar(268))*_tmp103*_tmp51 - Scalar(1)/Scalar(268)*_tmp30)), Scalar(2));
    const Scalar _tmp120 = 1 - std::pow(Scalar(std::tanh((Scalar(1)/Scalar(268))*_tmp103*_tmp68 - Scalar(1)/Scalar(268)*_tmp65)), Scalar(2));
    const Scalar _tmp121 = _tmp114*_tmp68;
    const Scalar _tmp122 = _tmp117*_tmp68;
    const Scalar _tmp123 = _tmp54*_tmp87;
    const Scalar _tmp124 = _tmp123*_tmp84;
    const Scalar _tmp125 = _tmp15*_tmp34;
    const Scalar _tmp126 = _tmp125*_tmp74;
    const Scalar _tmp127 = _tmp125*_tmp81;
    const Scalar _tmp128 = _tmp102*_tmp60;
    const Scalar _tmp129 = _tmp128*theta_rot(1, 0);
    const Scalar _tmp130 = 1 - std::pow(Scalar(std::tanh((Scalar(1)/Scalar(56))*_tmp129*_tmp71 - Scalar(1)/Scalar(56)*_tmp69)), Scalar(2));
    const Scalar _tmp131 = _tmp116*_tmp72;
    const Scalar _tmp132 = _tmp31*_tmp87;
    const Scalar _tmp133 = _tmp132*_tmp84;
    const Scalar _tmp134 = _tmp14*_tmp82;
    const Scalar _tmp135 = _tmp14*_tmp75;
    const Scalar _tmp136 = _tmp113*_tmp72;
    const Scalar _tmp137 = _tmp0*_tmp29;
    const Scalar _tmp138 = _tmp108*_tmp137;
    const Scalar _tmp139 = _tmp33*_tmp74;
    const Scalar _tmp140 = _tmp33*_tmp81;
    const Scalar _tmp141 = -_tmp139*theta_pos(0, 0) + _tmp140*theta_pos(0, 0) + _tmp83;
    const Scalar _tmp142 = _tmp138*theta_pos(0, 0) + _tmp14*_tmp141 - _tmp54*_tmp92 + _tmp54*_tmp95;
    const Scalar _tmp143 = _tmp0*_tmp87;
    const Scalar _tmp144 = _tmp143*_tmp29;
    const Scalar _tmp145 = _tmp14*_tmp144;
    const Scalar _tmp146 = _tmp29*_tmp54;
    const Scalar _tmp147 = _tmp143*_tmp146;
    const Scalar _tmp148 = _tmp13*_tmp141;
    const Scalar _tmp149 = _tmp74*_tmp84;
    const Scalar _tmp150 = _tmp149*_tmp31;
    const Scalar _tmp151 = 2*_tmp36;
    const Scalar _tmp152 = 2*_tmp32;
    const Scalar _tmp153 = _tmp152*_tmp73;
    const Scalar _tmp154 = -_tmp151*theta_pos(1, 0) + _tmp153*theta_pos(1, 0);
    const Scalar _tmp155 = _tmp78*_tmp85;
    const Scalar _tmp156 = _tmp18*_tmp36;
    const Scalar _tmp157 = _tmp155*_tmp156;
    const Scalar _tmp158 = _tmp157*_tmp31;
    const Scalar _tmp159 = (Scalar(1)/Scalar(8))*_tmp41;
    const Scalar _tmp160 = _tmp159*_tmp40;
    const Scalar _tmp161 = _tmp14*_tmp160;
    const Scalar _tmp162 = _tmp155*_tmp48;
    const Scalar _tmp163 = _tmp29*_tmp93;
    const Scalar _tmp164 = _tmp154*_tmp162 - _tmp163*theta_pos(1, 0);
    const Scalar _tmp165 = (Scalar(1)/Scalar(8))*_tmp57;
    const Scalar _tmp166 = _tmp165*_tmp40;
    const Scalar _tmp167 = _tmp157*_tmp54;
    const Scalar _tmp168 = _tmp54*_tmp84;
    const Scalar _tmp169 = _tmp168*_tmp74;
    const Scalar _tmp170 = _tmp14*_tmp164 + _tmp154*_tmp166 - _tmp154*_tmp167 + _tmp169*theta_pos(1, 0);
    const Scalar _tmp171 = _tmp160*_tmp54;
    const Scalar _tmp172 = _tmp107*_tmp90;
    const Scalar _tmp173 = _tmp172*_tmp36;
    const Scalar _tmp174 = _tmp15*_tmp84;
    const Scalar _tmp175 = _tmp174*_tmp74;
    const Scalar _tmp176 = _tmp160*_tmp31;
    const Scalar _tmp177 = _tmp14*_tmp157;
    const Scalar _tmp178 = _tmp14*_tmp149;
    const Scalar _tmp179 = _tmp44*theta_pos(2, 0);
    const Scalar _tmp180 = _tmp91*theta_pos(2, 0);
    const Scalar _tmp181 = _tmp94*theta_pos(2, 0);
    const Scalar _tmp182 = _tmp180 - _tmp181;
    const Scalar _tmp183 = _tmp104*theta_pos(2, 0) - _tmp105*theta_pos(2, 0) + _tmp110*theta_pos(2, 0) + _tmp14*_tmp182 - _tmp179*_tmp54;
    const Scalar _tmp184 = _tmp31*theta_pos(2, 0);
    const Scalar _tmp185 = _tmp33*theta_pos(2, 0);
    const Scalar _tmp186 = _tmp179 - _tmp185*_tmp74 + _tmp185*_tmp81;
    const Scalar _tmp187 = _tmp138*theta_pos(2, 0) + _tmp14*_tmp186 - _tmp180*_tmp54 + _tmp181*_tmp54;
    const Scalar _tmp188 = _tmp13*_tmp186;
    const Scalar _tmp189 = -_tmp151*theta_pos(3, 0) + _tmp153*theta_pos(3, 0);
    const Scalar _tmp190 = _tmp162*_tmp189 - _tmp163*theta_pos(3, 0);
    const Scalar _tmp191 = _tmp14*_tmp190 + _tmp166*_tmp189 - _tmp167*_tmp189 + _tmp169*theta_pos(3, 0);
    const Scalar _tmp192 = _tmp91*theta_pos(4, 0);
    const Scalar _tmp193 = _tmp18*theta_pos(4, 0);
    const Scalar _tmp194 = _tmp193*_tmp93;
    const Scalar _tmp195 = _tmp192 - _tmp194;
    const Scalar _tmp196 = _tmp54*theta_pos(4, 0);
    const Scalar _tmp197 = _tmp44*theta_pos(4, 0);
    const Scalar _tmp198 = _tmp193*_tmp29;
    const Scalar _tmp199 = _tmp108*_tmp198 + _tmp14*_tmp195 + _tmp196*_tmp75 - _tmp196*_tmp82 - _tmp197*_tmp54;
    const Scalar _tmp200 = _tmp31*theta_pos(4, 0);
    const Scalar _tmp201 = -_tmp139*theta_pos(4, 0) + _tmp140*theta_pos(4, 0) + _tmp197;
    const Scalar _tmp202 = _tmp138*theta_pos(4, 0) + _tmp14*_tmp201 - _tmp192*_tmp54 + _tmp194*_tmp54;
    const Scalar _tmp203 = _tmp13*_tmp201;
    const Scalar _tmp204 = -_tmp151*theta_pos(5, 0) + _tmp153*theta_pos(5, 0);
    const Scalar _tmp205 = _tmp18*_tmp54;
    const Scalar _tmp206 = _tmp204*_tmp36;
    const Scalar _tmp207 = _tmp155*_tmp206;
    const Scalar _tmp208 = _tmp162*_tmp204 - _tmp163*theta_pos(5, 0);
    const Scalar _tmp209 = _tmp14*_tmp208 + _tmp166*_tmp204 + _tmp169*theta_pos(5, 0) - _tmp205*_tmp207;
    const Scalar _tmp210 = _tmp18*_tmp31;
    const Scalar _tmp211 = _tmp14*_tmp18;
    const Scalar _tmp212 = _tmp7/(_tmp4 * std::sqrt(_tmp4));
    const Scalar _tmp213 = _tmp16*_tmp212;
    const Scalar _tmp214 = _tmp1*_tmp213;
    const Scalar _tmp215 = _tmp214*logR(2, 0);
    const Scalar _tmp216 = std::pow(_tmp15, Scalar(2));
    const Scalar _tmp217 = _tmp10*logR(0, 0);
    const Scalar _tmp218 = _tmp216*_tmp217;
    const Scalar _tmp219 = _tmp217*_tmp8;
    const Scalar _tmp220 = std::pow(_tmp4, Scalar(-2));
    const Scalar _tmp221 = 4*_tmp19*_tmp220;
    const Scalar _tmp222 = _tmp1*_tmp221;
    const Scalar _tmp223 = 2*_tmp19*_tmp9;
    const Scalar _tmp224 = logR(0, 0)*logR(1, 0);
    const Scalar _tmp225 = _tmp213*_tmp224;
    const Scalar _tmp226 = 4*_tmp8;
    const Scalar _tmp227 = _tmp220*_tmp226;
    const Scalar _tmp228 = _tmp227*logR(0, 0);
    const Scalar _tmp229 = [&]() { const Scalar base = logR(0, 0); return base * base * base; }();
    const Scalar _tmp230 = _tmp213*_tmp3;
    const Scalar _tmp231 = logR(0, 0)*logR(2, 0);
    const Scalar _tmp232 = _tmp213*_tmp231;
    const Scalar _tmp233 = -_tmp221*_tmp224 + _tmp232*logR(1, 0);
    const Scalar _tmp234 = _tmp156*_tmp40;
    const Scalar _tmp235 = _tmp107*_tmp234;
    const Scalar _tmp236 = _tmp217*_tmp235;
    const Scalar _tmp237 = _tmp212*_tmp45;
    const Scalar _tmp238 = _tmp224*_tmp237;
    const Scalar _tmp239 = _tmp159*_tmp39;
    const Scalar _tmp240 = (Scalar(1)/Scalar(2))*_tmp50;
    const Scalar _tmp241 = _tmp212*_tmp49;
    const Scalar _tmp242 = -_tmp1*_tmp241 - _tmp14*_tmp239 + _tmp236 - _tmp238 + _tmp240*_tmp26 + _tmp52;
    const Scalar _tmp243 = _tmp231*_tmp237;
    const Scalar _tmp244 = _tmp212*_tmp47;
    const Scalar _tmp245 = _tmp20*logR(2, 0);
    const Scalar _tmp246 = _tmp235*_tmp245;
    const Scalar _tmp247 = _tmp13*_tmp47;
    const Scalar _tmp248 = _tmp165*_tmp39;
    const Scalar _tmp249 = _tmp234*_tmp85;
    const Scalar _tmp250 = _tmp231*_tmp241;
    const Scalar _tmp251 = _tmp240*_tmp245;
    const Scalar _tmp252 = _tmp217*_tmp248 - _tmp224*_tmp244;
    const Scalar _tmp253 = _tmp13*_tmp45;
    const Scalar _tmp254 = _tmp241*logR(1, 0);
    const Scalar _tmp255 = _tmp217*_tmp240 - _tmp254*logR(0, 0);
    const Scalar _tmp256 = -_tmp231*_tmp244 + _tmp245*_tmp248;
    const Scalar _tmp257 = _tmp221*_tmp3;
    const Scalar _tmp258 = _tmp230*logR(2, 0);
    const Scalar _tmp259 = _tmp227*logR(1, 0);
    const Scalar _tmp260 = [&]() { const Scalar base = logR(1, 0); return base * base * base; }();
    const Scalar _tmp261 = _tmp235*_tmp24 - _tmp237*_tmp3 - _tmp239*_tmp54 + _tmp253 + _tmp255;
    const Scalar _tmp262 = logR(1, 0)*logR(2, 0);
    const Scalar _tmp263 = _tmp11*_tmp235 - _tmp237*_tmp262;
    const Scalar _tmp264 = _tmp11*_tmp240;
    const Scalar _tmp265 = _tmp254*logR(2, 0);
    const Scalar _tmp266 = _tmp11*_tmp248 - _tmp244*_tmp262;
    const Scalar _tmp267 = 2*_tmp8;
    const Scalar _tmp268 = _tmp213*logR(1, 0);
    const Scalar _tmp269 = _tmp2*_tmp9;
    const Scalar _tmp270 = -_tmp239*_tmp31 - _tmp250 + _tmp251 + _tmp263;
    const Scalar _tmp271 = 2*_tmp0;
    const Scalar _tmp272 = 2*_tmp18;
    const Scalar _tmp273 = -_tmp271*ierr(1, 0) - _tmp272*ierr(0, 0);
    const Scalar _tmp274 = _tmp18*_tmp273;
    const Scalar _tmp275 = _tmp155*_tmp79;
    const Scalar _tmp276 = _tmp275*_tmp54;
    const Scalar _tmp277 = _tmp166*_tmp73;
    const Scalar _tmp278 = _tmp277*_tmp29;
    const Scalar _tmp279 = _tmp44*ierr(0, 0);
    const Scalar _tmp280 = _tmp0*_tmp275;
    const Scalar _tmp281 = _tmp143*_tmp273 - _tmp273*_tmp280 + _tmp44*ierr(1, 0);
    const Scalar _tmp282 = -_tmp123*_tmp274 + _tmp14*_tmp281 - _tmp273*_tmp278 + _tmp274*_tmp276 - _tmp279*_tmp54;
    const Scalar _tmp283 = _tmp274*_tmp275;
    const Scalar _tmp284 = _tmp159*_tmp86;
    const Scalar _tmp285 = _tmp284*_tmp29;
    const Scalar _tmp286 = _tmp273*_tmp285;
    const Scalar _tmp287 = _tmp146*_tmp284;
    const Scalar _tmp288 = _tmp172*_tmp79;
    const Scalar _tmp289 = _tmp73*ierr(2, 0);
    const Scalar _tmp290 = -_tmp151*ierr(2, 0) + _tmp152*_tmp289;
    const Scalar _tmp291 = _tmp289*_tmp43;
    const Scalar _tmp292 = -_tmp137*_tmp291 + _tmp162*_tmp290;
    const Scalar _tmp293 = _tmp14*_tmp292 + _tmp166*_tmp290 - _tmp167*_tmp290 + _tmp168*_tmp291;
    const Scalar _tmp294 = _tmp291*_tmp84;
    const Scalar _tmp295 = _tmp290*_tmp31;
    const Scalar _tmp296 = _tmp13*_tmp292;
    const Scalar _tmp297 = -p(0, 0) + p_d(0, 0);
    const Scalar _tmp298 = -p(1, 0) + p_d(1, 0);
    const Scalar _tmp299 = _tmp271*_tmp298 + _tmp272*_tmp297;
    const Scalar _tmp300 = _tmp18*_tmp299;
    const Scalar _tmp301 = _tmp275*_tmp300;
    const Scalar _tmp302 = _tmp285*_tmp299;
    const Scalar _tmp303 = _tmp143*_tmp299 - _tmp280*_tmp299 - _tmp298*_tmp44;
    const Scalar _tmp304 = _tmp297*_tmp44;
    const Scalar _tmp305 = -_tmp123*_tmp300 + _tmp14*_tmp303 + _tmp276*_tmp300 - _tmp278*_tmp299 + _tmp304*_tmp54;
    const Scalar _tmp306 = _tmp13*_tmp303;
    const Scalar _tmp307 = -p(2, 0) + p_d(2, 0);
    const Scalar _tmp308 = _tmp151*_tmp307 - _tmp153*_tmp307;
    const Scalar _tmp309 = _tmp157*_tmp308;
    const Scalar _tmp310 = _tmp162*_tmp308 + _tmp163*_tmp307;
    const Scalar _tmp311 = _tmp13*_tmp310;
    const Scalar _tmp312 = _tmp166*_tmp308 - _tmp169*_tmp307 - _tmp309*_tmp54 + _tmp311*logR(0, 0);
    const Scalar _tmp313 = _tmp160*_tmp308;
    const Scalar _tmp314 = -v(0, 0) + v_d(0, 0);
    const Scalar _tmp315 = -v(1, 0) + v_d(1, 0);
    const Scalar _tmp316 = _tmp271*_tmp315 + _tmp272*_tmp314;
    const Scalar _tmp317 = _tmp316*_tmp87;
    const Scalar _tmp318 = _tmp314*_tmp44;
    const Scalar _tmp319 = _tmp275*_tmp316;
    const Scalar _tmp320 = _tmp0*_tmp316;
    const Scalar _tmp321 = -_tmp275*_tmp320 - _tmp315*_tmp44 + _tmp320*_tmp87;
    const Scalar _tmp322 = _tmp29*_tmp316;
    const Scalar _tmp323 = _tmp14*_tmp321 - _tmp205*_tmp317 + _tmp205*_tmp319 - _tmp277*_tmp322 + _tmp318*_tmp54;
    const Scalar _tmp324 = _tmp284*_tmp322;
    const Scalar _tmp325 = _tmp13*_tmp321;
    const Scalar _tmp326 = -v(2, 0) + v_d(2, 0);
    const Scalar _tmp327 = _tmp151*_tmp326 - _tmp153*_tmp326;
    const Scalar _tmp328 = _tmp162*_tmp327 + _tmp163*_tmp326;
    const Scalar _tmp329 = _tmp13*_tmp328;
    const Scalar _tmp330 = _tmp166*_tmp327 - _tmp167*_tmp327 - _tmp169*_tmp326 + _tmp329*logR(0, 0);
    const Scalar _tmp331 = _tmp160*_tmp327;

    // Output terms (2)
    if ( thrust_torque != nullptr ) {
        Eigen::Matrix<Scalar, 4, 1>& _thrust_torque = (*thrust_torque);


        _thrust_torque(0, 0) = _tmp0*_tmp17 + _tmp18*_tmp23 + _tmp28*_tmp29;
        _thrust_torque(1, 0) = 268*std::tanh((Scalar(1)/Scalar(268))*_tmp30 - Scalar(1)/Scalar(268)*_tmp51*_tmp64);
        _thrust_torque(2, 0) = -268*std::tanh((Scalar(1)/Scalar(268))*_tmp64*_tmp68 - Scalar(1)/Scalar(268)*_tmp65);
        _thrust_torque(3, 0) = -56*std::tanh((Scalar(1)/Scalar(56))*_tmp60*_tmp63*_tmp72 - Scalar(1)/Scalar(56)*_tmp69);
    }

    if ( jacobian != nullptr ) {
        Eigen::Matrix<Scalar, 4, 25>& _jacobian = (*jacobian);


        _jacobian(0, 0) = -_tmp23*theta_pos(0, 0);
        _jacobian(1, 0) = _tmp119*(-_tmp103*(-_tmp15*_tmp96 - _tmp31*_tmp83 + _tmp76*theta_pos(0, 0) - _tmp77*_tmp82 + _tmp89*theta_pos(0, 0)) + _tmp111*_tmp115 - _tmp111*_tmp118);
        _jacobian(2, 0) = _tmp120*(-_tmp103*(-_tmp106*logR(2, 0) + _tmp124*theta_pos(0, 0) - _tmp126*theta_pos(0, 0) + _tmp127*theta_pos(0, 0) + _tmp15*_tmp83) + _tmp111*_tmp121 - _tmp111*_tmp122);
        _jacobian(3, 0) = _tmp130*(-_tmp111*_tmp131 + _tmp111*_tmp136 - _tmp129*(_tmp106*logR(1, 0) + _tmp133*theta_pos(0, 0) + _tmp134*theta_pos(0, 0) - _tmp135*theta_pos(0, 0) + _tmp14*_tmp83));
        _jacobian(0, 1) = -_tmp17*theta_pos(0, 0);
        _jacobian(1, 1) = _tmp119*(-_tmp103*(-_tmp141*_tmp15 + _tmp145*theta_pos(0, 0) - _tmp31*_tmp92 + _tmp31*_tmp95) + _tmp115*_tmp142 - _tmp118*_tmp142);
        _jacobian(2, 1) = _tmp120*(-_tmp103*(_tmp147*theta_pos(0, 0) - _tmp148*logR(2, 0) + _tmp15*_tmp92 - _tmp15*_tmp95) + _tmp121*_tmp142 - _tmp122*_tmp142);
        _jacobian(3, 1) = _tmp130*(-_tmp129*(_tmp14*_tmp92 - _tmp14*_tmp95 + _tmp144*_tmp77 + _tmp148*logR(1, 0)) - _tmp131*_tmp142 + _tmp136*_tmp142);
        _jacobian(0, 2) = -_tmp28*theta_pos(1, 0);
        _jacobian(1, 2) = _tmp119*(-_tmp103*(-_tmp15*_tmp164 + _tmp150*theta_pos(1, 0) - _tmp154*_tmp158 + _tmp154*_tmp161) + _tmp115*_tmp170 - _tmp118*_tmp170);
        _jacobian(2, 2) = _tmp120*(-_tmp103*(_tmp154*_tmp171 + _tmp154*_tmp173 - _tmp164*_tmp31 - _tmp175*theta_pos(1, 0)) + _tmp121*_tmp170 - _tmp122*_tmp170);
        _jacobian(3, 2) = _tmp130*(-_tmp129*(_tmp154*_tmp176 + _tmp154*_tmp177 + _tmp164*_tmp54 - _tmp178*theta_pos(1, 0)) - _tmp131*_tmp170 + _tmp136*_tmp170);
        _jacobian(0, 3) = -_tmp23*theta_pos(2, 0);
        _jacobian(1, 3) = _tmp119*(-_tmp103*(-_tmp15*_tmp182 - _tmp179*_tmp31 - _tmp184*_tmp82 + _tmp76*theta_pos(2, 0) + _tmp89*theta_pos(2, 0)) + _tmp115*_tmp183 - _tmp118*_tmp183);
        _jacobian(2, 3) = _tmp120*(-_tmp103*(_tmp124*theta_pos(2, 0) - _tmp126*theta_pos(2, 0) + _tmp127*theta_pos(2, 0) + _tmp15*_tmp179 - _tmp182*_tmp31) + _tmp121*_tmp183 - _tmp122*_tmp183);
        _jacobian(3, 3) = _tmp130*(-_tmp129*(_tmp133*theta_pos(2, 0) + _tmp134*theta_pos(2, 0) - _tmp135*theta_pos(2, 0) + _tmp14*_tmp179 + _tmp182*_tmp54) - _tmp131*_tmp183 + _tmp136*_tmp183);
        _jacobian(0, 4) = -_tmp17*theta_pos(2, 0);
        _jacobian(1, 4) = _tmp119*(-_tmp103*(_tmp145*theta_pos(2, 0) - _tmp15*_tmp186 - _tmp180*_tmp31 + _tmp181*_tmp31) + _tmp115*_tmp187 - _tmp118*_tmp187);
        _jacobian(2, 4) = _tmp120*(-_tmp103*(_tmp147*theta_pos(2, 0) + _tmp15*_tmp180 - _tmp15*_tmp181 - _tmp188*logR(2, 0)) + _tmp121*_tmp187 - _tmp122*_tmp187);
        _jacobian(3, 4) = _tmp130*(-_tmp129*(_tmp14*_tmp180 - _tmp14*_tmp181 + _tmp144*_tmp184 + _tmp188*logR(1, 0)) - _tmp131*_tmp187 + _tmp136*_tmp187);
        _jacobian(0, 5) = -_tmp28*theta_pos(3, 0);
        _jacobian(1, 5) = _tmp119*(-_tmp103*(-_tmp15*_tmp190 + _tmp150*theta_pos(3, 0) - _tmp158*_tmp189 + _tmp161*_tmp189) + _tmp115*_tmp191 - _tmp118*_tmp191);
        _jacobian(2, 5) = _tmp120*(-_tmp103*(_tmp171*_tmp189 + _tmp173*_tmp189 - _tmp175*theta_pos(3, 0) - _tmp190*_tmp31) + _tmp121*_tmp191 - _tmp122*_tmp191);
        _jacobian(3, 5) = _tmp130*(-_tmp129*(_tmp176*_tmp189 + _tmp177*_tmp189 - _tmp178*theta_pos(3, 0) + _tmp190*_tmp54) - _tmp131*_tmp191 + _tmp136*_tmp191);
        _jacobian(0, 6) = -_tmp23*theta_pos(4, 0);
        _jacobian(1, 6) = _tmp119*(-_tmp103*(-_tmp15*_tmp195 - _tmp197*_tmp31 + _tmp198*_tmp88 - _tmp200*_tmp82 + _tmp76*theta_pos(4, 0)) + _tmp115*_tmp199 - _tmp118*_tmp199);
        _jacobian(2, 6) = _tmp120*(-_tmp103*(_tmp123*_tmp198 - _tmp126*theta_pos(4, 0) + _tmp127*theta_pos(4, 0) + _tmp15*_tmp197 - _tmp195*_tmp31) + _tmp121*_tmp199 - _tmp122*_tmp199);
        _jacobian(3, 6) = _tmp130*(-_tmp129*(_tmp132*_tmp198 + _tmp134*theta_pos(4, 0) - _tmp135*theta_pos(4, 0) + _tmp14*_tmp197 + _tmp195*_tmp54) - _tmp131*_tmp199 + _tmp136*_tmp199);
        _jacobian(0, 7) = -_tmp17*theta_pos(4, 0);
        _jacobian(1, 7) = _tmp119*(-_tmp103*(_tmp145*theta_pos(4, 0) - _tmp15*_tmp201 - _tmp192*_tmp31 + _tmp194*_tmp31) + _tmp115*_tmp202 - _tmp118*_tmp202);
        _jacobian(2, 7) = _tmp120*(-_tmp103*(_tmp144*_tmp196 + _tmp15*_tmp192 - _tmp15*_tmp194 - _tmp203*logR(2, 0)) + _tmp121*_tmp202 - _tmp122*_tmp202);
        _jacobian(3, 7) = _tmp130*(-_tmp129*(_tmp14*_tmp192 - _tmp14*_tmp194 + _tmp144*_tmp200 + _tmp203*logR(1, 0)) - _tmp131*_tmp202 + _tmp136*_tmp202);
        _jacobian(0, 8) = -_tmp28*theta_pos(5, 0);
        _jacobian(1, 8) = _tmp119*(-_tmp103*(-_tmp15*_tmp208 + _tmp150*theta_pos(5, 0) + _tmp161*_tmp204 - _tmp207*_tmp210) + _tmp115*_tmp209 - _tmp118*_tmp209);
        _jacobian(2, 8) = _tmp120*(-_tmp103*(_tmp171*_tmp204 + _tmp172*_tmp206 - _tmp175*theta_pos(5, 0) - _tmp208*_tmp31) + _tmp121*_tmp209 - _tmp122*_tmp209);
        _jacobian(3, 8) = _tmp130*(-_tmp129*(_tmp176*_tmp204 - _tmp178*theta_pos(5, 0) + _tmp207*_tmp211 + _tmp208*_tmp54) - _tmp131*_tmp209 + _tmp136*_tmp209);
        _jacobian(0, 9) = _tmp0*(_tmp214 - _tmp216*_tmp26 - _tmp22 + _tmp233 + _tmp27) + _tmp18*(_tmp215 + _tmp218 - _tmp219 - _tmp222 + _tmp223 - _tmp225) + _tmp29*(-_tmp20*_tmp226 - _tmp213*_tmp229 + _tmp227*_tmp229 + _tmp228*_tmp3 - _tmp230*logR(0, 0));
        _jacobian(1, 9) = _tmp119*(-_tmp103*(-_tmp1*_tmp244 - _tmp243 + _tmp246 + _tmp247 + _tmp248*_tmp26 + (Scalar(1)/Scalar(2))*_tmp53) + _tmp115*_tmp242 - _tmp118*_tmp242);
        _jacobian(2, 9) = _tmp120*(-_tmp103*(_tmp14*_tmp249 + _tmp250 - _tmp251 + _tmp252) + _tmp121*_tmp242 - _tmp122*_tmp242);
        _jacobian(3, 9) = _tmp130*(-_tmp129*(_tmp1*_tmp237 - _tmp235*_tmp26 - _tmp253 + _tmp255 + _tmp256) - _tmp131*_tmp242 + _tmp136*_tmp242);
        _jacobian(0, 10) = _tmp0*(-_tmp218 + _tmp219 + _tmp223 + _tmp225 - _tmp257 + _tmp258) + _tmp18*(_tmp216*_tmp24 + _tmp22 - _tmp230 + _tmp233 - _tmp25) + _tmp29*(_tmp1*_tmp259 - _tmp10*_tmp226 - _tmp213*_tmp260 - _tmp214*logR(1, 0) + _tmp227*_tmp260);
        _jacobian(1, 10) = _tmp119*(-_tmp103*(_tmp252 + _tmp263 + (Scalar(1)/Scalar(2))*_tmp70) + _tmp115*_tmp261 - _tmp118*_tmp261);
        _jacobian(2, 10) = _tmp120*(-_tmp103*(_tmp24*_tmp248 - _tmp244*_tmp3 + _tmp247 + _tmp249*_tmp54 - _tmp264 + _tmp265) + _tmp121*_tmp261 - _tmp122*_tmp261);
        _jacobian(3, 10) = _tmp130*(-_tmp129*(-_tmp236 + _tmp238 + _tmp24*_tmp240 - _tmp241*_tmp3 + _tmp266 + _tmp52) - _tmp131*_tmp261 + _tmp136*_tmp261);
        _jacobian(0, 11) = _tmp0*(_tmp10*_tmp267 - _tmp2*_tmp259 + _tmp2*_tmp268 + _tmp21 - _tmp216*_tmp245 + _tmp232) + _tmp18*(_tmp11*_tmp216 - _tmp12 + _tmp2*_tmp213*logR(0, 0) - _tmp2*_tmp228 + _tmp20*_tmp267 - _tmp268*logR(2, 0)) + _tmp29*(-_tmp215 + _tmp222 + _tmp257 - _tmp258);
        _jacobian(1, 11) = _tmp119*(-_tmp103*(-_tmp2*_tmp237 + _tmp235*_tmp269 + _tmp253 + _tmp256 + (Scalar(1)/Scalar(2))*_tmp67) + _tmp115*_tmp270 - _tmp118*_tmp270);
        _jacobian(2, 11) = _tmp120*(-_tmp103*(_tmp2*_tmp241 - _tmp240*_tmp269 + _tmp249*_tmp31 + _tmp266 - _tmp52) + _tmp121*_tmp270 - _tmp122*_tmp270);
        _jacobian(3, 11) = _tmp130*(-_tmp129*(-_tmp2*_tmp244 + _tmp243 - _tmp246 + _tmp247 + _tmp248*_tmp269 + _tmp264 - _tmp265) - _tmp131*_tmp270 + _tmp136*_tmp270);
        _jacobian(0, 12) = 0;
        _jacobian(1, 12) = -_tmp119*theta_rot(2, 0);
        _jacobian(2, 12) = 0;
        _jacobian(3, 12) = 0;
        _jacobian(0, 13) = 0;
        _jacobian(1, 13) = 0;
        _jacobian(2, 13) = -_tmp120*theta_rot(2, 0);
        _jacobian(3, 13) = 0;
        _jacobian(0, 14) = 0;
        _jacobian(1, 14) = 0;
        _jacobian(2, 14) = 0;
        _jacobian(3, 14) = -_tmp130*theta_rot(3, 0);
        _jacobian(0, 15) = -_tmp17*ierr(1, 0) - _tmp23*ierr(0, 0);
        _jacobian(1, 15) = _tmp119*(-_tmp103*(-_tmp132*_tmp274 - _tmp14*_tmp286 - _tmp15*_tmp281 - _tmp279*_tmp31 + _tmp283*_tmp31) + _tmp115*_tmp282 - _tmp118*_tmp282);
        _jacobian(2, 15) = _tmp120*(-_tmp103*(_tmp108*_tmp274 - _tmp273*_tmp287 - _tmp273*_tmp288 - _tmp281*_tmp31 + _tmp66*ierr(0, 0)) + _tmp121*_tmp282 - _tmp122*_tmp282);
        _jacobian(3, 15) = _tmp130*(-_tmp129*(_tmp14*_tmp279 - _tmp14*_tmp283 + _tmp274*_tmp88 + _tmp281*_tmp54 - _tmp286*_tmp31) - _tmp131*_tmp282 + _tmp136*_tmp282);
        _jacobian(0, 16) = -_tmp28*ierr(2, 0);
        _jacobian(1, 16) = _tmp119*(-_tmp103*(-_tmp15*_tmp292 - _tmp157*_tmp295 + _tmp161*_tmp290 + _tmp294*_tmp31) + _tmp115*_tmp293 - _tmp118*_tmp293);
        _jacobian(2, 16) = _tmp120*(-_tmp103*(_tmp171*_tmp290 + _tmp173*_tmp290 - _tmp174*_tmp291 - _tmp296*logR(2, 0)) + _tmp121*_tmp293 - _tmp122*_tmp293);
        _jacobian(3, 16) = _tmp130*(-_tmp129*(-_tmp14*_tmp294 + _tmp160*_tmp295 + _tmp177*_tmp290 + _tmp296*logR(1, 0)) - _tmp131*_tmp293 + _tmp136*_tmp293);
        _jacobian(0, 17) = _tmp17*_tmp298 + _tmp23*_tmp297;
        _jacobian(1, 17) = _tmp119*(-_tmp103*(-_tmp132*_tmp300 - _tmp14*_tmp302 - _tmp15*_tmp303 + _tmp301*_tmp31 + _tmp304*_tmp31) + _tmp115*_tmp305 - _tmp118*_tmp305);
        _jacobian(2, 17) = _tmp120*(-_tmp103*(_tmp109*_tmp299 - _tmp287*_tmp299 - _tmp288*_tmp299 - _tmp297*_tmp66 - _tmp306*logR(2, 0)) + _tmp121*_tmp305 - _tmp122*_tmp305);
        _jacobian(3, 17) = _tmp130*(-_tmp129*(-_tmp14*_tmp301 - _tmp14*_tmp304 + _tmp300*_tmp88 - _tmp302*_tmp31 + _tmp306*logR(1, 0)) - _tmp131*_tmp305 + _tmp136*_tmp305);
        _jacobian(0, 18) = _tmp28*_tmp307;
        _jacobian(1, 18) = _tmp119*(-_tmp103*(-_tmp15*_tmp310 - _tmp150*_tmp307 + _tmp161*_tmp308 - _tmp309*_tmp31) + _tmp115*_tmp312 - _tmp118*_tmp312);
        _jacobian(2, 18) = _tmp120*(-_tmp103*(_tmp173*_tmp308 + _tmp175*_tmp307 - _tmp311*logR(2, 0) + _tmp313*_tmp54) + _tmp121*_tmp312 - _tmp122*_tmp312);
        _jacobian(3, 18) = _tmp130*(-_tmp129*(_tmp177*_tmp308 + _tmp178*_tmp307 + _tmp31*_tmp313 + _tmp311*logR(1, 0)) - _tmp131*_tmp312 + _tmp136*_tmp312);
        _jacobian(0, 19) = _tmp17*_tmp315 + _tmp23*_tmp314;
        _jacobian(1, 19) = _tmp119*(-_tmp103*(-_tmp14*_tmp324 - _tmp15*_tmp321 - _tmp210*_tmp317 + _tmp210*_tmp319 + _tmp31*_tmp318) + _tmp115*_tmp323 - _tmp118*_tmp323);
        _jacobian(2, 19) = _tmp120*(-_tmp103*(_tmp109*_tmp316 - _tmp288*_tmp316 - _tmp314*_tmp66 - _tmp324*_tmp54 - _tmp325*logR(2, 0)) + _tmp121*_tmp323 - _tmp122*_tmp323);
        _jacobian(3, 19) = _tmp130*(-_tmp129*(-_tmp14*_tmp318 + _tmp18*_tmp316*_tmp88 - _tmp211*_tmp319 - _tmp31*_tmp324 + _tmp325*logR(1, 0)) - _tmp131*_tmp323 + _tmp136*_tmp323);
        _jacobian(0, 20) = _tmp28*_tmp326;
        _jacobian(1, 20) = _tmp119*(-_tmp103*(_tmp14*_tmp331 - _tmp15*_tmp328 - _tmp150*_tmp326 - _tmp158*_tmp327) + _tmp115*_tmp330 - _tmp118*_tmp330);
        _jacobian(2, 20) = _tmp120*(-_tmp103*(_tmp173*_tmp327 + _tmp175*_tmp326 - _tmp329*logR(2, 0) + _tmp331*_tmp54) + _tmp121*_tmp330 - _tmp122*_tmp330);
        _jacobian(3, 20) = _tmp130*(-_tmp129*(_tmp177*_tmp327 + _tmp178*_tmp326 + _tmp31*_tmp331 + _tmp329*logR(1, 0)) - _tmp131*_tmp330 + _tmp136*_tmp330);
        _jacobian(0, 21) = 0;
        _jacobian(1, 21) = -_tmp119*_tmp128*_tmp51;
        _jacobian(2, 21) = -_tmp120*_tmp128*_tmp68;
        _jacobian(3, 21) = 0;
        _jacobian(0, 22) = 0;
        _jacobian(1, 22) = 0;
        _jacobian(2, 22) = 0;
        _jacobian(3, 22) = -_tmp128*_tmp130*_tmp71;
        _jacobian(0, 23) = 0;
        _jacobian(1, 23) = _tmp119*(-w(0, 0) + w_d(0, 0));
        _jacobian(2, 23) = _tmp120*(-w(1, 0) + w_d(1, 0));
        _jacobian(3, 23) = 0;
        _jacobian(0, 24) = 0;
        _jacobian(1, 24) = 0;
        _jacobian(2, 24) = 0;
        _jacobian(3, 24) = _tmp130*(-w(2, 0) + w_d(2, 0));
    }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym

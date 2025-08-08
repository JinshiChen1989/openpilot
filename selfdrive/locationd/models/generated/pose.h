#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_5871344221582215751);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_241695378812077950);
void pose_H_mod_fun(double *state, double *out_6164610465264093335);
void pose_f_fun(double *state, double dt, double *out_1969944452673306282);
void pose_F_fun(double *state, double dt, double *out_381226404886081050);
void pose_h_4(double *state, double *unused, double *out_6288155756564553297);
void pose_H_4(double *state, double *unused, double *out_5410449463209380728);
void pose_h_10(double *state, double *unused, double *out_8113307794108590847);
void pose_H_10(double *state, double *unused, double *out_781315721660582811);
void pose_h_13(double *state, double *unused, double *out_5617290323579445144);
void pose_H_13(double *state, double *unused, double *out_2200181745107320201);
void pose_h_14(double *state, double *unused, double *out_8750371077921366803);
void pose_H_14(double *state, double *unused, double *out_1447208606869896199);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
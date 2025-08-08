#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_3598480170507539390);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3464032024592457063);
void car_H_mod_fun(double *state, double *out_6792944004458032914);
void car_f_fun(double *state, double dt, double *out_2318859909626675730);
void car_F_fun(double *state, double dt, double *out_5702720615743298578);
void car_h_25(double *state, double *unused, double *out_4888315426605084309);
void car_H_25(double *state, double *unused, double *out_3732462036277658897);
void car_h_24(double *state, double *unused, double *out_8875778684049224946);
void car_H_24(double *state, double *unused, double *out_2921743421830526860);
void car_h_30(double *state, double *unused, double *out_8884877201835593290);
void car_H_30(double *state, double *unused, double *out_8260158366405267095);
void car_h_26(double *state, double *unused, double *out_2485835684421744144);
void car_H_26(double *state, double *unused, double *out_7473965355151715121);
void car_h_27(double *state, double *unused, double *out_2283910105796386658);
void car_H_27(double *state, double *unused, double *out_8011822395503859610);
void car_h_29(double *state, double *unused, double *out_2126723437445257513);
void car_H_29(double *state, double *unused, double *out_7749927022090874911);
void car_h_28(double *state, double *unused, double *out_4972233611297065051);
void car_H_28(double *state, double *unused, double *out_5786296750525548660);
void car_h_31(double *state, double *unused, double *out_1267131769235575820);
void car_H_31(double *state, double *unused, double *out_8100173457385066597);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
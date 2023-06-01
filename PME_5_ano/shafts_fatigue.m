%% Dimensionamento dos veios à fadiga

clc;
clear all;
close all;

gear_shafts;

%% Veio 1 - FADIGA


idx = 1;

% Carateristicas para o material CK45 - Revenido

teta_rotura(idx) = 650; % MPa
teta_cedencia(idx) = 430; % MPa
teta_lim_fad(idx) = 0.5*teta_rotura(idx); % MPa

d_min(idx) = 0.035;

Kas(idx) = 0.9; % Retificado, fabrico comercial
Ks(idx) = 1.189*(d_min(idx)*10^3)^(-0.097);

teta_lim_fad_c(idx) = Kas(idx)*Ks(idx)*teta_lim_fad(idx);

s = 1; % 23mm - r = 0.5

Mfletor_XY(s) = 46;
Mfletor_ZY(s) = 169;

Mfletor(s) = sqrt((Mfletor_ZY(s)^2)+Mfletor_XY(s)^2);

teta_a(s) = ((32*Mfletor(s))/(pi*d_min(idx)^3))*10^-6;% MPa
teta_m(s) = ((4*Fx(idx))/(pi*d_min(idx)^2))*10^-6; % MPa
tao_m(s) = ((16*M_torsor(idx))/(pi*d_min(idx)^3))*10^-6; % MPa

teta_max(s) = max([teta_a(s) teta_m(s) tao_m(s)]);

r_conc1(s) = 0.0005;

d_sec1(s) = 0.036;

q1(s) = 0.6;
razao1_r_d(s) = r_conc1(s)/d_sec1(s);
razao1_D_d(s) = d_sec1(s)/d_min(idx);
Kt1(s) = 2.4;


Kf1(s) = 1 + q1(s)*(Kt1(s)-1);

teta_f_max1(s) = Kf1(s)*teta_max(s);

tensao_est_eq_normal1(s) = teta_m(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max1(s);

o_von_mises1(s) = sqrt((tensao_est_eq_normal1(s)^2)+3*(tao_m(s)^2));

FS1(s) = teta_cedencia(idx)/o_von_mises1(s);

s = 2; % 58mm - sem raio de concordância

d_sec1(s) = 0.036;

teta_a(s) = ((32*M_fletor(idx))/(pi*d_sec1(s)^3))*10^-6;% MPa
teta_m(s) = ((4*Fx(idx))/(pi*d_sec1(s)^2))*10^-6; % MPa
tao_m(s) = ((16*M_torsor(idx))/(pi*d_sec1(s)^3))*10^-6; % MPa

teta_max(s) = max([teta_a(s) teta_m(s) tao_m(s)]);

r_conc1(s) = 0;

q1(s) = 0;
razao1_r_d(s) = r_conc1(s)/d_sec1(s);
Kt1_flexao(s) = 1;
Kt1_torsao(s) = 3.5;

Kf1(s) = 1 + q1(s)*(Kt1_flexao(s)-1);

teta_f_max1(s) = Kf1(s)*teta_max(s);

tensao_est_eq_normal1(s) = teta_m(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max1(s);

tensao_corte_eq1(s) = Kt1_torsao(s)*tao_m(s);

o_von_mises1(s) = sqrt((tensao_est_eq_normal1(s)^2)+3*(tensao_corte_eq1(s)^2));

FS1(s) = teta_cedencia(idx)/o_von_mises1(s);


s = 3; % 65.5mm - raio conc 2

Mfletor_XY(s) = 227;
Mfletor_ZY(s) = 310;

Mfletor(s) = sqrt((Mfletor_ZY(s)^2)+Mfletor_XY(s)^2);

d_sec1(s) = 0.042;

teta_a(s) = ((32*Mfletor(s))/(pi*d_sec1(s-1)^3))*10^-6;% MPa
teta_m(s) = ((4*Fx(idx))/(pi*d_sec1(s-1)^2))*10^-6; % MPa
tao_m(s) = ((16*M_torsor(idx))/(pi*d_sec1(s-1)^3))*10^-6; % MPa

teta_max(s) = max([teta_a(s) teta_m(s) tao_m(s)]);

r_conc1(s) = 0.002;

q1(s) = 0.9;
razao1_r_d(s) = r_conc1(s)/d_sec1(s);
razao1_D_d(s) = d_sec1(s)/d_sec1(s-1);
Kt1(s) = 2.15;


Kf1(s) = 1 + q1(s)*(Kt1(s)-1);

teta_f_max1(s) = Kf1(s)*teta_max(s);

tensao_est_eq_normal1(s) = teta_m(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max1(s);

o_von_mises1(s) = sqrt((tensao_est_eq_normal1(s)^2)+3*(tao_m(s)^2));

FS1(s) = teta_cedencia(idx)/o_von_mises1(s); 

s = 4; % 178mm - r = 2

Mfletor_XY(s) = 31;
Mfletor_ZY(s) = 43;

Mfletor(s) = sqrt((Mfletor_ZY(s)^2)+Mfletor_XY(s)^2);

teta_a(s) = ((32*Mfletor(s))/(pi*d_min(idx)^3))*10^-6;% MPa
teta_m(s) = ((4*Fx(idx))/(pi*d_min(idx)^2))*10^-6; % MPa
tao_m(s) = ((16*M_torsor(idx))/(pi*d_min(idx)^3))*10^-6; % MPa

teta_max(s) = max([teta_a(s) teta_m(s) tao_m(s)]);

r_conc1(s) = 0.002;

d_sec1(s) = 0.035;

q1(s) = 0.9;
razao1_r_d(s) = r_conc1(s)/d_sec1(s);
razao1_D_d(s) = d_sec1(s-1)/d_sec1(s);
Kt1(s) = 2.0;


Kf1(s) = 1 + q1(s)*(Kt1(s)-1);

teta_f_max1(s) = Kf1(s)*teta_max(s);

tensao_est_eq_normal1(s) = teta_m(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max1(s);

o_von_mises1(s) = sqrt((tensao_est_eq_normal1(s)^2)+3*(tao_m(s)^2));

FS1(s) = teta_cedencia(idx)/o_von_mises1(s);


%% Veio 2 - FADIGA

idx = 2;

% Carateristicas para o material 34 Cr 4

teta_rotura(idx) = 800; % MPa
teta_cedencia(idx) = 590; % MPa
teta_lim_fad(idx) = 0.5*teta_rotura(idx); % MPa

d_min(idx) = 0.035;

Kas(idx) = 0.9; % Retificado, fabrico comercial
Ks(idx) = 1.189*(d_min(idx)*10^3)^(-0.097);

teta_lim_fad_c(idx) = Kas(idx)*Ks(idx)*teta_lim_fad(idx);

s = 1; % 42.5mm - raio conc 2

Mfletor_XY2(s) = 3;
Mfletor_ZY2(s) = 62;

Mfletor2(s) = sqrt((Mfletor_ZY2(s)^2)+Mfletor_XY2(s)^2);

teta_a2(s) = ((32*Mfletor2(s))/(pi*d_min(idx)^3))*10^-6;% MPa
teta_m2(s) = ((4*Fx(idx))/(pi*d_min(idx)^2))*10^-6; % MPa
tao_m2(s) = ((16*M_torsor(idx))/(pi*d_min(idx)^3))*10^-6; % MPa

teta_max2(s) = max([teta_a2(s) teta_m2(s) tao_m2(s)]);

r_conc2(s) = 0.002;

d_sec2(s) = 0.042;

q2(s) = 0.9;
razao2_r_d(s) = r_conc2(s)/d_min(idx);
razao2_D_d(s) = d_sec2(s)/d_min(idx);
Kt2(s) = 2.0;

Kf2(s) = 1 + q2(s)*(Kt2(s)-1);

teta_f_max2(s) = Kf2(s)*teta_max2(s);

tensao_est_eq_normal2(s) = teta_m2(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max2(s);

o_von_mises2(s) = sqrt((tensao_est_eq_normal2(s)^2)+3*(tao_m2(s)^2));

FS2(s) = teta_cedencia(idx)/o_von_mises2(s); 


s = 2; % 25mm - sem raio de concordância

d_sec2(s) = 0.048;

teta_a2(s) = ((32*M_fletor(idx-1))/(pi*d_sec2(s-1)^3))*10^-6;% MPa
teta_m2(s) = ((4*Fx(idx-1))/(pi*d_sec2(s-1)^2))*10^-6; % MPa
tao_m2(s) = ((16*M_torsor(idx))/(pi*d_sec2(s-1)^3))*10^-6; % MPa

teta_max2(s) = max([teta_a2(s) teta_m2(s) tao_m2(s)]);

r_conc2(s) = 0;

Kas2(s) = 0.9; % Retificado, fabrico comercial
Ks2(s) = 1.189*(d_sec2(s)*10^3)^(-0.097);

q2(s) = 0;
razao2_r_d(s) = r_conc2(s)/d_sec2(s);
Kt2_flexao(s) = 1;
Kt2_torsao(s) = 3.5;

Kf2(s) = 1 + q2(s)*(Kt2_flexao(s)-1);

teta_f_max2(s) = Kf2(s)*teta_max2(s);

tensao_est_eq_normal2(s) = teta_m2(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max2(s);

tensao_corte_eq2(s) = Kt2_torsao(s) * tao_m2(s);

o_von_mises2(s) = sqrt((tensao_est_eq_normal2(s)^2)+3*(tensao_corte_eq2(s)^2));

FS2(s) = teta_cedencia(idx)/o_von_mises2(s);


s = 3; % 42.5mm - raio conc 2

Mfletor_XY2(s) = 568;
Mfletor_ZY2(s) = 46;

Mfletor2(s) = sqrt((Mfletor_ZY2(s)^2)+Mfletor_XY2(s)^2);

d_sec2(s) = 0.042;

teta_a2(s) = ((32*Mfletor2(s))/(pi*d_sec2(s)^3))*10^-6;% MPa
teta_m2(s) = ((4*Fx(idx))/(pi*d_sec2(s)^2))*10^-6; % MPa
tao_m2(s) = ((16*M_torsor(idx))/(pi*d_sec2(s)^3))*10^-6; % MPa

teta_max2(s) = max([teta_a2(s) teta_m2(s) tao_m2(s)]);

r_conc2(s) = 0.002;

q2(s) = 0.9;
razao2_r_d(s) = r_conc2(s)/d_min(idx);
razao2_D_d(s) = d_sec2(s)/d_sec2(s-1);
Kt2(s) = 2.1;

Kf2(s) = 1 + q2(s)*(Kt2(s)-1);

teta_f_max2(s) = Kf2(s)*teta_max2(s);

tensao_est_eq_normal2(s) = teta_m2(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max2(s);

o_von_mises2(s) = sqrt((tensao_est_eq_normal2(s)^2)+3*(tao_m2(s)^2));

FS2(s) = teta_cedencia(idx)/o_von_mises2(s); 

s = 4; % 87.5mm - raio conc 2

Mfletor_XY2(s) = 391;
Mfletor_ZY2(s) = 227;

Mfletor2(s) = sqrt((Mfletor_ZY2(s)^2)+Mfletor_XY2(s)^2);

d_sec2(s) = 0.042;

teta_a2(s) = ((32*Mfletor2(s))/(pi*d_sec2(s)^3))*10^-6;% MPa
teta_m2(s) = ((4*Fx(idx))/(pi*d_sec2(s)^2))*10^-6; % MPa
tao_m2(s) = ((16*M_torsor(idx))/(pi*d_sec2(s)^3))*10^-6; % MPa

teta_max2(s) = max([teta_a2(s) teta_m2(s) tao_m2(s)]);

r_conc2(s) = 0.002;

q2(s) = 0.9;
razao2_r_d(s) = r_conc2(s)/d_min(idx);
razao2_D_d(s) = d_sec2(s-1)/d_sec2(s);
Kt2(s) = 2.1;

Kf2(s) = 1 + q2(s)*(Kt2(s)-1);

teta_f_max2(s) = Kf2(s)*teta_max2(s);

tensao_est_eq_normal2(s) = teta_m2(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max2(s);

o_von_mises2(s) = sqrt((tensao_est_eq_normal2(s)^2)+3*(tao_m2(s)^2));

FS2(s) = teta_cedencia(idx)/o_von_mises2(s); 

s = 5; % 117mm - sem raio de concordância

d_sec2(s) = 0.042;

teta_a2(s) = ((32*M_fletor(idx))/(pi*d_sec2(s)^3))*10^-6;% MPa
teta_m2(s) = ((4*Fx(idx))/(pi*d_sec2(s)^2))*10^-6; % MPa
tao_m2(s) = ((16*M_torsor(idx))/(pi*d_sec2(s)^3))*10^-6; % MPa

teta_max2(s) = max([teta_a2(s) teta_m2(s) tao_m2(s)]);

r_conc2(s) = 0;

Kas2(s) = 0.9; % Retificado, fabrico comercial
Ks2(s) = 1.189*(d_sec2(s)*10^3)^(-0.097);

q2(s) = 0;
razao2_r_d(s) = r_conc2(s)/d_sec2(s);
Kt2_flexao(s) = 1;
Kt2_torsao(s) = 3.5;

Kf2(s) = 1 + q2(s)*(Kt2_flexao(s)-1);

teta_f_max2(s) = Kf2(s)*teta_max2(s);

tensao_est_eq_normal2(s) = teta_m2(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max2(s);

tensao_corte_eq2(s) = Kt2_torsao(s) * tao_m2(s);

o_von_mises2(s) = sqrt((tensao_est_eq_normal2(s)^2)+3*(tensao_corte_eq2(s)^2));

FS2(s) = teta_cedencia(idx)/o_von_mises2(s);

s = 6; % 42.5mm - raio conc 2

Mfletor_XY2(s) = 23;
Mfletor_ZY2(s) = 128;

Mfletor2(s) = sqrt((Mfletor_ZY2(s)^2)+Mfletor_XY2(s)^2);

teta_a2(s) = ((32*Mfletor2(s))/(pi*d_min(idx)^3))*10^-6;% MPa
teta_m2(s) = ((4*Fx(idx))/(pi*d_min(idx)^2))*10^-6; % MPa
tao_m2(s) = ((16*M_torsor(idx))/(pi*d_min(idx)^3))*10^-6; % MPa

teta_max2(s) = max([teta_a2(s) teta_m2(s) tao_m2(s)]);

r_conc2(s) = 0.002;

d_sec2(s) = 0.035;

q2(s) = 0.9;
razao2_r_d(s) = r_conc2(s)/d_min(idx);
razao2_D_d(s) = d_sec2(s-1)/d_sec2(s);
Kt2(s) = 2.0;

Kf2(s) = 1 + q2(s)*(Kt2(s)-1);

teta_f_max2(s) = Kf2(s)*teta_max2(s);

tensao_est_eq_normal2(s) = teta_m2(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max2(s);

o_von_mises2(s) = sqrt((tensao_est_eq_normal2(s)^2)+3*(tao_m2(s)^2));

FS2(s) = teta_cedencia(idx)/o_von_mises2(s); 

%% Veio 3 - FADIGA

idx = 3;

% Carateristicas para o material 30 CrNIMo 8

teta_rotura(idx) = 1100; % MPa
teta_cedencia(idx) = 900; % MPa
teta_lim_fad(idx) = 0.5*teta_rotura(idx); % MPa

d_min(idx) = 0.040;

Kas(idx) = 0.9; % Retificado, fabrico comercial
Ks(idx) = 1.189*(d_min(idx)*10^3)^(-0.097);

teta_lim_fad_c(idx) = Kas(idx)*Ks(idx)*teta_lim_fad(idx);

s = 1; % 42.5mm - raio conc 2

Mfletor_XY3(s) = 50;
Mfletor_ZY3(s) = 73;

Mfletor3(s) = sqrt((Mfletor_ZY3(s)^2)+Mfletor_XY3(s)^2);

teta_a3(s) = ((32*Mfletor3(s))/(pi*d_min(idx)^3))*10^-6;% MPa
teta_m3(s) = ((4*Fx(idx))/(pi*d_min(idx)^2))*10^-6; % MPa
tao_m3(s) = ((16*M_torsor(idx))/(pi*d_min(idx)^3))*10^-6; % MPa

teta_max3(s) = max([teta_a3(s) teta_m3(s) tao_m3(s)]);

r_conc3(s) = 0.002;

d_sec3(s) = 0.046;

q3(s) = 0.9;
razao3_r_d(s) = r_conc3(s)/d_min(idx);
razao3_D_d(s) = d_sec3(s)/d_min(idx);
Kt3(s) = 1.9;

Kf3(s) = 1 + q3(s)*(Kt3(s)-1);

teta_f_max3(s) = Kf3(s)*teta_max3(s);

tensao_est_eq_normal3(s) = teta_m3(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max3(s);

o_von_mises3(s) = sqrt((tensao_est_eq_normal3(s)^2)+3*(tao_m3(s)^2));

FS3(s) = teta_cedencia(idx)/o_von_mises3(s);

s = 2; % 25mm - sem raio de concordância

d_sec3(s) = 0.050;

teta_a3(s) = ((32*M_fletor(idx-1))/(pi*d_sec3(s)^3))*10^-6;% MPa
teta_m3(s) = ((4*Fx(idx-1))/(pi*d_sec3(s)^2))*10^-6; % MPa
tao_m3(s) = ((16*M_torsor(idx))/(pi*d_sec3(s)^3))*10^-6; % MPa

teta_max3(s) = max([teta_a3(s) teta_m3(s) tao_m3(s)]);

r_conc3(s) = 0;

Kas3(s) = 0.9; % Retificado, fabrico comercial
Ks3(s) = 1.189*(d_sec3(s)*10^3)^(-0.097);

q3(s) = 0;
razao3_r_d(s) = r_conc3(s)/d_sec3(s);
Kt3_flexao(s) = 1;
Kt3_torsao(s) = 3.5;

Kf3(s) = 1 + q3(s)*(Kt3_flexao(s)-1);

teta_f_max3(s) = Kf3(s)*teta_max3(s);

tensao_est_eq_normal3(s) = teta_m3(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max3(s);

tensao_corte_eq3(s) = Kt3_torsao(s) * tao_m3(s);

o_von_mises3(s) = sqrt((tensao_est_eq_normal3(s)^2)+3*(tensao_corte_eq3(s)^2));

FS3(s) = teta_cedencia(idx)/o_von_mises3(s);


s = 3; % 42.5mm - raio conc 2

Mfletor_XY3(s) = 1182;
Mfletor_ZY3(s) = 101;

Mfletor3(s) = sqrt((Mfletor_ZY3(s)^2)+Mfletor_XY3(s)^2);

d_sec3(s) = 0.058;

teta_a3(s) = ((32*Mfletor3(s))/(pi*d_sec3(s-1)^3))*10^-6;% MPa
teta_m3(s) = ((4*Fx(idx))/(pi*d_sec3(s-1)^2))*10^-6; % MPa
tao_m3(s) = ((16*M_torsor(idx))/(pi*d_sec3(s-1)^3))*10^-6; % MPa

teta_max3(s) = max([teta_a3(s) teta_m3(s) tao_m3(s)]);

r_conc3(s) = 0.002;

q3(s) = 0.8;
razao3_r_d(s) = r_conc3(s)/d_min(idx);
razao3_D_d(s) = d_sec3(s)/d_sec3(s-1);
Kt3(s) = 2.2;

Kf3(s) = 1 + q3(s)*(Kt3(s)-1);

teta_f_max3(s) = Kf3(s)*teta_max3(s);

tensao_est_eq_normal3(s) = teta_m3(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max3(s);

o_von_mises3(s) = sqrt((tensao_est_eq_normal3(s)^2)+3*(tao_m3(s)^2));

FS3(s) = teta_cedencia(idx)/o_von_mises3(s); 

s = 4; % 87.5mm - raio conc 2

Mfletor_XY3(s) = 1168;
Mfletor_ZY3(s) = 54;

Mfletor3(s) = sqrt((Mfletor_ZY3(s)^2)+Mfletor_XY3(s)^2);

d_sec3(s) = 0.046;

teta_a3(s) = ((32*Mfletor3(s))/(pi*d_sec3(s)^3))*10^-6;% MPa
teta_m3(s) = ((4*Fx(idx))/(pi*d_sec3(s)^2))*10^-6; % MPa
tao_m3(s) = ((16*M_torsor(idx))/(pi*d_sec3(s)^3))*10^-6; % MPa

teta_max3(s) = max([teta_a3(s) teta_m3(s) tao_m3(s)]);

r_conc3(s) = 0.002;

q3(s) = 0.8;
razao3_r_d(s) = r_conc3(s)/d_min(idx);
razao3_D_d(s) = d_sec3(s-1)/d_sec3(s);
Kt3(s) = 2.2;

Kf3(s) = 1 + q3(s)*(Kt3(s)-1);

teta_f_max3(s) = Kf3(s)*teta_max3(s);

tensao_est_eq_normal3(s) = teta_m3(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max3(s);

o_von_mises3(s) = sqrt((tensao_est_eq_normal3(s)^2)+3*(tao_m3(s)^2));

FS3(s) = teta_cedencia(idx)/o_von_mises3(s); 

s = 5; % 117mm - sem raio de concordância

d_sec3(s) = 0.046;

teta_a3(s) = ((32*M_fletor(idx))/(pi*d_sec3(s)^3))*10^-6;% MPa
teta_m3(s) = ((4*Fx(idx))/(pi*d_sec3(s)^2))*10^-6; % MPa
tao_m3(s) = ((16*M_torsor(idx))/(pi*d_sec3(s)^3))*10^-6; % MPa

teta_max3(s) = max([teta_a3(s) teta_m3(s) tao_m3(s)]);

r_conc3(s) = 0;

Kas3(s) = 0.9; % Retificado, fabrico comercial
Ks3(s) = 1.189*(d_sec3(s)*10^3)^(-0.097);

q3(s) = 0;
razao3_r_d(s) = r_conc3(s)/d_sec3(s);
Kt3_flexao(s) = 1;
Kt3_torsao(s) = 3.5;

Kf3(s) = 1 + q3(s)*(Kt3_flexao(s)-1);

teta_f_max3(s) = Kf3(s)*teta_max3(s);

tensao_est_eq_normal3(s) = teta_m3(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max3(s);

tensao_corte_eq3(s) = Kt3_torsao(s) * tao_m3(s);

o_von_mises3(s) = sqrt((tensao_est_eq_normal3(s)^2)+3*(tensao_corte_eq3(s)^2));

FS3(s) = teta_cedencia(idx)/o_von_mises3(s);

s = 6; % 42.5mm - raio conc 2

Mfletor_XY3(s) = 45;
Mfletor_ZY3(s) = 78;

Mfletor3(s) = sqrt((Mfletor_ZY3(s)^2)+Mfletor_XY3(s)^2);

teta_a3(s) = ((32*Mfletor3(s))/(pi*d_min(idx)^3))*10^-6;% MPa
teta_m3(s) = ((4*Fx(idx))/(pi*d_min(idx)^2))*10^-6; % MPa
tao_m3(s) = ((16*M_torsor(idx))/(pi*d_min(idx)^3))*10^-6; % MPa

teta_max3(s) = max([teta_a3(s) teta_m3(s) tao_m3(s)]);

r_conc3(s) = 0.002;

d_sec3(s) = 0.040;

q3(s) = 0.9;
razao3_r_d(s) = r_conc3(s)/d_min(idx);
razao3_D_d(s) = d_sec3(s-1)/d_sec3(s);
Kt3(s) = 1.9;

Kf3(s) = 1 + q3(s)*(Kt3(s)-1);

teta_f_max3(s) = Kf3(s)*teta_max3(s);

tensao_est_eq_normal3(s) = teta_m3(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max3(s);

o_von_mises3(s) = sqrt((tensao_est_eq_normal3(s)^2)+3*(tao_m3(s)^2));

FS3(s) = teta_cedencia(idx)/o_von_mises3(s);

%% Veio 4 - FADIGA


idx = 4;

% Carateristicas para o material CK45 - Revenido

teta_rotura(idx) = 1250; % MPa
teta_cedencia(idx) = 1050; % MPa
teta_lim_fad(idx) = 0.5*teta_rotura(idx); % MPa

d_min(idx) = 0.045;

Kas(idx) = 0.9; % Retificado, fabrico comercial
Ks(idx) = 1.189*(d_min(idx)*10^3)^(-0.097);

teta_lim_fad_c(idx) = Kas(idx)*Ks(idx)*teta_lim_fad(idx);

s = 1; % 42.5mm - raio conc 2

Mfletor_XY(s) = 129;
Mfletor_ZY(s) = 155;

Mfletor(s) = sqrt((Mfletor_ZY(s)^2)+Mfletor_XY(s)^2);

teta_a4(s) = ((32*Mfletor(s))/(pi*d_min(idx)^3))*10^-6;% MPa
teta_m4(s) = ((4*Fx(idx-1))/(pi*d_min(idx)^2))*10^-6; % MPa
tao_m4(s) = ((16*M_torsor(idx))/(pi*d_min(idx)^3))*10^-6; % MPa

teta_max4(s) = max([teta_a4(s) teta_m4(s) tao_m4(s)]);

r_conc4(s) = 0.002;

d_sec4(s) = 0.052;

q4(s) = 0.9;
razao4_r_d(s) = r_conc4(s)/d_sec4(s);
razao4_D_d(s) = d_sec4(s)/d_min(idx);
Kt4(s) = 2.2;


Kf4(s) = 1 + q4(s)*(Kt4(s)-1);

teta_f_max4(s) = Kf4(s)*teta_max(s);

tensao_est_eq_normal4(s) = teta_m4(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max4(s);

o_von_mises4(s) = sqrt((tensao_est_eq_normal4(s)^2)+3*(tao_m4(s)^2));

FS4(s) = teta_cedencia(idx)/o_von_mises4(s); 


s = 2; % 25mm - sem raio de concordância

d_sec4(s) = 0.052;

teta_a4(s) = ((32*M_fletor(idx))/(pi*d_sec4(s)^3))*10^-6;% MPa
teta_m4(s) = ((4*Fx(idx-1))/(pi*d_sec4(s)^2))*10^-6; % MPa
tao_m4(s) = ((16*M_torsor(idx))/(pi*d_sec4(s)^3))*10^-6; % MPa

teta_max4(s) = max([teta_a4(s) teta_m4(s) tao_m4(s)]);

r_conc4(s) = 0;

q4(s) = 0;
razao4_r_d(s) = r_conc4(s)/d_sec4(s);
Kt4_flexao(s) = 1;
Kt4_torsao(s) = 3.5;

Kf4(s) = 1 + q4(s)*(Kt4_flexao(s)-1);

teta_f_max4(s) = Kf4(s)*teta_max4(s);

tensao_est_eq_normal4(s) = teta_m4(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max4(s);

tensao_corte_eq4(s) = Kt4_torsao(s)*tao_m4(s);

o_von_mises4(s) = sqrt((tensao_est_eq_normal4(s)^2)+3*(tensao_corte_eq4(s)^2));

FS4(s) = teta_cedencia(idx)/o_von_mises4(s);


s = 3; % 42.5mm - raio conc 2

Mfletor_XY(s) = 1215;
Mfletor_ZY(s) = 524;

Mfletor(s) = sqrt((Mfletor_ZY(s)^2)+Mfletor_XY(s)^2);

d_sec4(s) = 0.060;

teta_a4(s) = ((32*Mfletor(s))/(pi*d_sec4(s-1)^3))*10^-6;% MPa
teta_m4(s) = ((4*Fx(idx-1))/(pi*d_sec4(s-1)^2))*10^-6; % MPa
tao_m4(s) = ((16*M_torsor(idx))/(pi*d_sec4(s-1)^3))*10^-6; % MPa

teta_max4(s) = max([teta_a4(s) teta_m4(s) tao_m4(s)]);

r_conc4(s) = 0.002;

q4(s) = 0.9;
razao4_r_d(s) = r_conc4(s)/d_sec4(s);
razao4_D_d(s) = d_sec4(s)/d_sec4(s-1);
Kt4(s) = 2.2;


Kf4(s) = 1 + q4(s)*(Kt4(s)-1);

teta_f_max4(s) = Kf4(s)*teta_max(s);

tensao_est_eq_normal4(s) = teta_m4(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max4(s);

o_von_mises4(s) = sqrt((tensao_est_eq_normal4(s)^2)+3*(tao_m4(s)^2));

FS4(s) = teta_cedencia(idx)/o_von_mises4(s); 

s = 4; % 42.5mm - raio conc 2

Mfletor_XY(s) = 311;
Mfletor_ZY(s) = 134;

Mfletor(s) = sqrt((Mfletor_ZY(s)^2)+Mfletor_XY(s)^2);

teta_a4(s) = ((32*Mfletor(s))/(pi*d_min(idx)^3))*10^-6;% MPa
teta_m4(s) = ((4*Fx(idx-1))/(pi*d_min(idx)^2))*10^-6; % MPa
tao_m4(s) = ((16*M_torsor(idx))/(pi*d_min(idx)^3))*10^-6; % MPa

teta_max4(s) = max([teta_a4(s) teta_m4(s) tao_m4(s)]);

r_conc4(s) = 0.002;

d_sec4(s) = 0.052;

q4(s) = 0.9;
razao4_r_d(s) = r_conc4(s)/d_sec4(s);
razao4_D_d(s) = d_sec4(s)/d_min(idx);
Kt4(s) = 2.2;


Kf4(s) = 1 + q4(s)*(Kt4(s)-1);

teta_f_max4(s) = Kf4(s)*teta_max(s);

tensao_est_eq_normal4(s) = teta_m4(s) + (teta_cedencia(idx)/teta_lim_fad_c(idx))...
    *teta_f_max4(s);

o_von_mises4(s) = sqrt((tensao_est_eq_normal4(s)^2)+3*(tao_m4(s)^2));

FS4(s) = teta_cedencia(idx)/o_von_mises4(s);


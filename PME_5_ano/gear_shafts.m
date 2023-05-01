%% Cálculo das forças de engrenamento

clc;
clear all;
close all;

gear_defenition;

% Forças de engrenamento para o par Z1 e Z2 em N; a função v_veio serve
% para converter a a velocidade do veio de rpm para m/s

% Assume-se que existem forças axiais e escolheu-se rolamentos angulares de
% esferas como hipótese inicial.

idx = 1;

v_veio(idx) = 2*pi*(pinhao(idx).diametro_primitivo*0.001/2)*w_entrada/60;

M_torsor(idx) = torque_motor;

Ft(idx) = M_torsor(idx)/((pinhao(idx).diametro_primitivo*0.001)/2); %momento aplicado é o torque do motor
Fr(idx) = Ft(idx)*(tan(alpha)/cos(beta(idx)));
Fx(idx) = Ft(idx)*tan(beta(idx));
Mz_pinhao(idx) = Fx(idx)*(pinhao(idx).diametro_primitivo*0.001/2);
Mz_roda(idx) = Fx(idx)*(roda(idx).diametro_primitivo*0.001/2);

R_a_xy(idx) = 100;
R_b_xy(idx) = 3900;

R_a_zy(idx) = -3900;
R_b_zy(idx) = 5900;

Fr_rolamento_a(idx) = sqrt((R_a_xy(idx)^2)+R_a_zy(idx)^2);
Fr_rolamento_b(idx) = sqrt((R_b_xy(idx)^2)+R_b_zy(idx)^2);

    % Para este caso, optou-se por rolamentos de carreira simples com
    % contactoangular de 25º para o 1º e 2º apoio (inicialmente)

alpha_contacto_rol(idx) = deg2rad(25);
X0(idx) = 0.5;
Y0(idx) = 0.42;
razao_forcas_rol_a(idx) = Fx(idx)/Fr_rolamento_a(idx);
razao_forcas_rol_b(idx) = Fx(idx)/Fr_rolamento_b(idx);
ceof_carga_axial(idx) = 0.68;
P0_axial_a(idx) = 2.3*Fr_rolamento_a(idx)*tan(alpha_contacto_rol(idx))...
    + Fx(idx);
P0_axial_b(idx) = 2.3*Fr_rolamento_b(idx)*tan(alpha_contacto_rol(idx))...
    + Fx(idx);

if razao_forcas_rol_a > ceof_carga_axial
    X(idx) = 0.41;
    Y(idx) = 0.87;
else
    X(idx) = 1.0;
    Y(idx) = 0;
end

if razao_forcas_rol_b > ceof_carga_axial
    X(idx) = 0.41;
    Y(idx) = 0.87;
else
    X(idx) = 1.0;
    Y(idx) = 0;
end


% Forças de engrenamento para o par Z3 e Z4 em N

idx = 2;

v_veio(idx) = 2*pi*(pinhao(idx).diametro_primitivo*0.001/2)...
    *pinhao(idx).rotacao/60;

M_torsor(idx) = P_motor_catalogo/v_veio(idx);

Ft(idx) = M_torsor(idx)/((pinhao(idx).diametro_primitivo*0.001)/2);
Fr(idx) = Ft(idx)*(tan(alpha)/cos(beta(idx)));
Fx(idx) = Ft(idx)*tan(beta(idx));
Mz_pinhao(idx) = Fx(idx)*(pinhao(idx).diametro_primitivo*0.001/2);
Mz_roda(idx) = Fx(idx)*(roda(idx).diametro_primitivo*0.001/2);

R_a_xy(idx) = 700;
R_b_xy(idx) = 3400;

R_a_zy(idx) = 3900;
R_b_zy(idx) = 8800;

Fr_rolamento_a(idx) = sqrt((R_a_xy(idx)^2)+R_a_zy(idx)^2);
Fr_rolamento_b(idx) = sqrt((R_b_xy(idx)^2)+R_b_zy(idx)^2);

    % Para este caso, optou-se por rolamentos de carreira simples com
    % contactoangular de 25º para o 1º e 2º apoio (inicialmente)

alpha_contacto_rol(idx) = deg2rad(25);
X0(idx) = 0.5;
Y0(idx) = 0.42;
razao_forcas_rol_a(idx) = Fx(idx)/Fr_rolamento_a(idx);
razao_forcas_rol_b(idx) = Fx(idx)/Fr_rolamento_b(idx);
ceof_carga_axial(idx) = 0.68;
P0_axial_a(idx) = 2.3*Fr_rolamento_a(idx)*tan(alpha_contacto_rol(idx))...
    + Fx(idx)-Fx(idx-1);
P0_axial_b(idx) = 2.3*Fr_rolamento_b(idx)*tan(alpha_contacto_rol(idx))...
    + Fx(idx)-Fx(idx-1);

if razao_forcas_rol_a > ceof_carga_axial
    X(idx) = 0.41;
    Y(idx) = 0.87;
else
    X(idx) = 1.0;
    Y(idx) = 0;
end

if razao_forcas_rol_b > ceof_carga_axial
    X(idx) = 0.41;
    Y(idx) = 0.87;
else
    X(idx) = 1.0;
    Y(idx) = 0;
end

% Forças de engrenamento para o par Z5 e Z6 em N

idx = 3;

v_veio(idx) = 2*pi*(pinhao(idx).diametro_primitivo*0.001/2)*pinhao(idx).rotacao/60;

M_torsor(idx) = P_motor_catalogo/v_veio(idx);

Ft(idx) = M_torsor(idx)/((pinhao(idx).diametro_primitivo*0.001)/2);
Fr(idx) = Ft(idx)*(tan(alpha)/cos(beta(idx)));
Fx(idx) = Ft(idx)*tan(beta(idx));
Mz_pinhao(idx) = Fx(idx)*(pinhao(idx).diametro_primitivo*0.001/2);
Mz_roda(idx) = Fx(idx)*(roda(idx).diametro_primitivo*0.001/2);

R_a_xy(idx) = -7100;
R_b_xy(idx) = -5700;

R_a_zy(idx) = -3600;
R_b_zy(idx) = -1600;

Fr_rolamento_a(idx) = sqrt((R_a_xy(idx)^2)+R_a_zy(idx)^2);
Fr_rolamento_b(idx) = sqrt((R_b_xy(idx)^2)+R_b_zy(idx)^2);

    % Para este caso, optou-se por rolamentos de carreira simples com
    % contactoangular de 25º para o 1º e 2º apoio (inicialmente)

alpha_contacto_rol(idx) = deg2rad(25);
X0(idx) = 0.5;
Y0(idx) = 0.42;
razao_forcas_rol_a(idx) = Fx(idx)/Fr_rolamento_a(idx);
razao_forcas_rol_b(idx) = Fx(idx)/Fr_rolamento_b(idx);
ceof_carga_axial(idx) = 0.68;
P0_axial_a(idx) = 2.3*Fr_rolamento_a(idx)*tan(alpha_contacto_rol(idx))...
    + Fx(idx)-Fx(idx-1);
P0_axial_b(idx) = 2.3*Fr_rolamento_b(idx)*tan(alpha_contacto_rol(idx))...
    + Fx(idx)-Fx(idx-1);

if razao_forcas_rol_a > ceof_carga_axial
    X(idx) = 0.41;
    Y(idx) = 0.87;
else
    X(idx) = 1.0;
    Y(idx) = 0;
end

if razao_forcas_rol_b > ceof_carga_axial
    X(idx) = 0.41;
    Y(idx) = 0.87;
else
    X(idx) = 1.0;
    Y(idx) = 0;
end


%Ultimo veio

idx = 4;

R_a_xy(idx) = -25100;
R_b_xy(idx) = -33000;

R_a_zy(idx) = 3900;
R_b_zy(idx) = -12900;

Fr_rolamento_a(idx) = sqrt((R_a_xy(idx)^2)+R_a_zy(idx)^2);
Fr_rolamento_b(idx) = sqrt((R_b_xy(idx)^2)+R_b_zy(idx)^2);

    % Para este caso, optou-se por rolamentos de carreira simples com
    % contactoangular de 25º para o 1º e 2º apoio (inicialmente)

alpha_contacto_rol(idx) = deg2rad(25);
X0(idx) = 0.5;
Y0(idx) = 0.42;
razao_forcas_rol_a(idx) = Fx(idx)/Fr_rolamento_a(idx);
razao_forcas_rol_b(idx) = Fx(idx)/Fr_rolamento_b(idx);
ceof_carga_axial(idx) = 0.68;
P0_axial_a(idx) = 2.3*Fr_rolamento_a(idx)*tan(alpha_contacto_rol(idx))...
    + Fx(idx-1);
P0_axial_b(idx) = 2.3*Fr_rolamento_b(idx)*tan(alpha_contacto_rol(idx))...
    + Fx(idx-1);

if razao_forcas_rol_a > ceof_carga_axial
    X(idx) = 0.41;
    Y(idx) = 0.87;
else
    X(idx) = 1.0;
    Y(idx) = 0;
end

if razao_forcas_rol_b > ceof_carga_axial
    X(idx) = 0.41;
    Y(idx) = 0.87;
else
    X(idx) = 1.0;
    Y(idx) = 0;
end


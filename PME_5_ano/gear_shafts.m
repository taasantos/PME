%% Cálculo das forças de engrenamento

clc;
clear all;
close all;

gear_defenition;

% Forças de engrenamento para o par Z1 e Z2 em N; a função v_veio serve
% para converter a a velocidade do veio de rpm para m/s

idx = 1;

v_veio(idx) = 2*pi*(pinhao(idx).diametro_primitivo*0.001/2)*w_entrada/60;

M_torsor(idx) = torque_motor;

Ft(idx) = M_torsor(idx)/((pinhao(idx).diametro_primitivo*0.001)/2); %momento aplicado é o torque do motor
Fr(idx) = Ft(idx)*(tan(alpha)/cos(beta(idx)));
Fx(idx) = Ft(idx)*tan(beta(idx));
Mz_pinhao(idx) = Fx(idx)*(pinhao(idx).diametro_primitivo*0.001/2);
Mz_roda(idx) = Fx(idx)*(roda(idx).diametro_primitivo*0.001/2);

% Forças de engrenamento para o par Z3 e Z4 em N

idx = 2;

v_veio(idx) = 2*pi*(pinhao(idx).diametro_primitivo*0.001/2)*pinhao(idx).rotacao/60;

M_torsor(idx) = P_motor_catalogo/v_veio(idx);

Ft(idx) = M_torsor(idx)/((pinhao(idx).diametro_primitivo*0.001)/2);
Fr(idx) = Ft(idx)*(tan(alpha)/cos(beta(idx)));
Fx(idx) = Ft(idx)*tan(beta(idx));
Mz_pinhao(idx) = Fx(idx)*(pinhao(idx).diametro_primitivo*0.001/2);
Mz_roda(idx) = Fx(idx)*(roda(idx).diametro_primitivo*0.001/2);

% Forças de engrenamento para o par Z5 e Z6 em N

idx = 3;

v_veio(idx) = 2*pi*(pinhao(idx).diametro_primitivo*0.001/2)*pinhao(idx).rotacao/60;

M_torsor(idx) = P_motor_catalogo/v_veio(idx);

Ft(idx) = M_torsor(idx)/((pinhao(idx).diametro_primitivo*0.001)/2);
Fr(idx) = Ft(idx)*(tan(alpha)/cos(beta(idx)));
Fx(idx) = Ft(idx)*tan(beta(idx));
Mz_pinhao(idx) = Fx(idx)*(pinhao(idx).diametro_primitivo*0.001/2);
Mz_roda(idx) = Fx(idx)*(roda(idx).diametro_primitivo*0.001/2);


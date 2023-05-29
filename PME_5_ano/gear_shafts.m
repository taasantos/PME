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

Mf_XY(idx) = 203; %N.m
Mf_ZX(idx) = 214; %N.m
M_fletor(idx) = sqrt((Mf_ZX(idx)^2)+Mf_XY(idx)^2);
fs_veio(idx) = 1.5;
oy(idx) = 430000000; %Pa - aço Ck45 tempera + revenido

    % Critério de cedencia de Tresca

D(idx).tresca = (((32*fs_veio(idx))/(pi*oy(idx))*sqrt((M_fletor(idx)^2)+...
    M_torsor(idx)^2)))^(1/3); % m

D(idx).von_mises = (((32*fs_veio(idx))/(pi*oy(idx))*sqrt((M_fletor(idx)^2)+...
    0.75*M_torsor(idx)^2)))^(1/3); % m

R_a_xy(idx) = -2114;
R_b_xy(idx) = 1645;

R_a_zy(idx) = -8139;
R_b_zy(idx) = 1565;

Fr_rolamento_a(idx) = sqrt((R_a_xy(idx)^2)+R_a_zy(idx)^2);
Fr_rolamento_b(idx) = sqrt((R_b_xy(idx)^2)+R_b_zy(idx)^2);

    % Para este caso, optou-se por rolamentos de cilindros cónicos com 15º de carreira simples
    % para ambos os casos.

alpha_contacto_rol(idx) = deg2rad(40);
alpha_contacto_rol_cilindro(idx) = deg2rad(15);

X0_a(idx) = 0.5;
Y0_a(idx) = 0.22*cot(alpha_contacto_rol_cilindro(idx));

X0_b(idx) = 0.5;
Y0_b(idx) = 0.22*cot(alpha_contacto_rol_cilindro(idx));

razao_forcas_rol_a(idx) = Fx(idx)/Fr_rolamento_a(idx);
razao_forcas_rol_b(idx) = Fx(idx)/Fr_rolamento_b(idx);

ceof_carga_axial(idx) = 1.5*tan(alpha_contacto_rol_cilindro(idx));
ceof_carga_axial_cilindro(idx)= 1.5*tan(alpha_contacto_rol_cilindro);

if razao_forcas_rol_a(idx) > ceof_carga_axial_cilindro(idx)
    X_a(idx) = 0.4;
    Y_a(idx) = 0.4*cot(alpha_contacto_rol_cilindro);
else
    X_a(idx) = 1.0;
    Y_a(idx) = 0;
end

if razao_forcas_rol_b(idx) > ceof_carga_axial_cilindro(idx)
    X_b(idx) = 0.4;
    Y_b(idx) = 0.4*cot(alpha_contacto_rol_cilindro);
else
    X_b(idx) = 1.0;
    Y_b(idx) = 0;
end

fs(idx) = 1.2; % coefeciente de segurança - normal
ft(idx) = 1.0; % fator de temperatura - <120º
fl(idx) = 2; % fator de esforço dinâmico - acionamento motores
fn(idx) = 0.322; % fator de rotação - 1500 rpm

P0_a(idx) = X0_a(idx)*Fr_rolamento_a(idx) + Y0_a(idx)*Fx(idx);
C0_a(idx) = fs(idx)*P0_a(idx);

P0_b(idx) = X0_b(idx)*Fr_rolamento_b(idx) + Y0_b(idx)*Fx(idx);
C0_b(idx) = fs(idx)*P0_b(idx);

P_a(idx) = X_a(idx)*Fr_rolamento_a(idx) + Y_a(idx)*Fx(idx);
C_a(idx) = (fl(idx)/(fn(idx)*ft(idx)))*P_a(idx);

P_b(idx) = X_b(idx)*Fr_rolamento_b(idx) + Y_b(idx)*Fx(idx);
C_b(idx) = (fl(idx)/(fn(idx)*ft(idx)))*P_b(idx);

    %Tamanho do veio

Rd(idx) = 23; %mm
Re(idx) = 18; %mm
a0 = 7.5; %mm
a = 5; %mm
Bz1 = pinhao(1).largura; %mm
Bz2 = pinhao(2).largura;
Bz3 = pinhao(3).largura;

comp_min_veio(idx) = Rd(idx) + Re(idx) + Bz1 + Bz2 + Bz3 +...
    2*a0 + 2*a;

    % Dimensionamento de chaveta

D_veio_pinhao(idx) = 0.040; % Secção do veio que acolhe a chaveta

Comp_cubo_max_recomendado_pinhao(idx) = 2.5*D_veio_pinhao(idx); % N cumpre critério

t_corte(idx) = 85000000; % MPa
t_esmagamento(idx) = 80000000;

b_chaveta_pinhao(idx) = 12; % 38 < d < 44
h_chaveta_pinhao(idx) = 8;
t1_chaveta_pinhao(idx) = 5;
t2_chaveta_pinhao(idx) = 3.3;

L_min_t_corte_pinhao(idx) = (2*M_torsor(idx))/(t_corte(idx)*(b_chaveta_pinhao(idx)*10^(-3))*...
    D_veio_pinhao(idx)); % m
L_min_t_esmagamento_pinhao(idx) = (2*M_torsor(idx))/(t_esmagamento(idx)*...
    D_veio_pinhao(idx)*((h_chaveta_pinhao(idx)*10^(-3))-(t1_chaveta_pinhao(idx)*10^(-3)))); % m

    %É necessário veio estriado

    %Dimensionamento de um veio estriado

%ligeiro 8x36x40

t_esmagamento_estriado(idx) = 80000000; %Pa

kt_pinhao(idx) = 1.3; 

z_estriado_pinhao(idx) = 8;
d_estriado_pinhao(idx) = 0.036;
D_estriado_pinhao(idx) = 0.040;
b_estriado_pinhao(idx) = 0.007;
h_estriado_pinhao(idx) = 0.75*((D_estriado_pinhao(idx)+d_estriado_pinhao(idx))/2);

comp_util_pinhao(idx) = 1.5*d_estriado_pinhao(idx);

L_min_estriado_pinhao(idx) = (2*M_torsor(idx)*kt_pinhao(idx))/(((D_estriado_pinhao(idx)...
    +d_estriado_pinhao)/2)*z_estriado_pinhao(idx)*t_esmagamento_estriado(idx)*...
    h_estriado_pinhao(idx));

L_estriado_selecionado_pinhao(idx) = pinhao(idx).largura - 5;

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

Mf_XY(idx) = 327; %N.m
Mf_ZX(idx) = 642; %N.m
M_fletor(idx) = sqrt((Mf_ZX(idx)^2)+Mf_XY(idx)^2);
fs_veio(idx) = 1.5;
oy(idx) = 430000000; %Pa - aço Ck45 tempera + revenido

    % Critério de cedencia de Tresca

D(idx).tresca = (((32*fs_veio(idx))/(pi*oy(idx))*sqrt((M_fletor(idx)^2)+...
    M_torsor(idx)^2)))^(1/3); % m

D(idx).von_mises = (((32*fs_veio(idx))/(pi*oy(idx))*sqrt((M_fletor(idx)^2)+...
    0.75*M_torsor(idx)^2)))^(1/3); % m

R_a_xy(idx) = -30;
R_b_xy(idx) = 1473;

R_a_zy(idx) = 4852;
R_b_zy(idx) = 8732;

Fr_rolamento_a(idx) = sqrt((R_a_xy(idx)^2)+R_a_zy(idx)^2);
Fr_rolamento_b(idx) = sqrt((R_b_xy(idx)^2)+R_b_zy(idx)^2);

    % Para este caso, optou-se por rolamentos de carreira simples com
    % contacto angular de 40º e cilindros cónicos com contacto angular de
    % 15º

alpha_contacto_rol(idx) = deg2rad(40);
alpha_contacto_rol_cilindro(idx) = deg2rad(15);

X0_a(idx) = 0.5;
Y0_a(idx) = 0.26;

X0_b(idx) = 0.5;
Y0_b(idx) = 0.22*cot(alpha_contacto_rol_cilindro(idx));

razao_forcas_rol_a(idx) = Fx(idx)/Fr_rolamento_a(idx);
razao_forcas_rol_b(idx) = Fx(idx)/Fr_rolamento_b(idx);

ceof_carga_axial(idx) = 1.14;
ceof_carga_axial_cilindro(idx) = 1.5*tan(alpha_contacto_rol_cilindro(idx));

P0_axial_a(idx) = 2.3*Fr_rolamento_a(idx)*tan(alpha_contacto_rol(idx))...
    + Fx(idx)-Fx(idx-1);
P0_axial_b(idx) = 2.3*Fr_rolamento_b(idx)*tan(alpha_contacto_rol(idx))...
    + Fx(idx)-Fx(idx-1);

if razao_forcas_rol_a(idx) > ceof_carga_axial(idx)
    X_a(idx) = 0.35;
    Y_a(idx) = 0.57;
else
    X_a(idx) = 1.0;
    Y_a(idx) = 0;
end

if razao_forcas_rol_b(idx) > ceof_carga_axial_cilindro(idx)
    X_b(idx) = 0.4;
    Y_b(idx) = 0.4*cot(alpha_contacto_rol_cilindro(idx));
else
    X_b(idx) = 1.0;
    Y_b(idx) = 0;
end

fs(idx) = 1.2; % coefeciente de segurança - normal
ft(idx) = 1.0; % fator de temperatura - <120º
fl(idx) = 2; % fator de esforço dinâmico - acionamento de motores
fn(idx) = 0.405; % fator de rotação - 500 rpm

P0_a(idx) = X0_a(idx)*Fr_rolamento_a(idx) + Y0_a(idx)*(Fx(idx)-Fx(idx-1));
C0_a(idx) = fs(idx)*P0_a(idx);

P0_b(idx) = X0_b(idx)*Fr_rolamento_b(idx) + Y0_b(idx)*(Fx(idx)-Fx(idx-1));
C0_b(idx) = fs(idx)*P0_b(idx);

P_a(idx) = X_a(idx)*Fr_rolamento_a(idx) + Y_a(idx)*(Fx(idx)-Fx(idx-1));
C_a(idx) = (fl(idx)/(fn(idx)*ft(idx)))*P_a(idx);

P_b(idx) = X_b(idx)*Fr_rolamento_b(idx) + Y_b(idx)*(Fx(idx)-Fx(idx-1));
C_b(idx) = (fl(idx)/(fn(idx)*ft(idx)))*P_b(idx);

    %Tamanho do veio

Rd(idx) = 17; %mm
Re(idx) = 17; %mm
a0 = 7.5; %mm
a = 5; %mm
Bz1 = pinhao(1).largura; %mm
Bz2 = pinhao(2).largura;
Bz3 = pinhao(3).largura;

comp_min_veio(idx) = Rd(idx) + Re(idx) + Bz1 + Bz2 + Bz3 +...
    2*a0 + 2*a;

    % Dimensionamento de chaveta

D_veio_pinhao(idx) = 0.040; %38<d<44
D_veio_roda(idx) = 0.037;% 30<d<38

Comp_cubo_max_recomendado_pinhao(idx) = 2.5*D_veio_pinhao(idx); %cumpre critério
Comp_cubo_max_recomendado_roda(idx) = 2.5*D_veio_roda(idx); %cumpre critério

t_corte(idx) = 85000000; % MPa
t_esmagamento(idx) = 80000000;

b_chaveta_roda(idx) = 10;
h_chaveta_roda(idx) = 8;
t1_chaveta_roda(idx) = 5;
t2_chaveta_roda(idx) = 3.3;

b_chaveta_pinhao(idx) = 12;
h_chaveta_pinhao(idx) = 8;
t1_chaveta_pinhao(idx) = 5;
t2_chaveta_pinhao(idx) = 3.3;

L_min_t_corte_roda(idx) = (2*M_torsor(idx))/(t_corte(idx)*(b_chaveta_roda(idx)*10^(-3))*...
    D_veio_roda(idx)); % m
L_min_t_esmagamento_roda(idx) = (2*M_torsor(idx))/(t_esmagamento(idx)*...
    D_veio_roda(idx)*((h_chaveta_roda(idx)*10^(-3))-(t1_chaveta_roda(idx)*10^(-3)))); % m

L_min_t_corte_pinhao(idx) = (2*M_torsor(idx))/(t_corte(idx)*(b_chaveta_pinhao(idx)*10^(-3))*...
    D_veio_pinhao(idx)); % m
L_min_t_esmagamento_pinhao(idx) = (2*M_torsor(idx))/(t_esmagamento(idx)*...
    D_veio_pinhao(idx)*((h_chaveta_pinhao(idx)*10^(-3))-(t1_chaveta_pinhao(idx)*10^(-3)))); % m

    %Tanto o pinhao como a roda precisam de veio estriado

     %Dimensionamento de um veio estriado 

%ligeiro 8x36x40

t_esmagamento_estriado(idx) = 80000000; %Pa

kt_roda(idx) = 1.1; 

z_estriado_roda(idx) = 8;
d_estriado_roda(idx) = 0.036;
D_estriado_roda(idx) = 0.040;
b_estriado_roda(idx) = 0.007;
h_estriado_roda(idx) = 0.75*((D_estriado_roda(idx)+d_estriado_roda(idx))/2);

comp_util_roda(idx) = 1.5*d_estriado_roda(idx);

L_min_estriado_roda(idx) = (2*M_torsor(idx)*kt_roda(idx))/(((D_estriado_roda(idx)...
    +d_estriado_roda(idx))/2)*z_estriado_roda(idx)*t_esmagamento_estriado(idx)*...
    h_estriado_roda(idx));

L_estriado_selecionado_roda(idx) = roda(idx-1).largura - 5;

%ligeiro 8x36x40

kt_pinhao(idx) = 1.1; 

z_estriado_pinhao(idx) = 8;
d_estriado_pinhao(idx) = 0.036;
D_estriado_pinhao(idx) = 0.040;
b_estriado_pinhao(idx) = 0.007;
h_estriado_pinhao(idx) = 0.75*((D_estriado_pinhao(idx)+d_estriado_pinhao(idx))/2);

comp_util_pinhao(idx) = 1.5*d_estriado_pinhao(idx);

L_min_estriado_pinhao(idx) = (2*M_torsor(idx)*kt_pinhao(idx))/(((D_estriado_pinhao(idx)...
    +d_estriado_pinhao(idx))/2)*z_estriado_pinhao(idx)*t_esmagamento_estriado(idx)*...
    h_estriado_pinhao(idx));

L_estriado_selecionado_pinhao(idx) = pinhao(idx).largura - 5;

% Forças de engrenamento para o par Z5 e Z6 em N

idx = 3;

v_veio(idx) = 2*pi*(pinhao(idx).diametro_primitivo*0.001/2)*pinhao(idx).rotacao/60;

M_torsor(idx) = P_motor_catalogo/v_veio(idx);

Ft(idx) = M_torsor(idx)/((pinhao(idx).diametro_primitivo*0.001)/2);
Fr(idx) = Ft(idx)*(tan(alpha)/cos(beta(idx)));
Fx(idx) = Ft(idx)*tan(beta(idx));
Mz_pinhao(idx) = Fx(idx)*(pinhao(idx).diametro_primitivo*0.001/2);
Mz_roda(idx) = Fx(idx)*(roda(idx).diametro_primitivo*0.001/2);

Mf_XY(idx) = 301; % N.m
Mf_ZX(idx) = 1234; %N.m
M_fletor(idx) = sqrt((Mf_ZX(idx)^2)+Mf_XY(idx)^2);
fs_veio(idx) = 1.5;
oy(idx) = 750000000; %Pa - aço 42CrMo4 tempera + revenido

    % Critério de cedencia de Tresca

D(idx).tresca = (((32*fs_veio(idx))/(pi*oy(idx))*sqrt((M_fletor(idx)^2)+...
    M_torsor(idx)^2)))^(1/3); % m

D(idx).von_mises = (((32*fs_veio(idx))/(pi*oy(idx))*sqrt((M_fletor(idx)^2)+...
    0.75*M_torsor(idx)^2)))^(1/3); % m

R_a_xy(idx) = -3414;
R_b_xy(idx) = -2987;

R_a_zy(idx) = -5010;
R_b_zy(idx) = -5058;

Fr_rolamento_a(idx) = sqrt((R_a_xy(idx)^2)+R_a_zy(idx)^2);
Fr_rolamento_b(idx) = sqrt((R_b_xy(idx)^2)+R_b_zy(idx)^2);

    % Para este caso, optou-se por rolamentos de carreira simples com
    % contacto angular de 40º e cilindros cónicos com contacto angular de
    % 15º

alpha_contacto_rol(idx) = deg2rad(40);
alpha_contacto_rol_cilindro(idx) = deg2rad(15);

X0_a(idx) = 0.5;
Y0_a(idx) = 0.26;

X0_b(idx) = 0.05;
Y0_b(idx) = 0.22*cot(alpha_contacto_rol_cilindro(idx));

razao_forcas_rol_a(idx) = Fx(idx)/Fr_rolamento_a(idx);
razao_forcas_rol_b(idx) = Fx(idx)/Fr_rolamento_b(idx);

ceof_carga_axial(idx) = 1.14;
ceof_carga_axial_cilindro(idx) = 1.5*tan(alpha_contacto_rol_cilindro(idx));

P0_axial_a(idx) = 2.3*Fr_rolamento_a(idx)*tan(alpha_contacto_rol(idx))...
    + Fx(idx)-Fx(idx-1);

if razao_forcas_rol_a(idx) > ceof_carga_axial(idx)
    X_a(idx) = 0.35;
    Y_a(idx) = 0.57;
else
    X_a(idx) = 1.0;
    Y_a(idx) = 0;
end

if razao_forcas_rol_b(idx) > ceof_carga_axial_cilindro(idx)
    X_b(idx) = 0.4;
    Y_b(idx) = 0.4*cot(alpha_contacto_rol_cilindro(idx));
else
    X_b(idx) = 1.0;
    Y_b(idx) = 0;
end

fs(idx) = 1.2; % coefeciente de segurança - normal
ft(idx) = 1.0; % fator de temperatura - <120º
fl(idx) = 2; % fator de esforço dinâmico - acionamento de motores
fn(idx) = 0.693; % fator de rotação - 100 rpm

P0_a(idx) = X0_a(idx)*Fr_rolamento_a(idx) + Y0_a(idx)*(Fx(idx)-Fx(idx-1));
C0_a(idx) = fs(idx)*P0_a(idx);

P0_b(idx) = X0_b(idx)*Fr_rolamento_b(idx) + Y0_b(idx)*(Fx(idx)-Fx(idx-1));
C0_b(idx) = fs(idx)*P0_b(idx);

P_a(idx) = X_a(idx)*Fr_rolamento_a(idx) + Y_a(idx)*(Fx(idx)-Fx(idx-1));
C_a(idx) = (fl(idx)/(fn(idx)*ft(idx)))*P_a(idx);

P_b(idx) = X_b(idx)*Fr_rolamento_b(idx) + Y_b(idx)*(Fx(idx)-Fx(idx-1));
C_b(idx) = (fl(idx)/(fn(idx)*ft(idx)))*P_b(idx);

    %Tamanho do veio

Rd(idx) = 18; %mm
Re(idx) = 19; %mm
a0 = 7.5; %mm
a = 5; %mm
Bz1 = pinhao(1).largura; %mm
Bz2 = pinhao(2).largura;
Bz3 = pinhao(3).largura;

comp_min_veio(idx) = Rd(idx) + Re(idx) + Bz1 + Bz2 + Bz3 +...
    2*a0 + 2*a;

    % Dimensionamento de chaveta

D_veio_pinhao(idx) = 0.042; %38<d<44
D_veio_roda(idx) = 0.042;% 30<d<38

Comp_cubo_max_recomendado_pinhao(idx) = 2.5*D_veio_pinhao(idx); %cumpre critério
Comp_cubo_max_recomendado_roda(idx) = 2.5*D_veio_roda(idx); %cumpre critério

t_corte(idx) = 85000000; % MPa
t_esmagamento(idx) = 80000000;

b_chaveta_roda(idx) = 12;
h_chaveta_roda(idx) = 8;
t1_chaveta_roda(idx) = 5;
t2_chaveta_roda(idx) = 3.3;

b_chaveta_pinhao(idx) = 12;
h_chaveta_pinhao(idx) = 8;
t1_chaveta_pinhao(idx) = 5;
t2_chaveta_pinhao(idx) = 3.3;

L_min_t_corte_roda(idx) = (2*M_torsor(idx))/(t_corte(idx)*(b_chaveta_roda(idx)*10^(-3))*...
    D_veio_roda(idx)); % m
L_min_t_esmagamento_roda(idx) = (2*M_torsor(idx))/(t_esmagamento(idx)*...
    D_veio_roda(idx)*((h_chaveta_roda(idx)*10^(-3))-(t1_chaveta_roda(idx)*10^(-3)))); % m

L_min_t_corte_pinhao(idx) = (2*M_torsor(idx))/(t_corte(idx)*(b_chaveta_pinhao(idx)*10^(-3))*...
    D_veio_pinhao(idx)); % m
L_min_t_esmagamento_pinhao(idx) = (2*M_torsor(idx))/(t_esmagamento(idx)*...
    D_veio_pinhao(idx)*((h_chaveta_pinhao(idx)*10^(-3))-(t1_chaveta_pinhao(idx)*10^(-3)))); % m

    %tanto pinhao como roda irão levar veios estriados

    %Dimensionamento de um veio estriado 

%ligeiro 8x42x46

t_esmagamento_estriado(idx) = 80000000; %Pa

kt_roda(idx) = 1.1; 

z_estriado_roda(idx) = 8;
d_estriado_roda(idx) = 0.042;
D_estriado_roda(idx) = 0.046;
b_estriado_roda(idx) = 0.008;
h_estriado_roda(idx) = 0.75*((D_estriado_roda(idx)+d_estriado_roda(idx))/2);

comp_util_roda(idx) = 1.5*d_estriado_roda(idx);

L_min_estriado_roda(idx) = (2*M_torsor(idx)*kt_roda(idx))/(((D_estriado_roda(idx)...
    +d_estriado_roda(idx))/2)*z_estriado_roda(idx)*t_esmagamento_estriado(idx)*...
    h_estriado_roda(idx));

L_estriado_selecionado_roda(idx) = roda(idx-1).largura - 5;

%ligeiro 8x42x46

kt_pinhao(idx) = 1.1; 

z_estriado_pinhao(idx) = 8;
d_estriado_pinhao(idx) = 0.042;
D_estriado_pinhao(idx) = 0.046;
b_estriado_pinhao(idx) = 0.008;
h_estriado_pinhao(idx) = 0.75*((D_estriado_pinhao(idx)+d_estriado_pinhao(idx))/2);

comp_util_pinhao(idx) = 1.5*d_estriado_pinhao(idx);

L_min_estriado_pinhao(idx) = (2*M_torsor(idx)*kt_pinhao(idx))/(((D_estriado_pinhao(idx)...
    +d_estriado_pinhao(idx))/2)*z_estriado_pinhao(idx)*t_esmagamento_estriado(idx)*...
    h_estriado_pinhao(idx));

L_estriado_selecionado_pinhao(idx) = pinhao(idx).largura - 5;

%Ultimo veio

idx = 4;

v_veio(idx) = 2*pi*(pinhao(idx-1).diametro_primitivo*0.001/2)*...
    pinhao(idx-1).rotacao/u(3)/60;
M_torsor(idx) = P_motor_catalogo/v_veio(idx);

Mf_XY(idx) = 498; % N.m
Mf_ZX(idx) = 1453; %N.m
M_fletor(idx) = sqrt((Mf_ZX(idx)^2)+Mf_XY(idx)^2);
fs_veio(idx) = 1.5;
oy(idx) = 750000000; %Pa - aço 42CrMo4 tempera + revenido

    % Critério de cedencia de Tresca

D(idx).tresca = (((32*fs_veio(idx))/(pi*oy(idx))*sqrt((M_fletor(idx)^2)+...
    M_torsor(idx)^2)))^(1/3); % m

D(idx).von_mises = (((32*fs_veio(idx))/(pi*oy(idx))*sqrt((M_fletor(idx)^2)+...
    0.75*M_torsor(idx)^2)))^(1/3); % m


R_a_xy(idx) = -9610;
R_b_xy(idx) = -15299;

R_a_zy(idx) = 8296;
R_b_zy(idx) = -5240;

Fr_rolamento_a(idx) = sqrt((R_a_xy(idx)^2)+R_a_zy(idx)^2);
Fr_rolamento_b(idx) = sqrt((R_b_xy(idx)^2)+R_b_zy(idx)^2);

 % Para este caso, optou-se por rolamentos de carreira simples com
    % contacto angular de 40º para ambos os casos

alpha_contacto_rol(idx) = deg2rad(40);

X0_a(idx) = 0.5;
Y0_a(idx) = 0.26;

X0_b(idx) = 0.5;
Y0_b(idx) = 0.26;

razao_forcas_rol_a(idx) = Fx(idx-1)/Fr_rolamento_a(idx);
razao_forcas_rol_b(idx) = Fx(idx-1)/Fr_rolamento_b(idx);

ceof_carga_axial(idx) = 1.14;

P0_axial_a(idx) = 2.3*Fr_rolamento_a(idx)*tan(alpha_contacto_rol(idx))...
    + Fx(idx-1);

if razao_forcas_rol_a(idx) > ceof_carga_axial(idx)
    X_a(idx) = 0.35;
    Y_a(idx) = 0.57;
else
    X_a(idx) = 1.0;
    Y_a(idx) = 0;
end

if razao_forcas_rol_b(idx) > ceof_carga_axial(idx)
    X_b(idx) = 0.35;
    Y_b(idx) = 0.57;
else
    X_b(idx) = 1.0;
    Y_b(idx) = 0;
end

fs(idx) = 1.2; % coefeciente de segurança - normal
ft(idx) = 1.0; % fator de temperatura - <120º
fl(idx) = 1.5; % fator de esforço dinâmico - acionamento motores
fn(idx) = 0.693; % fator de rotação - 1500 rpm

P0_a(idx) = X0_a(idx)*Fr_rolamento_a(idx) + Y0_a(idx)*Fx(idx-1);
C0_a(idx) = fs(idx)*P0_a(idx);

P0_b(idx) = X0_b(idx)*Fr_rolamento_b(idx) + Y0_b(idx)*Fx(idx-1);
C0_b(idx) = fs(idx)*P0_b(idx);

P_a(idx) = X_a(idx)*Fr_rolamento_a(idx) + Y_a(idx)*Fx(idx-1);
C_a(idx) = (fl(idx)/(fn(idx)*ft(idx)))*P_a(idx);

P_b(idx) = X_b(idx)*Fr_rolamento_b(idx) + Y_b(idx)*Fx(idx-1);
C_b(idx) = (fl(idx)/(fn(idx)*ft(idx)))*P_b(idx);

    %Tamanho do veio

Rd(idx) = 19; %mm
Re(idx) = 25; %mm
a0 = 7.5; %mm
a = 5; %mm
Bz1 = pinhao(1).largura; %mm
Bz2 = pinhao(2).largura;
Bz3 = pinhao(3).largura;

comp_min_veio(idx) = Rd(idx) + Re(idx) + Bz1 + Bz2 + Bz3 +...
    2*a0 + 2*a;

    % Dimensionamento de chaveta

D_veio_roda(idx) = 0.050;% 30<d<38

Comp_cubo_max_recomendado_roda(idx) = 2.5*D_veio_roda(idx); %cumpre critério

t_corte(idx) = 85000000; % MPa
t_esmagamento(idx) = 80000000;

b_chaveta_roda(idx) = 14;
h_chaveta_roda(idx) = 9;
t1_chaveta_roda(idx) = 5.5;
t2_chaveta_roda(idx) = 3.8;

L_min_t_corte_roda(idx) = (2*M_torsor(idx))/(t_corte(idx)*(b_chaveta_roda(idx)*10^(-3))*...
    D_veio_roda(idx)); % m
L_min_t_esmagamento_roda(idx) = (2*M_torsor(idx))/(t_esmagamento(idx)*...
    D_veio_roda(idx)*((h_chaveta_roda(idx)*10^(-3))-(t1_chaveta_roda(idx)*10^(-3)))); % m

%Irá necessitar de um veio estriado também

% Dimensionamento do veio estriado

%Ligeiro 8x46x50

t_esmagamento_estriado(idx) = 80000000; %Pa

kt_roda(idx) = 1.1; 

z_estriado_roda(idx) = 8;
d_estriado_roda(idx) = 0.046;
D_estriado_roda(idx) = 0.050;
b_estriado_roda(idx) = 0.009;
h_estriado_roda(idx) = 0.75*((D_estriado_roda(idx)+d_estriado_roda(idx))/2);

comp_util_roda(idx) = 1.5*d_estriado_roda(idx);

L_min_estriado_roda(idx) = (2*M_torsor(idx)*kt_roda(idx))/(((D_estriado_roda(idx)...
    +d_estriado_roda(idx))/2)*z_estriado_roda(idx)*t_esmagamento_estriado(idx)*...
    h_estriado_roda(idx));

L_estriado_selecionado_roda(idx) = roda(idx-1).largura - 5;

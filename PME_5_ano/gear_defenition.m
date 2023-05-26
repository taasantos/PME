%% Defenição das rodas - 3 andares

clc;
clear all;
close all;

problem_variables;

% velocidade de saida do redutor

w_saida = (v_elevacao*60)/(2*pi*dist); % rpm

% Força aplicada no pior cenário: escada vertical

F_carga = m_carga*a_grav; % N

% Torque do motor no pior cenário

torque_motor = F_carga*dist; % Nm

% Potencia do motor

P_motor = (torque_motor*w_saida*2*pi)/(60*1000);% kW
P_motor_catalogo = 1.5*1000; %W

% Numero de andares do redutor - função 'ceil' arredonda por excesso

N_andares = ceil(log10(w_entrada/w_saida)/log10(4));

% Relação de transmissão

U_total = w_entrada/w_saida;

% Relação de transmição

u(1) = 4;
u(2) = 3.5;
u(3) = 1.96;

U_sizing_relation = u(1)*u(2)*u(3);

%% Defenição de constantes para o cálculo do pinhão e das rodas

% 20 anos, 250 dias, 4h, 60min, w de cada andar rpm
Nciclos(1) = 20*250*4*60*w_entrada;
Nciclos(2) = 20*250*4*60*w_entrada/u(1);
Nciclos(3) = 20*250*4*60*(w_entrada/u(1))/u(2);

beta(1) = deg2rad(20); % graus para radianos 
beta(2) = deg2rad(20);
beta(3) = deg2rad(30);
alpha = deg2rad(20); % graus para radianos
alpha_t(1) = atan(tan(alpha)/cos(beta(1))); % radianos
alpha_t(2) = atan(tan(alpha)/cos(beta(2)));
alpha_t(3) = atan(tan(alpha)/cos(beta(3)));
KM = 1; % uniforme / uniforme
Kbl(1) = log10(Nciclos(1))/8;
Kbl(2) = log10(Nciclos(2))/8;
Kbl(3) = log10(Nciclos(3))/8;
Ye(1) = 0.8; 
Ye(2) = 0.8;
Ye(3) = 0.8;
CL = 10; % Redutores P<=10kW; Mecânica Geral;
Clb(1) = CL/cos(beta(1));
Clb(2) = CL/cos(beta(2));
Clb(3) = CL/cos(beta(3));
Ka = 1; % Assumidos L/d1 < 0
Yf = 0.4; %assumido inicialmente n correção
Ybeta(1) = 1/cos(beta(1));
Ybeta(2) = 1/cos(beta(2));
Ybeta(3) = 1/cos(beta(3));
Khl(1) = 8/log10(Nciclos(1));
Khl(2) = 8/log10(Nciclos(2));
Khl(3) = 8/log10(Nciclos(3));
oblim(1) = 285; % MPa - aço ao carbono C45
oblim(2) = 340; % MPa - aço ao carbono temperado (óleo) 33Mn5
oblim(3) = 410; % MPa - aço ao carbono temperado (óleo) 33Mn5 MUDAR
ohlim(1) = 745; % MPa - aço ao carbono C45
ohlim(2) = 825; % MPa - aço ao carbono temperado (óleo) 33Mn5
ohlim(3) = 980; % MPa - aço ao carbono temperado (óleo) 33Mn5 MUDAR
E(1) = 200000;%MPa
E(2) = 210000;%MPa
E(3) = 210000;%MPa
v = 0.3;

%% Cálculo dos parâmetros dos pinhões e das rodas

idx = 1; % primeira engrenagem

% Defenição dos valores iniciais do pinhão para o cálculo do módulo

pinhao(idx).dentes = 19;
pinhao(idx).rotacao = w_entrada; %rpm
pinhao(idx).dentes_virtual = pinhao(idx).dentes/(cos(beta(idx))^3);

% Cálculo do módulo

mn_fad(idx) = ((19600*P_motor_catalogo*cos(beta(idx))*KM*Khl(idx)*Ye(idx)*(u(idx)+1))/...
    (Clb(idx)*oblim(idx)*pinhao(idx).rotacao*Ka*pinhao(idx).dentes_virtual*Yf*Ybeta(idx)...
    *u(idx)))^(1/3);
mn_hertz(idx) = ((60000*P_motor_catalogo*cos(beta(idx))*KM*2*E(idx)*(u(idx)+1))...
    /((pi^2)*Clb(idx)*(ohlim(idx)^2)*pinhao(idx).rotacao*Ka*pinhao(idx).dentes_virtual...
    *Khl(idx)*sin(2*alpha)*(1-v^2)*u(idx)))^(1/3);

if mn_fad(idx) > mn_hertz(idx)
    mn(idx) = 1;
else
    mn(idx) = 4.5;
end

%calculo das variáveis dependentes do módulo para o pinhão e roda

mt(idx) = mn(idx)/cos(beta(idx));
passo(idx) = mn(idx)*pi;
passo_aparente(idx) = mt(idx)*pi;
hz(idx) = 2.25*mn(idx);
hc(idx) = mn(idx);
hf(idx) = 1.25*mn(idx);
s(idx) = 0.25*mn(idx);

% Cálculo do resto dos valores do pinhão

pinhao(idx).diametro_primitivo = pinhao(idx).dentes*mt(idx);
pinhao(idx).diametro_base = pinhao(idx).diametro_primitivo*cos(alpha_t(idx));
pinhao(idx).diametro_pe_dente = pinhao(idx).diametro_primitivo-2*hf;
pinhao(idx).diametro_externo = pinhao(idx).diametro_primitivo+2*mn(idx);
pinhao(idx).largura = 35; % valores assumidos e a serem alterados nos cálculos da fadiga
pinhao(idx).largura_dentado = pinhao(idx).largura/cos(beta(idx));

% Cálculo dos parametros da roda da primeira engrenagem 

roda(idx).dentes = u(idx)*pinhao(idx).dentes;
roda(idx).dentes_virtual = roda(idx).dentes/cos(beta(idx))^3;
roda(idx).diametro_primitivo = roda(idx).dentes*mt(idx);
roda(idx).diametro_base = roda(idx).diametro_primitivo*cos(alpha_t(idx));
roda(idx).diametro_pe_dente = roda(idx).diametro_primitivo-2*hf;
roda(idx).diametro_externo = roda(idx).diametro_primitivo+2*mn(idx);
roda(idx).largura = 35; % valores assumidos e a serem alterados nos cálculos da fadiga
roda(idx).largura_dentado = roda(idx).largura/cos(beta(idx));

% Cálculo do entre eixo da primeira engrenagem

entre_eixo(idx) =((roda(idx).dentes + pinhao(idx).dentes)/2)*mt(idx); 

idx = 2; % Segunda engrenagem

% Defenição dos valores iniciais do pinhão para o cálculo do módulo

pinhao(idx).dentes = 22;
pinhao(idx).rotacao = pinhao(idx-1).rotacao/u(1); %rpm
pinhao(idx).dentes_virtual = pinhao(idx).dentes/cos(beta(idx))^3;

% Cálculo do módulo

mn_fad(idx) = ((19600*P_motor_catalogo*cos(beta(idx))*KM*Khl(idx)*Ye(idx)*(u(idx)+1))/...
    (Clb(idx)*oblim(idx)*pinhao(idx).rotacao*Ka*pinhao(idx).dentes_virtual*Yf*Ybeta(idx)...
    *u(idx)))^(1/3);
mn_hertz(idx) = ((60000*P_motor_catalogo*cos(beta(idx))*KM*2*E(idx)*(u(idx)+1))...
    /((pi^2)*Clb(idx)*(ohlim(idx)^2)*pinhao(idx).rotacao*Ka*pinhao(idx).dentes_virtual...
    *Khl(idx)*sin(2*alpha)*(1-v^2)*u(idx)))^(1/3);


if mn_fad(idx) > mn_hertz(idx)
    mn(idx) = 1.0;
else
    mn(idx) = 6;
end

%calculo das variáveis dependentes do módulo para o pinhão e roda

mt(idx) = mn(idx)/cos(beta(idx));
passo(idx) = mn(idx)*pi;
passo_aparente(idx) = mt(idx)*pi;
hz(idx) = 2.25*mn(idx);
hc(idx) = mn(idx);
hf(idx) = 1.25*mn(idx);
s(idx) = 0.25*mn(idx);

% Cálculo do resto dos valores do pinhão

pinhao(idx).diametro_primitivo = pinhao(idx).dentes*mt(idx);
pinhao(idx).diametro_base = pinhao(idx).diametro_primitivo*cos(alpha_t(idx));
pinhao(idx).diametro_pe_dente = pinhao(idx).diametro_primitivo-2*hf(idx);
pinhao(idx).diametro_externo = pinhao(idx).diametro_primitivo+2*mn(idx);
pinhao(idx).largura = 60; % valores assumidos e a serem alterados nos cálculos da fadiga
pinhao(idx).largura_dentado = pinhao(idx).largura/cos(beta(idx));

% Cálculo dos parametros da roda da primeira engrenagem 

roda(idx).dentes = u(idx)*pinhao(idx).dentes;
roda(idx).dentes_virtual = roda(idx).dentes/cos(beta(idx))^3;
roda(idx).diametro_primitivo = roda(idx).dentes*mt(idx);
roda(idx).diametro_base = roda(idx).diametro_primitivo*cos(alpha_t(idx));
roda(idx).diametro_pe_dente = roda(idx).diametro_primitivo-2*hf(idx);
roda(idx).diametro_externo = roda(idx).diametro_primitivo+2*mn(idx);
roda(idx).largura = 40; % valores assumidos e a serem alterados nos cálculos da fadiga
roda(idx).largura_dentado = roda(idx).largura/cos(beta(idx));

% Cálculo do entre eixo da primeira engrenagem

entre_eixo(idx) =((roda(idx).dentes + pinhao(idx).dentes)/2)*mt(idx);

idx = 3; % terceira engrenagem 

% Defenição dos valores iniciais do pinhão para o cálculo do módulo

pinhao(idx).dentes =38;
pinhao(idx).rotacao = pinhao(idx-1).rotacao/u(2); %rpm
pinhao(idx).dentes_virtual = pinhao(idx).dentes/cos(beta(idx))^3;

% Cálculo do módulo

mn_fad(idx) = ((19600*P_motor_catalogo*cos(beta(idx))*KM*Khl(idx)*Ye(idx)*(u(idx)+1))/...
    (Clb(idx)*oblim(idx)*pinhao(idx).rotacao*Ka*pinhao(idx).dentes_virtual*Yf*Ybeta(idx)...
    *u(idx)))^(1/3);
mn_hertz(idx) = ((60000*P_motor_catalogo*cos(beta(idx))*KM*2*E(idx)*(u(idx)+1))...
    /((pi^2)*Clb(idx)*(ohlim(idx)^2)*pinhao(idx).rotacao*Ka*pinhao(idx).dentes_virtual...
    *Khl(idx)*sin(2*alpha)*(1-v^2)*u(idx)))^(1/3);


if mn_fad(idx) > mn_hertz(idx)
    mn(idx) = 1.25;
else
    mn(idx) = 6;
end

%calculo das variáveis dependentes do módulo para o pinhão e roda

mt(idx) = mn(idx)/cos(beta(idx));
passo(idx) = mn(idx)*pi;
passo_aparente(idx) = mt(idx)*pi;
hz(idx) = 2.25*mn(idx);
hc(idx) = mn(idx);
hf(idx) = 1.25*mn(idx);
s(idx) = 0.25*mn(idx);

% Cálculo do resto dos valores do pinhão

pinhao(idx).diametro_primitivo = pinhao(idx).dentes*mt(idx);
pinhao(idx).diametro_base = pinhao(idx).diametro_primitivo*cos(alpha_t(idx));
pinhao(idx).diametro_pe_dente = pinhao(idx).diametro_primitivo-2*hf(idx);
pinhao(idx).diametro_externo = pinhao(idx).diametro_primitivo+2*mn(idx);
pinhao(idx).largura = 35; % valores assumidos e a serem alterados nos cálculos da fadiga
pinhao(idx).largura_dentado = pinhao(idx).largura/cos(beta(idx));

% Cálculo dos parametros da roda da primeira engrenagem 

roda(idx).dentes = ceil(u(idx)*pinhao(idx).dentes);
roda(idx).dentes_virtual = roda(idx).dentes/cos(beta(idx))^3;
roda(idx).diametro_primitivo = roda(idx).dentes*mt(idx);
roda(idx).diametro_base = roda(idx).diametro_primitivo*cos(alpha_t(idx));
roda(idx).diametro_pe_dente = roda(idx).diametro_primitivo-2*hf(idx);
roda(idx).diametro_externo = roda(idx).diametro_primitivo+2*mn(idx);
roda(idx).largura = 20; % valores assumidos e a serem alterados nos cálculos da fadiga
roda(idx).largura_dentado = roda(idx).largura/cos(beta(idx));

% Cálculo do entre eixo da primeira engrenagem

entre_eixo(idx) =((roda(idx).dentes + pinhao(idx).dentes)/2)*mt(idx);

% Confirmação do erro da velocidade de Saida - DIN 804

w_saida_real = pinhao(idx).rotacao/u(3);
erro_relativo = abs(w_saida_real-w_saida)/w_saida; % erro muito inferior ao intervalo estabelecido pela norma
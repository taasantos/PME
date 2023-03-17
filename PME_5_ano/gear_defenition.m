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
u(2) = 4;
u(3) = 2.23;

U_sizing_relation = u(1)*u(2)*u(3);

%% Defenição de constantes para o cálculo do pinhão e das rodas

% 20 anos, 300 dias, 8h, 60min, w de cada andar rpm
Nciclos(1) = 20*300*8*60*w_entrada;
Nciclos(2) = 20*300*8*60*w_entrada/u(1);
Nciclos(3) = 20*300*8*60*w_entrada/u(2);

beta = deg2rad(20); % graus
alpha = deg2rad(20); % graus
alpha_t = atan(tan(alpha)/cos(beta)); % graus
KM = 1; % uniforme / uniforme
Kbl(1) = log10(Nciclos(1))/8;
Kbl(2) = log10(Nciclos(2))/8;
Kbl(3) = log10(Nciclos(3))/8;
Ye = 0.8; % valor admitido
CL = 10; % Redutores P<=10kW; Mecânica Geral;
Clb = CL/cos(beta);
Ka = 1; % Assumido
Yf = 0.4; %assumido inicialmente n correção
Ybeta = 1/cos(beta);
Khl(1) = 8/log10(Nciclos(1));
Khl(2) = 8/log10(Nciclos(2));
Khl(3) = 8/log10(Nciclos(3));
oblim = 240; % MPa - aço ao carbono C45
ohlim = 500; % MPa - aço ao carbono C45
E = 200000; %MPa
v = 0.3;

%% Cálculo dos parâmetros dos pinhões e das rodas

idx = 1; % primeira engrenagem

% Defenição dos valores iniciais do pinhão para o cálculo do módulo

pinhao(idx).dentes = 17;
pinhao(idx).rotacao = w_entrada; %rpm
pinhao(idx).dentes_virtual = pinhao(idx).dentes/cos(beta)^3;

% Cálculo do módulo

mn_fad(idx) = ((19600*P_motor_catalogo*cos(beta)*KM*Kbl(idx)*Ye)/(Clb*oblim...
    *pinhao(idx).rotacao*Ka*pinhao(idx).dentes_virtual*Yf*Ybeta)...
    *((u(idx))+1)/u(idx)).^(1/3);
mn_hertz(idx) = ((60000*P_motor_catalogo*cos(beta)*KM*2*E*(u(idx)+1))...
    /((pi^2)*Clb*(ohlim^2)*pinhao(idx).rotacao*Ka*pinhao(idx).dentes_virtual...
    *Khl(idx)*sin(2*alpha)*(1-v^2)*u(idx)))^(1/3);

if mn_fad(idx) > mn_hertz(idx)
    mn(idx) = 1.25;
else
    mn(idx) = 5;
end

%calculo das variáveis dependentes do módulo para o pinhão e roda

mt(idx) = mn(idx)/cos(beta);
passo(idx) = mn(idx)*pi;
passo_aparente(idx) = mt(idx)*pi;
hz(idx) = 2.25*mn(idx);
hc(idx) = mn(idx);
hf(idx) = 1.25*mn(idx);
s(idx) = 0.25*mn(idx);

% Cálculo do resto dos valores do pinhão

pinhao(idx).diametro_primitivo = pinhao(idx).dentes*mt(idx);
pinhao(idx).diametro_base = pinhao(idx).diametro_primitivo*cos(alpha_t);
pinhao(idx).diametro_pe_dente = pinhao(idx).diametro_primitivo-2*hf;
pinhao(idx).diametro_externo = pinhao(idx).diametro_primitivo+2*mn(idx);

% Cálculo dos parametros da roda da primeira engrenagem 

roda(idx).dentes = u(idx)*pinhao(idx).dentes;
roda(idx).dentes_virtual = roda(idx).dentes/cos(beta)^3;
roda(idx).diametro_primitivo = roda(idx).dentes*mt(idx);
roda(idx).diametro_base = roda(idx).diametro_primitivo*cos(alpha_t);
roda(idx).diametro_pe_dente = roda(idx).diametro_primitivo-2*hf;
roda(idx).diametro_externo = roda(idx).diametro_primitivo+2*mn(idx);

% Cálculo do entre eixo da primeira engrenagem

entre_eixo(idx) =((roda(idx).dentes + pinhao(idx).dentes)/2)*mt(idx); 

idx = 2; % Segunda engrenagem

% Defenição dos valores iniciais do pinhão para o cálculo do módulo

pinhao(idx).dentes = 17;
pinhao(idx).rotacao = w_entrada; %rpm
pinhao(idx).dentes_virtual = pinhao(idx).dentes/cos(beta)^3;

% Cálculo do módulo

mn_fad(idx) = ((19600*P_motor_catalogo*cos(beta)*KM*Kbl(idx)*Ye)/(Clb*oblim...
    *pinhao(idx).rotacao*Ka*pinhao(idx).dentes_virtual*Yf*Ybeta)...
    *((u(idx))+1)/u(idx)).^(1/3);
mn_hertz(idx) = ((60000*P_motor_catalogo*cos(beta)*KM*2*E*(u(idx)+1))...
    /((pi^2)*Clb*(ohlim^2)*pinhao(idx).rotacao*Ka*pinhao(idx).dentes_virtual...
    *Khl(idx)*sin(2*alpha)*(1-v^2)*u(idx)))^(1/3);


if mn_fad(idx) > mn_hertz(idx)
    mn(idx) = 1;
else
    mn(idx) = 5;
end

%calculo das variáveis dependentes do módulo para o pinhão e roda

mt(idx) = mn(idx)/cos(beta);
passo(idx) = mn(idx)*pi;
passo_aparente(idx) = mt(idx)*pi;
hz(idx) = 2.25*mn(idx);
hc(idx) = mn(idx);
hf(idx) = 1.25*mn(idx);
s(idx) = 0.25*mn(idx);

% Cálculo do resto dos valores do pinhão

pinhao(idx).diametro_primitivo = pinhao(idx).dentes*mt(idx);
pinhao(idx).diametro_base = pinhao(idx).diametro_primitivo*cos(alpha_t);
pinhao(idx).diametro_pe_dente = pinhao(idx).diametro_primitivo-2*hf(idx);
pinhao(idx).diametro_externo = pinhao(idx).diametro_primitivo+2*mn(idx);

% Cálculo dos parametros da roda da primeira engrenagem 

roda(idx).dentes = u(idx)*pinhao(idx).dentes;
roda(idx).dentes_virtual = roda(idx).dentes/cos(beta)^3;
roda(idx).diametro_primitivo = roda(idx).dentes*mt(idx);
roda(idx).diametro_base = roda(idx).diametro_primitivo*cos(alpha_t);
roda(idx).diametro_pe_dente = roda(idx).diametro_primitivo-2*hf(idx);
roda(idx).diametro_externo = roda(idx).diametro_primitivo+2*mn(idx);

% Cálculo do entre eixo da primeira engrenagem

entre_eixo(idx) =((roda(idx).dentes + pinhao(idx).dentes)/2)*mt(idx);

idx = 3; % terceira engrenagem 

% Defenição dos valores iniciais do pinhão para o cálculo do módulo

pinhao(idx).dentes = 17;
pinhao(idx).rotacao = w_entrada; %rpm
pinhao(idx).dentes_virtual = pinhao(idx).dentes/cos(beta)^3;

% Cálculo do módulo

mn_fad(idx) = ((19600*P_motor_catalogo*cos(beta)*KM*Kbl(idx)*Ye)/(Clb*oblim...
    *pinhao(idx).rotacao*Ka*pinhao(idx).dentes_virtual*Yf*Ybeta)...
    *((u(idx))+1)/u(idx)).^(1/3);
mn_hertz(idx) = ((60000*P_motor_catalogo*cos(beta)*KM*2*E*(u(idx)+1))...
    /((pi^2)*Clb*(ohlim^2)*pinhao(idx).rotacao*Ka*pinhao(idx).dentes_virtual...
    *Khl(idx)*sin(2*alpha)*(1-v^2)*u(idx)))^(1/3);


if mn_fad(idx) > mn_hertz(idx)
    mn(idx) = 1.25;
else
    mn(idx) = 5;
end

%calculo das variáveis dependentes do módulo para o pinhão e roda

mt(idx) = mn(idx)/cos(beta);
passo(idx) = mn(idx)*pi;
passo_aparente(idx) = mt(idx)*pi;
hz(idx) = 2.25*mn(idx);
hc(idx) = mn(idx);
hf(idx) = 1.25*mn(idx);
s(idx) = 0.25*mn(idx);

% Cálculo do resto dos valores do pinhão

pinhao(idx).diametro_primitivo = pinhao(idx).dentes*mt(idx);
pinhao(idx).diametro_base = pinhao(idx).diametro_primitivo*cos(alpha_t);
pinhao(idx).diametro_pe_dente = pinhao(idx).diametro_primitivo-2*hf(idx);
pinhao(idx).diametro_externo = pinhao(idx).diametro_primitivo+2*mn(idx);

% Cálculo dos parametros da roda da primeira engrenagem 

roda(idx).dentes = ceil(u(idx)*pinhao(idx).dentes);
roda(idx).dentes_virtual = roda(idx).dentes/cos(beta)^3;
roda(idx).diametro_primitivo = roda(idx).dentes*mt(idx);
roda(idx).diametro_base = roda(idx).diametro_primitivo*cos(alpha_t);
roda(idx).diametro_pe_dente = roda(idx).diametro_primitivo-2*hf(idx);
roda(idx).diametro_externo = roda(idx).diametro_primitivo+2*mn(idx);

% Cálculo do entre eixo da primeira engrenagem

entre_eixo(idx) =((roda(idx).dentes + pinhao(idx).dentes)/2)*mt(idx);


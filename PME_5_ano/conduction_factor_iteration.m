%% Iterações para a razão de condução

gear_defenition;

idx = 1;

epslon(idx) = (1/(pi*mt(idx)*cos(alpha_t)))*(sqrt(((pinhao(idx).diametro_externo...
    /2)^2)-((pinhao(idx).diametro_base/2)^2))+sqrt(((roda(idx).diametro_externo...
    /2)^2)-((roda(idx).diametro_base/2)^2))-entre_eixo(idx)*sin(alpha_t)); 

idx = 2;

epslon(idx) = (1/(pi*mt(idx)*cos(alpha_t)))*(sqrt(((pinhao(idx).diametro_externo...
    /2)^2)-((pinhao(idx).diametro_base/2)^2))+sqrt(((roda(idx).diametro_externo...
    /2)^2)-((roda(idx).diametro_base/2)^2))-entre_eixo(idx)*sin(alpha_t));

idx = 3;

epslon(idx) = (1/(pi*mt(idx)*cos(alpha_t)))*(sqrt(((pinhao(idx).diametro_externo...
    /2)^2)-((pinhao(idx).diametro_base/2)^2))+sqrt(((roda(idx).diametro_externo...
    /2)^2)-((roda(idx).diametro_base/2)^2))-entre_eixo(idx)*sin(alpha_t));
clc; clear; close all;

%% Parametrização do sistema

%Parâmetros da bateria
R = 100; %R interno da bateria (ohms)
C = 1e-3; %C da bateria (farads)
tau = R * C; %Constante de tempo (s)

%Parâmetros do atuador (regulador de tensão)
K_A = 1; %Ganho do atuador
tau_A = 0.5; %Constante de tempo do atuador

%Parâmetros do sensor (medidor de tensão)
K_S = 1; %Ganho do sensor
tau_S = 0.2; %Constante de tempo do sensor

%% Criação das funções de transferência

%Função de transferência da bateria
num = 1; %Numerador da FT da bateria
den = [R*C 1]; %Denominador da FT da bateria
G = tf(num, den)

%Função de transferência do atuador
num_a = K_A;
den_a = [tau_A 1];
G_atuador = tf(num_a, den_a)

%Função de transferência do sensor
num_s = K_S;
den_s = [tau_S 1];
G_sensor = tf(num_s, den_s)

%Função de transferência da bateria considerando realimentação
num_MF = 1; 
den_MF = [R*C 2]; 
G_MF = tf(num_MF, den_MF)

%% Modelagem dos controladores PID diversos

%Projeto do controlador PID pelo método de ZN
L = 0.5; %Atraso utilizado nos processos (0,5 s)
Kp_ZN = 1.2*tau/(num*L);
tauI_ZN = 2*L;
Ki_ZN = Kp_ZN/tauI_ZN;
tauD_ZN = 0.5*L;
Kd_ZN = Kp_ZN*tauD_ZN;

%Projeto do controlador PID pelo método CC
Kp_CC = (1.35+(0.25*L/tau))*(tau/(num*L));
tauI_CC = ((1.35+(0.25*L/tau))*L)/(0.54+(0.33*L/tau));
Ki_CC = Kp_CC/tauI_CC;
tauD_CC = (0.5*L)/(1.35+(0.25*L/tau));
Kd_CC = Kp_CC*tauD_CC;

%Projeto do controlador PID pelo método de CHR
%Sistema regulatório
Kp_CHR = 0.95*tau/(num*L);
tauI_CHR = 2.375*L;
Ki_CHR = Kp_CHR/tauI_CHR;
tauD_CHR = 0.421*L;
Kd_CHR = Kp_CHR*tauD_CHR;

%Projeto do controlador PID pelo método de Skogestad 2004
%Sistema de Primeira Ordem com Atraso
Kp_Skd = tau/(num*(2*L)); %λ = L
tauI_Skd = tau; %min(tau, 4(λ+L))
Ki_Skd = Kp_Skd/tauI_Skd;
tauD_Skd = 0;
Kd_Skd = Kp_Skd*tauD_Skd;

%% Método Anti-Windup
%Foi escolhido o método CC para a realização desse método devido ao fator
%de controlabilidade
Kp_AW = (0.9+(0.083*(L/tau)))*(tau/(num*L));
tauI_AW = (0.9+(0.083*(L/tau)))*L/(1.27+0.6*(L/tau));
Ki_AW = Kp_AW/tauI_AW;
Tw = tauI_AW/Kp_AW; %Constante de tempo utilizada para a back-calculation

%% Modelo da perturbação e controlador feed-forward
%Nesta situação, a bateria está esquentando, o que aumenta sua resistência
%interna

R_novo = 10e3; 

%Parâmetros da função de transferência da perturbação
K_p = 1; %O ganho permanece igual pois R não influencia
tau_p = R_novo*C;

% Controlador feed-forward dinâmico
num_Ca = [(-K_p*tau) (-K_p)];
den_Ca = [(num*tau_p) (num)];

%% Modelo de controle em cascata

% Controlador da malha rápida (corrente)
Kp_corrente = 0.8; 
Ki_corrente = 5; 
Kd_corrente = 0.01; 


% Controlador da malha lenta (tensão)
Kp_tensao = 0.5;
Ki_tensao = 0.05; 
Kd_tensao = 0.1; 

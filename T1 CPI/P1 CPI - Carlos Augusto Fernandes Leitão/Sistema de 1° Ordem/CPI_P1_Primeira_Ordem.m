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
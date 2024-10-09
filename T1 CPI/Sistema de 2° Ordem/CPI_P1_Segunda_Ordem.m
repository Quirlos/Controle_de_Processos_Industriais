clc; clear; close all;

%% Parametrização do sistema

%Parâmetros do motor dde indução
Rs = 1.5; %R do estator (ohms)
Ls = 0.05; %L do estator (H)
J = 0.02; %Inércia do rotor (kg.m^2)
B = 0.001; %Coeficiente de atrito viscoso (N.m.s)
Kt = 0.1; %Constante de torque (N.m/A)

%Parâmetros do sensor (tacômetro) 
K_t = 0.05; %Ganho do tacômetro (V/(rad/s))
tau_s = 0.01; %Constante de tempo do tacômetro (s)

%Parâmetros do atuador (inversor de frequência)
K_inv = 10; % Ganho do inversor (rad/s / V)
tau_a = 0.02; %Constante de tempo do inversor (s)

%% Criação das funções de transferência

%Função de transferência do motor de indução
num = Kt; %Numerador da FT do motor
den = conv([J B], [Ls Rs]); %Denominador da FT do motor
G = tf(num, den)

%Função de transferência do motor de indução em malha fechada com realimentação unitária
G_MF = feedback(G, 1)

%Obtenção do numerador e denominador da função de transferência em malha fechada
[num_MF, den_MF] = tfdata(G_MF, 'v');  %Armazenamento nas variáveis num_MF e den_MF

%Função de transferência do sensor
num_s = K_t;           
den_s = [tau_s 1];   
G_s = tf(num_s, den_s)

%Função de transferência do atuador
num_a = K_inv;         
den_a = [tau_a 1];   
G_a = tf(num_a, den_a)
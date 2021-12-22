% Defining parameters
Ra = 3; Ke = 0.01; Kt = 0.01; I = 6e-4; b = 1e-4;
thetades = pi/2;
tau_l = 0.01;
d_bar = (Ra/Kt)* tau_l;
Vlim = 5;

% Define LTI object G
A = Kt / (Ra* I);
B = (b + Ke * Kt /Ra)/I;
G = zpk([],[0,-B],A);

%%% Designing the lead controller %%%

initial_crossover = 20;  % TUNE THIS!!!!!!!!! %

% Find Gain
K = 1/ abs(evalfr(G,initial_crossover * 1i));

% Verified magnitude plot crosses 0 db at initial_crossover
% bode(K * G);
% margin(K * G);
% grid on;

alpha = 0.1;

[~, ~, ~, omega_bar] = margin((K * G)/sqrt(alpha));
T = 1/(sqrt(alpha) * omega_bar);

% Define Lead Controller C1
C1 = K * tf([T 1], [alpha * T 1]);
figure
hold on


% Plot bode plots of G, KG & C1* G. 
% Verified crossover frequency and phase margin. Using margin function.

bode(G);
% margin(G)
bode(K * G);
% margin(K * G)
bode(C1 * G);
% margin(C1* G)
title('Bode Plots of G, K * G & C1 * G')
legend('G','K * G','C1 * G');
grid on




% Display bode plots of C1 * G with its gain & phase margin
% figure
% bode(C1 * G);
% margin(C1 * G);
% grid on

%%% Design PI Controller %%%
[Gm, Pm, Wcg, Wcp] = margin(C1 * G);

TI = 20/Wcp; % TUNE THIS!!!!!!!!! %

C2 = tf ([TI 1], [TI 0]);

C = C1 * C2;

% Verifyied magnitude plot of G/ (1+CG) is below -34 dB using bode plots
figure
bode (G/(1 + C * G));
margin(G/(1 + C * G));
grid on
% Define T_R & T_D
T_R = C * G / (1 + C * G);
T_D = G / (1 + C * G);
T_R = minreal(T_R);
T_D = minreal(T_D);


figure 

subplot (2,1,1);
step(thetades * T_R);
title('Step Response of thetades * T_R')
grid on

subplot(2,1,2);
step(thetades * T_D);
title('Step Response of thetades * T_D')
grid on

stepinfo(thetades * T_R)

Kaw =  1.3/TI; %%% TUNE THIS

% Define motor variables
La = 0.02; Ra = 3; Ke = 0.01; Kt = 0.01; I = 6e-4; b = 1e-4;

% Motor model matrices
A = [0 1 0; 0 -b/I Kt/I ; 0 -Ke/La -Ra/La];
B = [0 ; 0 ; 1/La];
C = [0 1 0];
D = 0;

% simplified_motor model matrices
A1 = [0 1; 0 -(b + (Ke*Kt/Ra))/I];
B1 = [0 ; Kt/(Ra*I)];
C1 = [0 1];
D1 = 0;

% define state space model
motor = ss(A, B, C, D);
motor_simplified = ss(A1, B1, C1, D1);

% convert objects to transfer functions
G_motor = tf(motor);
G_motor_simplified = tf(motor_simplified);

% convert object to zpk form
zpk_motor = zpk(motor);

% extract list of poles of transfer function G_motor
[z, p, k] = zpkdata(G_motor);
poles = cell2mat(p);

% extract numerator and denominator arrays from the object G_motor
[num,den] = tfdata(G_motor);
num = cell2mat(num);
den = cell2mat(den);

% extract numerator and denominator arrays from the object
% G_motor_simplified
[num1,den1] = tfdata(G_motor_simplified);
num1 = cell2mat(num1);
den1 = cell2mat(den1);

%%% FIRST FIGURE %%%
% Step Response of motor versus T
T = linspace(0,30,1000);
subplot(311);
[Y1,T,X1] = step(motor, T);
plot(T, Y1);
title('Step Response of motor');
xlabel('Time') 
ylabel('Y1') 

% Step Response of motor_simplified versus T
subplot(312);
[Y2,T,X2] = step(motor_simplified, T);
plot(T, Y2);
title('Step Response of simplified motor');
xlabel('Time') 
ylabel('Y2') 

% Y1 - Y2 
subplot(313);
plot(T, Y1-Y2);
title('Difference');
xlabel('Time') 
ylabel('Y2 - Y1') 
% Compare with asymptotic value calculated by Final Value Theorem 

%%% SECOND FIGURE %%%
% The armature current is just the third column of the matrix X when 
% we use the step function. [Y,T,X]=step(sys,T).

figure;
plot(T,X1(:, end));
title('Armature Current vs Time');
xlabel('Time') 
ylabel('Armature Current I_a') 

%%% THIRD FIGURE %%%
% The initial condition is X0=[0;-1;.5]
% And the input signal is sinusoidal

figure;
plot(T,(lsim(motor,sin(T),T,[0;-1;5])));
title('Output Response vs Time');

xlabel('Time') 
ylabel('Output Response') 

% Amplitude: ~5.45
amp = abs(evalfr(G_motor, 1i));

% initialize constant variables
M = 1000;  B = 1; g = 9.81; a = B/M;
vdes =14;
theta = -pi/6;
dbar = g * sin(theta);

% Define LTI object G
G = tf (1,[1 a]);

% Define variables p1,p2 meeting Spec 5: p1 + p2 < 2.144
p1 = 1.07; 
p2 = 1.07;

% Define PI control parameters in terms of p1 and p2
K = p1 + p2 - a;
Tl = (p1+p2-a)/(p1*p2);

% Define LTI object C
zeros = [-1/Tl];
poles = 0;
gain = K;
C = zpk(zeros,poles,gain);

% Define LTI object T representing Y(s)/R(s).
% Perform pole-zero cancellation in T
T = (C*G)/(1+C*G);
T = minreal(T);

% Extract poles of T and verify that they are the same as p1 & p2
[z,p,k] = zpkdata(T);
poles_of_T = cell2mat(p);
% Define constants
La = 0.05; Ra = 3; M = 0.1; km = 0.1; g = 9.81;

% equilibrium of y,x,u
ybar = 0.1;
xbar = [ybar ; 0 ;ybar * sqrt(M*g/km) ];
ubar = Ra * sqrt(M*g/km) * ybar;

% Find numerically derived matrices A,B,C,D 
[A,B,C,D] = linmod('lab2_1',xbar,ubar);

% Declare theoretical matrices A1,B1,C1,D1 
A1 = [ 0 1 0; 
       2*g/ybar 0  -2 * km * sqrt(M*g/km)/(M*ybar);
       0 0 -Ra/La];
   
B1 = [0; 0; 1/La];

C1 = [1 0 0];

D1 = 0;

% Compute errors between theoretical and numerically derived matrices
A_diff = norm (A-A1);
B_diff = norm (B-B1);

% Find the transfer function from the State-Space model
% Name the transfer function G
magball_ss = ss(A1, B1, C1, D1);
G = tf(magball_ss);

% Extract the poles from the transfer function's zpk form
[z, p, k] = zpkdata(G);
poles = cell2mat(p);

% eigenvalues of A1
% poles are subset of eigenvalues
eig_A1 = eig(A1);

% Introduce LTI system object with given parameters
% K value is the final K value that we found which stabilizes the output at
% y(0) =0.15m
CONTROLLER = zpk(-10,-100,100);

% Finding the poles of the closed-loop system transfer function
[z1,p1,k1] = zpkdata(1- CONTROLLER*G);
zeros = cell2mat(z1);
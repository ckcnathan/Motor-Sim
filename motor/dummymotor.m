
name = 'dummy motor';

% Resistance %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r_a = 0.2;  %phase a resistance (ohms)
r_b = 0.2;  %phase b resistance (ohms)
r_c = 0.2;  %phase c resistance (ohms)

R = [r_a,0,0;0,r_b,0;0,0,r_c];  %resistance matrix

% Inductance %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% lookup table based to account for current dependence
i_l_ref = [0;200];              %current reference for lookup table
l_d_lut = [0.005;0.005001];      %D-Axis Inductance
l_q_lut = [0.005;0.005001];      %Q-Axis Inductance
l_m = 0;            %Phase Mutual Inductance, assuming a constant for now
l_d = @(id) interp1(i_l_ref,l_d_lut,abs(id));      
l_q = @(iq) interp1(i_l_ref,l_q_lut,abs(iq));

% self inductance calculation from dq inductance
l_self = @(theta_r, phase, id, iq) 0.5*((l_d(id)-l_q(iq))*cos(2*(theta_r + phase))+(l_d(id)+l_q(iq)));
% Inductance matrix
L = @(theta_r,id,iq) [l_self(theta_r, 0, id, iq), l_m, l_m; l_m, l_self(theta_r, -2*pi/3, id, iq), l_m; l_m, l_m, l_self(theta_r, 2*pi/3, id, iq)];

% Rotor pm flux linkage %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% back emf harmonic coefficients
k1 = 0.023; 
k2 = 0;
k3 = 0;
k4 = 0;
k5 = 0;
k6 = 0;
k7 = 0;
k8 = 0;
k9 = 0; 

% flux linkage %
% sinusoidal, can be made to any arbitrary shape using lookup table.
pm = @(theta_r, phase) k1*cos(theta_r + phase)+k2*cos(2*theta_r + phase)+k3*cos(3*theta_r + phase)+k4*cos(4*theta_r + phase)+k5*cos(5*theta_r + phase)+k6*cos(6*theta_r + phase)+k7*cos(7*theta_r + phase)+k8*cos(8*theta_r + phase)+k9*cos(9*theta_r + phase);
pm_r = @(theta_r) [pm(theta_r,0);pm(theta_r,-2*pi/3);pm(theta_r,2*pi/3)];   % pm flux linkage vector.

% Mechanical properties %%%%%%%%%%%%%%%%%%%%%%%%%%%
J = 0.1;    % moment of inertial of rotor;
B = 0;      % rotor friction;
mech_eff = 1;   % mechanical efficiency
ppairs = 7;     % pole pairs
termination = "wye";    % termination type, either delta and wye.





%   
% l_p = @(theta_r, theta_p, id, iq) .5*(l_d - l_q)*(cos(2*(-theta_r + theta_p)))+(l_d + l_q)/2;
% 
% L = @(theta_r, id, iq) [l_p(theta_r, 0, id, iq), l_m, l_m; l_m, l_p(theta_r, 2*pi/3, id, iq), l_m; l_m, l_m, l_p(theta_r, -2*pi/3, id, iq)]; 
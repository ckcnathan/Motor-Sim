function [V_uvw,I_int,I_error] = PI_current_controller(I_ref,I_old,theta,I_int,I_max,V_bus)
% Inputs:
% I_ref = Reference current [A]
% I_old = Current feedback [A]
% theta = Rotor position [rad]
% I_int = Current error integral [A]
% I_max = Current limit [A]
% V_bus = Bus voltage [V]
%
% Outputs:
% V_uvw = terminal voltages, 3x1 vector [V]
% I_int = Current error integral [A]
%
% Desciprtion: 
% Closed loop PI current controller.
%

ki = 0.0247;    %0.0247
k = 6.3621;          %6.3621

I_error = I_ref-I_old;

I_int = k*ki*I_error+I_int;
% if abs(I_int)>I_max
%     I_int = sign(I_int)*I_max;
% end

V_dq0 = k*I_error + I_int;
V_abc = abc(theta)*V_dq0;
V_uvw = V_abc - 0.5*(min(V_abc)+max(V_abc));
V_uvw = 0.5*(V_uvw + V_bus);



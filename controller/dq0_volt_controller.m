function [V_uvw] = dq0_volt_controller(V_bus,theta,V_d,V_q)
% Inputs:
% V_bus = DC bus voltage [V]
% theta = rotor position [rad]
% V_d = d-axis voltage (control input) [V]
% V_q = q-axis voltage (control input) [V]
%
% Outputs:
% V_uvw = terminal voltages, 3x1 vector [V]
%
% Desciprtion: 
% Control motor by setting d and q axis voltages, open loop.
%

V_dq0 = [V_d;-V_q;0];
V_abc = dq0(theta)\V_dq0;
V_uvw = V_abc - 0.5*(min(V_abc)+max(V_abc));
V_uvw = V_uvw*0.5 + 0.5*V_bus;
V_uvw = max(min(V_uvw,V_bus),0);
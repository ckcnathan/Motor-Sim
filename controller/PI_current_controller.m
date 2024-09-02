function [V_uvw,I_int] = PI_current_controller(I_ref,I_old,theta,I_int,I_max,V_bus)


ki = 0.0247;
k = 6.3621;

I_error = I_ref-I_old;

I_int = k*ki*I_error+I_int;
if abs(I_int)>I_max
    I_int = sign(I_int)*I_max;
end

V_dq0 = k*I_error + I_int;
V_abc = abc(theta)*V_dq0;
V_uvw = V_abc - 0.5*(min(V_abc)+max(V_abc));
V_uvw = 0.5*(V_uvw + V_bus);
V_uvw = max(min(V_uvw,V_bus),0);



clear
clc

% main script for running the simulator, duplicate for different scenarios

% Motor %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Select motor by running the script containing it's parameters
motor = 'dummymotor';
run(strcat('motor/', motor))

% foc functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath('Functions\');
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% add controller folder path %%%%%%%%%%%%%%%%%%%%
addpath("controller\");
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% time %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_end = 5;  %end time [s]
sample_fq = 20000;  %sampling frequency [Hz]
dt = 1/sample_fq;   %sampling period [s]
output.time = 0:dt:t_end;    %time vector
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i_abc = [0;0;0];
theta_e = 0;
theta = 0;
dtheta = 0.0001;
ddtheta = 0;

dq0_tf = dq0(theta_e);
i_dq0 = dq0_tf*i_abc;
L_mat_old = L(theta_e,i_dq0(1),i_dq0(2));
pm_mat_old = pm_r(theta_e);

% controller specific
i_int = [0;0;0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
for k=1:length(output.time)

    V_bus = 24;
    trq_load = 0;

    dq0_tf = dq0(theta_e);
    abc_tf = abc(theta_e);

    % Controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % V_uvw = dq0_volt_controller(V_bus,dq0_tf,0,1);

    % if output.time(k)>1
    %    i_ref = [0;5;0];
    % else
    %    i_ref = [0;0;0];
    % end
    i_ref = [0;5*sin(output.time(k));0];
    [V_uvw,i_int,i_error] = PI_current_controller(i_ref,i_dq0,abc_tf,i_int,60,V_bus);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    i_dq0 = dq0_tf*i_abc;

    R_mat = R;
    L_mat = L(theta_e,i_dq0(1),i_dq0(2));
    pm_mat = pm_r(theta_e);
    dL_mat = (L_mat-L_mat_old)./dt;     % Inductance derivative matrix [3x3]
    dpm_mat = (pm_mat-pm_mat_old)./dt;  % pm flux linkage derivative vector [3x1]

    if strcmp(termination,'wye')
        R_solv = [R_mat,[0;0;0];[0,0,0,0]];
        L_solv = [L_mat,[1;1;1];[1,1,1,0]];
        dL_solv = [dL_mat,[0;0;0];[0,0,0,0]];
        dpm_solv = [dpm_mat;0];
        i_solv = [i_abc;0];
        V_solv = [V_uvw;0];
    elseif strcmp(termination,'delta')
        R_solv = R_mat;
        L_solv = L_mat;
        dL_solv = dL_mat;
        dpm_solv = dpm_mat;
        i_solv = i_abc;
        V_solv = [V_uvw(1)-V_uvw(2);V_uvw(2)-V_uvw(3);V_uvw(3)-V_uvw(1)];
        V_abc = V_solv;
    else
        disp('Termination type error')
        return
    end

    % % forward euler's method %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    di_abc = L_solv\(V_solv-R_solv*i_solv-dL_solv*i_solv-dpm_solv);
    i_abc = di_abc(1:3).*dt+i_abc;
    i_dq0 = dq0_tf*i_abc;
    if strcmp(termination,'wye')
        V_abc = V_uvw-di_abc(4); % V_uvw-V_n
    end
    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % ode45 %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % tspan = [0 dt];
    % [t,i_out] = ode45(@(t,i_solv) solvplant45(L_solv,V_solv,R_solv,dL_solv,i_solv,dpm_solv),tspan,i_solv);
    % i_abc = i_out(end,1:3)';
    % i_dq0 = dq0_tf*i_abc;
    % if strcmp(termination,'wye')
    %     V_abc = V_uvw-i_out(end,4); % V_uvw-V_n
    % end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % POWER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    p_elec = V_abc'*i_abc;   % electrical power
    %p_elec_bemf = dpm_mat*i_abc;   % power "used" by back emf
    %p_elec_rel = dL_mat*i_abc*i_abc;   % power "used" by reluctance

    % mechanical power = power "used" by back emf + reluctance %%%%%%%%%%%%
    % p_mech1 = mech_eff*(dpm_mat(1:3)'*i_abc+(dL_mat(1:3,1:3)*i_abc)'*i_abc);
    % trq1 = p_mech/dtheta;        % mechanical torque
      % different method (mech power), same results %%%%%%%%%%%%%%%%%%%%%%%%%
    pm_f = dq0_tf*pm_mat;
    trq = mech_eff*(1.5)*ppairs*(pm_f(1)+(l_d(i_dq0(1))-l_q(i_dq0(2)))*i_dq0(1))*i_dq0(2);
    p_mech = trq*dtheta;

    ddtheta = (trq+trq_load-dtheta*B)/J;        % rotor angular acceleration [rad/s^2]
    dtheta = dtheta+ddtheta*dt;     % rotor angular velocity [rad/s]
    theta = theta+dtheta*dt;        % mechanical angle [rad]
    theta = mod(theta,2*pi);    
    theta_e = theta_e+ppairs*dtheta*dt; % electrical angle [rad]
    theta_e = mod(theta_e,2*pi);

    L_mat_old = L_mat;
    pm_mat_old = pm_mat;

    output.V.bus(k) = V_bus;
    output.V.uvw(k,:) = V_uvw;
    output.V.abc(k,:) = V_abc;
    output.I.abc(k,:) = i_abc;
    output.I.dq0(k,:) = i_dq0;
    output.theta(k) = theta;
    output.thetaE(k) = theta_e;
    output.dtheta(k) = dtheta;
    output.ddtheta(k) = ddtheta;
    output.power.elec(k) = p_elec;
    output.power.mech(k) = p_mech;
    % output.power.mech1(k) = p_mech1;  
    output.torque(k) = trq;
    % output.torque1(k) = trq1;

    output.I.error(k,:) = i_error;
    output.I.ref(k,:) = i_ref;
end
toc

%%
plot(output.time,output.I.dq0)
hold on
plot(output.time,output.I.ref)
grid on




%%
function [di_abc] = solvplant45(L_solv,V_solv,R_solv,dL_solv,i_solv,dpm_solv)
    di_abc = L_solv\(V_solv-R_solv*i_solv-dL_solv*i_solv-dpm_solv);

end
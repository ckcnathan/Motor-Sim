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
sample_fq = 10000;  %sampling frequency [Hz]
dt = 1/sample_fq;   %sampling period [s]
output.time = 0:dt:t_end;    %time vector
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% initialization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i_abc_old = [0;0;0];    % initial abc current [A]
theta_old = 0;        % initial rotor position [rad]
dtheta_old = -0.0001;   % initial rotor velocity [rad/s] (cannot be 0)
i_dq0_old = dq0(theta_old)*i_abc_old;   % initial dq0 current [A]

R_mat_old = R;      % Resistance matrix [3x3]
L_mat_old = L(theta_old,i_dq0_old(1),i_dq0_old(2)); % Inductance matrix [3x3]
pm_mat_old = pm_r(theta_old);       % pm flux linkage vector [3x1]
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
for k = 1:length(output.time)

    V_bus = 10;     % bus voltage (constant for now)
    
    % Controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % dq0 voltage controller, 
    % Set Vd and Vq, open loop.
    V_uvw = dq0_volt_controller(V_bus,theta_old,0,1);  

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    R_mat_new = R;
    L_mat_new = L(theta_old,i_dq0_old(1),i_dq0_old(2));
    pm_mat_new = pm_r(theta_old);
    dL_mat = (L_mat_new-L_mat_old)./dt;     % Inductance derivative matrix [3x3]
    dpm_mat = (pm_mat_new-pm_mat_old)./dt;  % pm flux linkage derivative vector [3x1]

    if strcmp(termination,'wye')
        R_mat = [R_mat_new,[0;0;0];[0,0,0,0]];
        L_mat = [L_mat_new,[1;1;1];[1,1,1,0]];
        dL_mat = [dL_mat,[0;0;0];[0,0,0,0]];
        dpm_mat = [dpm_mat;0];
        i_abc = [i_abc_old;0];
        V_uvw = [V_uvw;0];
    elseif strcmp(termination,'delta')
        R_mat = R_mat_new;
        L_mat = L_mat_new;
        dL_mat = dL_mat;
        dpm_mat = dpm_mat;
        i_abc = i_abc_old;
        V_abc = V_uvw(1:3);
        V_uvw = [V_uvw(1)-V_uvw(2);V_uvw(2)-V_uvw(3);V_uvw(3)-V_uvw(1)];
    else
        disp('Termination type error')
        return
    end

    % euler's method %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    di_abc_mat = L_mat\(V_uvw-R_mat*i_abc-dL_mat*i_abc-dpm_mat);
    i_abc_new = di_abc_mat(1:3).*dt+i_abc_old;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    i_dq0_new = dq0(theta_old)*i_abc_new;

    % mechanical calculations %%%%%%%%%%%%%%%%%%%
    if strcmp(termination,'wye')
        V_abc = V_uvw(1:3)-di_abc_mat(4);
    end
    p_elec_new = ppairs*V_abc'*i_abc_new;   % electrical power
    %p_elec_bemf_new = dpm_mat*i_abc_new;   % power "used" by back emf
    %p_elec_rel_new = dL_mat*i_abc*i_abc;   % power "used" by reluctance

    % mechanical power = power "used" by back emf + reluctance %%%%%%%%%%%%
    p_mech_new = ppairs*mech_eff*(dpm_mat(1:3)'*i_abc_new+(dL_mat(1:3,1:3)*i_abc_new)'*i_abc_new);
    trq_new = p_mech_new/dtheta_old;        % mechanical torque

    % different method (mech power), same results %%%%%%%%%%%%%%%%%%%%%%%%%
    % pm_f = dq0(theta_old)*pm_mat_old;
    % trq_new = (1)*ppairs*(pm_f(1)+(l_d(i_dq0_new(1))-l_q(i_dq0_new(2)))*i_dq0_new(1))*i_dq0_new(2);

    ddtheta_new = trq_new/J;        % rotor angular acceleration [rad/s^2]
    dtheta_new = dtheta_old+ddtheta_new*dt;     % rotor angular velocity [rad/s]
    theta_new = theta_old+dtheta_new*dt;        % rotor angle [rad]
    theta_new = mod(theta_new,2*pi);        

    % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    R_mat_old = R_mat_new;
    L_mat_old = L_mat_new;
    pm_mat_old = pm_mat_new;
    i_abc_old = i_abc_new;
    i_dq0_old = i_dq0_new;
    theta_old = theta_new;
    dtheta_old = dtheta_new;
    ddtheta_old = ddtheta_new;

    output.V.bus(k) = V_bus;
    output.V.uvm(k,:) = V_uvw;
    output.V.abc(k,:) = V_abc;
    output.I.abc(k,:) = i_abc_new;
    output.theta(k) = theta_new;
    output.dtheta(k) = dtheta_new;
    output.ddtheta(k) = ddtheta_new;
    output.power.elec(k) = p_elec_new;
    output.power.mech(k) = p_mech_new;
    output.torque(k) = trq_new;
end
toc
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plot(output.time,output.I.abc)
figure 
plot(output.time,output.theta)




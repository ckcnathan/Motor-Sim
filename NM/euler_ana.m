clear
clc
close

% syms i(t) L R V
%
% ode = L*diff(i,t) + i*R == V*cos(t);
% iSolu = dsolve(ode,i(0)==0);
% simplify(iSolu,teps',50)

L = 0.05;
R = 0.2;

s = tf('s');
%syms s
% fqc = 100000;
% dtc = 1/fqc;
% tc = 0:dtc:25;
% uc = 0.1*tc.*sin(tc);
% Ic = 1/(L*s+R);
% yc = lsim(Ic,uc,tc);
% [pkt(:,1),pkind(:,1)] = findpeaks(yc);
% peakt(:,1) = tc(pkind(:,1));

spvec = [100;500;1000;5000;10000;50000];
for k=1:length(spvec)
    fq = spvec(k);
    dt = 1/fq;
    t = 0:dt:20;
    u = sin(t);
    Ic = 1/(L*s+R);
    yc = lsim(Ic,u,t);

    I_old = 0;
    I = zeros(length(t),1);
    for i=1:length(t)
        V = u(i);
        di = (1/L)*(V - I_old*R);
        I_new = I_old+di*dt;
        I_old = I_new;

        I(i) = I_new;
    end

    error = abs(yc-I)./I;
    err(k) = mean(error);

    plot(t,error)
    hold on
    % [pkt(:,k+1),pkind(:,k+1)] = findpeaks(I);
    % peakt(:,k+1) = t(pkind(:,k+1));
end

legend('100','500','1000','5000','10000','50000')
grid on


% figure
% plot(t,yc)
% grid on
% hold on
% plot(t,u)
% grid on
% plot(t,u)

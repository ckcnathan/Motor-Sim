
% dq0 transform (power invariant form)
dq0 = @(theta) sqrt(2/3)*[cos(theta), cos(theta-(2*pi/3)), cos(theta+(2*pi/3));
    -sin(theta), -sin(theta-2*pi/3), -sin(theta+2*pi/3);
    1/sqrt(2), 1/sqrt(2), 1/sqrt(2)];

% abc transform
abc = @(theta) inv(dq0(theta));

% abc = @(theta) sqrt(2/3)*[cos(theta), sin(-theta), 1/sqrt(2);
%     cos(theta-(2*pi/3)), -sin(theta-(2*pi/3)), 1/sqrt(2);
%     cos(theta+(2*pi/3)), -sin(theta+(-2*pi/3)), 1/(sqrt(2))];
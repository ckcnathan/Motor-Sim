function [dq_mat] = dq0(theta)
% dq0 transform (power invariant form)


dq_mat = sqrt(2/3)*[cos(theta), cos(theta-(2*pi/3)), cos(theta+(2*pi/3));
    -sin(theta), -sin(theta-2*pi/3), -sin(theta+2*pi/3);
    1/sqrt(2), 1/sqrt(2), 1/sqrt(2)];
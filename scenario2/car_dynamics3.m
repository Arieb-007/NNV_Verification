function [dx]=car_dynamics3(x,a_ego)

mu=0.0001; % friction parameter

% x1 = lead_car position
% x2 = lead_car velocity

% x3 = ego_car position
% x4 = ego_car velocity
% x5 = ego_car internal state

% lead car dynamics
a_lead = 0 ; 
dx(1,1)=x(2);
dx(2,1)=-0.5*x(2)+a_lead ;
% ego car dyanmics
dx(3,1)= x(4); 
dx(4,1) = x(5);
dx(5,1) = -2 * x(5) + 2 * a_ego - mu*x(4)^2;

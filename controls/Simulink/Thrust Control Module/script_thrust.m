clear; 
%Desired trajectory
t1 = 0:0.01:5;
t2 = 5.01:0.01:10;
t = [t1, t2];

% x_des = zeros(size(t));
% y_des = zeros(size(t));
z_des1 = 5 * (1 - exp(-t1/1.085));
z_des2 = 5 * ones(1, 500);
z_des = [z_des1, z_des2];

% pos_des= [t' x_des' y_des' z_des'];
pos_des= [t' z_des'];

%Initial position


%mass
m = 1.2;
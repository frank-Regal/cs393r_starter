%% Autonomous Robots Particle Filter
% Predict Step
close all;
clear all;
clc;

%% Spoofed State and Odom Data

points = 10;

x = 0:0.5:5;
y = 0:0.1:max(x);
state = zeros(length(x),3);
odom = zeros(length(y),3);
var = rand(1,length(y)).*0.15;


% State
for i = 1:length(x)
    state(i,1) = x(i);
    state(i,2) = x(i)^2 - x(i) - 25*sin(x(i));
    if (i == 1)
        state(i,3) =0;
    else
        state(i,3) = tan(state(i,1)/state(i,2));
    end
end



% Odom
for j = 1:length(y)
    odom(j,1) = y(j)+var(j);
    odom(j,2) = y(j)^2 - y(j) - 25*sin(y(j)) + var(j);
    if (j == 1)
        odom(j,3) =0;
    else
        odom(j,3) = tan(odom(j,1)/odom(j,2));
    end
end

plot(odom(:,1),odom(:,2),"bx");
hold on;
grid on;

%% Initialize

init = 50; % particles
particles{1,length(odom)} = zeros(init,3);
for t = 1:length(odom)
    particles{1,t} = zeros(init,3);
end

for k = 1:init
    particles{1,1}(k,1) = state(1,1) + rng_Gaussian(-0.1,0.1,1);
    particles{1,1}(k,2) = state(1,2) + rng_Gaussian(-0.5,0.5,1);
    particles{1,1}(k,3) = state(1,3) + rng_Gaussian(-0.1,0.1,1);
    plot(particles{1,1}(:,1),particles{1,1}(:,2),'o', 'MarkerFaceColor', 'y');
end
pause(7);
hold on;

%% Predict
h = animatedline('Color','r','Marker','o','MarkerFaceColor','r');
a1 = 1;
a2 = 1;
a3 = 1;
a4 = 1;

for f =1:length(odom)-1
    for e = 1:init-1
        del_x = odom(f+1,1) - odom(f,1);
        del_y = odom(f+1,2) - odom(f,2);
        del_theta = get_angle_diff(odom(f+1,3),odom(f,3));
        del_trans = sqrt(del_x^2 + del_y^2)
        
        
        % Convert to Map Coordinates
        rotation_angle = get_angle_diff(particles{1,f}(e,3), odom(f,3));
        
        % Rotate Previous Odom about Last Known Particle Location
        map_frame_del_x = del_x*cos(rotation_angle) - del_y*sin(rotation_angle);
        map_frame_del_y = del_x*cos(rotation_angle) + del_y*sin(rotation_angle);
        map_frame_del_trans = sqrt(map_frame_del_x^2 + map_frame_del_y^2);
        
        % Add Variance to deltas
        del_x_hat = map_frame_del_x + rng_Gaussian(-(a1*map_frame_del_trans + a2*abs(del_theta)),(a1*map_frame_del_trans + a2*abs(del_theta)),1);
        del_y_hat = map_frame_del_y + rng_Gaussian(-(a1*map_frame_del_trans + a2*abs(del_theta)),(a1*map_frame_del_trans + a2*abs(del_theta)),1);
        del_theta_hat = del_theta + rng_Gaussian(-( a3*map_frame_del_trans + a4*abs(del_theta) ),( a3*map_frame_del_trans + a4*abs(del_theta) ),1);

        particles{1,f}((e+1),1) = particles{1,f}(e,1) + del_x_hat;
        particles{1,f}((e+1),2) = particles{1,f}(e,1) + del_y_hat;
        particles{1,f}((e+1),3) = particles{1,f}(e,1) + del_theta_hat;
    end
    
    plot(particles{1,f}(:,1),particles{1,f}(:,2),'go');
    
    if (rem(f,5) == 0)
        addpoints(h,state(f/5,1),state(f/5,2));
        drawnow
    end
    hold on;
    pause(0.5);
    
end
addpoints(h,state(11,1),state(11,2));
drawnow
% frank = rng_Gaussian(0,0,1)

%%
% 
% axis([0,4*pi,-1,1])
% 
% x = linspace(0,4*pi,1000);
% y = sin(x);
% for k = 1:length(x)
%     addpoints(h,x(k),y(k));
%     drawnow
% end

%% Functions
function [theta]=get_angle_diff(a, b)
  theta = a-b;
  if (theta < -pi())
    while (theta < -pi())
        theta = theta + 2*pi();
    end
  elseif (theta > pi())
    while (theta > pi())
        theta = theta - 2*pi();
    end
  end
end
 
function [rand_num] = rng_Gaussian(min,max,length)
    rand_num = min + (max+max)*rand(length,1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Author:             Manav Wadhawan          mwadhawa@purdue.edu
%%% Course:             ECET 58100
%%% Course Instructor:  Dr. Richard Voyles
%%%
%%% Press F5 to run; or click Run
%%% The following code simulates a very simplified kinematic model of 
%%% ackermann steering commonly used in automobiles 
%%% Ref link: https://www.xarg.org/book/kinematics/ackerman-steering/ 
%%% A simple PD controller is used to main a constant lateral distance. 
%%% For the sake of simplicity we keep the forward velocity constant during 
%%% the control action. The objective of this code was to biuld an
%%% intuition for tuning PD conitrol of an ackerman steered vehicle.
%%%
%%% Controller: Its worthwhile to mention that steering angle is not the 
%%%             same as car angle. Our control input is the steering angle 
%%%             not car angle, The objective of controller(100Hz) is slow the 
%%%             steering angle enough so that car angle catches up to the
%%%             steering angle. To achieve this we have the Derevative component of
%%%             PID. A dead band is also added in the middle, though it is 
%%%             not neccesary. Car dimensions to a 1/10 scale of an actual car.
%%%
%%% Markers:    Box --              Car
%%%             Light Green line -- Car Angle (theta)
%%%             Blue Line --        Steering Angle (phi)        
%%%             Red Line --         Lateral step or goal
%%%             All dimensions are in meters
%%%             Car angle and Steering angle plotted at origin. 
%%%
%%% Output: To record a video change "vid_recorder" from 0 to 1. The file
%%%         name by default is 'ackermann_simulator.avi'.
%%% 
%%% Testing:        1) Kp = 1.5, Kd = 25  demos a working control ( but not best)
%%%                 2) Kp = 1.5, Kd = 30  faster settling time but with 
%%%                    steady state error         
%%%                 3) Kp = 1.5, Kd = 5   demos the need of Kd               
%%%                 4) change 'speed' to visualize the affect of car speed on
%%%                    controller
%%%                 5) change 'goal' to change the goal or lateral step of the car
%%%
%%% Remarks: The setting point as default is 1.5 m given that car width is 
%%% only 0.15m the goal is extermenly large for any controller to reach.
%%% a more pragmatic value of 'goal' would be 0.5m. 
%%%
%%% Rev: 1.0.0          First Commit
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clear workspace and graphs
clf;
clc;
%%% PID parameters
Kp = 1.5;           
Kd = 30;
err_old = 0;
del_t = 0.01;                   % controller frequency of 100Hz
%%% Vehicle Parameters
speed = 5.2;                    % Car speed = 5.2 m/s
goal = 1.5;                     % Wall distance = 1.5 m
car_length = 0.3;               % length of vehicle
%%% Kinematic model : X_dot = AX + BU 
phi = 0;
theta = 0;
A = [1  0  0;
     0  1  0;
     0  0  1];
B = [del_t*cos(theta)   0              0;
     0           del_t*sin(theta)      0;
     0                   0      del_t/car_length;];
U = [speed; speed; tan(phi)];
vid_recorder = 0;                     % to record a video change to 0 to 1
if vid_recorder == 1
    vidObj = VideoWriter('ackermann_simulation.avi'); open(vidObj); end
% define handle objects for animation [line 71 to line 84]
X = [0; 0; 0;];
i = 1;
X_dot = [];
timer_loc = [5 5];
time_ind = text(timer_loc(1), timer_loc(2), 'Time = 0.00 s', 'FontSize', 10, 'color', 'k');
data_c = [0 3;0 0];
T_c = [cos(phi) -sin(phi); sin(phi) cos(phi)];  % transform matrix for animation
car_dim = [0 0 0.3 0.15];                       % car dimensions to scale
car = rectangle('Position', car_dim, 'FaceColor',[0 0.5 0.5]);
data_steer_angle = [0 1; 0 0]; 
data_car_angle = [0 1; 0 0]; 
steer_angle = line('xdata',data_steer_angle(1,:), 'ydata', data_steer_angle(2,:),'linewidth', 2);
car_angle = line('xdata',data_car_angle(1,:), 'ydata', data_car_angle(2,:),'linewidth', 2);
a1 = -0.5; a2 = 15; a3 = -0.5; a4 = 8;
axis([a1 a2 a3 a4])
axis('square')
xlabel('in meters','Interpreter','latex')
ylabel('in meters','Interpreter','latex')
figure(1)
title('Ackermann Simulator','Interpreter','latex')
line([-5 20],[goal goal],'Color','red','LineStyle','-.');
legend('Steering Angle($\phi$)','Car Angle($\theta$)','Goal','Interpreter', 'latex');
%%% Control Loop
t = 0;
i = 1;
t_final = 3.5;              % total time of simulation
t_all = [];
while (t < t_final)
    % run model
    X(1, i+1) = X(1, i) + speed*del_t*cos(theta);
    X(2, i+1) = X(2, i) + speed*del_t*sin(theta);
    X(3, i+1) = X(3, i) + del_t*tan(phi)/car_length;
    theta = X(3, i);
    % control action
    err = -X(2, i) + goal;
    % adding a dead band for more stability
    if err < 0.07 && err > -0.07  err = 0; end 
    phi = Kp * err + Kd*(err - err_old);
    if phi > 50*pi/180 phi = 50*pi/180; end
    if phi < -50*pi/180 phi = -50*pi/180; end
    err_old = err;
    % time and index propogation
    t = t + del_t;
    i = i + 1;
    %%% animation [line 116 to line 128]
    axis([a1 a2 a3 a4])
    T_car_angle = [cos(theta) -sin(theta);sin(theta) cos(theta)];
    data_new_car_angle = T_car_angle * data_car_angle;
    set(car_angle,'xdata',data_new_car_angle(1,:), 'ydata', data_new_car_angle(2,:), 'color', 'green');
    T_steer_angle = [cos(phi) -sin(phi);sin(phi) cos(phi)];
    data_new_steer_angle = T_steer_angle * data_steer_angle;
    set(steer_angle,'xdata',data_new_steer_angle(1,:), 'ydata', data_new_steer_angle(2,:), 'color', 'blue');
    set(car,'Position', [car_dim(1)+X(1,end) car_dim(2)+X(2,end) car_dim(3) car_dim(4)]);
    set(time_ind,'String', sprintf ('Time = %.2f s', t));
    drawnow;
    grid on;
    hold on
    if t_final - t == del_t
        plot(X(1,:),X(2,:));
    end
    t_all = [t_all t];
    if vid_recorder == 1  currFrame = getframe(gcf); 
        writeVideo(vidObj,currFrame);end
end
hold on 
plot(X(1,:),X(2,:));        % plot path of the car
legend('Steering Angle($\phi$)','Car Angle($\theta$)','Goal','Path','Interpreter', 'latex');
if vid_recorder == 1
     currFrame = getframe(gcf);
     writeVideo(vidObj,currFrame);
     close(vidObj);  
end
vid_recorder = 0;

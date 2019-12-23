%% Author: Manav Wadhawan
%% Modelling
syms x1 x2 x3 x4 m1 m2 g r1 u1 u2 
% x1 = theta; x2 = r; x3 = theta_dot; x4 = r_dot
% The model is as follows
M = [x3;
     x4;
    (-2*m2*x4*x3*x2 -g*cos(x1)*(m1*r1 + m2*x2) + u1)/((m1*(r1^2)) + m2*(x3^2));
    (x3^2)*x2 - g*sin(x1) + u2/m2];
% C matrix -- We have 2 outputs theta=x1 and r=x2
C = [1 0 0 0;
     0 1 0 0];
D = zeros(2, 2);
disp(" The model is as follows: ");
disp(M);
M  = subs(M, {x1, x2, x3, x4, m1, m2, g, r1}, {pi/4, 2, 0, 0, 10, 3, 9.81, 1});
u1_e = double(solve(M(3), u1));
u2_e = double(solve(M(4), u2));

%% Linearizing about equllibrium
A = [x3;
     x4;
    (-2*m2*x4*x3*x2 -g*cos(x1)*(m1*r1 + m2*x2))/((m1*(r1^2)) + m2*(x3^2));
    (x3^2)*x2 - g*sin(x1)];
B = [0;
     0;
     u1/(m1*r1^2 + m2*x3^2);
     u2/m2];
 A_lin = jacobian(A, [x1, x2, x3, x4]);
 B_lin = jacobian(B, [u1, u2]);
 A_lin_e = double(subs(A_lin, {x1, x2, x3, x4, m1, m2, g, r1,}, {pi/4, 2, 0, 0, 10, 3, 9.81, 1}));
 B_lin_e = double(subs(B_lin, {x1, x2, x3, x4,m1, m2, g, r1, u1, u2}, {pi/4, 2, 0, 0,10, 3, 9.81, 1, u1_e, u2_e}));

%% Designing a state feedback controller using the linearized model
p = [-12 -11 -12 -13];          
K = place(A_lin_e, B_lin_e, p);                     % pole placement
eigs(A_lin_e - B_lin_e*K);                          % verifying the placed poles
sys_cl = ss(A_lin_e - B_lin_e*K, B_lin_e, C, D);    % transfer function
tf(sys_cl);
% checking the lyapunov matrix equation for closed loop system which is driven by state feedback controller
Q = eye(4);
P = lyap(A_lin_e - B_lin_e*K, Q);
eig(P);
% all eigen values of the P matrix are positive, Therefore we can say that
% P matrix is positive definite and Closed loop system is asymptotically
% stable.

%% Using linearized model to design a Luenberger Observer

rank(obsv(A_lin_e, C));                         % checking for observability
L = place(A_lin_e', C', [-25 -20 -24 -22])';    

% The system poles are at [-12 -11 -12 -13] we select the observer poles close to twice that of the system poles. 
%% Q10. Animation for the combined controller-observer compensator driving the nonlinear robot.

% const for subs 
m1 = 10;
r1 = 1;
g = 9.81;
m2 = 3;
% define handle objects
cylinder = line('xdata',data_c(1,:), 'ydata', data_c(2,:),'linewidth', 6);
hinge = line('xdata',0, 'ydata', 0,'linewidth', 3);
rod = line('xdata',data_r(1,:), 'ydata', data_r(2,:),'linewidth', 3);
% time consts
t = 0;
dt = 0.01;
t_final = 8;
% system initial conditions
theta = pi/2;    
r = 1.5;
theta_dot = 0;
r_dot = 0;
% equillbrium 
Xe = [pi/4 2 0 0]';
Ue = [u1_e u2_e]';
Ye = C * Xe;
% observer initial condition is set to zero.
Z = [0 0 0 0]';
% ----------- Animation and other indicators-------------------------------
vid_recorder = 0;
if vid_recorder == 1
vidObj = VideoWriter('Observer.avi'); open(vidObj); end
clf;
figure(1);
data_c = [0 1; 0 0];   
data_r = [0 2; 0 0];  
phi = 0;
T_c = [cos(phi) -sin(phi); sin(phi) cos(phi)];
data_c = T_c * data_c;
axis([-0.5 4 -0.5 4])
axis('square')
timer_loc = [0 3.75];
text_blk1_loc = [0 2.75];
text_blk2_loc = [2 2.75];
box_pos = [0.8 -0.9 2.2 1.05];
cylinder = line('xdata',data_c(1,:), 'ydata', data_c(2,:),'linewidth', 6);
rod = line('xdata',data_r(1,:), 'ydata', data_r(2,:),'linewidth', 3);
time_ind = text(timer_loc(1), timer_loc(2), 'Time = 0.00 s', 'FontSize', 10, 'color', 'k'); 
states_ind = text(text_blk1_loc(1), text_blk1_loc(2), {['$\theta = $', sprintf('%.4f', theta/pi*180, '$\radian$')], ...
['$r = $', sprintf('%.4f', r, 'm')], ...
['$\dot{\theta} = $', sprintf('%.4f'), theta_dot, '$\radian$ /s'], ... 
['$\dot{r} = $', sprintf('%.4f'), r_dot, '$m/s$']}, 'FontSize', 10, 'color', 'k');
est_ind = text(text_blk2_loc(1), text_blk2_loc(2), {['$\hat{\theta} = $', sprintf('%.4f', theta/pi*180, '$\radian$')], ...
['$\hat{r} = $', sprintf('%.4f', r, 'm')], ...
['$\dot{\hat{\theta}} = $', sprintf('%.4f'), theta_dot, '$\radian$ /s'], ... 
['$\dot{\hat{r}} = $', sprintf('%.4f'), r_dot, '$m/s$']}, 'FontSize', 10, 'color', 'k');
% initialization for plotting
t_all = [];
r_all = [];
i = 1;
Z_all = [];
Xest_all = [];
X_all = [];
theta_all = [];
while (t < t_final)
    t = t + dt;
    X = [theta r theta_dot r_dot]'; 
    Y = C*X;
    U = -K*Z + Ue;
    % model
    theta = theta + theta_dot*dt;
    theta_dot = theta_dot + dt*(-2*m2*theta_dot*r*r_dot -g*cos(theta)*(m1*r1 + m2*r) + U(1))/((m1*(r1^2)) + m2*(r^2));
    r = r + r_dot*dt;
    r_dot = r_dot + dt*(U(2)/m2 - 9.81*sin(theta) + m2*(theta_dot^2)*r);    
    % observer
    Z = Z + dt*(( A_lin_e-L*C)*Z + B_lin_e*(U-Ue) + L*(Y - Ye));
    Xest = Z + Xe;
    %------------------------ animation------------------------------------
    T_c = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    data_new_c = T_c * data_c;      % transforming cylinder to x,y
    data_r = [0 r; 0 0];
    data_new_r = T_c * data_r;      % transforming rod to x,y
    set(cylinder, 'xdata',data_new_c(1,:) ,'ydata',data_new_c(2, :), 'color', 'red');
    set(rod,'xdata',data_new_r(1,:), 'ydata', data_new_r(2,:), 'color', 'blue');
    set(time_ind,'String', sprintf ('Time = %.2f s', t), 'Interpreter','latex');
    set(states_ind, 'String', {['$\theta = $', sprintf('%.4f', theta/pi*180), ' $^\circ$'],... 
    ['$r = $', sprintf('%.2f', r), ' m'],... 
    ['$\dot{\theta} = $', sprintf('%.2f', theta_dot), ' $^\circ$/s'],... 
    ['$\dot{r} = $', sprintf('%.2f', r_dot), ' m/s']}, 'Interpreter', 'latex');
    set(est_ind, 'String', {['$\hat{\theta} = $', sprintf('%.4f', Xest(1)/pi*180), ' $^\circ$'],... 
    ['$\hat{r} = $', sprintf('%.2f', Xest(2)), ' m'],... 
    ['$\dot{\hat{\theta}} = $', sprintf('%.2f', Xest(3)), ' $^\circ$/s'],... 
    ['$\dot{\hat{r}} = $', sprintf('%.2f', Xest(4)), ' m/s']}, 'Interpreter', 'latex');  
    grid on;
    drawnow;
    % collecting data for plotting
    Z_all = [Z_all Z];
    X_all = [X_all X];
    t_all = [t_all t]; 
    Xest_all = [Xest_all Xest];
    i = i + 1;
    % frames for video
    if vid_recorder == 1  currFrame = getframe(gcf); 
    writeVideo(vidObj,currFrame);end
    
end
if vid_recorder == 1; close(vidObj); 
end

%% Q 11. Plotting $\theta$ and $r$ vs time
f2 = figure;
subplot(2, 1, 1);
plot(t_all,X_all(1,:))
title('$\theta$ vs time','Interpreter','latex')
xlabel('time in seconds','Interpreter','latex')
ylabel('$\theta$ in radians','Interpreter','latex')
xlim([0 3])

subplot(2, 1, 2);
plot(t_all, X_all(2,:))
title('$r$ vs time','Interpreter','latex')
xlabel('time in seconds','Interpreter','latex')
ylabel('$r$ in meters','Interpreter','latex')
xlim([0 3])

%% Q 12. Plotting State estimate and State vs Time
subplot(4, 1, 1);
plot(t_all, Xest_all(1,:), t_all, X_all(1,:))
title('$\theta$ estimate vs $\theta$','Interpreter','latex')
legend ('$\theta$ estimate ','$\theta$ ','Interpreter','latex')
xlim([0 3])

subplot(4, 1, 2);
plot(t_all, Xest_all(2, :), t_all, X_all(2,:))
title('$r$ estimate vs $r$ ','Interpreter','latex')
legend ('$r$ estimate','$r$ ','Interpreter','latex')
xlim([0 3])

subplot(4, 1, 3);
plot(t_all, Xest_all(3, :), t_all, X_all(3,:))
title('$\dot{\theta}$  estimate vs $\dot{\theta}$','Interpreter','latex')
legend ('$\dot{\theta}$ estimate','$\dot{\theta}$','Interpreter','latex')
xlim([0 3])

subplot(4, 1, 4);
plot(t_all,Xest_all(4, :), t_all, X_all(4,:))
title('$\dot{r}$ estimate vs $\dot{r}$','Interpreter','latex')
legend ('$\dot{r}$ estimate','$\dot{r}$','Interpreter','latex')
xlim([0 3]) 
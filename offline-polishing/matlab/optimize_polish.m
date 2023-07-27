%% Find the optimal solution for input data of the dataset

% change the input file to view another solution
load_data

%% Optimization Problem
% frequency of the dataset
dt = 1/50; 

%%%% You can smooth the data (optional)
data = F2{1}';
% data(:,1) = smooth(data(:,1),10);
% data(:,2) = smooth(data(:,2),10);

%%% ------------------ ATTENTION --------------------- %%%
%%% sometimes you need to flip the data (why?)
% data(:,1) = flip(data(:,1));
% data(:,2) = flip(data(:,2));

Xdata = data(1:end-1,:);
Xvel = diff(data) ./ dt;

% convert to polar coordinates
r(:,1) = sqrt(sum(Xdata.^2,2));
r(:,2) = atan2(Xdata(:,2),Xdata(:,1));

% convert to polar velocity coordinates
dr(:,1) = sum(Xdata.*Xvel,2) ./ sqrt(sum(Xdata.^2,2));
dr(:,2) = (Xvel(:,2).*Xdata(:,1) - Xdata(:,2).*Xvel(:,1)) ./ sum(Xdata(:,1:2).^2,2);

%% Ellipsoid - regression option with constraints 

xdot_r1 = Xvel(:,1); %dr(:,1).*cos(r(:,2)) - r(:,1).*dr(:,2).*sin(r(:,2));
xdot_r2 = Xvel(:,2); %dr(:,1).*sin(r(:,2)) + r(:,1).*dr(:,2).*cos(r(:,2));

% Initial parameters
% alpha, omega, radius, a, b theta
p0 = [10, 1, 0.001, 0.001, 0.1, 1, 0, 0];

x_hat =  @(p) p(4).*cos(p(6)).*(Xdata(:,1) + p(7)) + p(4).*sin(p(6)).*(Xdata(:,2) + p(8)); 
y_hat =  @(p) -p(5).*sin(p(6)).*(Xdata(:,1) + p(7)) + p(5).*cos(p(6)).*(Xdata(:,2) + p(8));

r_ = @(p) sqrt(x_hat(p).^2 + y_hat(p).^2);
phi = @(p) atan2(y_hat(p),x_hat(p));

r_dot = @(p) -1*p(1)*(r_(p) - p(3));
phi_dot = @(p) p(2); % rads per sec

% Limit Cycle Dynamical System in Polar Coordinates
xd_hat =  @(p) r_dot(p).*cos(phi(p)) - r_(p).*phi_dot(p).*sin(phi(p));
yd_hat =  @(p) r_dot(p).*sin(phi(p)) + r_(p).*phi_dot(p).*cos(phi(p));

% Dynamical System diffeomorphism (transformation matrix)
xdot_d1 = @(p) cos(p(6))*(1/p(4))*xd_hat(p) - sin(p(6))*(1/p(5))*yd_hat(p);
xdot_d2 = @(p) sin(p(6))*(1/p(4))*xd_hat(p) + cos(p(6))*(1/p(5))*yd_hat(p);
    
objective = @(p) sum(((r_(p) - r(:,1))/norm(r(:,1))).^2) + sum(((phi(p) - r(:,2))/norm(r(:,2))).^2) + ...
   sum(((xdot_d1(p) - xdot_r1)/norm(xdot_r1)).^2) + sum(((xdot_d2(p) - xdot_r2)/norm(xdot_r2)).^2) + ...
   sum(((r_dot(p) - dr(:,1))/norm(dr(:,1))).^2);

disp(['Initial Objective: ' num2str(objective(p0))])
% linear constraints
A = [];
b = [];
Aeq = [];
beq = [];
% bounds
lb = [ 10,  -2*pi, 0.0001,  1,     1,   -2*pi, -2, -2];
ub = [ 100,  2*pi,   0.09,   inf,  inf,   2*pi,  2,  2];

% compute the solver time
tic
popt = fmincon(objective,p0,A,b,Aeq,beq,lb,ub);
toc

disp(['Final Objective: ' num2str(objective(popt))])
disp('Optimal Parameters:');
disp([num2str(char(945)) 9 num2str(char(969)) 9 'r0' 9 'a' 9 'b' 9 num2str(char(952)) 9 'x1' 9 'x2']);
disp([num2str(round(popt(1),3)) 9 num2str(round(popt(2),3)) 9 num2str(round(popt(3),3)) ...
    9 num2str(round(popt(4),3)) 9 num2str(round(popt(5),3)) 9 num2str(round(popt(6),3)) ...
    9 num2str(round(popt(7),3)) 9 num2str(round(popt(8),3))]);

%% Plotting the full ellipse with all parameters

fig = figure('name','Streamlines');

xgrid = -popt(7);
ygrid = -popt(8);

plot_space = 0.1;      % How zoom out you want the plot to be?

[x, y] = meshgrid(xgrid-popt(3)-plot_space:0.01:xgrid+popt(3)+plot_space, ygrid-popt(3)-plot_space:0.01:ygrid+popt(3)+plot_space);

% Limit Cycle
% define variables
alpha = popt(1);
r0 = popt(3);        % radius = sqrt(r0)

% diffeomorphism
x_hat = popt(4).*cos(popt(6)).*(x + popt(7)) + popt(4).*sin(popt(6)).*(y + popt(8)); 
y_hat = -popt(5).*sin(popt(6)).*(x + popt(7)) + popt(5).*cos(popt(6)).*(y + popt(8));

r_desired = sqrt(x_hat.^2 + y_hat.^2);
phi_desired = atan2(y_hat,x_hat);

r_dot_desired = -1*alpha*(r_desired-r0);
phi_dot_desired = popt(2); % rads per sec

% Limit Cycle Dynamical System in Polar Coordinates
xd_hat =  r_dot_desired.*cos(phi_desired) - r_desired.*phi_dot_desired.*sin(phi_desired);
yd_hat =  r_dot_desired.*sin(phi_desired) + r_desired.*phi_dot_desired.*cos(phi_desired);

% Dynamical System diffeomorphism (transformation matrix)
xd = cos(popt(6)).*popt(4)^(-1).*xd_hat - sin(popt(6)).*popt(5)^(-1).*yd_hat;
yd = sin(popt(6)).*popt(4)^(-1).*xd_hat + cos(popt(6)).*popt(5)^(-1).*yd_hat;

streamslice(x, y, xd, yd, 3);
axis([xgrid-popt(3), xgrid+popt(3), ygrid-popt(3), ygrid+popt(3)]);

hold on;
plot(data(:,1), data(:,2), '.r');

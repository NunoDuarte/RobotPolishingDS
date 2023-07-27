clear r
clear dr

%%
% frequency
dt = 1/50; 

%%%% smoothing the data
data = F2{1}';
% data(:,1) = smooth(data(:,1),10);
% data(:,2) = smooth(data(:,2),10);
% sometimes you need to smooth with more datapoints

%%% ------------------ ATTENTION --------------------- %%%
%%% sometimes you need to flip the data (why?)
% data(:,1) = flip(data(:,1));
% data(:,2) = flip(data(:,2));

% %%% ------------------ ATTENTION --------------------- %%%
% %%% -- IN CASE YOU ARE RUNNING EXAMPLE TRAJECTORIES -- %%%
% %%% running perfect data
% data = CircleUnit{1}';

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

plot(xdot_r1, xdot_r2, '.r');

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

tic
popt = fmincon(objective,p0,A,b,Aeq,beq,lb,ub);
toc

% try out our initial values p0
hold on;
plot(xdot_d1(p0), xdot_d2(p0), '.b');
plot(xdot_d1(popt), xdot_d2(popt), '.g');
legend('measured', 'initial predicted', 'optimal predicted')
ylabel('y')
xlabel('x')

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

% Limit Cycle - circle
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

%% For adding to paper

fig.Units               = 'centimeters';
fig.Position(3)         = 8;
fig.Position(4)         = 6;

set(fig.Children, ...
    'FontName',     'Times', ...
    'FontSize',     9);
set(gca,'LooseInset', max(get(gca,'TightInset'), 0.02))

%% plot optimal polishing motion

alpha = popt(1);
r0 = popt(3);        % radius = sqrt(r0)
omega = popt(2);
theta = popt(6);      % angle rotation (radians)
a = popt(4);         % scaling coefficients
b = popt(5);
x1 = -popt(7);        % translation coefficients
x2 = -popt(8);      % radius = sqrt(r0)

xtest = data(1,1);
ytest = data(1,2);

figure()
xunit = [];
yunit = [];

for i=1:300
    hold on;
    
    plot(xtest,ytest, '.r');    

    % diffeomorphism
    x_hat = a.*cos(theta).*(xtest - x1) + a.*sin(theta).*(ytest - x2); 
    y_hat = -b.*sin(theta).*(xtest - x1) + b.*cos(theta).*(ytest - x2);

    r_ = sqrt(x_hat.^2 + y_hat.^2);
    phi = atan2(y_hat,x_hat);

    r_dot = -1*alpha*(r_-r0);
    phi_dot = omega; % rads per sec

    % Limit Cycle Dynamical System in Polar Coordinates
    xd_hat =  r_dot.*cos(phi) - r_.*phi_dot.*sin(phi);
    yd_hat =  r_dot.*sin(phi) + r_.*phi_dot.*cos(phi);

    % Dynamical System diffeomorphism (transformation matrix)
    xd = cos(theta).*a^(-1).*xd_hat - sin(theta).*b^(-1).*yd_hat;
    yd = sin(theta).*a^(-1).*xd_hat + cos(theta).*b^(-1).*yd_hat;
    
    xtest = xtest + xd*(1/50);
    ytest = ytest + yd*(1/50);
    xunit = [xunit, xtest];
    yunit = [yunit, ytest];
    
end

CircleUnit{1} = [xunit; yunit];


%% find closest points
i = 1;
gen_data = CircleUnit{1}';
real_data = data;

k_nearest = [];
dist_nearest = [];
for i=1:length(data)
    
    % remove repeating closest numbers (found most but not all)
    [k,dist] = dsearchn(gen_data, real_data(i:end,:));
    gen_data(k(1),:) = [];
    
    k_nearest = [k_nearest, k(1) + i-1];
    dist_nearest = [dist_nearest, dist(1)];
    
end

% compute the mean square root
y = rms(dist_nearest)

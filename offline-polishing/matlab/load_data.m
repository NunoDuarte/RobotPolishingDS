% for the glitch of the Command Window
%MATLAB_JAVA = '/usr/lib/jvm/java-8-openjdk/jre matlab -desktop -nosplash';
% Add this to ~/.bashrc
% export MATLAB_JAVA=/usr/lib/jvm/java-8-openjdk/jre

addpath('../data')
addpath('../process_data')

%% you need to open {W|E}lposhing# files

load('Wlpolish1.mat')

Wl{1} = Wlpolish1';
% Wlpolish{1} = Wlpolish5';
% Wlpolish{1} = Elpolish1';
% Wlpolish{2} = Wlpolish2';
% Wlpolish{6} = Wlpolish6';

plotting = 2;    % do you want to plot 2D or 3D?
[F3, F2] = preprocessing(Wl, plotting);

%% Save to one file

load('Wlpolish1.mat')
load('Wlpolish2.mat')
load('Wlpolish3.mat')
load('Wlpolish4.mat')
load('Wlpolish5.mat')
load('Wlpolish6.mat')
load('Elpolish1.mat')
load('Elpolish4.mat')

Polish{1} = Wlpolish1';
Polish{2} = Wlpolish2';
Polish{3} = Wlpolish3';
Polish{4} = Wlpolish4';
Polish{5} = Wlpolish5';
Polish{6} = Wlpolish6';
Polish{7} = Elpolish1';
Polish{8} = Elpolish4';

plotting = 2;    % do you want to plot 2D or 3D?
[~, P2] = preprocessing(Polish, [], plotting);


addpath('../../dataset')

% load data
load('input1.mat')

% you can load one file or multiple
Wl{1} = data';

% plot 2D or 3D?
plotting = 2;    
[F3, F2] = preprocess_data(Wl, plotting);

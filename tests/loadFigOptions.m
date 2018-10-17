
set(gcf, 'Paperunits' , 'centimeters')
set(gcf, 'PaperSize', [40 20]);
set(gcf, 'PaperPosition', [0 0 40  20]); 


xgraph = 0.25; %from left
%coordinates  for 4 subplots
four_y1 = 0.85;
four_y2 = 0.68;%s2nd graph
four_y3 = 0.42; %1st graph
four_y4 = 0.15; %1st graph
four_w = 0.8; %width
four_h = 0.22;
small_h = 0.13
%coordinates  for 3 subplots
three_y1 = 0.7;
three_y2 = 0.4;%s2nd graph
three_y3 = 0.1; %1st graph
three_w = 0.75; %width
three_h = 0.25;
%coordinates  for 2 subplots
two_xgraph = 0.2; %from left
two_y1 = 0.58;%s2nd graph
two_y2 = 0.15; %1st graph
two_w = 0.75; %width
two_h = 0.4;
%coordinates for 1 subplot
one_y1 = 0.25;
one_w = 0.8;
one_h = 0.7;
one_xgraph = 0.15

%% Set grids on all axis.
set(0,'defaultAxesXGrid','on');
set(0,'defaultAxesYGrid','on');
set(0,'defaultAxesZGrid','on');


set(0,'defaultAxesXGrid','on');set(0,'defaultAxesYGrid','on');set(0,'defaultAxesZGrid','on');
% set some other default values
set(0, 'RecursionLimit', 20);set(0, 'DefaultFigurePaperType', 'A4');
set(0, 'Defaultlinelinewidth',3);set(0,'defaultaxeslinewidth',1)
set(0, 'defaultpatchlinewidth',1);set(0, 'DefaultFigureWindowStyle', 'normal');
set(0, 'DefaultAxesBox', 'on');set(0, 'DefaultTextFontSize', 20);
set(0, 'DefaultAxesFontSize', 20);set(0, 'DefaultUicontrolFontSize', 20);
set(0, 'Defaulttextinterpreter','latex')
label_size=20


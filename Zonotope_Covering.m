clear all;
clc;

subplot(1, 2, 1)

%% Obstacle 

v = [5, 0; 6, 0; 6, 2; 5, 2] ;
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [4, 3.5; 6.5, 3.5; 6.5, 5; 4, 5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [0, 1; 0, 3; 2, 3; 2, 1];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [8, 2; 8, 4; 10, 4; 10, 2];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [1 6.5; 1 8.5; 3 8.5; 3 6.5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [4, 5; 4, 8.5; 5.8, 8.5; 5.8, 5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [8, 8; 8, 10; 10, 10; 10, 8];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [6, 0; 6, 1; 8, 1; 8, 0];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [7, 6; 7, 7; 9, 7; 9, 6];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;

%% Door 
x = linspace(8.5, 10);
y = 4+0.03*sin(16.6*(x-8.5));
plot(y, x, '-', 'color', '#949494', 'linewidth', 2); 

%% Initial Region
v = [1, 0.2; 1, 0.7; 1.5, 0.7; 1.5, 0.2];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'r', 'FaceColor', 'r'); hold on;

%% Goal 
v = [1.5, 9; 1.5, 9.5; 2, 9.5; 2, 9];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'm', 'FaceColor', 'm'); hold on;
v = [6 5; 6 5.5; 6.5 5.5; 6.5 5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'm', 'FaceColor', 'm'); hold on;
v = [9 1.5; 9 2; 9.5 2; 9.5, 1.5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'm', 'FaceColor', 'm'); hold on;

%% Workspace

lb = [0; 0];
ub = [10; 10];
WS= interval(lb, ub);
plot(WS, [1 2], 'k', 'linewidth', 2); hold on; 

v = [5.95, 0; 6.05, 0; 6.05, 5; 5.95, 5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'k', 'FaceColor', 'k'); hold on;
v = [3.95, 5; 4.05, 5; 4.05, 8.5; 3.95, 8.5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'k', 'FaceColor', 'k'); hold on;
v = [0 4.95; 0 5.05; 2 5.05; 2 4.95];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'k', 'FaceColor', 'k'); hold on;
v = [3 4.95; 3 5.05; 7 5.05; 7 4.95];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'k', 'FaceColor', 'k'); hold on;
v = [8 4.95; 8 5.05; 10 5.05; 10 4.95];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'k', 'FaceColor', 'k'); hold on;

axis([-0.2 10.2, -0.2 10.2]); 
box on; 
set(gca,'FontSize', 20);
xlabel('$x_{1}$', 'Interpreter', 'latex', 'fontsize', 24); 
ylabel('$x_{2}$', 'Interpreter', 'latex', 'fontsize', 24); 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(1, 2, 2)

%% Obstacle 

v = [5, 0; 6, 0; 6, 2; 5, 2] ;
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [4, 3.5; 6.5, 3.5; 6.5, 5; 4, 5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [0, 1; 0, 3; 2, 3; 2, 1];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [8, 2; 8, 4; 10, 4; 10, 2];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [1 6.5; 1 8.5; 3 8.5; 3 6.5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [4, 5; 4, 8.5; 5.8, 8.5; 5.8, 5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [8, 8; 8, 10; 10, 10; 10, 8];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [6, 0; 6, 1; 8, 1; 8, 0];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [7, 6; 7, 7; 9, 7; 9, 6];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;

%% Door 
x = linspace(8.5, 10);
y = 4+0.03*sin(16.6*(x-8.5));
plot(y, x, '-', 'color', '#949494', 'linewidth', 2); 

%% Initial Region
v = [1, 0.2; 1, 0.7; 1.5, 0.7; 1.5, 0.2];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'r', 'FaceColor', 'r'); hold on;

%% Goal 
v = [1.5, 9; 1.5, 9.5; 2, 9.5; 2, 9];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'm', 'FaceColor', 'm'); hold on;
v = [6 5; 6 5.5; 6.5 5.5; 6.5 5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'm', 'FaceColor', 'm'); hold on;
v = [9 1.5; 9 2; 9.5 2; 9.5, 1.5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'm', 'FaceColor', 'm'); hold on;

%% Workspace

lb = [0; 0];
ub = [10; 10];
WS= interval(lb, ub);
plot(WS, [1 2], 'k', 'linewidth', 2); hold on; 

v = [5.95, 0; 6.05, 0; 6.05, 5; 5.95, 5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'k', 'FaceColor', 'k'); hold on;
v = [3.95, 5; 4.05, 5; 4.05, 8.5; 3.95, 8.5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'k', 'FaceColor', 'k'); hold on;
v = [0 4.95; 0 5.05; 2 5.05; 2 4.95];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'k', 'FaceColor', 'k'); hold on;
v = [3 4.95; 3 5.05; 7 5.05; 7 4.95];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'k', 'FaceColor', 'k'); hold on;
v = [8 4.95; 8 5.05; 10 5.05; 10 4.95];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'k', 'FaceColor', 'k'); hold on;

%%  Zonotope-based Partition 

%Zonotopes 
Zc1= [5; 2.5];
ZG1 = 1.2*[-1  -1 -2; 0.75 -0.75 0];
Z1= zonotope(Zc1, ZG1);
plot(Z1, [1 2], 'b', 'linewidth', 1); hold on;

Zc2= [1; 2.5];
ZG2 = 1.2*[2 1 1; 0 0.75 -0.75];
Z2= zonotope(Zc2, ZG2);
plot(Z2, [1 2], 'g',  'linewidth', 1); hold on;

Zc3= [9; 2.5];
ZG3 = 1.2*[2 1 1; 0 0.75 -0.75];
Z3= zonotope(Zc3, ZG3);
plot(Z3, [1 2], 'c',  'linewidth', 1); hold on;

Zc4= [5; 7.5];
ZG4 = 1.2*[-1  -1 -2; 0.75 -0.75 0];
Z4= zonotope(Zc4, ZG4);
plot(Z4, [1 2], 'b', 'linewidth', 1); hold on;

Zc5= [1; 7.5];
ZG5 = 1.2*[2 1 1; 0 0.75 -0.75];
Z5= zonotope(Zc5, ZG5);
plot(Z5, [1 2], 'g',  'linewidth', 1); hold on;

Zc6= [9; 7.5];
ZG6 = 1.2*[2 1 1; 0 0.75 -0.75];
Z6= zonotope(Zc6, ZG6);
plot(Z6, [1 2], 'c',  'linewidth', 1); hold on;

Zc7= [3; 4];
ZG7 = 1.2*[1 0 -1; -0.75 -1.5 -0.75];
Z7= zonotope(Zc7, ZG7);
plot(Z7, [1 2], 'r',  'linewidth', 1); hold on;

Zc8= [7; 4];
ZG8 = 1.2*[1 0 -1; -0.75 -1.5 -0.75];
Z8= zonotope(Zc8, ZG8);
plot(Z8, [1 2], 'y',  'linewidth', 1); hold on;

Zc9= [3; 1];
ZG9 = 1.2*[1 0 -1; 0.75 1.5 0.75];
Z9= zonotope(Zc9, ZG9);
plot(Z9, [1 2],  'm', 'linewidth', 1); hold on;

Zc10= [3; 9];
ZG10 = 1.2*[1 0 -1; 0.75 1.5 0.75];
Z10= zonotope(Zc10, ZG10);
plot(Z10, [1 2],  'm', 'linewidth', 1); hold on;

Zc11= [7; 9];
ZG11 = 1.2*[1 0 -1; 0.75 1.5 0.75];
Z11= zonotope(Zc11, ZG11);
plot(Z11, [1 2], 'color', '#80B3FF',  'linewidth', 1); hold on;

Zc12= [7; 1];
ZG12 = 1.2*[1 0 -1; 0.75 1.5 0.75];
Z12= zonotope(Zc12, ZG12);
plot(Z12, [1 2], 'color', '#80B3FF',  'linewidth', 1); hold on;

%Constrained Zonotopes 
Zc13= [0.5; 5];
ZG13 = 1.2*[0.5 0; 0 -1];
Z13= zonotope(Zc13, ZG13);
plot(Z13, [1 2], 'color', '#B2CAF6', 'linewidth', 1); hold on;

Zc14= [0.5; 0.5];
ZG14 = 1.2*[0.5 0; 0 -0.5];
Z14= zonotope(Zc14, ZG14);
plot(Z14, [1 2], 'color', '#B14BA3',  'linewidth', 1); hold on;

Zc15= [0.5; 9.5];
ZG15 = 1.2*[0.5 0; 0 -0.5];
Z15= zonotope(Zc15, ZG15);
plot(Z2, [1 2], 'color', '#B14BA3', 'linewidth', 1); hold on;

Zc16= [9.5; 0.5];
ZG16 = 1.2*[0.5 0; 0 -0.5];
Z16= zonotope(Zc16, ZG16);
plot(Z16, [1 2], 'color', '#B15A4B',  'linewidth', 1); hold on;

Zc17= [9.5; 9.5];
ZG17 = 1.2*[0.5 0; 0 -0.5];
Z17= zonotope(Zc17, ZG17);
plot(Z17, [1 2], 'color', '#B15A4B',  'linewidth', 1); hold on;

Zc18= [9.5; 5];
ZG18 = 1.2*[0.5 0; 0 -1];
Z18= zonotope(Zc18, ZG18);
plot(Z18, [1 2], 'color', '#765E32',  'linewidth', 1); hold on;

axis([-0.2 10.2, -0.2 10.2]); box on; 
set(gca,'FontSize', 20);
xlabel('$x_{1}$', 'Interpreter', 'latex', 'fontsize', 24); 
ylabel('$x_{2}$', 'Interpreter', 'latex', 'fontsize', 24); 

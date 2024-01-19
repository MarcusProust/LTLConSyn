clear all;
clc;

%% Workspace ------------------------------------------------------------------------------------------------------------------------

%Obstacle 

v = [4.5 0; 5.5 0; 5.5 2; 4.5 2] ;
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [4.5 3; 5.5 3; 5.5 5; 4.5 5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [0 2; 0 3; 2 3; 2 2];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [8 2; 8 3; 10 3; 10 2];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [0 4.2; 0 5; 0.8 5; 0.8 4.2];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [3.5 4.5; 3.5 5; 4.5 5; 4.5 4.5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [8 4; 8 5; 10 5; 10 4];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;
v = [5.5 0.5; 5.5 1; 7 1; 7 0.5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', '#949494', 'FaceColor', '#949494'); hold on;

%Door 
% v = [4.98 2; 5.02 2; 5.02 3; 4.98 3];
% f = [1 2 3 4];
% patch('Faces', f, 'Vertices', v, 'EdgeColor', '#242c32', 'FaceColor', '#242c32'); hold on;

%Initial Region
v = [0 0; 0.5 0; 0.5 0.5; 0 0.5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'r', 'FaceColor', 'r'); hold on;

%Goal 
v = [1 3.5; 1.5 3.5; 1.5 4; 1 4];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'm', 'FaceColor', 'm'); hold on;
v = [6.75 4; 7.25 4; 7.25 4.5; 6.75 4.5];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'm', 'FaceColor', 'm'); hold on;
v = [9 1.5; 9.5 1.5; 9.5 2; 9 2];
f = [1 2 3 4];
patch('Faces', f, 'Vertices', v, 'EdgeColor', 'm', 'FaceColor', 'm'); hold on;

% Workspace

lb = [0; 0];
ub = [10; 5];
WS= interval(lb, ub);
plot(WS, [1 2], 'k', 'linewidth', 2); hold on; 

lb = [5; 0];
ub = [5; 1.995];
WS= interval(lb, ub);
plot(WS, [1 2], 'k', 'linewidth', 2); hold on; 
lb = [5; 3];
ub = [5; 5];
WS= interval(lb, ub);
plot(WS, [1 2], 'k', 'linewidth', 2); hold on; 

%%  Zonotope-based Partition ----------------------------------------------------------------------------------------------------------

%Zonotopes 
Zc1= [5; 2.5];
ZG1 = 1.2*[-1  -1 -2; 0.75 -0.75 0];
Z1= zonotope(Zc1, ZG1);
plot(Z1, [1 2], 'b', 'linewidth', 1); hold on;

Zc2= [3; 4];
ZG2 = 1.2*[1 0 -1; -0.75 -1.5 -0.75];
Z2= zonotope(Zc2, ZG2);
plot(Z2, [1 2], 'r',  'linewidth', 1); hold on;

Zc3= [3; 1];
ZG3 = 1.2*[1 0 -1; 0.75 1.5 0.75];
Z3= zonotope(Zc3, ZG3);
plot(Z3, [1 2],  'm', 'linewidth', 1); hold on;

Zc4= [1; 2.5];
ZG4 = 1.2*[2 1 1; 0 0.75 -0.75];
Z4= zonotope(Zc4, ZG4);
plot(Z4, [1 2], 'g',  'linewidth', 1); hold on;

Zc5= [9; 2.5];
ZG5 = 1.2*[2 1 1; 0 0.75 -0.75];
Z5= zonotope(Zc5, ZG5);
plot(Z5, [1 2], 'c',  'linewidth', 1); hold on;

Zc6= [7; 4];
ZG6 = 1.2*[1 0 -1; -0.75 -1.5 -0.75];
Z6= zonotope(Zc6, ZG6);
plot(Z6, [1 2], 'y',  'linewidth', 1); hold on;

Zc7= [7; 1];
ZG7 = 1.2*[1 0 -1; 0.75 1.5 0.75];
Z7= zonotope(Zc7, ZG7);
plot(Z7, [1 2], 'color', '#80B3FF',  'linewidth', 1); hold on;

%Constrained Zonotopes 
CZc1= [0.5; 4.5];
CZG1 = 1.2*[0.5 0; 0 -0.5];
CZ1= zonotope(CZc1, CZG1);
plot(CZ1, [1 2], 'color', '#B2CAF6', 'linewidth', 1); hold on;

CZc2= [0.5; 0.5];
CZG2 = 1.2*[0.5 0; 0 -0.5];
CZ2= zonotope(CZc2, CZG2);
plot(CZ2, [1 2], 'color', '#B14BA3',  'linewidth', 1); hold on;

CZc3= [9.5; 0.5];
CZG3 = 1.2*[0.5 0; 0 -0.5];
CZ3= zonotope(CZc3, CZG3);
plot(CZ3, [1 2], 'color', '#B15A4B',  'linewidth', 1); hold on;

CZc4= [9.5; 4.5];
CZG4 = 1.2*[0.5 0; 0 -0.5];
CZ4= zonotope(CZc4, CZG4);
plot(CZ4, [1 2], 'color', '#765E32',  'linewidth', 1); hold on;

% CZc5= [8.5; 1.8];
% CZG5 = [2 0; 0 0.5];
% CZ5= zonotope(CZc5, CZG5);
% plot(CZ5, [1 2], 'color', '#324A76',  'linewidth', 1.5); hold on;
% 
% CZc6= [9.2; 3.8];
% CZG6 = [1.1 0; 0 0.7];
% CZ6= zonotope(CZc6, CZG6);
% plot(CZ6, [1 2], 'color', '#B2CAF6',  'linewidth', 1.5); hold on;
% 
% CZc7= [4; 9];
% CZG7 = [1.5 0; 0 1.2];
% CZ7= zonotope(CZc7, CZG7);
% plot(CZ7, [1 2], 'color', '#ED9564',  'linewidth', 1.5); hold on;

axis([-0.2 10.2, -0.2 5.2]); box on; %grid on; 
xlabel('$x_{1}$', 'Interpreter', 'latex', 'fontsize', 16); 
ylabel('$x_{2}$', 'Interpreter', 'latex', 'fontsize', 16); 

% %%
% figure;
% plot(c1(1), c1(2), 'k*', 'linewidth', 10);     hold on; 
% plot(c2(1), c2(2), 'k*', 'linewidth', 10);     hold on; 
% plot(c3(1), c3(2), 'k*', 'linewidth', 10);     hold on; 
% plot(c4(1), c4(2), 'k*', 'linewidth', 10);     hold on; 
% plot(c5(1), c5(2), 'r*', 'linewidth', 10);     hold on; 
% plot(c6(1), c6(2), 'k*', 'linewidth', 10);     hold on; 
% plot(c7(1), c7(2), 'b*', 'linewidth', 10);     hold on; 
% plot(c8(1), c8(2), 'k*', 'linewidth', 10);     hold on; 
% 
% %plot([c1(1), c2(1)], [c1(2), c2(2)], 'b-');   hold on; 
% %plot([c1(1), c3(1)], [c1(2), c3(2)], 'b-');   hold on; 
% %plot([c1(1), c4(1)], [c1(2), c4(2)], 'b-');   hold on; 
% plot([c1(1), c5(1)], [c1(2), c5(2)], 'b-',  'linewidth', 1.5);   hold on; 
% plot([c1(1), c6(1)], [c1(2), c6(2)], 'b-',  'linewidth', 1.5);   hold on; 
% 
% %plot([c2(1), c3(1)], [c2(2), c3(2)], 'b-');   hold on; 
% %plot([c2(1), c4(1)], [c2(2), c4(2)], 'b-');   hold on; 
% plot([c2(1), c6(1)], [c2(2), c6(2)], 'b-',  'linewidth', 1.5);   hold on; 
% plot([c2(1), c7(1)], [c2(2), c7(2)], 'b-',  'linewidth', 1.5);   hold on; 
% 
% %plot([c3(1), c4(1)], [c3(2), c4(2)], 'b-');   hold on; 
% plot([c3(1), c7(1)], [c3(2), c7(2)], 'b-',  'linewidth', 1.5);   hold on; 
% plot([c3(1), c8(1)], [c3(2), c8(2)], 'b-',  'linewidth', 1.5);   hold on; 
% 
% plot([c4(1), c8(1)], [c4(2), c8(2)], 'b-',  'linewidth', 1.5);   hold on; 
% plot([c4(1), c5(1)], [c4(2), c5(2)], 'b-',  'linewidth', 1.5);   hold on; 
% 
% L1= texlabel("$\mathbb{O}_{1}$");   
% text(8, 1.6, L1, 'Interpreter', 'latex', 'fontsize', 20); hold on; 
% L2= texlabel('pi_{2}');   
% text(-0.1, 1.1, L2, 'fontsize', 20); hold on; 
% L3= texlabel('pi_{3}');   
% text(1.6, 1.6, L3, 'fontsize', 20);hold on; 
% 
% L4= texlabel('pi_{4}');   
% text(1, 0, L4, 'fontsize', 20); hold on; 
% L5= texlabel('pi_{5}');   
% text(1.6, -1.6, L5, 'fontsize', 20); hold on; 
% L6= texlabel('pi_{6}');   
% text(-0.1, -1.1, L6, 'fontsize', 20);hold on; 
% 
% L7= texlabel('pi_{7}');   
% text(-1.8, -1.6, L7, 'fontsize', 20); hold on; 
% L8= texlabel('pi_{8}');   
% text(-1.2, 0, L8, 'fontsize', 20);hold on; 
% 
% set(gca,'visible','off')

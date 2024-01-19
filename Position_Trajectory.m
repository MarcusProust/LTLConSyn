clear all;
clc;
addpath(genpath('../../../mfiles/'));

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
lb = [6; 0];
ub = [6; 5];
WS= interval(lb, ub);
plot(WS, [1 2], 'k', 'linewidth', 2); hold on; 
lb = [4; 5];
ub = [4; 8.5];
WS= interval(lb, ub);
plot(WS, [1 2], 'k', 'linewidth', 2); hold on; 
lb = [3; 5];
ub = [7; 5];
WS= interval(lb, ub);
plot(WS, [1 2], 'k', 'linewidth', 2); hold on; 
lb = [0; 5];
ub = [2; 5];
WS= interval(lb, ub);
plot(WS, [1 2], 'k', 'linewidth', 2); hold on; 
lb = [8; 5];
ub = [10; 5];
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

%% simulation

% initial state
x0=[1.2 0.5 0];
y=x0;
con=[];
YY=[];
U={};

%% Reachability
for(i=1:1:8)
    controller=SymbolicSet(['Controller', num2str(i), '.bdd'], 'projection', [1 2 3]);
    target=SymbolicSet(['Target', num2str(i), '.bdd']);
    v=[];

while(1) 
    if (target.isElement(y(end, :))) 
        break;
    end
    
    u=controller.getInputs(y(end, :));
    U=[U, u];
    v=[v; u(end, :)];
    [t x]=ode45(@unicycle_ode, [0, 0.2], y(end, :), [], u(end, :));
    y=[y; x(end, :)];
    YY=[YY; y];
end
x0=y(end, :);
con=[con; v];
plot(y(:,1), y(:,2), 'b-', 'linewidth', 1.5); hold on;
end

%% Safety
C=SymbolicSet('Controller9.bdd', 'projection', [1 2 3]);
y=y(end, :);
T=10; 
for t=1:T/0.2
    v=[];
    u=C.getInputs(y(end, :));
    v=[v; u(end, :)];
    U=[U, u];
    [t x]=ode45(@unicycle_ode, [0 .2], y(end, :), odeset('abstol', 1e-4, 'reltol', 1e-4), u(end, :));
    plot(y(:, 1), y(:, 2), 'b-', 'linewidth', 1.5); hold on;
    con=[con; v];
    y=[y; x(end, :)];
end

%% Global approach 
G=load('States.mat');
plot(G.YY(:, 1), G.YY(:, 2), 'k:', 'linewidth', 1.5);
hold on;

axis([0, 10, 0, 10]); box on;  
xlabel('$x_{1}$', 'Interpreter', 'latex', 'fontsize', 16); 
ylabel('$x_{2}$', 'Interpreter', 'latex', 'fontsize', 16); 

%% Control Inputs 

figure;
t=0:0.2:(size(con, 1)-1)*0.2;
subplot(2,1,1);
for(i=1:1:length(U))
    CI=U{i};
    plot(0.2*(i-1), CI(:, 1), 'b.', 'MarkerSize', 10);
    hold on;
end 
[x1, y1]=stairs(t, con(:, 1));
plot(x1, y1, 'b-', 'linewidth', 2);
axis([0, 37, -1, 1]); box on;  
set(gca, 'FontSize', 20);
ylabel('$u_{1}$', 'Interpreter', 'latex', 'FontSize', 30);

subplot(2,1,2);
for(i=1:1:length(U))
    CI=U{i};
    plot(0.2*(i-1), CI(:, 2), 'b.', 'MarkerSize', 10);
    hold on;
end 
[x2, y2]=stairs(t, con(:, 2));
plot(x2, y2, 'b-', 'linewidth', 2);
axis([0, 37, -1, 1]); box on;  
set(gca, 'FontSize', 20);
xlabel('$t$', 'Interpreter', 'latex', 'FontSize', 30);
ylabel('$u_{2}$', 'Interpreter', 'latex', 'FontSize', 30);

function dxdt = unicycle_ode(t, x, u)

  dxdt = zeros(3, 1);
  c=atan(tan(u(2))/2);
  dxdt(1)=u(1)*cos(c+x(3))/cos(c);
  dxdt(2)=u(1)*sin(c+x(3))/cos(c);
  dxdt(3)=u(1)*tan(u(2));
end

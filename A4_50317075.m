clc;            close all;          clear all;

t  = 0:0.1:10;
q0 = zeros(1,length(t));
q1 = (pi/3)*cos(t);
q2 = (pi/2)*(1+t);
q3 = (pi/2)*(1-t);
q4 = pi/2*ones(1,length(t));

d = [20 0 0 0 0];       a = [0 50 50 0 0];             al = [-pi/2 0 0 -pi/2 0];


%% Robot joint positions
x1 = zeros(1,length(t));     y1 = zeros(1,length(t));   z1 = 20*ones(1,length(t));
x2 = x1+50*cos(-q0-q1);       y2 = zeros(1,length(t));   z2 = z1+50*sin(-q0-q1);
x3 = x2+50*cos(-q0-q1-q2);    y3 = zeros(1,length(t));   z3 = z2+50*sin(-q0-q1-q2);
x4 = x3+5*cos(-q0-q1-q2-q3-q4);  y4 = zeros(1,length(t));   z4 = z3+5*sin(-q0-q1-q2-q3-q4);

Pos=[];
for i=1:length(t)
    
    cla
    %% Robot visualization
    xx = [0 x1(i) x2(i) x3(i) x4(i)];
    yy = [0 y1(i) y2(i) y3(i) y4(i)];
    zz = [0 z1(i) z2(i) z3(i) z4(i)];
    plot3(xx,yy,zz,'-o','markersize',5,'Linewidth',3,'MarkerFacecolor','b','MarkerEdgeColor','g')
    hold on
    plot3(x4(1:i),y4(1:i),z4(1:i),'k:')
    %% Forward Kinematics
    th = [q0(i) q1(i) q2(i) q3(i) q4(i)];
    H = eye(4);
    
    for j=1:5
        Hb = [cos(th(j)) -sin(th(j))*cos(al(j)) sin(th(j))*sin(al(j)) a(j)*cos(th(j));
              sin(th(j)) cos(th(j))*cos(al(j))  -cos(th(j))*sin(al(j)) a(j)*sin(th(j)); 
              0 sin(al(j)) cos(al(j)) d(j); 
              0 0 0 1];
          H = H*Hb;
    end
    Pe = H*[0;0;5;1];
    Pos=[Pos Pe];
    plot3(Pe(1),Pe(2),Pe(3),'ro','MarkerfaceColor','r','Markersize',5,'MarkerEdgeColor','y')
    xlim([-100 100])
    ylim([-100 100])
    zlim([-100 120])
    pause(0.1)
end
Pos
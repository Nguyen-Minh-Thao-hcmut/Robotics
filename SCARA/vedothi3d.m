function [EF] = vedothi3d(theta1,theta2,d3,theta4)
%T = 0.03
    


[T01,T02,T03,T04] = EF_HomoTransform(theta1,theta2,d3,theta4);
A = T01*[0; 0; 0; 1];
B = T02*[0; 0; 0; 1];
C = T03*[0; 0; 0; 1];
EF = T04*[0; 0; 0; 1];

x_link1 = [0; 0];
y_link1 = [0; 0];
z_link1 = [0; 10];

x_link2 = [0; A(1)];
y_link2 = [0; A(2)];
z_link2 = [10; A(3)];

x_link3 = [A(1); B(1)];
y_link3 = [A(2); B(2)];
z_link3 = [A(3); B(3)];

x_link4 = [B(1); C(1)];
y_link4 = [B(2); C(2)];
z_link4 = [B(3); C(3)];

x_link5 = [C(1); EF(1)];
y_link5 = [C(2); EF(2)];
z_link5 = [C(3); EF(3)];

unit_vecto_Ox_RF1 = [3; 0; 0; 1];
unit_vecto_Oy_RF1 = [0; 3; 0; 1];
unit_vecto_Oz_RF1 = [0; 0; 3; 1];

unit_vecto_Ox_RF2 = [3; 0; 0; 1];
unit_vecto_Oy_RF2 = [0; 3; 0; 1];
unit_vecto_Oz_RF2 = [0; 0; 3; 1];

unit_vecto_Ox_RF3 = [3; 0; 0; 1];
unit_vecto_Oy_RF3 = [0; 3; 0; 1];
unit_vecto_Oz_RF3 = [0; 0; 3; 1];

unit_vecto_Ox_RF4 = [3; 0; 0; 1];
unit_vecto_Oy_RF4 = [0; 3; 0; 1];
unit_vecto_Oz_RF4 = [0; 0; 3; 1];



plot3([0;3],[0;0],[10;10],'Color','red');
hold on;
plot3([0;0],[0;3],[10;10],'Color','blue');
plot3([0;0],[0;0],[10;13],'color','black');
plot3([0;3],[0;0],[0;0],'Color','red');
plot3([0;0],[0;3],[0;0],'Color','blue');
plot3([0;0],[0;0],[0;3],'color','black');

plot_link(x_link1,y_link1,z_link1,'black','.');
plot_RF(A(1),A(2),A(3),unit_vecto_Ox_RF1,unit_vecto_Oy_RF1,unit_vecto_Oz_RF1,T01);
plot_link(x_link2,y_link2,z_link2,'blue','.');
plot_RF(B(1),B(2),B(3),unit_vecto_Ox_RF2,unit_vecto_Oy_RF2,unit_vecto_Oz_RF2,T02);
plot_link(x_link3,y_link3,z_link3,'red','.');
plot_RF(C(1),C(2),C(3),unit_vecto_Ox_RF3,unit_vecto_Oy_RF3,unit_vecto_Oz_RF3,T03);
plot_link(x_link4,y_link4,z_link4,'yellow','.');
plot_RF(EF(1),EF(2),EF(3),unit_vecto_Ox_RF4,unit_vecto_Oy_RF4,unit_vecto_Oz_RF4,T04);
plot_link(x_link5,y_link5,z_link5,'green','.');
pause(0.001);
axis([-20 20 -20 20 0 30]);
th = linspace(0,2*pi);
x_workspace = 10*cos(th);
y_workspace = 10*sin(th);
plot(x_workspace,y_workspace,'red');
hold off;
grid on;
end

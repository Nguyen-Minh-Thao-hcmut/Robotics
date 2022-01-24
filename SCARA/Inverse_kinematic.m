function [theta1,theta2,d3,theta4,mark] = Inverse_kinematic(q_x,q_y,q_z,dir,pre_mark,q_phi)
% - dir = [x1,x2]; x = 1:chieu duong, x = 0:chieu am; x1 la chieu cua theta1,
%   x2 la chieu cua theta2; dir = [3,3]: Diem dau tien => pre_mark = [3,3]
% - pre_mark = [x1,x2]; x1 = 1: theta1_pre co dau duong, x1 = 0: theta1_pre
%   mang dau am; x1 = 3: theta1 la diem dau tien. Tuong tu voi theta2
%   
d3 = q_z - 15;
c2 = (q_x^2 + q_y^2 - 2*25)/(2*25);

s2 = +sqrt(1-c2^2);
theta2 = atan2(s2,c2);

s1 = ((5 + 5*c2)*q_y - 5*s2*q_x)/(q_x^2+q_y^2);
c1 = ((5 + 5*c2)*q_x + 5*s2*q_y)/(q_x^2+q_y^2);
theta1 = atan2(s1,c1);


% -Xu li chieu quay cua theta1, tranh tinh trang atan(theta1 > pi) am hoac
%   atan(theta1 < -pi) duong dan den khong the dieu khien PID duoc
if ((dir(1) == 1)||(dir(1) == 3))&&(pre_mark(1) == 1)&&(theta1 < -0.04)
    theta1 = theta1 + 2*pi;
elseif ((dir(1) == 0)||(dir(1) == 3))&&(pre_mark(1) == 0)&&(theta1 > 0.04)
    theta1 = theta1 - 2*pi;
%elseif (dir(1) == 3)&&(theta1 == pi)&&(pre_mark(1) == 3)
 %   theta1 = -pi;
end
if theta1 >= 0
    mark(1) = 1;
else 
    mark(1) = 0;
end
if theta2 >= 0
    mark(2) = 1;
else
    mark(2) = 0;
end
theta4 = q_phi - theta1 - theta2;




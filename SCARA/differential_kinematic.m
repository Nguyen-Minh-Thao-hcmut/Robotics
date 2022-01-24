function [singular,warning,theta1_dot,theta2_dot,d3_dot,theta4_dot,index] = differential_kinematic(q_x,q_y,q_z,q_phi)
global tmp_x;
global tmp_y;
global tmp_z;
global tmp_phi
theta1_dot = zeros(1);
theta2_dot = zeros(1);
d3_dot = zeros(1);
theta4_dot = zeros(1);
qx = tmp_x;
qy = tmp_y;
qz = tmp_z;
phi_pre = 0;
T = 0.01;

%Phuc vu cho viec hien thi, canh bao va ve do thi cac diem Kinematic
%Singularitis
singular = zeros(1,3); %Ma tran chua to do (q_x,q_y,q_z) cua cac diem Kinematic Singularities
warning = 0;% So diem Singularities
index = [];% chi so cua cac diem Singularities trong ma tran quy dao q_x, q_y, q_z
%

% Tinh theta1, theta2, d3, theta4 t? Inverse Kinematic
dir = [3,3];
pre_mark = [3,3];
[theta1_pre,theta2_pre,d3_pre,theta4_pre, mark] =  Inverse_kinematic(tmp_x,tmp_y,tmp_z,dir,pre_mark,tmp_phi);
pre_mark = mark;

for i =1:length(q_x)
    [theta1,theta2,d3,theta4,mark] = Inverse_kinematic(q_x(i),q_y(i),q_z(i),dir,pre_mark,q_phi(i));
    pre_mark = mark;
    if (theta1 - theta1_pre) > 0
        dir(1) = 1;
    elseif theta1 - theta1_pre < 0
        dir(1) = 0;
    end
    if (theta2 - theta2_pre) > 0
       dir(2) = 1;
    elseif theta2 - theta2_pre < 0
       dir(2) = 0;
    end
%
%Tinh toan differential Kinematics
    [T01, T02, T03, T04] = EF_HomoTransform(theta1,theta2,d3,theta4);
    J = Jacobian(T01,T02,T03,T04);% Ma tran Jacobian chua rut gon
    J(4:5,:) = [];% Rut gon ma tran Jacobian ve ma tran vuong
    v_x = (q_x(i) - qx)/T;% Vx c?a EF so voi RF0
    v_y = (q_y(i) - qy)/T;% Vy c?a EF so voi RF0
    v_z = (q_z(i) - qz)/T;% Vz c?a EF so voi RF0
    phi = theta1 + theta2 + theta4;% Goc phi
    w_z = (phi - phi_pre)/T;% Wz cua EF so voi RF0
    check = 1;


    if (round(det(J),0) == 0)&&(i>=5)% Kiem tra xem ma tran J co bi suy bien (Vi tri Kinematic Singularity)
        check = 0;
    end
    if check == 0
        % Neu J bi suy bien thi van toc cac Joins rat lon, 
        %dan den khong quan sat duoc tren do thi nen em gioi han lai 
        %de co the tuan sat tren do thi
       theta1_dot(i) = (theta1 - theta1_pre)/T;
       if theta1_dot(i) > 2*pi
           theta1_dot(i) = 2*pi;
       elseif theta1_dot(i) < -2*pi
           theta1_dot(i) = -2*pi;
       end
       theta2_dot(i) = (theta2 - theta2_pre)/T;
       if theta2_dot(i)>2*pi
           theta2_dot(i) = 2*pi;
       elseif theta2_dot(i) < -2*pi
           theta2_dot(i) = -2*pi;
       end
       d3_dot(i) = (d3 - d3_pre)/T;
       if d3_dot(i) > 50
           d3_dot(i) = 50;
       elseif d3_dot(i) < -50
           d3_dot(i) = -50;
       end
       theta4_dot(i) = (theta4 - theta4_pre)/T;
       if theta4_dot(i) > 2*pi
           theta4_dot(i) = 2*pi;
       elseif theta4_dot(i) < -2*pi
           theta4_dot(i) = -2*pi;
       end
       %Ghi lai toa do cac diem Singularities
       if i>10 
           % i>10 vi tai nhung diem dau tien cua quy dao, det(J) xap xi 0 
           % nhung day khong phai nhung diem Kinematic Singularities
           % nen chon i>10 de loai bo nhung diem nay
        warning = warning + 1;
        singular(warning,:) = [q_x(i),q_y(i),q_z(i)];
        index = [index, i];
       end
       %
    else% Van toc cac Join tai nhung diem khong phai Singularities duoc tinh tu ma tran J 
        Join = inv(J)*[v_x; v_y; v_z; w_z]; 
        theta1_dot(i) = Join(1,:);
        theta2_dot(i) = Join(2,:);
        d3_dot(i) = Join(3,:);
        theta4_dot(i) = Join(4,:);
        %
    end
    qx = q_x(i);
    qy = q_y(i);
    qz = q_z(i);
    theta1_pre = theta1;
    theta2_pre = theta2;
    d3_pre = d3;
    theta4_pre = theta4;
end
    
        
    
    
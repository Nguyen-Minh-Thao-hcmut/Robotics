function [q_x,q_y,q_z,q_phi] = linear_path_planning(amax,v_max)
global x;
global y;
global z;
global tmp_phi;
global tmp_x;
global tmp_y;
global tmp_z;
global phi;
global run_enable;%% run_enable(1): Kiem tra dieu kien singularity, run_enable(2): Kiem tra dieu kien suy bien cua van toc
q_x = zeros(1);
q_y = zeros(1);
q_z = zeros(1);
q_phi = zeros(1);
% Tim vmax_rel va hinh chieu cua amax len 3 truc toa do
vecto_delta = [x - tmp_x; y - tmp_y; z - tmp_z]; % Vector do doi EF
MS = vecto_delta(1)^2 + vecto_delta(2)^2 + vecto_delta(3)^2;
q_max = sqrt(MS); % ?? dài qu? ??o c?a EF
cosa_x = (dot(vecto_delta,[1;0;0]))/sqrt(MS); % - He so goc cua vecto_delta len 3 truc toa do
cosa_y = (dot(vecto_delta,[0;1;0]))/sqrt(MS); %   trong do [1;0;0],[0;1;0],[0;0;1] la cac vector don vi
cosa_z = (dot(vecto_delta,[0;0;1]))/sqrt(MS); %
%
[delta_q] = velocity_trajectory(q_max,amax,v_max);% Vi phan do dai dich chuyen


%

%- Quy hoach quy dao tren 3 truc toa do
%  Vi phan toa do dich chuyen duoc tinh bang cach nhan vi phan do dai dich
%  chuyen voi he so goc vector_delta len 3 truc toa do

if (run_enable(2) == 0)
    q_x = 0;
    q_y = 0;
    q_z = 0;
    return
else  
    %Trên Ox
    qx = tmp_x;
    for i=1:length(delta_q)
        q_x(i) = qx + delta_q(i)*cosa_x;
        qx = q_x(i);
    end
    %

    % Trên Oy
    qy = tmp_y;
    for i=1:length(delta_q)
        q_y(i) = qy + delta_q(i)*cosa_y;
        qy = q_y(i);
    end
    %

    %Trên Oz
    qz = tmp_z;
    for i=1:length(delta_q)
        q_z(i) = qz + delta_q(i)*cosa_z;
        qz = q_z(i);
    end
    %Quy hoach goc phi
    dphi = phi - tmp_phi;
    for i=1:length(delta_q)
        q_phi(i) = tmp_phi + dphi*delta_q(i);
    end
end



function [q_x,q_y,q_z,q_phi] = circle_path_planning(amax,v_max)
global x;
global y;
global z;
global phi;
global tmp_phi;
global tmp_x;
global tmp_y;
global tmp_z;
global run_enable;%% run_enable(1): Kiem tra dieu kien singularity, run_enable(2): Kiem tra dieu kien suy bien cua van toc
q_x = zeros(1);
q_y = zeros(1);
q_z = zeros(1);
q_phi = zeros(1);
% Tim trung diem AB, voi A la toa do ban dau cua EF, B la toa do muon di
% chuyen den
center_x = (x + tmp_x)/2;
center_y = (y + tmp_y)/2;

Bt = (x - tmp_x)^2 + (y - tmp_y)^2;
R = sqrt(Bt)/2;% Ban kinh cua duong tron duong kinh AB
q_max = pi*R;  % Nua chu vi cua duong tron, la do dai quy dao dich chuyem
delta_q = velocity_trajectory(q_max,amax,v_max); %Vi phan do dai dich chuyen (Do dai dai so)
% ptr duong tron: x = R*cos(t) + center_x
%                 y = R*sin(t) + center_y


if (run_enable(2) == 0)
    q_x = 0;
    q_y = 0;
    q_z = 0;
    return
else 
    t = atan2(tmp_y - center_y, tmp_x - center_x);%Goc tuong ung voi diem dau cua EF
    
    % Quy hoach quy dao duong tron dua vao delta_q va t
    delta_q_z = (z - tmp_z);
    qz = tmp_z;
    if y >= 0 %y >= 0 thi theta1 quay nguoc chieu kim dong ho, y<0 thi nguoc lai de tranh tinh trang Workspace Singularities
        for i = 1:length(delta_q)
            delta_t = delta_q(i)/R;
            t = t - delta_t;
            q_x(i) = R*cos(t) + center_x;
            q_y(i) = R*sin(t) + center_y;
            q_z(i) = qz + delta_q(i)*(delta_q_z/q_max);
            qz = q_z(i);
        end 
    else 
        for i = 1:length(delta_q)
            delta_t = delta_q(i)/R;
            t = t + delta_t;
            q_x(i) = R*cos(t) + center_x;
            q_y(i) = R*sin(t) + center_y;
            q_z(i) = qz + delta_q(i)*(delta_q_z/q_max);
            qz = q_z(i);
        end
    end
    d_phi = phi - tmp_phi;
    for i = 1: length(delta_q)
        q_phi(i) = tmp_phi + d_phi*delta_q(i);
    end
end




    

            

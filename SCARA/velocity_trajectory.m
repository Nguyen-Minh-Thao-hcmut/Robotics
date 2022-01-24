function [delta_q] = velocity_trajectory(q_max,amax,v_max)
%q_max la quang duong dich chuyen cua EF
%amax la gia toc toi da mong muon
%v_max la van toc toi da mong muon
%delta_q la vi phan quang duong dich chuyen cua EF
global TrajectoryMode;
global run_enable;
T = 0.01;

if (TrajectoryMode == 1)% Quy hoach quy dao hinh thang, co dieu chinh vmax khi van toc bi suy bien
    
    MS1 = 2*amax*q_max/2;
    vmax_real = sqrt(MS1); %vmax thuc te, khi do dai dich chuyen qua ngan, van toc khong the dat toi gia tri vmax_set
    if (vmax_real < v_max)
        vmax = vmax_real;
    else
        vmax = v_max;
    end

        % Tim cac thoi diem t1,t2,t3
    t1 = vmax/amax;
    n1 = floor(t1/T);
    t2 = q_max/vmax;
    if floor(t2/T) > n1 
        n2 = floor(t2/T);
    else
        n2 = n1;
    end
    n3 = n1 + n2;
    % Quy hoach van toc hinh thang
    q = zeros(1);
    delta_q = zeros(1);
    q(1) = 0;
    for i = 1:n1
        q(i+1) = 0.5*amax*(i*T)^2;
        delta_q(i) = q(i+1) - q(i);
    end
    for i = (n1+1):n2
        q(i+1) = q(n1+1) + amax*(n1*T)*(i*T - n1*T);
        delta_q(i) = q(i+1) - q(i);
    end
    for i = (n2+1):n3
        q(i+1) = q(n2+1) - 0.5*amax*(i*T - n2*T)^2 + amax*(n1*T)*(i*T - n2*T);
        delta_q(i) = q(i+1) - q(i);
    end
 
    run_enable(2) = 1;
else
    %Quy hoach van toc hinh S curve, khong tu dieu chinh vmax khi van
    %toc bi suy bien, nhung canh bao va khong cho phep chay mo phong
    t1 = v_max/amax;
    t2 = 2*t1;
    t3 = q_max/v_max;
    t4 = t3 + t1;
    t5 = t3 + 2*t1;
    q = zeros(1);
    delta_q = zeros(1);

    if (t3 < t2)
        run_enable(2) = 0;
        delta_q = 0;

        return
        
    else
        n1 = floor(t1/T);
        n2 = 2*n1;
        n3 = floor(t3/T);
        n4 = n3 + n1;
        n5 = n3 + 2*n1;
        q(1) = 0;
        k = (amax^2)/v_max;
        for i = 1:n1
            q(i+1) = (k*(i*T)^3)/6;
            delta_q(i) = q(i+1) - q(i);
        end
        for i = n1+1:n2
            q(i+1) = q(n1+1) + (k*((T*n1)^2)*((i-n1)*T))/2 + (k*(n1*T)*(((i-n1)*T)^2))/2 -(k*(((i-n1)*T)^3))/6;
            delta_q(i) = q(i+1) - q(i);
        end
            Vmax = k*((n1*T)^2)/2 + k*(n1*T)*(n2-n1)*T - k*(((n2-n1)*T)^2)/2;
        for i = n2+1:n3
            q(i+1) = q(n2+1) + Vmax*(i-n2)*T;
            delta_q(i) = q(i+1) - q(i);
        end
       
        for i = 1:n2
            delta_q(n3+i) = delta_q(n2+1-i);
        end
        run_enable(2) = 1;
    
    end
end
    

    
    
    
    
    
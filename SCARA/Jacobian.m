function [Jacobian] = Jacobian(T01,T02,T03,T04)
    z0 = [0; 0; 1];
    p0 = [0; 0; 0];
    z1 = T01(1:3,3);
    p1 = T01(1:3,4);
    z2 = T02(1:3,3);
    p2 = T02(1:3,4);
    z3 = T03(1:3,3);
    p3 = T03(1:3,4);
    z4 = T04(1:3,3);
    p4 = T04(1:3,4);
    %Jp1 = z0x(p4 - p0); Jp2 = z1x(p4 - p1); Jp3 = z2; jp4 =z3x(p4-p3)
    % uxp = [-uzpy + uypz; uzpx - uxpz; -uypx + uxpy]
    a = p4 - p0;
    Jp1 = [-z0(3)*a(2)+z0(2)*a(3); z0(3)*a(1)-z0(1)*a(3); -z0(2)*a(1)+z0(1)*a(2)];
    a = p4 - p1;
    Jp2 = [-z1(3)*a(2)+z1(2)*a(3); z1(3)*a(1)-z1(1)*a(3); -z1(2)*a(1)+z1(1)*a(2)];
    %a = p4 - p2;
    Jp3 = z2;
    a = p4-p3;
    Jp4 = [-z3(3)*a(2)+z3(2)*a(3); z3(3)*a(1)-z3(1)*a(3); -z3(2)*a(1)+z3(1)*a(2)];
    Jo1 = z0; Jo2 = z1; Jo3 = [0; 0; 0]; Jo4 = z3;
    A = cat(1,Jp1,Jo1);
    B = cat(1,Jp2,Jo2);
    C = cat(1,Jp3,Jo3);
    D = cat(1,Jp4,Jo4);
    Jacobian = cat(2,A,B,C,D);
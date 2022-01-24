
function varargout = SCARA1(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SCARA1_OpeningFcn, ...
                   'gui_OutputFcn',  @SCARA1_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before SCARA1 is made visible.
function SCARA1_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SCARA1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);

global x;
global y;
global z;
global tmp_x; 
global tmp_y; 
global tmp_z; 
global tmp_phi
global Path_Planning_Mode;
global TrajectoryMode;
global Kp;
global Ki;
global Kd;
global vmax_set;
global amax_set;
global run_enable;
run_enable = [0, 0];%Sua lai sau
global PID_Ready
PID_Ready = 0;
vmax_set = 0;
amax_set = 0;
Path_Planning_Mode = 1;
TrajectoryMode = 1;
Kp = 0;
Ki = 0;
Kd = 0;
x = -5;
y = -5;
z = 15;
[theta1,theta2,d3,theta4,mark] = Inverse_kinematic(x,y,z,[3,3],[3,3],0);
EF = vedothi3d(theta1,theta2,d3,theta4);
axis([-20 20 -20 20 0 30]);
grid on;
tmp_x = EF(1);
tmp_y = EF(2);
tmp_z = EF(3);
tmp_phi = 0;
rotate3d on;
figure(1);
figure(2);



% --- Outputs from this function are returned to the command line.
function varargout = SCARA1_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;


% --- Executes on slider movement.
function sliderTheta1_Callback(hObject, eventdata, handles)

global theta1;
theta1 = get(handles.sliderTheta1,'value');
set(handles.editTheta1,'string',num2str(theta1));




% --- Executes during object creation, after setting all properties.
function sliderTheta1_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function sliderTheta2_Callback(hObject, eventdata, handles)

global theta2;
theta2 = get(handles.sliderTheta2,'value');
set(handles.editTheta2,'string',num2str(theta2));

% --- Executes during object creation, after setting all properties.
function sliderTheta2_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function sliderD3_Callback(hObject, eventdata, handles)

global d3;
d3 = get(handles.sliderD3,'value');
set(handles.editD3,'string',num2str(d3));

% --- Executes during object creation, after setting all properties.
function sliderD3_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function sliderTheta4_Callback(hObject, eventdata, handles)

global theta4;
theta4 = get(handles.sliderTheta4,'value');
set(handles.editTheta4,'string',num2str(theta4));

% --- Executes during object creation, after setting all properties.
function sliderTheta4_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function editTheta1_Callback(hObject, eventdata, handles)

global theta1;
theta1 = str2num(get(handles.editTheta1,'string'));
if (theta1 > 6.283185)
    theta1 = 6.283185;
elseif (theta1 < -6.283185)
    theta1 = -6.283185;
end
set(handles.sliderTheta1,'value',theta1);

% --- Executes during object creation, after setting all properties.
function editTheta1_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editTheta2_Callback(hObject, eventdata, handles)

global theta2;
theta2 = str2num(get(handles.editTheta2,'string'));
if (theta2 > 6.283185)
    theta2 = 6.283185;
elseif (theta2 < -6.283185)
    theta2 = -6.283185;
end
set(handles.sliderTheta2,'value',theta2);

% --- Executes during object creation, after setting all properties.
function editTheta2_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editD3_Callback(hObject, eventdata, handles)

global d3;
d3 = str2num(get(handles.editD3,'string'));
if (d3 > 20)
    d3 = 20;
elseif (d3 < 0)
    d3 = 0;
end
set(handles.sliderD3,'value',d3);

% --- Executes during object creation, after setting all properties.
function editD3_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editTheta4_Callback(hObject, eventdata, handles)
global theta4;
theta4 = str2num(get(handles.editTheta4,'string'));
if (theta4 > 6.283185)
    theta4 = 6.283185;
elseif (theta4 < -6.283185)
    theta4 = -6.283185;
end
set(handles.sliderTheta4,'string',num2str(theta4));

% --- Executes during object creation, after setting all properties.
function editTheta4_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonForward.
function pushbuttonForward_Callback(hObject, eventdata, handles)
global theta1;
global theta2;
global d3;
global theta4;
global tmp1;
global tmp2;
global tmp3;
global tmp4;


[T01,T02,T03,T04] = EF_HomoTransform(theta1,theta2,d3,theta4);
EF = vedothi3d(theta1,theta2,d3,theta4);
tmp1 = theta1;
tmp2 = theta2;
tmp3 = d3;
tmp4 = theta4;

set(handles.editX,'string',num2str(EF(1)));
set(handles.editY,'string',num2str(EF(2)));
set(handles.editZ,'string',num2str(EF(3)));
phi = atan((EF(3)/sqrt(EF(1)^2 + EF(2)^2)));
set(handles.editPhi,'string',num2str(phi));





function editX_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function editX_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editY_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function editY_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editZ_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function editZ_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editPhi_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function editPhi_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_X_Callback(hObject, eventdata, handles)
global x;
global y;
global z;
global phi;

x = get(handles.slider_X,'value');
if (x^2 + y^2) > 100
    y = sqrt(100 - x^2);
end
set(handles.edit_X,'string',num2str(x));


% --- Executes during object creation, after setting all properties.
function slider_X_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_Z_Callback(hObject, eventdata, handles)
global x;
global y;
global z;
global phi;

z = get(handles.slider_Z,'value');
if z < 15
    z =15;
end
set(handles.edit_Z,'string',num2str(z));


% --- Executes during object creation, after setting all properties.
function slider_Z_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_Y_Callback(hObject, eventdata, handles)
global x;
global y;
global z;
global phi;

y = get(handles.slider_Y,'value');
if (y^2 + x^2) > 100
    x = sqrt(100 - y^2);
end
set(handles.edit_Y,'string',num2str(y));

% --- Executes during object creation, after setting all properties.
function slider_Y_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider_Phi_Callback(hObject, eventdata, handles)
global x;
global y;
global z;
global phi;

phi = get(handles.slider_Phi,'value');
set(handles.edit_Phi,'string',num2str(phi));

% --- Executes during object creation, after setting all properties.
function slider_Phi_CreateFcn(hObject, eventdata, handles)

if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit_X_Callback(hObject, eventdata, handles)

global x;
global y;
global z;
global phi;

x = str2num(get(handles.edit_X,'string'));
set(handles.slider_X,'value',x);

% --- Executes during object creation, after setting all properties.
function edit_X_CreateFcn(hObject, eventdata, handles)


if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_Y_Callback(hObject, eventdata, handles)
global x;
global y;
global z;
global phi;

y = str2num(get(handles.edit_Y,'string'));
set(handles.slider_Y,'value',y);

% --- Executes during object creation, after setting all properties.
function edit_Y_CreateFcn(hObject, eventdata, handles)


if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_Z_Callback(hObject, eventdata, handles)

global x;
global y;
global z;
global phi;

z = str2num(get(handles.edit_Z,'string'));
set(handles.slider_Z,'value',z);

% --- Executes during object creation, after setting all properties.
function edit_Z_CreateFcn(hObject, eventdata, handles)


if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_Phi_Callback(hObject, eventdata, handles)

global x;
global y;
global z;
global phi;

phi = str2num(get(handles.edit_Phi,'string'));
set(handles.slider_Phi,'value',phi);

% --- Executes during object creation, after setting all properties.
function edit_Phi_CreateFcn(hObject, eventdata, handles)


if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonInverse.
function pushbuttonInverse_Callback(hObject, eventdata, handles)
global x;
global y;
global z;
global phi;
global tmp_x;
global tmp_y;
global tmp_z;
global tmp_phi;
global Path_Planning_Mode;
global TrajectoryMode;
global vmax_set;
global amax_set;
global run_enable;
global q_x;
global q_y;
global q_z;
global q_phi;
global qx_tun;
global qy_tun;
global qz_tun;
global PID_Ready;
if PID_Ready == 1
    q_x = qx_tun;
    q_y = qy_tun;
    q_z = qz_tun;
end


T = 0.01;
axes(handles.axes1);
error_flag = 0;
if (Path_Planning_Mode == 1)
    if (run_enable(2) == 0)
        set(handles.textWarning,'string','Error: Something went wrong, run invalid!');
        error_flag = 1;
    else             
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
            theta1_pre = theta1;
            theta2_pre = theta2;
            EF = vedothi3d(theta1,theta2,d3,theta4);
            x_EF(i) = EF(1);
            y_EF(i) = EF(2);
            z_EF(i) = EF(3);
            line(x_EF,y_EF,z_EF,'LineWidth',3,'Marker','.');
        end
        grid on;
        BT = (q_x(1) - tmp_x)^2 + (q_y(1) - tmp_y)^2 + (q_z(1) - tmp_z)^2;
        v(1) = sqrt(BT)/T;
        a(1) = v(1)/T;
        q(1) = sqrt(BT);
        for i = 2:length(q_x)
            BT = (q_x(i) - q_x(i-1))^2 + (q_y(i) - q_y(i-1))^2 + (q_z(i) - q_z(i-1))^2;
            q(i) = q(i-1) + sqrt(BT);
            v(i) = sqrt(BT)/T;
            a(i) = (v(i) - v(i-1))/T;            
        end
        [singular,warning,theta1_dot,theta2_dot,d3_dot,theta4_dot,index] = differential_kinematic(q_x,q_y,q_z,q_phi);
        if PID_Ready == 1
            tmp_x = x;
            tmp_y = y;
            tmp_z = z;
            tmp_phi = phi;
        end
    end
elseif (Path_Planning_Mode == 2)
    if((run_enable(1) == 0)||(run_enable(2) == 0))
         set(handles.textWarning,'string','Error: Something went wrong, run invalid!');
         error_flag = 1;
    else
        for i =1:length(q_x)  
            if (i == 1)
                dir = [3,3];
                pre_mark = [3,3];
                [theta1_pre,theta2_pre,d3_pre,theta4_pre, mark] =  Inverse_kinematic(tmp_x,tmp_y,tmp_z,dir,pre_mark,tmp_phi);
                pre_mark = mark;
            end
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
            theta1_pre = theta1;
            theta2_pre = theta2;
            EF = vedothi3d(theta1,theta2,d3,theta4);
            x_EF(i) = EF(1);
            y_EF(i) = EF(2);
            z_EF(i) = EF(3);
            line(x_EF,y_EF,z_EF,'LineWidth',3,'Marker','.');
        end
        grid on;
        BT = (q_x(1) - tmp_x)^2 + (q_y(1) - tmp_y)^2 + (q_z(1) - tmp_z)^2;
        q(1) = sqrt(BT);
        v(1) = sqrt(BT)/T;
        a(1) = v(1)/T;
        for i = 2:length(q_x)
            BT = (q_x(i) - q_x(i-1))^2 + (q_y(i) - q_y(i-1))^2 + (q_z(i) - q_z(i-1))^2;
            v(i) = sqrt(BT)/T;
            q(i) = q(i-1) + sqrt(BT);
            a(i) = (v(i) - v(i-1))/T;
        end
        [singular,warning,theta1_dot,theta2_dot,d3_dot,theta4_dot,index] = differential_kinematic(q_x,q_y,q_z,q_phi);
        if PID_Ready == 1
        tmp_x = x;
        tmp_y = y;
        tmp_z = z;
        tmp_phi = phi;
        end
    end
end

      
if(error_flag == 0)    
set(handles.edit_X,'string',num2str(x));
set(handles.edit_Y,'string',num2str(y));
set(handles.edit_Z,'string',num2str(z));
set(handles.edit_Theta1,'string',num2str(theta1));
set(handles.edit_Theta2,'string',num2str(theta2));
set(handles.edit_D3,'string',num2str(d3));
set(handles.edit_Theta4,'string',num2str(theta4));
rotate3d on;

index
singular
for i = 1:length(q_x)
    tgian(i) = i*T;
end
% Ve do thi

if PID_Ready == 0
    color = 'red';
elseif PID_Ready == 1
    color = 'blue';
end

figure(1);
subplot(3,3,1);
hold on;
plot(tgian,q_x,color);
if warning ~= 0
    plot(index*T,singular(:,1),'*c');
end
xlabel('tgian');
ylabel('q_x(t)');
title('Quy hoach tren Ox');
grid on;
subplot(3,3,2);
hold on;
plot(tgian,q_y,color);
if warning ~= 0
    plot(index*T,singular(:,2),'*c');
end
xlabel('tgian');
ylabel('q_y(t)');
title('Quy hoach tren Oy');
grid on;
subplot(3,3,3);
hold on;
plot(tgian,q_z,color);
if warning ~= 0
    plot(index*T,singular(:,3),'*c');
end
xlabel('tgian');
ylabel('q_z(t)');
title('Quy hoach tren Oz');
grid on;
subplot(3,3,4);
hold on;
plot(tgian,v,color);
xlabel('tgian');
ylabel('v(t)');
title('Quy hoach van toc');
grid on;
subplot(3,3,5);
hold on;
plot(tgian,a,color);
xlabel('tgian');
ylabel('a(t)');
title('Gia toc');
grid on;
subplot(3,3,6);
hold on;
plot(tgian,q,color);
xlabel('tgian');
ylabel('q(t');
grid on;
plot_Joins_velocities(theta1_dot,theta2_dot,d3_dot,theta4_dot,tgian,index,color);    
end
set(handles.textWarning,'string','Succesfully, would you want to try again?');
PID_Ready = 0;

function edit_Theta1_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function edit_Theta1_CreateFcn(hObject, eventdata, handles)


if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_Theta2_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function edit_Theta2_CreateFcn(hObject, eventdata, handles)


if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_D3_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function edit_D3_CreateFcn(hObject, eventdata, handles)



if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_Theta4_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function edit_Theta4_CreateFcn(hObject, eventdata, handles)




if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonClose.
function pushbuttonClose_Callback(hObject, eventdata, handles)

close



function editWarning_Callback(hObject, eventdata, handles)





% --- Executes during object creation, after setting all properties.
function editWarning_CreateFcn(hObject, eventdata, handles)




if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonPID_SaveVal.
function pushbuttonPID_SaveVal_Callback(hObject, eventdata, handles)
global Kp;
global Ki;
global Kd;
global q_x;
global q_y;
global q_z;
global q_phi
global tmp_x;
global tmp_y;
global tmp_z;
global tmp_phi
global qx_tun;
global qy_tun;
global qz_tun
global PID_Ready;
PID_Ready = 0;
qx_tun = zeros(1);
qy_tun = zeros(1);
qz_tun = zeros(1);
theta1 = zeros(1);
theta2 = zeros(1);
d3 = zeros(1);
time = zeros(1);
del_theta1 = zeros(1);
del_theta2 = zeros(1);
del_d3 = zeros(1);
del_theta4 = zeros(1);
theta4 = zeros(1);
Ts = 0.01;
T_stop = length(q_x)*Ts;
save('TunPID.mat','Kp','Ki','Kd','Ts','T_stop');
evalin('base', 'load(''TunPID.mat'')');
dir = [3,3];
pre_mark = [3,3];
[theta1_0,theta2_0,d3_0,theta4_0,mark] = Inverse_kinematic(tmp_x,tmp_y,tmp_z,dir,pre_mark,tmp_phi);
pre_mark = mark;
theta1_pre = theta1_0;
theta2_pre = theta2_0;

%index = 1
%length(q_x)
%length(q_y)
%length(q_z)


for i = 1: length(q_x)
    time(i) = i*Ts;
    
    [theta1(i), theta2(i), d3(i), theta4(i),mark] = Inverse_kinematic(q_x(i),q_y(i),q_z(i),dir,pre_mark,q_phi(i));
    
    
    pre_mark = mark;
    del_theta1(i) = theta1(i) - theta1_0;
    del_theta2(i) = theta2(i) - theta2_0;
    del_d3(i) = d3(i) - d3_0;
    del_theta4(i) = theta4(i) - theta4_0;
    dtheta1 = theta1(i) - theta1_pre;
    dtheta2 = theta2(i) - theta2_pre;
    if (dtheta1 > 0)
        dir(1) = 1;
    elseif dtheta1 < 0
        dir(1) = 0;
    end
    if (dtheta2 > 0)
        dir(2) = 1;
    elseif dtheta2 < 0
        dir(2) = 0;
    end
    theta1_pre = theta1(i);
    theta2_pre = theta2(i);
end


%theta1
%theta1(1) - 2*pi
%theta2

[singular,warning,theta1_dot,theta2_dot,d3_dot,theta4_dot] = differential_kinematic(q_x,q_y,q_z,q_phi);
    if warning ~= 0
        set(handles.textWarning,'string',strcat('Warning: Kinematic Singularities at coordinate:',int2str(singular)));
    end

sig = [time;del_theta1];
save Signal.mat -v7.3 sig;
out = sim('PID_Tun.slx');
theta1_tun = out.theta.signals.values;
theta1_tun(1) = [];
for i = 1: length(theta1_tun)
    theta1_tun(i) = theta1_tun(i) + theta1_0;
end



sig = [time; del_theta2];
save Signal.mat -v7.3 sig;
out = sim('PID_Tun.slx');
theta2_tun = out.theta.signals.values;
theta2_tun(1) = [];
for i = 1: length(theta2_tun)
    theta2_tun(i) = theta2_tun(i) + theta2_0;
end



sig = [time; del_d3];
save Signal.mat -v7.3 sig;
out = sim('PID_Tun.slx');
d3_tun = out.theta.signals.values;
d3_tun(1) = [];
for i = 1:length(d3_tun)
    d3_tun(i) = d3_tun(i) + d3_0;
end



sig = [time; del_theta4];
save Signal.mat -v7.3 sig;
out = sim('PID_Tun.slx');
theta4_tun = out.theta.signals.values;
theta4_tun(1) = [];
for i = 1:length(theta4_tun)
    theta4_tun(i) = theta4_tun(i) + theta4_0;
end



for i = 1:length(theta1)
    [T01,T02,T03,T04] = EF_HomoTransform(theta1_tun(i), theta2_tun(i), d3_tun(i), theta4_tun(i));
    EF = T04*[0;0;0;1];
    qx_tun(i) = EF(1);
    qy_tun(i) = EF(2);
    qz_tun(i) = EF(3);
end



figure;
subplot(3,3,1);
plot(time,theta1_tun,'blue');
hold on;
plot(time,theta1,'red');
grid on;
title('theta1');
xlabel('t(s)');
ylabel('theta1(t)');

subplot(3,3,2);
plot(time,theta2_tun,'blue');
hold on;
plot(time,theta2,'red');
grid on;
title('theta2');
xlabel('t(s)');
ylabel('theta2(t)');

subplot(3,3,3);
plot(time,d3_tun,'blue');
hold on;
plot(time,d3,'red');
grid on;
title('d3');
xlabel('t(s)');
ylabel('d3(t)');

subplot(3,3,4);
plot(time,theta4_tun,'blue');
hold on;
plot(time,theta4,'red');
grid on;
title('theta4');
xlabel('t(s)');
ylabel('theta4(t)');

subplot(3,3,5);
plot(time,qx_tun,'blue');
hold on;
plot(time,q_x,'red');
grid on;
title('q_x');
xlabel('t(s)');
ylabel('q_x(t)');

subplot(3,3,6);
plot(time,qy_tun,'blue');
hold on;
plot(time,q_y,'red');
grid on;
title('q_y');
xlabel('t(s)');
ylabel('q_y(t)');

subplot(3,3,7);
plot(time,qz_tun,'blue');
hold on;
plot(time,q_z,'red');
grid on;
title('q_z');
xlabel('t(s)');
ylabel('q_z(t)');
PID_Ready = 1;

% --- Executes on button press in pushbuttonPID_Tun.
function pushbuttonPID_Tun_Callback(hObject, eventdata, handles)
global Kp;
global Ki;
global Kd;

% Nguoi dung cai dat cac gia tri Kp, Ki, Kd

Ts = 9/60;
T_stop = 9; %[s]
time = linspace(0,9,61);
data = linspace(3,3,61);
sig = [time;data];
save Signal.mat -v7.3 sig;
save('TunPID.mat','Kp','Ki','Kd','Ts','T_stop');
evalin('base', 'load(''TunPID.mat'')');


out = sim('PID_Tun.slx');
theta = out.theta.signals.values;
t = out.tout;

axes(handles.axes2);
axis([0 9 0 6]);
plot(t,theta,'Color','blue','LineWidth',3);
hold on;
plot(t,data,'Color','red','LineWidth',2);
grid on;
hold off;




% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)




% --- Executes on button press in pushbuttonGet_Val_Plan.
function pushbuttonGet_Val_Plan_Callback(hObject, eventdata, handles)
global tmp_x; 
global tmp_y; 
global tmp_z; 
global Path_Planning_Mode;
global TrajectoryMode;
global vmax_set;
global amax_set;
global run_enable;
global q_x;
global q_y;
global q_z;
global q_phi;
q_x = zeros(1);
q_y = zeros(1);
q_z = zeros(1);
time = zeros(1);
T = 0.01;
if((vmax_set == 0)||(amax_set == 0))
    set(handles.textWarning,'string','Error: Please type vmax and amax');
else
    if Path_Planning_Mode == 1
        [q_x,q_y,q_z,q_phi] = linear_path_planning(amax_set,vmax_set);
         [singular,warning,theta1_dot,theta2_dot,d3_dot,theta4_dot,index] = differential_kinematic(q_x,q_y,q_z,q_phi);
         
         singular
        if warning ~= 0
            set(handles.textWarning,'string',strcat(' Kinematic singularity at coordinate: ',num2str(singular)));
            w = questdlg('Press Yes to continuous, No to choose another coordinate');
            switch w
                case 'Yes'
                    run_enable(1) = 1;
                case 'No'
                    run_enable(1) = 0;
            end
        end
        if run_enable(1) == 0
            set(handles.textWarning,'string','Error: Singularities. Please choose another coordinate');
        elseif run_enable(2) == 0
            set(handles.textWarning,'string','Error: Velocity degenerate');
        else
            set(handles.textWarning,'string','Planning valid, you can run inverse');
        end
    elseif (Path_Planning_Mode == 2)
        [q_x,q_y,q_z,q_phi] = circle_path_planning(amax_set,vmax_set);
        [singular,warning,theta1_dot,theta2_dot,d3_dot,theta4_dot,index] = differential_kinematic(q_x,q_y,q_z,q_phi);
        
        if warning ~= 0
            set(handles.textWarning,'String',strcat('Kinematic singularity at coordinate: ', num2str(singular)));
            w = questdlg('Press Yes to continuous, No to choose another coordinate');
            switch w
                case 'Yes'
                    run_enable(1) = 1;
                case 'No'
                    run_enable(1) = 0;
            end
        end
        if (run_enable(1) == 0)
            set(handles.textWarning,'string','Error: Singularities. Please choose another coordinate');
        elseif (run_enable(2) == 0)
            set(handles.textWarning,'string','Error: Velocity degenerate');
        else
            set(handles.textWarning,'string','Planning valid, you can run inverse');
        end
    end
    for i=1:length(q_x)
        time(i) = i*T;
    end
    %theta4_dot
    plot_Joins_velocities(theta1_dot,theta2_dot,d3_dot,theta4_dot,time,index,'red');
end

        
            

%----------------------------------------------------------------------------
function editPID_Kp_Callback(hObject, eventdata, handles)
    global Kp;
    Kp = str2num(get(handles.editPID_Kp,'string'));

% --- Executes during object creation, after setting all properties.
function editPID_Kp_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editPID_Ki_Callback(hObject, eventdata, handles)
    global Ki;
    Ki = str2num(get(handles.editPID_Ki,'string'));

% --- Executes during object creation, after setting all properties.
function editPID_Ki_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editPID_Kd_Callback(hObject, eventdata, handles)
    global Kd;
    Kd = str2num(get(handles.editPID_Kd,'string'));



function editPID_Kd_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function help_Callback(hObject, eventdata, handles)




% --------------------------------------------------------------------
function tool_Callback(hObject, eventdata, handles)




% --------------------------------------------------------------------
function PID_Save_Callback(hObject, eventdata, handles)




% --------------------------------------------------------------------
function PID_Tun_Callback(hObject, eventdata, handles)




% --------------------------------------------------------------------
function plan_Callback(hObject, eventdata, handles)




% --------------------------------------------------------------------
function run_inverse_Callback(hObject, eventdata, handles)




% --------------------------------------------------------------------
function quit_Callback(hObject, eventdata, handles)
    choice = questdlg('Would you like to close?',...
        'Choice menu',...
        'Yes','No','Yes');
    switch choice
        case 'Yes'
            close
        case 'No'
    end

% --------------------------------------------------------------------
function about_Callback(hObject, eventdata, handles)
    web SCARA ROBOT SIMULATION HELP.htm


% --- Executes on selection change in popupmenuTrajectoryMode.
function popupmenuTrajectoryMode_Callback(hObject, eventdata, handles)
    global TrajectoryMode;
    TrajectoryMode = get(handles.popupmenuTrajectoryMode,'value');
    
    
% --- Executes during object creation, after setting all properties.
function popupmenuTrajectoryMode_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in radiobuttonLinear_Mode.
function radiobuttonLinear_Mode_Callback(hObject, eventdata, handles)
    global Path_Planning_Mode;
    Path_Planning_Mode = get(handles.radiobuttonLinear_Mode,'value');
    set(handles.radiobuttonCircle_Mode,'value',0);
        


% --- Executes on button press in radiobuttonCircle_Mode.
function radiobuttonCircle_Mode_Callback(hObject, eventdata, handles)
    global Path_Planning_Mode;
    Path_Planning_Mode = (get(handles.radiobuttonCircle_Mode,'value')) +  1
    set(handles.radiobuttonLinear_Mode,'value',0);



function editSet_vmax_Callback(hObject, eventdata, handles)
    global vmax_set;
    vmax_set = str2num(get(handles.editSet_vmax,'string'));

% --- Executes during object creation, after setting all properties.
function editSet_vmax_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editSet_amax_Callback(hObject, eventdata, handles)
    global amax_set;
    amax_set = str2num(get(handles.editSet_amax,'string'));

% --- Executes during object creation, after setting all properties.
function editSet_amax_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonSave_Setting.
function pushbuttonSave_Setting_Callback(hObject, eventdata, handles)
    global run_enable;
    global vmax_set;
    global q_max
    global amax_set;
    global Path_Planning_Mode;
    global x;
    global y;
    global z;
    global phi;
    global tmp_phi;
    if ((vmax_set == 0)||(amax_set == 0))
        set(handles.textWarning,'string','Error: You must type vmax, amax differ from zeros');
    else
        if (Path_Planning_Mode == 1)
            [qx,qy,qz] = linear_path_planning(amax_set,vmax_set);
            for i = 1: length(qx)
                BT = qx(i)^2 + qy(i)^2;
                if (sqrt(BT)>10)
                    run_enable(1) = 0;
                    break
                else
                    run_enable(1) = 1;
                end
            end
            if (run_enable(2) ==0)
               set(handles.textWarning,'string','Error: Velocity degenerate');
            elseif (run_enable(1) == 0)
               set(handles.textWarning,'string','Error: workspace singularities');
            else
               set(handles.textWarning,'string','Save valid, you can push Plan');
            end
        elseif (Path_Planning_Mode == 2)      
            [qx,qy,qz] = circle_path_planning(amax_set,vmax_set);
            for i = 1: length(qx)
                BT = qx(i)^2 + qy(i)^2;
                if (sqrt(BT)>10)
                    run_enable(1) = 0;
                    break
                else
                    run_enable(1) = 1;
                end
            end
           if (run_enable(1) == 0)
            set(handles.textWarning,'string','Error: workspace singularities.Please choose another coordinate');
           elseif (run_enable(2) == 0)
            set(handles.textWarning,'string','Error: Velocity degenerate');
           else
             set(handles.textWarning,'string','Save valid, you can push Plan');
           end
        end
    end
    
    
        
 
    
    


% --- Executes during object creation, after setting all properties.
function textWarning_CreateFcn(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function pushbuttonGet_Val_Plan_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbuttonGet_Val_Plan (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

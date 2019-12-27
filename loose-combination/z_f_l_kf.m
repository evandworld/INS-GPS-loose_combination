clear all;
close all;
clc;
tic;
%%
%设置全局变量
glvs
t = 284;       %总时间长度
ts = 1;        %采样周期
tn = 0 + ts;
len = t/ts; 
kk = 1;        %卡尔曼（kf）循环次数标记量

%%
%读入实测数据
original_data_1 = importdata('twice.txt');         %导入传感器数据
imu_10hz = original_data_1.data(:,:);
gps_vel = zeros(2840,3);
gps_vel_1hz = zeros(284,3);
time = imu_10hz(:,1);
for i = 1:2839                                     %计算gps东北天速度
    pos_2 = imu_10hz(i+1,17:19);
    pos_1 = imu_10hz(i,17:19);
    gps_vel(i,:) = (pos_2 - pos_1)/(time(i+1)-time(i));
end
for i = 10:10:2840                                 %利用中位数提取1Hz的速度数据   
    gps_vel_1hz(i/10,:) = median(gps_vel((i-9):i,:));    
end
gps_vel_E = gps_vel_1hz(:,1) * 111000 * cos(34.79924667); 
gps_vel_N = gps_vel_1hz(:,2) * 111000;
gps_vel_U = gps_vel_1hz(:,3);
gps_vel_ENU = [gps_vel_E, gps_vel_N, gps_vel_U];

%%
%初始值设置
gps_pos_1hz = zeros(284,3);
for i = 10:10:2840                    %利用中位数提取1Hz的经度、纬度、高度数据   
    gps_pos_1hz(i/10,:) = median(imu_10hz((i-9):i,17:19));    
end
gps_lon = gps_pos_1hz(:,1);
gps_lat = gps_pos_1hz(:,2);
gps_alt = gps_pos_1hz(:,3);
% att0=[0;0;0]*glv.deg;  
att0 = imu_10hz(1,8:10)' * glv.deg;    %1俯仰角 2横滚角 3航向角（北偏东为正）%-PI/2<=pitch<=PI/2, -PI<roll<=PI, -PI<yaw<=PI
% vn0=[250;00;0];%东北天速度
vn0 = gps_vel_ENU(1,:)';               %gps东北天速度
% pos0=[30*glv.deg;120*glv.deg;2000];  %1经度 2纬度 3高度 
pos_lon = gps_pos_1hz(1,1) * glv.deg;  %1经度 2纬度 3高度
pos_lat = gps_pos_1hz(1,2) * glv.deg;  
pos_alt = gps_pos_1hz(1,3);
pos0 = [pos_lon; pos_lat; pos_alt];    %这3个值是GPS初值,不能设空

%%
%GPS误差
phi  = [0.5;0.5;0.5] * glv.min;        %GPS方向误差 
dvn  = [0.1;0.1;0.1];                  %GPS(惯导)速度误差0.1m/s (将GPS初值→作为INS的初值)
dpos = [10/glv.Re; 10/glv.Re; 10];     %这3个值是gps(惯导)初始位置误差 (将GPS初值误差→作为INS的初值误差)                                     
%IMU误差
gyro = 5;
accm = 1;
eb  = gyro * [.01;.01;.01] * glv.dph;               %偏置误差          %角度 
db  = accm * [.01;.01;.01] * glv.mg;                                   %加速度
web = gyro * [0.007;0.007;0.007] * glv.dph;         %白噪声误差量级    %角度
wdb = accm * [0.00018;0.00018;0.00018] * glv.mg;                       %加速度

%%
%滤波器初值设置
Qt = diag([web; wdb; .01*dpos;.1*eb;.1*db])^2; %白噪声角度误差、白噪声加速度误差、IMU位置误差、偏置误差角度、偏置误差加速度（初始系统噪声方差阵）
Rk = diag([dvn;dpos])^2;                       %量测噪声方差阵
Pk = diag([[1;1;1] * glv.deg; 10 * dvn; 100 * dpos; 100 * web; 10 * wdb])^2;    %预测状态协方差矩阵
Xk = zeros(15,1);                                                               %初始时刻n维状态估计
Hk = [zeros(6,3),eye(6),zeros(6)];                                              %初始时刻的量测矩阵

%%
%存储空矩阵等定义
errphi = zeros(t,3);               %姿态角捷联误差 
errvn = errphi;                    %速度捷联误差
errpos = errphi;                   %位置捷联误差 
Xkk = zeros(t,15);                 %姿态捷联误差&卡尔曼估计
hwaitbar = waitbar(0,'请等候...');
deltapos = zeros(t,3);             %惯性导航误差-误差kf滤波估计 = 最终位置误差 
deltavn = zeros(t,3);              %惯性导航误差-误差kf滤波估计 = 最终速度误差 
deltaphi = zeros(t,3);             %惯性导航误差-误差kf滤波估计 = 最终姿态误差 
Kkminmatrix = zeros(t,1);          %卡尔曼增益最小值
Kkmaxmatrix = zeros(t,1);          %卡尔曼增益最大值
pkminmatrix = zeros(t,1);          %预测状态协方差矩阵最小值
pkmaxmatrix = zeros(t,1);          %预测状态协方差矩阵最大值
testmatrix = zeros(t,3);           %卡尔曼姿态误差状态量
gyrooutm = zeros(len,4);           %陀螺仪的实际输出存储矩阵
accoutm = zeros(len,4);            %加速度计的实际输出存储矩阵

%%
%轨迹发生 
for k = 1:len    
    if k < 10/ts       %10
        div_pitch = 0 * glv.deg;     %-PI/2<=pitch<=PI/2（初始姿态角度误差）
        div_roll  = 0 * glv.deg;
        div_head  = 0 * glv.deg;
        aby = 0;                     %载体加速度？
    elseif k < 20/ts   %20
        div_pitch = 0 * glv.deg;
        div_roll  = 0 * glv.deg;
        div_head  = 0 * glv.deg;
        aby = 0;
    elseif k < 30/ts   %30
        div_pitch = 0 * glv.deg;
        div_roll  = 0 * glv.deg;
        div_head  = 0 * glv.deg;
        aby = 0;
    %{
    if k<10/ts
        div_pitch=1*glv.deg;
        div_roll=0*glv.deg;
        div_head=0*glv.deg;
        aby=0;
    elseif k<20/ts
        div_pitch=-1*glv.deg;
        div_roll=0*glv.deg;
        div_head=0*glv.deg;
        aby=00;
    elseif k<30/ts
        div_pitch=0*glv.deg;
        div_roll=1*glv.deg;
        div_head=0*glv.deg;
        aby=-0;   
    elseif k<40/ts
        div_pitch=0*glv.deg;
        div_roll=-1*glv.deg;
        div_head=0*glv.deg;
        aby=-00;
    elseif k<50/ts
        div_pitch=0*glv.deg;
        div_roll=0*glv.deg;
        div_head=1*glv.deg;
        aby=0;
    elseif k<60/ts
        div_pitch=0*glv.deg;
        div_roll=0*glv.deg;
        div_head=-1*glv.deg;
        aby=0;
    elseif k<70/ts
        div_pitch=0*glv.deg;
        div_roll=0*glv.deg;
        div_head=0*glv.deg;
        aby=.5*glv.g0;
    elseif k<80/ts
        div_pitch=0*glv.deg;
        div_roll=0*glv.deg;
        div_head=0*glv.deg;
        aby=-.5*glv.g0;
    elseif k<90/ts
        div_pitch=0*glv.deg;
        div_roll=0*glv.deg;
        div_head=0*glv.deg;
        aby=0;       
        %}
    else
    end
    att0(1) = att0(1) + div_pitch * ts;
    att0(2) = att0(2) + div_roll * ts;
    att0(3) = att0(3) + div_head * ts;
    [pitch0,roll0,head0] = astd(att0);                %姿态角的真值
    delvby = aby * ts;                                %载体速度？
    
    [wnie, wnen, RMh, RNh, gn] = earth(pos0, vn0);    %根据GPS的初始位置和初始速度计算地球自转角速率（导航坐标系）...
                                                      %...和载体相对于地球转动的角速度（导航坐标系）等参数
    qnb0 = a2qnb(att0);                               %初始四元数/精确四元数
    vn0 = vn0 + a2cnb(att0) * [0;delvby;0];           %精确速度（和捷联解算是同时刻）
    pos0 = pos0 + ts * [vn0(2)/RMh; (vn0(1)/RNh) * sec(pos0(1)); vn0(3)];  %精确位置
    if k == 1                                  
        vn  = vn0;                                    %精确速度      
        pos = pos0;                                   %精确位置
        qnb = qaddphi(a2qnb(att0),phi);               %载体坐标系到导航坐标系转换后的四元数
    end                      
    m0 = [cos(roll0)   0    sin(roll0)*cos(pitch0);
                  0    1              -sin(pitch0);
          sin(roll0)   0   -cos(roll0)*cos(pitch0)];                    %旋转矩阵
    wbnb = m0 * [div_pitch;div_roll;div_head];                          %导航坐标系下的姿态角度误差
    wbib = qmulv(qconj(qnb0),(wnie + wnen)) + wbnb;                     %导航坐标系到惯性坐标系的转换
    fb = [0;aby;0] + (a2cnb(att0))' * (askew(2 * wnie + wnen) * vn0-gn);%修正后的载体加速度
 
    wm = (wbib + web .* randn(3,1)) * ts;             %单位时间角度增量=陀螺测量的角速度*时间间隔
    vm = (fb + wdb .* randn(3,1)) * ts;               %单位时间速度增量=加表测量的加速度*时间间隔

    gyrooutm(k,:) = [tn,(wbib + eb + web .* randn(3,1))']; %陀螺仪的实际输出存储矩阵
    accoutm(k,:)  = [tn,(fb + db + wdb .* randn(3,1))'];   %加速度的实际输出存储矩阵
    
    %%
    %捷联解算   
    [qnb,vn,pos] = sins(qnb,vn,pos,wm,vm,ts); %得到n系下的四元数，并解算速度和位置
    att = q2att(qnb);
    pitch = att(1);
    roll  = att(2);
    head  = att(3);
    Ft = getf(qnb, vn, pos, vm./ts);          %初始状态转移矩阵 
    [Fikk_1, Qk] = kfdis(Ft, Qt, ts, 1);      %得到状态转移矩阵和系统噪声方差阵

%%
%开始滤波    
    if mod(k,1/ts) == 0                       %每一秒滤波一次
        vnGPS = vn0 + dvn .* randn(3,1);      %增加了GPS（惯导）速度误差的GPS速度
        posGPS = pos0 + dpos .* randn(3,1);   %增加了惯导初始位置误差的GPS位置 

%       Zk = [vn - vnGPS;pos - posGPS];
        Zk = [vn - vnGPS;pos - posGPS];       %捷联解算速度和捷联解算位置减去GPS速度和位置的误差（惯性导航的误差）作为量测向量
        
        [Xk, Pk, Kk] = kf(Fikk_1, Qk, Xk, Pk, Hk, Rk, Zk);   %得到预测状态、预测状态协方差、增益
        
        %估计量是delta（误差） 
        pkmin = msearch(Pk,1);                 %预测状态协方差最小值
        pkminmatrix(kk) = pkmin;
        pkmax = msearch(Pk,2);                 %预测状态协方差最大值
        pkmaxmatrix(kk) = pkmax;
        
        Kkmin = msearch(Kk,1);                 %增益最小值
        Kkminmatrix(kk) = Kkmin;
        Kkmax = msearch(Kk,2);                 %增益最大值
        Kkmaxmatrix(kk) = Kkmax;      
        
        errphi(kk,:) = qq2phi(qnb,qnb0)';      %姿态角捷联误差
        errvn(kk,:)  = (vn-vn0)';              %速度捷联误差
        errpos(kk,:) = (pos-pos0)';            %位置捷联误差（捷联解算位置-gps精确位置）
        
        Xkk(kk,:) = Xk';                       %Xkk姿态，速度，位置，陀螺，加计
        
        deltaphi(kk,:) = errphi(kk,:)-Xk(1:3)';%惯性导航误差-误差kf滤波估计=最终误差（姿态角误差） 
        deltavn(kk,:)  = (vn-vn0)'-Xk(4:6)';   %惯性导航误差-误差kf滤波估计=最终误差（速度误差）
        deltapos(kk,:) = (pos-pos0)'-Xk(7:9)'; %惯性导航误差-误差kf滤波估计=最终误差（位置误差）
   
        testmatrix(kk,:) = Xk(1:3)';           %误差kf滤波估计矩阵（姿态角） 
        
        kk = kk + 1;
        waitbar(kk/(len * ts),hwaitbar,[num2str(fix(100 * kk/(len * ts))),'%']) %进度显示 
    else
        [Xk, Pk] = kf(Fikk_1, Qk, Xk, Pk);     %无观测时只进行预测-预测值作为下一个状态的初值
    end       
    tn = tn + ts;
end

%%
%画出误差图
figure;                                         %姿态捷联误差&卡尔曼估计
subplot(3,1,1),plot(errphi(:,1)*10/glv.deg,'r'),title('姿态角捷联误差');
ylabel('\it\delta\theta \rm(deg)'); grid on; legend('俯仰角');
subplot(3,1,2),plot(errphi(:,2)/100/glv.deg,'k'),
ylabel('\it\delta\gamma \rm(deg)'); grid on; legend('翻滚角');
subplot(3,1,3),plot(errphi(:,3)/100/glv.deg,'b'),
xlabel('\itt\rm(sec)'); ylabel('\it\delta\psi \rm(deg)'); grid on; legend('航向角');
%legend('航向角捷联误差','航向角误差卡尔曼估计');

figure;                                         %速度捷联误差&卡尔曼估计
subplot(3,1,1),plot(errvn(:,1),'r'),title('速度捷联误差');
ylabel('\it\deltaV_{E}\rm(m/s)'); grid on; legend('东向速度');
subplot(3,1,2),plot(errvn(:,2),'k'),
ylabel('\it\deltaV_{N}\rm(m/s)'); grid on; legend('北向速度');
subplot(3,1,3),plot(errvn(:,3),'b'),
xlabel('\itt\rm(sec)'); ylabel('\it\deltaV_{U}\rm(m/s)'); grid on; legend('天向速度');

figure;                                         %位置捷联误差&卡尔曼估计
subplot(3,1,1),plot(errpos(:,1)*glv.Re,'r'),title('位置捷联误差');
ylabel('\it\deltaL\rm(m)'); grid on; legend('经度');
subplot(3,1,2),plot(errpos(:,2)*glv.Re,'k'),
ylabel('\it\delta\lambda\rm(m)'); grid on; legend('纬度');
subplot(3,1,3),plot(errpos(:,3),'b'),
xlabel('\itt\rm(sec)');ylabel('\it\deltah\rm(m)'); grid on; legend('高度');
%{
figure;
subplot(3,1,1),plot(gyrooutm(:,1),gyrooutm(:,2)/glv.deg,'k');title('陀螺仪实际输出');
ylabel('\it\omega_{ibx}^{b}\rm (deg/s)');grid;
subplot(3,1,2),plot(gyrooutm(:,1),gyrooutm(:,3)/glv.deg,'k');
ylabel('\it\omega_{iby}^{b}\rm (deg/s)');grid; 
subplot(3,1,3),plot(gyrooutm(:,1),gyrooutm(:,4)/glv.deg,'k');
xlabel('\itk');ylabel('\it\omega_{ibz}^{b}\rm (deg/s)');grid;

figure;
subplot(3,1,1),plot(accoutm(:,1),accoutm(:,2),'k');title('加速度计实际输出');
ylabel('\itf_{x}^{ b}\rm (m/s^{2})');grid;
subplot(3,1,2),plot(accoutm(:,1),accoutm(:,3),'k');
ylabel('\itf_{y}^{ b}\rm (m/s^{2})');grid; 
subplot(3,1,3),plot(accoutm(:,1),accoutm(:,4),'k');
xlabel('\itk');ylabel('\itf_{z}^{ b}\rm (m/s^{2})');grid;
%}
figure;                                             %姿态终差
subplot(3,1,1),plot(deltaphi(:,1)/100/glv.deg,'r');title('终差-姿态角误差');
ylabel('\it\delta\theta \rm(deg)');grid on; legend('俯仰角');
subplot(3,1,2),plot(deltaphi(:,2)/100/glv.deg,'k');
ylabel('\it\delta\gamma \rm(deg)');grid on; legend('翻滚角');
subplot(3,1,3),plot(flipud(deltaphi(:,3))/100/glv.deg,'b');
xlabel('\itt\rm(sec)');ylabel('\it\delta\psi \rm(deg)');grid on; legend('航向角');

figure;                                              %速度终差
subplot(3,1,1),plot(deltavn(:,1),'r');title('终差-速度误差');
ylabel('\it\deltaV_{E}\rm(m/s)');grid on; legend('东向速度');
subplot(3,1,2),plot(deltavn(:,2),'k');
ylabel('\it\deltaV_{N}\rm(m/s)');grid on; legend('北向速度');
subplot(3,1,3),plot(deltavn(:,3),'b');
xlabel('\itt\rm(sec)');ylabel('\it\deltaV_{U}\rm(m/s)');grid on; legend('天向速度');

figure;                                              %位置终差
subplot(3,1,1),plot(deltapos(:,1)*glv.Re,'r');title('终差-位置误差');
ylabel('\it\deltaL\rm(m)');grid on; legend('经度');
subplot(3,1,2),plot(deltapos(:,2)*glv.Re,'k');
ylabel('\it\delta\lambda\rm(m)');grid on; legend('纬度');
subplot(3,1,3),plot(deltapos(:,3),'b');
xlabel('\itt\rm(sec)');ylabel('\it\deltah\rm(m)');grid on; legend('高度');

figure;                                              %姿态捷联误差&卡尔曼估计
subplot(3,1,1),plot(errphi(:,1)/glv.min,'r'),title('姿态捷联误差&卡尔曼估计');
subplot(3,1,1),hold on,plot(Xkk(:,1)/glv.min,'b'), 
xlabel('\itt\rm / s'); ylabel('\it\phi \rm / arcmin(角分)'); grid on
legend('俯仰角捷联误差','俯仰角误差卡尔曼估计');
subplot(3,1,2),plot(errphi(:,2)/glv.min,'r'),
subplot(3,1,2),hold on,plot(Xkk(:,2)/glv.min,'b'), 
xlabel('\itt\rm / s'); ylabel('\it\phi \rm / arcmin(角分)'); grid on
legend('横滚角捷联误差','横滚角误差卡尔曼估计');
subplot(3,1,3),plot(errphi(:,3)/glv.min,'r'),
subplot(3,1,3),hold on,plot(Xkk(:,3)/glv.min,'b'), 
xlabel('\itt\rm / s'); ylabel('\it\phi \rm / arcmin(角分)'); grid on
legend('航向角捷联误差','航向角误差卡尔曼估计');

figure;                                              %速度捷联误差&卡尔曼估计
subplot(3,1,1),plot(errvn(:,1),'r'),title('速度捷联误差&卡尔曼估计');
subplot(3,1,1),hold on,plot(Xkk(:,4),'b'), 
xlabel('\itt\rm / s'); ylabel('东向速度\it\delta V \rm / m/s'); grid on
legend('东向速度捷联误差','东向速度误差卡尔曼估计');
subplot(3,1,2),plot(errvn(:,2),'r'),
subplot(3,1,2),hold on,plot(Xkk(:,5),'b'), 
xlabel('\itt\rm / s'); ylabel('北向速度\it\delta V \rm / m/s'); grid on
legend('北向速度捷联误差','北向速度误差卡尔曼估计');
subplot(3,1,3),plot(errvn(:,3),'r'),
subplot(3,1,3),hold on,plot(Xkk(:,6),'b'), 
xlabel('\itt\rm / s'); ylabel('天向速度\it\delta V \rm / m/s'); grid on
legend('天向速度捷联误差','天向速度误差卡尔曼估计');

figure;                                              %位置捷联误差&卡尔曼估计
subplot(3,1,1),plot(errpos(:,1)*glv.Re,'r'),title('位置捷联误差&卡尔曼估计');
subplot(3,1,1),hold on,plot(Xkk(:,7)*glv.Re,'b'), 
xlabel('\itt\rm / s'); ylabel('\it\delta P \rm / m'); grid on
legend('经度捷联误差','经度误差卡尔曼估计');
subplot(3,1,2),plot(errpos(:,2)*glv.Re,'r'),
subplot(3,1,2),hold on,plot(Xkk(:,8)*glv.Re,'b'), 
xlabel('\itt\rm / s'); ylabel('\it\delta P \rm / m'); grid on
legend('纬度捷联误差','纬度误差卡尔曼估计');
subplot(3,1,3),plot(errpos(:,3),'r'),
subplot(3,1,3),hold on,plot(Xkk(:,9),'b'), 
xlabel('\itt\rm / s'); ylabel('\it\delta P \rm / m'); grid on
legend('高度捷联误差','高度误差卡尔曼估计');

figure;                                                 %陀螺漂移估计
subplot(3,1,1),plot(Xkk(:,10)/glv.dph,'r');title('陀螺漂移估计');
ylabel('x轴\it\epsilon \rm / \circ/h');grid on; legend('x轴');
subplot(3,1,2),plot(Xkk(:,11)/glv.dph,'k');
ylabel('y轴\it\epsilon \rm / \circ/h');grid on; legend('y轴');
subplot(3,1,3),plot(Xkk(:,12)/glv.dph,'b');
xlabel('\itt\rm / s');ylabel('z轴\it\epsilon \rm / \circ/h');grid on; legend('z轴');

figure;                                                 %加计漂移估计
subplot(3,1,1),plot(Xkk(:,13)/glv.mg,'r');title('加计漂移估计');
ylabel('x轴\it\nabla \rm / mg');grid on; legend('x轴');
subplot(3,1,2),plot(Xkk(:,14)/glv.mg,'k');
ylabel('y轴\it\nabla \rm / mg');grid on; legend('y轴');
subplot(3,1,3),plot(Xkk(:,15)/glv.mg,'b');
xlabel('\itt\rm / s');ylabel('z轴\it\nabla \rm / mg');grid on; legend('z轴');

figure;
plot(Kkminmatrix,'r');title('Kk变化');grid on
hold on;plot(Kkmaxmatrix,'b');
legend('min','max');

figure;
plot(pkminmatrix,'r');title('Pk变化');grid on
hold on;plot(pkmaxmatrix,'b');
legend('min','max');

close(hwaitbar);
toc;



function [sys,x0,str,ts] = MPC_Cons(t,x,u,flag)
switch flag
  case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
  case 2
    sys=mdlUpdate(t,x,u);
  case 3
    sys=mdlOutputs(t,x,u);
  case {1,4,9}
    sys=[];
  otherwise
    error.(['Unhandled flag=',num2str(flag)]);
end
end
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 4;
sizes.NumOutputs     = 7;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   
sys = simsizes(sizes);
x0  = [0.0001,0.0001,0.0001,0.0001]';
str = [];
ts  = [0.01 0];
global U yk xk_1
U=[0];
yk=zeros(4,1);
xk_1=zeros(4,1);
end
function sys=mdlUpdate(t,x,u)
sys = x;
end
function sys=mdlOutputs(t,x,u)
%% 定义全局变量
global u_piao U kesi yk xk_1 curve_1
if isempty(curve_1)
    curve_1=0;
end
Nx=4;%状态量4个，横向误差，航向角误差，侧向速度误差，横摆角速度误差
Nu=1;%控制量1个前轮转角
Np=6;%预测步长
Nc=4;%控制步长
Row=10;%松弛因子
T=0.01;%采用时间
%%车辆参数
m=1270+71*2;%整车质量
lf=1.015;%质心到前轴的距离
L = 2.910;
lr=L-lf;%质心到后轴的距离
Iz=1536.7;%绕Z轴的转动惯量
%%控制器设计
vx=u(6);
if vx<=0.1
    vx=5;
end
Cf=-46241;%前轴侧偏刚度
Cr=-66666;%后轴侧偏刚度
kesi=zeros(Nx,1);
% kesi=[u(1);u(2);u(3);u(4)]-xk;
X=u(1);
Y=u(2);
phi=u(3);
vy=u(4);
gama=u(5);
Dp=10;
[eymin,ephi,Xt,Yref]=ref(X,Y,phi,Dp);
kesi=[eymin;ephi;vy;gama]-xk_1;
xk_1=[eymin;ephi;vy;gama];
% xk_1=[u(6);u(7);u(8);u(9)];

q=[5000,0,0,0;
   0,5000,0,0;
   0,0,2000,0;
   0,0,0,2000];%10 10 2 2 
r=500000;       %100
Q=kron(eye(Np),q);
R=kron(eye(Nc),r);
A2=[0,vx,-1,-Dp;
   0,0,0,-1;
   0,0,2*(Cf+Cr)/m/vx,2*(lf*Cf-lr*Cr)/m/vx-vx;
   0,0,2*(lf*Cf-lr*Cr)/Iz/vx,2*(lf^2*Cf+lr^2*Cr)/Iz/vx];
B2=[0;0;-2*Cf/m;-2*lf*Cf/Iz];
eps1=-[0 0 -1/vx -lf/vx;
      0 0 -1/vx lr/vx];
eps2=-[1;0];
A=A2*T+eye(Nx);
B=B2*T;
D=[0;vx;0;0]*T;
C=eye(4);
curve=curvature(X);
delta_d=curve-curve_1;
curve_1=curve;
%% 预测时域矩阵升维
Sx_cell=cell(Np,1);
I_cell=cell(Np,1);
Ip_cell=cell(Np,1);
X1_cell=cell(Np,1);
X2_cell=cell(Np,1);
X3_cell=cell(Np,1);
kpa_cell=cell(Np,1);
Sd_cell=cell(Np,1);
for i=1:1:Np
    if i==1
        Sx_cell{i,1}=C*A^i;
        Sd_cell{i,1}=C*A^(i-1)*D;
        I_cell{i,1}=1;
        Ip_cell{i,1}=eye(2);
        X1_cell{i,1}=eps1;
        X2_cell{i,1}=eps2;
        X3_cell{i,1}=eps2;
        kpa_cell{i,1}=eps1*C*A;
    else
        Sx_cell{i,1}=Sx_cell{i-1,1}+C*A^i;
        Sd_cell{i,1}=Sd_cell{i-1,1}+C*A^(i-1)*D;
        I_cell{i,1}=1;
        Ip_cell{i,1}=eye(2);
        X1_cell{i,1}=eps1;
        X2_cell{i,1}=eps2;
        X3_cell{i,1}=eps2;
        kpa_cell{i,1}=eps1*Sx_cell{i,1};
    end
end
Sx=cell2mat(Sx_cell);
Sd=cell2mat(Sd_cell);
I=cell2mat(I_cell);
Ip=cell2mat(Ip_cell);
X1=cell2mat(X1_cell);
X2=cell2mat(X2_cell);
X3=cell2mat(X3_cell);
kpa=cell2mat(kpa_cell);
Su_cell=cell(Np,Nc);
omega_cell=cell(Np,Nc);
for i=1:1:Np
    for j=1:1:Nc
        if  i>=j
            if i == j
                Su_cell{i,j}=C*A^(i-j)*B;
                omega_cell{i,j}=eps1*Su_cell{i,j}+eps2;
                if j~=Nc
                    omega_cell{i,j+1}=eps2;
                end
            else
                Su_cell{i,j}=Su_cell{i-1,j}+C*A^(i-j)*B;
                omega_cell{i,j}=eps1*Su_cell{i,j}+eps2;
            end
        else
            Su_cell{i,j}=zeros(Nx,Nu);
            if j~=Nc
                omega_cell{i,j+1}=zeros(2,Nu);%2是两维的侧偏角
            end
        end
    end
end
Su=cell2mat(Su_cell);
omega=cell2mat(omega_cell);
H=[Su'*Q*Su+R zeros(Nu*Nc,1);
    zeros(1,Nu*Nc) Row];
% H=THETA'*Q*THETA+R;
H=(H+H');
error=Sx*kesi+kron(I,yk)+Sd*delta_d;
% error=Sx*kesi+kron(I,yk);
f=[2*error'*Q*Su 0];
% f=2*error'*Q*THETA;
A_t=zeros(Nc,Nc);
for i=1:1:Nc
    for j=1:1:Nc
        if i>=j
            A_t(i,j)=1;
        else
            A_t(i,j)=0;
        end
    end
end
A_I=kron(A_t,eye(Nu));
Ut=kron(ones(Nc,1),U);
umax=0.54;
umin=-0.54;
umax_dt=0.0082;
umin_dt=-0.0082;
% alpha_max=[0.02;0.02];
% alpha_min=[-0.02;-0.02];
alpha_max=[0.07;0.07];%双移线
alpha_min=[-0.07;-0.07];
Umax=kron(ones(Nc,1),umax);
Umin=kron(ones(Nc,1),umin);
Umax_dt=kron(ones(Nc,1),umax_dt);
Umin_dt=kron(ones(Nc,1),umin_dt);
A_cons_cell=cell(4,2);
B_cons_cell=cell(4,1);
A_cons_cell{1,1}=A_I;
A_cons_cell{1,2}=zeros(Nu*Nc, 1);
A_cons_cell{2,1}=-A_I;
A_cons_cell{2,2}=zeros(Nu*Nc, 1);
%% 不需要侧偏角软约束的话注释这一部分
% A_cons_cell{3,1}=omega;
% A_cons_cell{3,2}=zeros(2*Np, 1);
% A_cons_cell{4,1}=-omega;
% A_cons_cell{4,2}=zeros(2*Np, 1);
% % B_cons_cell{3,1}=Ip*alpha_max-X1*x-X2*U-X3*U(1)-kpa*kesi;
% % B_cons_cell{4,1}=-Ip*alpha_min+X1*x+X2*U+X3*U(1)+kpa*kesi;
% B_cons_cell{3,1}=Ip*alpha_max-X1*x-X2*U-kpa*kesi;
% B_cons_cell{4,1}=-Ip*alpha_min+X1*x+X2*U+kpa*kesi;
%%
B_cons_cell{1,1}=Umax-Ut;
B_cons_cell{2,1}=-Umin+Ut;
A_cons=cell2mat(A_cons_cell);
% B_cons_cell=cell(2,1);
B_cons=cell2mat(B_cons_cell);
M=10;
lb=[Umin_dt;0];
ub=[Umax_dt;M];
% lb=Umin_dt;
% ub=Umax_dt;
%% 二次规划问题
% options=optimset('Algorithm','interior-point-convex');
options = optimset('Algorithm','active-set');
x_start=zeros(Nu*Nc+1,1);
% [X,fval,exitflag] =quadprog(H,f,A_cons,B_cons,[],[],lb,ub,[],options);
[X,fval,exitflag] =quadprog(H,f,A_cons,B_cons,[],[],lb,ub,x_start,options);
%% 赋值输出
u_piao=X(1);
U=U+u_piao;
u_real=U;
Alpha=eps1*yk+eps2*U;
yk=yk+kesi;
alpha_f=(vy+gama*lf)/vx-U;
alpha_r=-(gama*lr-vy)/vx;
sys = [u_real;Xt;Yref;alpha_f;alpha_r;eymin;ephi];

    function [eymin,ephi,Xt,Yref]=ref(X,Y,phi,Dp)
        Xt=X+Dp*cos(phi);
        Yt=Y+Dp*sin(phi);
        %% 单移线工况预瞄横向偏差参数 %%
%         d=7.6;
%         l=120;
%         Yref=d/(2*pi)*(pi+2*pi*(Xt-l/2)/l+sin(2*pi/l*(Xt-l/2)));
%         phiref=(2723726541130763*pi)/135107988821114880 + (2723726541130763*pi*cos((pi*(Xt - 60))/60))/135107988821114880;
%         if Xt>120
%             Yref = 7.6;
%             phiref = 0;
%         end
        %% 双移线 %%
%         r11=0.096*(Xt-60)-1.2;
%         r22=0.096*(Xt-90)-1.2;
%         dm1=25;
%         dm2=25;
%         dn1=3;
%         dn2=3;
%         Yref=dn1/2*(1+tanh(r11))-dn2/2*(1+tanh(r22));
% %         phiref= atan(dn1*(1/cosh(r11))^2*(1.2/dm1)-dn2*(1/cosh(r22))^2*(1.2/dm2));
%         phiref=(18*tanh((12*Xt)/125 - 318/25)^2)/125 - (18*tanh((12*Xt)/125 - 174/25)^2)/125;

    shape=2.4;%参数名称，用于参考轨迹生成
    dx1=25;dx2=21.95;%没有任何实际意义，只是参数名称
    dy1=4.05;dy2=5.7;%没有任何实际意义，只是参数名称
    Xs1=27.19;Xs2=56.46;%参数名称
    z1=shape/dx1*(Xt-Xs1)-shape/2;
    z2=shape/dx2*(Xt-Xs2)-shape/2;
    Yref=dy1/2*(1+tanh(z1))-dy2/2*(1+tanh(z2));
    phiref=atan(dy1*(1/cosh(z1))^2*(1.2/dx1)-dy2*(1/cosh(z2))^2*(1.2/dx2));
    
%     eymin=-[-sin(phiref) cos(phiref)]*[0;Yt-Yref];
    eymin=Yref-Yt;
    ephi=phiref-phi;
    end
        function K=curvature(X)
        %将以下注释输入到命令行窗口，在工作区获得曲率方程
%         syms t;
%         d=7.6;
%         l=120;
%         fx=d/(2*pi)*(pi+2*pi*(t-l/2)/l+sin(2*pi/l*(t-l/2)));%单移线
%         fK=diff(fx,t,2)/(1+diff(fx,t,1)^2)^(3/2);
        %采用论文中的K
%         if  X<=20 || X>=105
%             K=0;
%         else if  X>20 && X<=60
%                 K=5*10^(-3);
%             else
%                 K=-5*10^(-3);
%             end
%         end
%% 单移线K
%         K=-(2723726541130763*pi^2*sin((pi*(X - 60))/60))/(8106479329266892800*(((2723726541130763*pi)/135107988821114880 + (2723726541130763*pi*cos((pi*(X - 60))/60))/135107988821114880)^2 + 1)^(3/2));
%% 双移线K
        K=((243*tanh((12*X)/125 - 11907/3125)*((12*tanh((12*X)/125 - 11907/3125)^2)/125 - 12/125))/625 - (1368*tanh((48*X)/439 - 80922/10975)*((48*tanh((48*X)/439 - 80922/10975)^2)/439 - 48/439))/2195)/(((243*tanh((12*X)/125 - 11907/3125)^2)/1250 - (684*tanh((48*X)/439 - 80922/10975)^2)/2195 + 64323/548750)^2 + 1)^(3/2);
        end
    end

content_list=whos;%获取workspace内变量名，结构体形式
for content_count=1:length(content_list)
    name_temp=content_list(content_count).name;
    name_new=[name_temp,'_n'];
    assignin('base',name_new,eval(name_temp));%为新变量赋值
end


%% 侧向速度+横摆角速度+XY+前轮转角+前后轮侧偏角 默认出图请运行该节
T1=0.001;
T2=0.01;
time1=0:T1:(length(vy)-1)*T1;
time2=0:T2:(length(alphaf)-1)*T2;
figure(1)%侧向速度图
plot(time1,vy,'color',[1,0,0],'LineStyle','-','linewidth',2);
hold on
plot(time1,vy_n,'color',[0,0,1],'LineStyle','--','linewidth',2);
set(gca,'FontName','Times New Roman','FontSize',14);
legend2=legend('\fontname{宋体}软约束 \fontname{Times New Roman}MPC','\fontname{Times New Roman}MPC');
set(legend2,'FontName','Times New Roman','FontSize',14,'FontWeight','normal'); 
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)','FontSize',16);
ylabel('\fontname{宋体}侧向速度 \fontname{Times New Roman}(m/s)','FontSize',16);
title('')
grid on;
figure(2)%横摆角速度图
plot(time1,gama,'color',[1,0,0],'LineStyle','-','linewidth',2);
hold on
plot(time1,gama_n,'color',[0,0,1],'LineStyle','--','linewidth',2);
set(gca,'FontName','Times New Roman','FontSize',14);
legend2=legend('\fontname{宋体}软约束 \fontname{Times New Roman}MPC','\fontname{Times New Roman}MPC');
set(legend2,'FontName','Times New Roman','FontSize',14,'FontWeight','normal'); 
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)','FontSize',16);
ylabel('\fontname{宋体}横摆角速度 \fontname{Times New Roman}(rad/s)','FontSize',16);
title('')
grid on;
figure(3)%XY图
plot(Xref,Yref,'color',[0,0,0],'LineStyle','--','linewidth',2);
hold on
plot(X,Y,'color',[1,0,0],'LineStyle','-','linewidth',2);
plot(X_n,Y_n,'color',[0,0,1],'LineStyle','-','linewidth',2);
set(gca,'FontName','Times New Roman','FontSize',14);
legend2=legend('\fontname{宋体}参考值','\fontname{宋体}软约束 \fontname{Times New Roman}MPC','\fontname{Times New Roman}MPC');
set(legend2,'FontName','Times New Roman','FontSize',14,'FontWeight','normal'); 
xlabel('\fontname{宋体}纵向位移 \fontname{Times New Roman}(m)','FontSize',16);
ylabel('\fontname{宋体}横向位移 \fontname{Times New Roman}(m)','FontSize',16);
title('')
grid on;
figure(4)%前轮转角
plot(time2,delta,'color',[1,0,0],'LineStyle','-','linewidth',2);
hold on
plot(time2,delta_n,'color',[0,0,1],'LineStyle','--','linewidth',2);
set(gca,'FontName','Times New Roman','FontSize',14);
legend2=legend('\fontname{宋体}软约束 \fontname{Times New Roman}MPC','\fontname{Times New Roman}MPC');
set(legend2,'FontName','Times New Roman','FontSize',14,'FontWeight','normal'); 
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)','FontSize',16);
ylabel('\fontname{宋体}前轮转角 \fontname{Times New Roman}(rad)','FontSize',16);
title('')
grid on;
figure(5)%前后轮侧偏角
plot(time2,alphaf,'color',[1,0,0],'LineStyle','-','linewidth',2);
hold on
plot(time2,alphar,'color',[1,0,1],'LineStyle','-','linewidth',2);
plot(time2,alphaf_n,'color',[1,0.7,0],'LineStyle','--','linewidth',2);
plot(time2,alphar_n,'color',[0,1,1],'LineStyle','--','linewidth',2);
set(gca,'FontName','Times New Roman','FontSize',14);
legend2=legend('\fontname{宋体}软约束前轮侧偏角','\fontname{宋体}软约束后轮侧偏角','\fontname{宋体}无约束前轮侧偏角','\fontname{宋体}无约束后轮侧偏角');
set(legend2,'FontName','Times New Roman','FontSize',14,'FontWeight','normal'); 
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)','FontSize',16);
ylabel('\fontname{宋体}前后轮侧偏角 \fontname{Times New Roman}(rad)','FontSize',16);
title('')
grid on;



%% 实时运行后出图请运行以下节 %%
T1=0.001;
T2=0.01;
time1=0:T1:(length(vy)-1)*T1;
time2=0:T2:(length(alphaf)-1)*T2;
figure(1)%侧向速度图
plot(time1,vy,'color',[1,0,0],'LineStyle','-','linewidth',2);
set(gca,'FontName','Times New Roman','FontSize',14);
legend2=legend('\fontname{宋体}软约束 \fontname{Times New Roman}MPC');
set(legend2,'FontName','Times New Roman','FontSize',14,'FontWeight','normal'); 
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)','FontSize',16);
ylabel('\fontname{宋体}侧向速度 \fontname{Times New Roman}(m/s)','FontSize',16);
title('')
grid on;
figure(2)%横摆角速度图
plot(time1,gama,'color',[1,0,0],'LineStyle','-','linewidth',2);
set(gca,'FontName','Times New Roman','FontSize',14);
legend2=legend('\fontname{宋体}软约束 \fontname{Times New Roman}MPC');
set(legend2,'FontName','Times New Roman','FontSize',14,'FontWeight','normal'); 
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)','FontSize',16);
ylabel('\fontname{宋体}横摆角速度 \fontname{Times New Roman}(rad/s)','FontSize',16);
title('')
grid on;
figure(3)%XY图
plot(Xref,Yref,'color',[0,0,0],'LineStyle','--','linewidth',2);
hold on
plot(X,Y,'color',[1,0,0],'LineStyle','-','linewidth',2);
set(gca,'FontName','Times New Roman','FontSize',14);
legend2=legend('\fontname{宋体}参考值','\fontname{宋体}软约束 \fontname{Times New Roman}MPC');
set(legend2,'FontName','Times New Roman','FontSize',14,'FontWeight','normal'); 
xlabel('\fontname{宋体}纵向位移 \fontname{Times New Roman}(m)','FontSize',16);
ylabel('\fontname{宋体}横向位移 \fontname{Times New Roman}(m)','FontSize',16);
title('')
grid on;
figure(4)%前轮转角
plot(time2,delta,'color',[1,0,0],'LineStyle','-','linewidth',2);
set(gca,'FontName','Times New Roman','FontSize',14);
legend2=legend('\fontname{宋体}软约束 \fontname{Times New Roman}MPC');
set(legend2,'FontName','Times New Roman','FontSize',14,'FontWeight','normal'); 
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)','FontSize',16);
ylabel('\fontname{宋体}前轮转角 \fontname{Times New Roman}(rad)','FontSize',16);
title('')
grid on;
figure(5)%前后轮侧偏角
plot(time2,alphaf,'color',[1,0,0],'LineStyle','-','linewidth',2);
hold on
plot(time2,alphar,'color',[1,0,1],'LineStyle','-','linewidth',2);
set(gca,'FontName','Times New Roman','FontSize',14);
legend2=legend('\fontname{宋体}前轮侧偏角','\fontname{宋体}后轮侧偏角');
set(legend2,'FontName','Times New Roman','FontSize',14,'FontWeight','normal'); 
xlabel('\fontname{宋体}时间 \fontname{Times New Roman}(s)','FontSize',16);
ylabel('\fontname{宋体}前后轮侧偏角 \fontname{Times New Roman}(rad)','FontSize',16);
title('')
grid on;
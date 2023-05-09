Q_plot = fbanim(Xout);
Q_plot = Q_plot';
px = Q_plot(:,1);
py = Q_plot(:,2);
pz = Q_plot(:,3);

theta_x = Q_plot(:,4);
theta_y = Q_plot(:,5);
theta_z = Q_plot(:,6);

FR_torque_HAA = Torqueout(:, 1);
FR_torque_HFE = Torqueout(:, 2);
FR_torque_KFE = Torqueout(:, 3);

FW_torque_X = Torqueout(:, 13);
FW_torque_Y = Torqueout(:, 14);
FW_torque_Z = Torqueout(:, 15);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(5) % foot velocity Plot
% set(gcf,'position',[600,400,400,300]) 
% plot(time_stamp, LF_foot_velox, 'g-.', 'LineWidth', 2);
% hold on;
% plot(time_stamp, LF_foot_veloy, 'r-', 'LineWidth', 2);
% hold on;
% fvz = plot(time_stamp, LF_foot_veloz, 'b:', 'LineWidth', 2);
% fvz.Color(4)=0.3;
% grid on;
% box on;
% legend({'$p_x$', '$p_y$', '$p_z$'}, 'fontsize', 14,'location', 'best', 'Interpreter','latex'); %图例
% xlabel('\fontname{宋体}\fontsize{14}时间\fontname{times new roman}\fontsize{14}/s','Fontname', 'Times New Roman');
% ylabel('\fontname{宋体}\fontsize{14}足端速度\fontname{times new roman}\fontsize{14}/ms^-^1','Fontname', 'Times New Roman');
% set(gca,'xlim',[0,40])%,'ylim',[-0.2,0.05]);
% % set(gca,'xtick',[0:5:45]);
% set(gca,'FontName','Times New Roman','FontSize',14);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(6) % position Plot
set(gcf,'position',[600,400,400,300]) 
plot(tout, px, 'r-.', 'LineWidth', 1.5);
hold on;
plot(tout, py, 'm:', 'LineWidth', 1.5);
hold on;
plot(tout, pz, 'b-', 'LineWidth', 1.5);

grid on;
box on;
legend({'$p_x$', '$p_y$', '$p_z$'}, 'fontsize', 14,'location', 'best', 'Interpreter','latex'); %图例
xlabel('\fontname{宋体}\fontsize{14}时间\fontname{times new roman}\fontsize{14}/s','Fontname', 'Times New Roman');
ylabel('\fontname{宋体}\fontsize{14}质心位置\fontname{times new roman}\fontsize{14}/m','Fontname', 'Times New Roman');
set(gca,'xlim',[0,20])
%,'ylim',[-0.2,0.05]);
set(gca,'FontName','Times New Roman','FontSize',14);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(7) % velocity Plot
% set(gcf,'position',[600,400,400,300]) 
% plot(time_stamp, vx*1.1, 'r-.', 'LineWidth', 1.5);
% hold on;
% plot(time_stamp, vy/6, 'm-', 'LineWidth', 1.5);
% hold on;
% vzz = plot(time_stamp, vz, 'b:', 'LineWidth', 1.5);
% vzz.Color(4) = 0.5;
% grid on;
% box on;
% legend({'$v_x$', '$v_y$', '$v_z$'}, 'fontsize', 14,'location', 'best', 'Interpreter','latex'); %图例
% xlabel('\fontname{宋体}\fontsize{14}时间\fontname{times new roman}\fontsize{14}/s','Fontname', 'Times New Roman');
% ylabel('\fontname{宋体}\fontsize{14}质心速度\fontname{times new roman}\fontsize{14}/m{\cdot}s^-^1','Fontname', 'Times New Roman');
% set(gca,'xlim',[0,40])%,'ylim',[-0.2,0.05]);
% % set(gca,'xtick',[0:5:45]);
% set(gca,'FontName','Times New Roman','FontSize',14);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1) % Attitude Plot
set(gcf,'position',[600,400,400,300]) 
plot(tout, (theta_x)/3.14159*180, 'r-.', 'LineWidth', 1.5); %/3.14159*180
hold on;
plot(tout, (theta_y)/3.14159*180, 'm:', 'LineWidth', 1.5);
hold on;
plot(tout, (theta_z)/3.14159*180, 'b-', 'LineWidth', 1.5);
% plot(time_stamp, theta_x/10+0.03, 'r:', 'LineWidth', 1.5);
% hold on;
% plot(time_stamp, theta_y+0.02, 'm-', 'LineWidth', 1.5);
% hold on;
% plot(time_stamp, theta_z/3, 'b-.', 'LineWidth', 1.5);

grid on;
box on;
legend({'$\varphi$', '$\theta_y$', '$\psi$'}, 'fontsize', 14,'location', 'best', 'Interpreter','latex'); %图例
xlabel('\fontname{宋体}\fontsize{14}时间\fontname{times new roman}\fontsize{14}/s','Fontname', 'Times New Roman');
ylabel('\fontname{宋体}\fontsize{14}姿态角\fontname{times new roman}\fontsize{14}/°','Fontname', 'Times New Roman');
set(gca,'xlim',[0,20])%,'ylim',[-0.2,0.05]);
% ylim([-1,7]);
set(gca,'FontName','Times New Roman','FontSize',14);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(2) % Contact Force Plot
% % plot(time_stamp, LF_contact_forcex, 'r-', 'LineWidth', 2);
% % hold on;
% % plot(time_stamp, LF_contact_forcey, 'b:', 'LineWidth', 2);
% % hold on;
% plot(time_stamp, LF_contact_forcez, 'r-', 'LineWidth', 1.5);
% hold on;
% % plot(time_stamp, Mode, 'b-.', 'LineWidth', 1.5);
% 
% grid on;
% box on;
% set(gcf,'position',[600,400,400,300]) 
% legend({'\fontname{宋体}\fontsize{14}法向接触力'}); %图例 'contact-force-x', 'contact-force-y', 
% xlabel('\fontname{宋体}\fontsize{14}时间\fontname{times new roman}\fontsize{14}/s','Fontname', 'Times New Roman');
% ylabel('\fontname{宋体}\fontsize{14}接触力\fontname{times new roman}\fontsize{14}/N','Fontname', 'Times New Roman');
% xlim([0,30]);
% % ylim([0,40]);
% set(gca,'FontName','Times New Roman','FontSize',14);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(3) % Joint Torque Plot
plot(tout, FR_torque_HAA, 'm--', 'LineWidth', 1.5);
hold on;
plot(tout, FR_torque_HFE, 'b:', 'LineWidth', 1.5);
hold on;
plot(tout, FR_torque_KFE, 'r-', 'LineWidth', 1.5);
hold on;

grid on;
box on;
set(gcf,'position',[600,400,400,300]) 
legend({'\fontname{宋体}\fontsize{14}侧摆关节', '\fontname{宋体}\fontsize{14}大腿关节', '\fontname{宋体}\fontsize{14}膝关节',}); %legend 
xlabel('\fontname{宋体}\fontsize{14}时间\fontname{times new roman}\fontsize{14}/s','Fontname', 'Times New Roman');
ylabel('\fontname{宋体}\fontsize{14}关节力矩\fontname{times new roman}\fontsize{14}/Nm','Fontname', 'Times New Roman');
xlim([0,20]);
ylim([-30,10]);
set(gca,'FontName','Times New Roman','FontSize',14);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(4) % Fly_wheel Torque Plot
plot(tout, FW_torque_X, 'm--', 'LineWidth', 1.5);
hold on;
plot(tout, FW_torque_Y, 'b-', 'LineWidth', 1.5);
hold on;
plot(tout, FW_torque_Z, 'r:', 'LineWidth', 1.5);
hold on;

grid on;
box on;
set(gcf,'position',[600,400,400,300]) 
legend({'\fontname{times new roman}\fontsize{14}x', '\fontname{times new roman}\fontsize{14}y', '\fontname{times new roman}\fontsize{14}z',}); %legend
xlabel('\fontname{宋体}\fontsize{14}时间\fontname{times new roman}\fontsize{14}/s','Fontname', 'Times New Roman');
ylabel('\fontname{宋体}\fontsize{14}飞轮力矩\fontname{times new roman}\fontsize{14}/Nm','Fontname', 'Times New Roman');
xlim([0,20]);
% ylim([-1.5,1.5]);
set(gca,'FontName','Times New Roman','FontSize',14);

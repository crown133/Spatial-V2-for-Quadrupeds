figure 
   set(gcf,'position',[100,100,400,300]) 
   plot(tout,Qout1,'-r','lineWidth',1.5)
   hold on
   plot(tout,Qout2,'-.b','lineWidth',1)
   ylabel('\fontname{helvetica}\fontsize{11}Pitch(\theta)\fontsize{12}/(rad)')
   xlabel('\fontname{helvetica}\fontsize{11}t\fontname{helvetica}\fontsize{11}/s')
%    d_gim_leg = legend('$ \dot{\delta}_{1,a}$','$ \dot{\delta}_{1,b}$');
   d_gim_leg = legend('$ ESO$','$ \theta$');
   set( d_gim_leg,'Position',[0.75,0.785,0.12,0.14],'Interpreter','latex')
%    xlim([0,tfinal])
   grid on
   hold on
   set(gca,'FontSize',12,'FontName','times new roman')
   
   axes('Position',[0.3,0.35,0.27,0.27]);
   plot(tout,Qout1,'-r','lineWidth',1.5)
   hold on
   plot(tout,Qout2,'-.b','lineWidth',1)
   xlim([5,8])
   ylim([0.55,0.7]);
   set(gca,'FontSize',9,'FontName','times new roman')
   set(gca,'YTick',0.55:0.05:0.7)
   grid on
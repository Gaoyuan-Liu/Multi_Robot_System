clear all
close all
clc

%% load mat file
filename = 'test_B21_alljoints_sine_5ms';
load(filename);

%% limit constants
q_min = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973];
q_max = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973];
dotq_min = [-2.1750, -2.1750, -2.1750, -2.1750,	-2.6100, -2.6100, -2.6100];
dotq_max = [2.1750,	2.1750, 2.1750,	2.1750,	2.6100,	2.6100,	2.6100];
tau_min = [-87.0, -87.0, -87.0, -87.0, -12.0, -12.0, -12.0]; 
tau_max = [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]; 

%% colors
red = [1 0.2 0.2];
orange = [1 0.5 0];
olive = [0.5 0.5 0];
green = [0 0.6 0.3];
blue = [0 0.5 1];
teal = [0 0.5 0.5];
purple = [0.5 0 0.5];
black = [0 0 0];

%% Graph joint configuration (q_min, q_max, q_r,q_v, q)
q_min=ones(length(time_jointstates),1)*q_min;
q_max=ones(length(time_jointstates),1)*q_max;
% 
% % q_start = [0.0000, -0.7854, 0.0000, -2.3562, 0.0000, 1.5708, 0.7854];
% % A=0.5; 
% % T=20;
% % omega=2*pi/T;
% % t0 = 5;
% % t=5:0.001:25;
% % for i = 1:7
% %     q_r(:,i)= q_start(i) + A*sin(omega*(t-t0));
% % end
% figurename = strcat('Panda robot: joint angles (', filename, ')');
% figure('name',figurename);
% 
% tabgroup = uitabgroup; % tabgroup
% for i = 1:7
%     tabtitle=char(strcat('Joint',{' '},num2str(i)));
%     thistab = uitab(tabgroup,'Title',tabtitle); % build iith tab
%     axes('Parent',thistab); % somewhere to plot
%     hold on
%     plot(time_jointstates,q_min(:,i),'-','Color',black,'linewidth',2);
%     plot(time_jointstates,q_max(:,i),'-','Color',black,'linewidth',2);
% %     plot(t,q_r(:,i),'-.','Color',red,'linewidth',1);
%     plot(time_user_ref,q_r(:,i),':','Color',teal, 'linewidth',2);
%     plot(time_applied_ref,q_v(:,i),'-.','Color',teal,'linewidth',2);
%     plot(time_jointstates,q(:,i),'-','Color',teal,'linewidth',1);
%     %legend('q_{min}','q_{max}','q_{r}','q_{v}','q')
%     xlabel('Time [s]','fontsize',10)
%     ylabel('Joint Angles [rad]','fontsize',10)
% end

%% Graph joint velocities
figurename = strcat('Panda robot: joint velocities (', filename, ')');
figure('name',figurename);
dotq_min=ones(length(time_jointstates),1)*dotq_min;
dotq_max=ones(length(time_jointstates),1)*dotq_max;

filtered_dotq = lowpass(dotq,0.001);
tabgroup = uitabgroup; % tabgroup
for i = 1:7
    tabtitle=char(strcat('Joint',{' '},num2str(i)));
    thistab = uitab(tabgroup,'Title',tabtitle); % build iith tab
    axes('Parent',thistab); % somewhere to plot
    hold on
    plot(time_jointstates,dotq_min(:,i),'-','Color',black,'linewidth',2);
    plot(time_jointstates,dotq_max(:,i),'-','Color',black,'linewidth',2);
    plot(time_jointstates,dotq(:,i),'-','Color',teal,'linewidth',1);
%     plot(time_jointstates,filtered_dotq(:,i),'-','Color',purple,'linewidth',1);
    xlabel('Time [s]','fontsize',10)
    ylabel('Joint velocities [rad/s]','fontsize',10)
end

%% Graph torques (tau_min, tau_max, tau, tau_cmd)
% figurename = strcat('Panda robot: DSM (', filename, ')');
% figure('name',figurename);
% hold on 
% 
tau_min = ones(length(time_jointstates),1)*tau_min;
tau_max = ones(length(time_jointstates),1)*tau_max;
% 
% figurename = strcat('Panda robot: tau (', filename, ')');
% figure('name',figurename);
% hold on
% 
% tabgroup = uitabgroup; % tabgroup
% for i = 1:7
%     tabtitle=char(strcat('Joint',{' '},num2str(i)));
%     thistab = uitab(tabgroup,'Title',tabtitle); % build iith tab
%     axes('Parent',thistab); % somewhere to plot
%     hold on
%     plot(time_jointstates,tau_min(:,i),'-','Color',black,'linewidth',2);
%     plot(time_jointstates,tau_max(:,i),'-','Color',black,'linewidth',2);
%     plot(time_tau_cmd(5:end,i),tau_cmd(5:end,i),'-.','Color',red, 'linewidth',1);
%     plot(time_jointstates,tau(:,i),'-','Color',teal,'linewidth',1);
%     %plot(time_gravity_effort,gravity_effort(:,i),'-','Color',orange,'linewidth',1);
%     %plot(time_coriolis_effort,coriolis_effort(:,i),'-','Color',purple,'linewidth',1);
%     xlabel('Time [s]','fontsize',10)
%     ylabel('Torques [Nm]','fontsize',10)
% end

%% Graph chrono single loop
figurename = strcat('Panda robot: time for single loop (', filename, ')');
figure('name',figurename);
hold on

scatter(time_chrono(2:end),time_singleloop(2:end),10,'filled');
ylim([0 max(time_singleloop(2:end))])
xlabel('Time [s]','fontsize',10)
ylabel('time for single loop [s]','fontsize',10)



%% Graph DSM (DSM_tau, DSM_q, DSM)
figurename = strcat('Panda robot: DSM (', filename, ')');
figure('name',figurename);
hold on

plot(time_DSM_tau,DSM_tau,'-','Color',blue, 'linewidth',2);
plot(time_DSM_q,DSM_q,'-','Color',green,'linewidth',2);
% plot(time_DSM_obst_sphere,DSM_obst_sphere,'-','Color',orange,'linewidth',2);
% plot(time_DSM_obst_cylinder,DSM_obst_cylinder,'-','Color',purple,'linewidth',2);
plot(time_DSM_overall,DSM,'-','Color',black,'linewidth',2);

legend('DSM_{\tau}','DSM_{q}','DSM')
% legend('DSM_{\tau}','DSM_{q}','DSM_{obst\_sphere}','DSM')
% legend('DSM_{\tau}','DSM_{q}','DSM_{obst\_cylinder}','DSM')
% legend('DSM_{\tau}','DSM_{q}','DSM_{obst\_sphere}','DSM_{obst\_cylinder}','DSM')
xlabel('Time [s]','fontsize',10)
ylabel('Dynamic Safety Margin','fontsize',10)

%% Graph DSM: prediction q 
figurename = strcat('Panda robot: DSM prediction q (', filename, ')');
figure('name',figurename);
hold on

% q_start = [1.0000-1, -0.7854+1, 0.0000, -2.3562, 0.0000, 1.5708, 0.7854];
% time = 0 : 0.01 : 30;
% q_r = zeros(size(time,2),size(q_start,2));
% for t= 1:size(time,2)
%     for i = 1:7
%         if (i==2)
%             if (time(t)<=5)
%                 q_r(t,i)= q_start(i)  ;
%             elseif (time(t)>5 && time(t) <=10)
%                 q_r(t,i)= q_start(i) +0.2;
%             elseif (time(t)>10 && time(t) <=15)
%                 q_r(t,i)= q_start(i) ;
%             elseif (time(t)>15 && time(t) <=20)
%                 q_r(t,i)= q_start(i) - 0.2;
%             elseif (time(t)>20 && time(t) <=25)
%                 q_r(t,i)= q_start(i) ;
%             else
%                 q_r(t,i)= q_start(i) +0.2;
%             end
%         else
%             q_r(t,i)= q_start(i);
%         end
%             
%     end
% end
% % A=0.5; 
% % T=20;
% % omega=2*pi/T;
% % t0 = 5;



tabgroup = uitabgroup; % tabgroup
for i = 1:7
    tabtitle=char(strcat('Joint',{' '},num2str(i)));
    thistab = uitab(tabgroup,'Title',tabtitle); % build iith tab
    axes('Parent',thistab); % somewhere to plot
    hold on
    plot(time_jointstates,q_min(:,i),'-','Color',black,'linewidth',2);
    plot(time_jointstates,q_max(:,i),'-','Color',black,'linewidth',2);
    plot(time_user_ref,q_r(:,i),':','Color',teal, 'linewidth',2);
%     plot(time,q_r(:,i),':','Color',teal, 'linewidth',2);
%     plot(time_applied_ref,q_v(:,i),'-.','Color',teal,'linewidth',2);
    plot(time_jointstates,q(:,i),'-','Color',teal,'linewidth',2);
    for k = 1 : size(q_pred,3)
        if (mod(k,2)==0)
            plot(q_pred(1,:,k),q_pred(i+1,:,k),'-','Color',orange,'linewidth',1);
        else
            plot(q_pred(1,:,k),q_pred(i+1,:,k),'-','Color',purple,'linewidth',1);
        end
    end
    grid minor
    ylim([q_min(1,i) q_max(1,i)])
    legend('q_{min}','q_{max}','q_{r}','q','q_{pred}')
    xlabel('Time [s]','fontsize',10)
    ylabel('Joint Angles [rad]','fontsize',10)
end

%% Graph DSM: prediction tau
figurename = strcat('Panda robot: DSM prediction tau (', filename, ')');
figure('name',figurename);
hold on

tabgroup = uitabgroup; % tabgroup
for i = 1:7
    tabtitle=char(strcat('Joint',{' '},num2str(i)));
    thistab = uitab(tabgroup,'Title',tabtitle); % build iith tab
    axes('Parent',thistab); % somewhere to plot
    hold on
    plot(time_jointstates,tau_min(:,i),'-','Color',black,'linewidth',2);
    plot(time_jointstates,tau_max(:,i),'-','Color',black,'linewidth',2);
    plot(time_tau_cmd(5:end,i),tau_cmd(5:end,i),'-.','Color',teal, 'linewidth',2);
    plot(time_jointstates,tau(:,i),'-','Color',teal,'linewidth',2);
        for k = 1 : size(tau_pred,3)
        if (mod(k,2)==0)
            plot(tau_pred(1,:,k),tau_pred(i+1,:,k),'-','Color',orange,'linewidth',1);
        else
            plot(tau_pred(1,:,k),tau_pred(i+1,:,k),'-','Color',purple,'linewidth',1);
        end
    
        end
    ylim([tau_min(1,i) tau_max(1,i)])
    legend('\tau_{min}','\tau_{max}','\tau_{cmd}','\tau','\tau_{pred}')
    xlabel('Time [s]','fontsize',10)
    ylabel('Torques [Nm]','fontsize',10)
end

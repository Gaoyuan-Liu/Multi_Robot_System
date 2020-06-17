clear all
%close all
clc

%% load bag file
filename = 'test_B21_alljoints_sine_5ms';
bagname = strcat(filename,'.bag');
bag = rosbag(bagname);
bagInfo = rosbag('info',bagname);

%% joint states bag: timeseries + readMessages
bag_jointstates = select(bag,'Topic','/panda/joint_states');

time_jointstates = timeseries(bag_jointstates).Time;
msgs_jointstates = readMessages(bag_jointstates);

%% joint states: q, dotq, tau
q = zeros(length(time_jointstates),7);
for t = 1 : length(time_jointstates)
    for i=1:7
        q(t,i)=msgs_jointstates{t}.Position(i+2);
    end
end

dotq = zeros(length(time_jointstates),7);
for t = 1 : length(time_jointstates)
    for i=1:7
        dotq(t,i)=msgs_jointstates{t}.Velocity(i+2);
    end
end

tau = zeros(length(time_jointstates),7);
for t = 1 : length(time_jointstates)
    for i=1:7
        tau(t,i)=msgs_jointstates{t}.Effort(i+2);
    end
end

%% torque commands bag: timeseries + readMessages
bag_tau_cmd1 = select(bag,'Topic','/panda/joint1_effort_controller/command');
bag_tau_cmd2 = select(bag,'Topic','/panda/joint2_effort_controller/command');
bag_tau_cmd3 = select(bag,'Topic','/panda/joint3_effort_controller/command');
bag_tau_cmd4 = select(bag,'Topic','/panda/joint4_effort_controller/command');
bag_tau_cmd5 = select(bag,'Topic','/panda/joint5_effort_controller/command');
bag_tau_cmd6 = select(bag,'Topic','/panda/joint6_effort_controller/command');
bag_tau_cmd7 = select(bag,'Topic','/panda/joint7_effort_controller/command');

time_tau_cmd1 = timeseries(bag_tau_cmd1).Time;
time_tau_cmd2 = timeseries(bag_tau_cmd2).Time;
time_tau_cmd3 = timeseries(bag_tau_cmd3).Time;
time_tau_cmd4 = timeseries(bag_tau_cmd4).Time;
time_tau_cmd5 = timeseries(bag_tau_cmd5).Time;
time_tau_cmd6 = timeseries(bag_tau_cmd6).Time;
time_tau_cmd7 = timeseries(bag_tau_cmd7).Time;

length_time_tau_cmd = [length(time_tau_cmd1),length(time_tau_cmd2),length(time_tau_cmd3),...
    length(time_tau_cmd4),length(time_tau_cmd5),length(time_tau_cmd6),length(time_tau_cmd7)];
time_tau_cmd=zeros(min(length_time_tau_cmd),7);
for t=1:min(length_time_tau_cmd)
    time_tau_cmd(t,1) = time_tau_cmd1(t,1);
    time_tau_cmd(t,2) = time_tau_cmd2(t,1);
    time_tau_cmd(t,3) = time_tau_cmd3(t,1);
    time_tau_cmd(t,4) = time_tau_cmd4(t,1);
    time_tau_cmd(t,5) = time_tau_cmd5(t,1);
    time_tau_cmd(t,6) = time_tau_cmd6(t,1);
    time_tau_cmd(t,7) = time_tau_cmd7(t,1);
end

msgs_tau_cmd1 = readMessages(bag_tau_cmd1);
msgs_tau_cmd2 = readMessages(bag_tau_cmd2);
msgs_tau_cmd3 = readMessages(bag_tau_cmd3);
msgs_tau_cmd4 = readMessages(bag_tau_cmd4);
msgs_tau_cmd5 = readMessages(bag_tau_cmd5);
msgs_tau_cmd6 = readMessages(bag_tau_cmd6);
msgs_tau_cmd7 = readMessages(bag_tau_cmd7);

%% torque commands: tau_cmd
tau_cmd = zeros(length(time_tau_cmd),7);
for t = 1 : length(time_tau_cmd)
    tau_cmd(t,1)=msgs_tau_cmd1{t}.Data;
    tau_cmd(t,2)=msgs_tau_cmd2{t}.Data;
    tau_cmd(t,3)=msgs_tau_cmd3{t}.Data;
    tau_cmd(t,4)=msgs_tau_cmd4{t}.Data;
    tau_cmd(t,5)=msgs_tau_cmd5{t}.Data;
    tau_cmd(t,6)=msgs_tau_cmd6{t}.Data;
    tau_cmd(t,7)=msgs_tau_cmd7{t}.Data;
end


%% chrono 1 loop bag: timeseries + readMessages
bag_chrono = select(bag,'Topic','/chrono_1loop');

time_chrono = timeseries(bag_chrono).Time;
msgs_chrono = readMessages(bag_chrono);

%% chrono 1 loop: time_singleloop
time_singleloop = zeros(length(time_chrono),1);
for t = 1 : length(time_chrono)
    time_singleloop(t)=msgs_chrono{t}.Data;
end


%% gravity effort and coriolis effort: timeseries + readMessages 
% bag_gravity_effort = select(bag,'Topic','/StabControl/gravity_effort');
% bag_coriolis_effort = select(bag,'Topic','/StabControl/coriolis_effort');
% 
% time_gravity_effort = timeseries(bag_gravity_effort).Time;
% msgs_gravity_effort = readMessages(bag_gravity_effort);
% 
% time_coriolis_effort = timeseries(bag_coriolis_effort).Time;
% msgs_coriolis_effort = readMessages(bag_coriolis_effort);

%% gravity effort and coriolis effort
% gravity_effort = zeros(length(time_gravity_effort),7);
% for t = 1 : length(time_gravity_effort)
%     for i=1:7
%         gravity_effort(t,i)=msgs_gravity_effort{t}.Data(i);
%     end
% end
% 
% coriolis_effort = zeros(length(time_coriolis_effort),7);
% for t = 1 : length(time_coriolis_effort)
%     for i=1:7
%         coriolis_effort(t,i)=msgs_coriolis_effort{t}.Data(i);
%     end
% end

%% navigation field: timeseries + readMessagesbag_user_ref = select(bag,'Topic','/ERG/NF/user_reference');
bag_user_ref = select(bag,'Topic','/ERG/NF/user_reference');
time_user_ref= timeseries(bag_user_ref).Time;
msgs_user_ref = readMessages(bag_user_ref);
% 
% bag_applied_ref = select(bag,'Topic','/ERG/NF/applied_reference');
% time_applied_ref = timeseries(bag_applied_ref).Time;
% msgs_applied_ref = readMessages(bag_applied_ref);

%% navigation field: q_r, q_v
q_r = zeros(length(time_user_ref),7);
for t = 1 : length(time_user_ref)
    for i=1:7
        q_r(t,i)=msgs_user_ref{t}.Data(i);
    end
end
% 
% q_v = zeros(length(time_applied_ref),7);
% for t = 1 : length(time_applied_ref)
%     for i=1:7
%         q_v(t,i)=msgs_applied_ref{t}.Data(i);
%     end
% end

%% dynamic safety margin: timeseries + readMessages
bag_DSM_overall = select(bag,'Topic','/ERG/DSM/overall');
time_DSM_overall = timeseries(bag_DSM_overall).Time;
msgs_DSM_overall = readMessages(bag_DSM_overall);

bag_DSM_tau = select(bag,'Topic','/ERG/DSM/tau');
time_DSM_tau = timeseries(bag_DSM_tau).Time;
msgs_DSM_tau = readMessages(bag_DSM_tau);

bag_DSM_q = select(bag,'Topic','/ERG/DSM/q');
time_DSM_q = timeseries(bag_DSM_q).Time;
msgs_DSM_q = readMessages(bag_DSM_q);

% bag_DSM_obst_sphere = select(bag,'Topic','/ERG/DSM/obst_sphere');
% time_DSM_obst_sphere = timeseries(bag_DSM_obst_sphere).Time;
% msgs_DSM_obst_sphere = readMessages(bag_DSM_obst_sphere);
% 
% bag_DSM_obst_cylinder = select(bag,'Topic','/ERG/DSM/obst_cylinder');
% time_DSM_obst_cylinder = timeseries(bag_DSM_obst_cylinder).Time;
% msgs_DSM_obst_cylinder = readMessages(bag_DSM_obst_cylinder);

%% dynamic safety margin: DSM, DSM_tau, DSM_q
DSM = zeros(length(time_DSM_overall),1);
for t = 1 : length(time_DSM_overall)
    DSM(t)=msgs_DSM_overall{t}.Data;
end

DSM_tau = zeros(length(time_DSM_tau),1);
for t = 1 : length(time_DSM_tau)
    DSM_tau(t)=msgs_DSM_tau{t}.Data;
end

DSM_q = zeros(length(time_DSM_q),1);
for t = 1 : length(time_DSM_q)
    DSM_q(t)=msgs_DSM_q{t}.Data;
end

% DSM_obst_sphere = zeros(length(time_DSM_obst_sphere),1);
% for t = 1 : length(time_DSM_obst_sphere)
%     DSM_obst_sphere(t)=msgs_DSM_obst_sphere{t}.Data;
% end
% 
% DSM_obst_cylinder = zeros(length(time_DSM_obst_cylinder),1);
% for t = 1 : length(time_DSM_obst_cylinder)
%     DSM_obst_cylinder(t)=msgs_DSM_obst_cylinder{t}.Data;
% end

%% dynamic safety margin: prediction
bag_prediction = select(bag,'Topic','/ERG/DSM/prediction');
time_prediction = timeseries(bag_prediction).Time;
msgs_prediction = readMessages(bag_prediction);
pred_horizon=150;

q_pred = zeros(8,pred_horizon,floor(length(time_prediction)/pred_horizon));
tau_pred = zeros(8,pred_horizon,floor(length(time_prediction)/pred_horizon));
num_matrix=1;
t=1;
t_pred=1;
t_predstart = time_prediction(t);
while t <= length(time_prediction)
    if round(time_prediction(t)-t_predstart,1)>=0.5 
        if (t_pred==151)
            if (num_matrix == floor(length(time_prediction)/pred_horizon))
                break;
            else
                num_matrix = num_matrix + 1;
            end
        end
        t_pred = 1; 
        t_predstart = time_prediction(t);
    end
    for i=1:8
        q_pred(i,t_pred,num_matrix) = msgs_prediction{t}.Data(i);
        if size(q_pred,2)>pred_horizon
            size(q_pred,2)
        end
        if i==1
            tau_pred(i,t_pred,num_matrix) = msgs_prediction{t}.Data(i);
        else 
            tau_pred(i,t_pred,num_matrix) = msgs_prediction{t}.Data(i+7);   
        end
    end  
    t = t + 1;
    t_pred = t_pred + 1;
end
        

% for t = 1 : floor(length(time_prediction_q)/150)
%     for t_pred = 1 : 150
%         for i=1:8
%             q_pred(i,t_pred,t) = msgs_prediction_q{150*(t-1)+t_pred+135}.Data(i);
%         end
%     end
% end

%% save data
% matname = strcat(filename,'.mat');
% save(matname,'bag','q','dotq','tau','tau_cmd','q_r','q_v',...
%     'DSM','DSM_tau','DSM_q','DSM_obst_sphere','DSM_obst_cylinder','q_pred', 'tau_pred',...
%     'time_jointstates','time_tau_cmd','time_user_ref','time_applied_ref',...
%     'time_DSM_overall','time_DSM_tau','time_DSM_q',...
%     'time_DSM_obst_sphere','time_DSM_obst_cylinder');

% matname = strcat(filename,'.mat');
% save(matname,'bag','q','dotq','tau','time_jointstates','tau_cmd','time_tau_cmd',...
%     'gravity_effort','time_gravity_effort','coriolis_effort','time_coriolis_effort');
% save(matname,'bag','q','dotq','tau','time_jointstates','tau_cmd','time_tau_cmd','time_singleloop','time_chrono');


matname = strcat(filename,'.mat');
save(matname,'bag','q','dotq','tau','tau_cmd',...
    'q_r', 'time_user_ref',...
    'time_singleloop','time_chrono',... 
    'DSM','DSM_tau','DSM_q','q_pred', 'tau_pred',...
    'time_jointstates','time_tau_cmd',...
    'time_DSM_overall','time_DSM_tau','time_DSM_q');

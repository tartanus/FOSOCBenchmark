%% plot benchmark data
close all
time=simData.benchData.Time;

kpOptim=simData.benchData.Data(:,1); 
kiOptim=simData.benchData.Data(:,2);
costJ=simData.benchData.Data(:,3);
ref=simData.benchData.Data(:,5);
error=simData.benchData.Data(:,6);
yOut=simData.benchData.Data(:,7);
OVOptim=simData.benchData.Data(:,8);
TSOptim=simData.benchData.Data(:,9);
yIC=simData.benchData.Data(:,10);     %response with IC only

Ts=0.01;

figure(1)
subplot(2,1,1)
plot(time,kpOptim)
title('Optimal kp');xlabel('Time (s)'); ylabel('Kp')
set(gca,'FontSize', 14);
subplot(2,1,2)
plot(time,kiOptim)
title('Optimal ki');xlabel('Time (s)'); ylabel('Ki')
set(gca,'FontSize', 14);

figure(2)
subplot(2,3,1:3)
plot(time,yOut)
hold on
plot(time,ref)
set(gca,'FontSize', 20);
legend('y GCNM','Reference')
title('Optimal y');xlabel('Time (s)'); ylabel('y')


subplot(2,3,4)
plot(time(300/Ts:600/Ts),yOut(300/Ts:600/Ts))
hold on
plot(time(300/Ts:600/Ts),yIC(300/0.01:600/Ts))
plot(time(300/Ts:600/Ts),ref(300/0.01:600/Ts))
set(gca,'FontSize', 20);
legend('y GCNM','y IC','Reference')
title('Optimal y from 300 to 600s');xlabel('Time (s)'); ylabel('y')

subplot(2,3,5)
plot(time(3000/Ts:3300/Ts),yOut(3000/Ts:3300/Ts))
hold on
plot(time(3000/Ts:3300/Ts),yIC(3000/Ts:3300/Ts))
plot(time(3000/Ts:3300/Ts),ref(3000/Ts:3300/Ts))
set(gca,'FontSize', 20);
legend('y GCNM','y IC','Reference')
title('Optimal y from 3000 to 3300s');xlabel('Time (s)'); ylabel('y')


subplot(2,3,6)
plot(time(28000/Ts:28300/Ts),yOut(28000/Ts:28300/Ts))
hold on
plot(time(28000/Ts:28300/Ts),yIC(28000/Ts:28300/Ts))
plot(time(28000/Ts:28300/Ts),ref(28000/Ts:28300/Ts))
set(gca,'FontSize', 20);
legend('y GCNM','y IC','Reference')
title('Optimal y from 28000 to 28300s');xlabel('Time (s)'); ylabel('y')

figure(3)
subplot(2,1,1)
plot(time,OVOptim)
set(gca,'FontSize', 14);
hold on
title('Optimal OV');xlabel('Time (s)'); ylabel('Overshoot')
subplot(2,1,2)
plot(time,TSOptim)
set(gca,'FontSize', 14);
title('Optimal Settling time');xlabel('Time (s)'); ylabel('Settling time (s)')

% figure(4)
counter=1;
clear costJSub timeSub kpOptimSub kiOptimSub
for i=1:300/Ts:length(costJ)-300/Ts
%     if i==1
%         index=6;
%     else
%         index=i;
%     end
%     
    costJSub(counter)=max(costJ(i:i+300/Ts));
    timeSub(counter)=max(time(i:i+300/Ts));
    kpOptimSub(counter)=kpOptim(i);
    kiOptimSub(counter)=kiOptim(i);
    counter=counter+1;
end

% reducedSampleFactor=600;
% costJSub=downsample(costJ,reducedSampleFactor)
% timeSub=downsample(time,reducedSampleFactor)

% plot(timeSub,costJSub)
% set(gca,'FontSize', 14);
% hold on
% title('J');xlabel('Time (s)'); ylabel('J')


%% plot ISE space
clc;

K=1;
tau=1;
% L=10;

if L<1 %big searching space(Kp,,ki>=100 for L<0.1)
    kp=1:1:15; ki=1:15/length(kp):15;
    t=0:0.1:10;
elseif L>=1 && L<10 %average searching space (kp,ki<1 for L>10)
    kp=0.1:0.1:1; ki=0.1:1/length(kp):1;%+1/length(kp);
    t=0:0.1:100;
elseif L>=10 %small searching space (kp,ki<1 for L>10)
    kp=0.01:0.01:0.1; ki=0.01:0.01:0.1;
    t=0:0.1:1500;
end

refAmplitude=1;
R=refAmplitude*ones(1,length(t));

%%  plot obtained gains vs ZN and ZNM L=0.1
s=tf('s');
p=((1*exp(-L*s))/(1*s+1));
[Gc01,KpZN01,TiZN01,TdZN01,H]=ziegler(2,[1 L 1, 10]);      %original ZN
[GcM01,KpZNM01,TiZNM01,TdZNM01,HM]=ziegler(3,[1 L 1, 10]); %modified ZN
KiZN01=KpZN01/TiZN01;
KiZNM01=KpZNM01/TiZNM01;

%closed loop response of the system with ZN PI tuning parameters
T=feedback(GcM01*p,1);
Y=lsim(T,R,t);
% figure()
% plot(t,Y)
% hold on

%closed loop response of the system with Modified ZN PI tuning parameters
T=feedback(Gc01*p,1);
Y=lsim(T,R,t);
% plot(t,Y)

%closed loop response of the system with SOC tuning parameters
GcSOC= kpOptim(end)+kiOptim(end)/s;
T=feedback(GcSOC*p,1);
Y=lsim(T,R,t);
% plot(t,Y)
% 
% legend('PI ZN','PI ZNM','PI SOC')
% xlabel('Time (s)')
% set(gca,'FontSize', 14);


%% SOC performance indices
% converTimeInvRealNoise(yin,sw,epsilon,refAmplitude)

SOCConvTime= timeSub(  converTimeInv1s(kiOptimSub,10,1e-3)); %convergence time SOC
% SOCConvTime= timeSub(  converTimeInvRealNoiseDown(costJSub,10,1e-3,1)); %convergence time SOC
OVFinal=OVOptim(end);   %Final OV
TSFinal=TSOptim(end);   %Final Settling time
RMSFinal=rms(yOut);             %RMS value of the system output
ISEFinal=     (1/length(yOut))*sum((yOut-ref).^2);
IAEFinal=     (1/length(yOut))*sum(abs(yOut));
ITAEFinal=    (1/length(yOut))*sum(time.*(yOut-ref).^2);
RMSE=    sqrt(  ((1/length(yOut)))*sum(yOut-ref).^2);
varNames={'SOC convergence time','OV','Settling Time','RMS','ISE','IAE','RMSE'};
table(SOCConvTime,OVFinal,TSFinal,RMSFinal,ISEFinal,IAEFinal,RMSE,...
    'VariableNames',varNames)




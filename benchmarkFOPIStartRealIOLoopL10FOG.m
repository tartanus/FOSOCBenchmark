%% SOC PID benchmark configuration file for Optimal randomness search
clear;
%initial condition PI control
kpo=0.2; %10 1.2 0.06
kio=0.2; %3  0.6  0.05
alphaO=0.85;
%optimization gains

% PI controller gains limits
maxKp=2; minKp=0.3;
maxKi=0.6; minKi=0.2;
%Globalized Constrained Nelder Mead GCNM parameters
GCNMEnable=1;
optimPeriod=300;        %optimization execution period (s)
resetThreshold=0.1;     %probabilistic restart threshold
refSignalPeriod=300;    %Reference signal period
refAmplitude=1;         %reference amplitude
%constraints limits
OVLim=5;                %overshoot
TSLim=80;               %Settling time limit (s)
L=10;                    %delay value L=0.1,1,10;
N=100;                  %derivative filter N (Zero for PI controller)
ts=0.01;                %sampling time
tSim=30000;             %simulation time

%% random noise performance evaluation
warning('off','all')
repetitions=2;
pathName="C:\Users\jairo\Dropbox\doctorado UC\yangquan assignments\DT peltier\DT SOC\benchmarkFOPIDataIOL10FO\IODataFO";
% alpha=1.5;
for k=2:repetitions
    for alpha=0.8:0.1:0.9
    %generate fractional order noise for the GCNM probabilistic restart
    timeVec=0:optimPeriod:tSim;
    timeVecFinal=0:ts:tSim;
%     kpRandAux=alphaStableNoise(alpha,1000)*maxKp;
%     kiRandAux=alphaStableNoise(alpha,1000)*maxKi;
    kpRandAux=ffGn(1000,alpha,1,0)*maxKp;
    kiRandAux=ffGn(1000,alpha,1,0)*maxKi;
    lambdaRandAux=ffGn(1000,alpha,0.05,0.95);
    kpRandAux=abs(kpRandAux/(max(abs(kpRandAux)))*maxKp);
    kiRandAux=abs(kiRandAux/(max(abs(kiRandAux)))*maxKi);
    lambdaRandAux=abs(lambdaRandAux/(max(abs(lambdaRandAux))))+0.1;
    
    for z=1:length(lambdaRandAux)
        
        if lambdaRandAux(z)<0.9
            lambdaRandAux(z)=0.9;
        end
    end
    
    
%     %constrain random integrator order to a minimum value
%     idx = lambdaRandAux<0.7;
%     lambdaRandAux(idx) = lambdaRandAux(idx) +0.7;
    
    
    randNoiseAux=alphaStableNoise(alpha,1000);
    kpRand=[]; kiRand=[]; randNoise=[]; lambdaRand=[];
    tic
    for j=1:length(timeVec)
        kpRand=[kpRand; ones(optimPeriod/ts,1)*kpRandAux(j)];
        kiRand=[kiRand; ones(optimPeriod/ts,1)*kiRandAux(j)];
        lambdaRand=[lambdaRand; ones(optimPeriod/ts,1)*lambdaRandAux(j)];
%         randNoise=[randNoise ones(optimPeriod/ts,1)*randNoiseAux(j)];
    end
  
    %random initial condition vector for the simulations
    randomKp=[timeVecFinal' kpRand(1:length(timeVecFinal))];
    randomKi=[timeVecFinal' kiRand(1:length(timeVecFinal))];
    lambdaRand=[timeVecFinal' lambdaRand(1:length(timeVecFinal))];
%     randomNoiseOutput=[timeVecFinal' randNoise(1:length(timeVecFinal))];
    toc
    tic
    % Run the Benchmark file
    simData=sim("FOPIDSOCOptimRand10.slx",'SrcWorkspace','current');
    benchmarkFiguresSOCRepeat
    execTimeIter=toc
    %saving data
    tic
    fileName=strcat(pathName,string(k),'alpha',string(alpha*10),'.mat');
    save(fileName)
    toc
    end
end





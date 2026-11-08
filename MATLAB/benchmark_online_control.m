function result=benchmark_online_control(verifyOnly)
% Complete numerical controller tick with internal state update, excluding I/O.
% This is a desktop MATLAB execution benchmark, not a hardware deadline proof.
if nargin<1,verifyOnly=false;end
here=fileparts(mfilename('fullpath'));root=fileparts(here);addpath(fullfile(here,'core'));
out=fullfile(root,'output');if ~exist(out,'dir'),mkdir(out);end
tracePath=fullfile(out,'revised_trajectories.mat');
if ~exist(tracePath,'file'),run_simulations();end
traces=load(tracePath);
fixed=load(fullfile(root,'reference_results','frozen_normalization.mat'));
rr=jsondecode(fileread(fullfile(root,'reference_results','selected_parameters.json')));
names=fieldnames(rr);C=numel(names);B=30;K=100;W=1000;dt=fixed.cfg(15);
inputs=cell(C,1);samples=round(linspace(1,size(traces.(names{1}),1),B*K));checks=struct();
for i=1:C
 r=rr.(names{i});d=traces.(names{i});theta=[r.parameters(:);zeros(7-numel(r.parameters),1)];
 integral=cumtrapz(d(:,1),d(:,12:13));
 inputs{i}=struct('data',d,'integral',integral,'theta',theta,'kind',r.kind,'phi',r.phi);
 maxError=0;
 for k=1:W
  j=1+mod(37*k-1,size(d,1));[q,dq,acc,t,state]=sampleInputs(inputs{i},j);
  [u,next]=controllerTick(r.kind,theta,q,dq,acc,t,state,fixed.cfg,r.phi);
  err=max(abs(u-d(j,8:9)')./max(1,abs(d(j,8:9)')));
  if r.kind==0 && j<size(d,1)
   err=max(err,max(abs(next.tau-d(j+1,8:9)')./max(1,abs(d(j+1,8:9)'))));
  end
  maxError=max(maxError,err);
 end
 assert(maxError<1e-8,'Controller tick differs from the reference trajectory.');checks.(names{i})=maxError;
end
fid=fopen(fullfile(out,'online_step_consistency.json'),'w','n','UTF-8');fwrite(fid,jsonencode(checks,'PrettyPrint',true),'char');fclose(fid);
if verifyOnly,result=checks;disp(checks);return;end
times=zeros(B*K,C);checksum=zeros(C,1);
for b=1:B
 % Rotate method order across batches to reduce fixed order bias.
 order=1+mod((0:C-1)+b-1,C);
 for i=order
  a=inputs{i};
  for k=1:K
   index=(b-1)*K+k;j=samples(index);[q,dq,acc,t,state]=sampleInputs(a,j);
   started=tic;
   [u,next]=controllerTick(a.kind,a.theta,q,dq,acc,t,state,fixed.cfg,a.phi);
   times(index,i)=toc(started)*1e6;
   checksum(i)=checksum(i)+sum(u)+sum(next.tau)+sum(next.integral);
  end
 end
end
rows=cell(C,8);
for i=1:C
 sorted=sort(times(:,i));p95=sorted(ceil(.95*numel(sorted)));
 rows(i,:)={names{i},median(sorted),mean(sorted),p95,max(sorted),median(sorted)/(dt*1e6)*100,sum(sorted>dt*1e6),numel(sorted)};
end
T=cell2table(rows,'VariableNames',{'Controller','Median_us','Mean_us','P95_us','MaxObserved_us','MedianPeriodPercent','ObservedOverPeriod','Samples'});
writetable(T,fullfile(out,'online_timing_summary.csv'));
protocol=struct('MATLAB',version,'OS',system_dependent('getos'),'CPU',getenv('PROCESSOR_IDENTIFIER'),...
 'controller_period_s',dt,'warmup_calls_per_method',W,'batches',B,'calls_per_batch',K,...
 'input_states','3000 samples distributed across the 20-second representative trajectories; extraction outside timed region',...
 'included','Reference/model/derivative evaluation; sliding variable; controller algebra; ASMC torque integration or PID integral-state update; returned controller state',...
 'excluded','Sensor acquisition, acceleration estimation, communication, actuator I/O, operating-system deadline guarantees, and simulated-plant RK4 integration',...
 'timing_method','Individual tic/toc durations without subtraction of timer overhead; fixed rotating method order across batches',...
 'p95_definition','Nearest-rank empirical 95th percentile',...
 'worst_case_execution_time_claimed',false);
save(fullfile(out,'online_timing_raw.mat'),'times','checksum','samples','names','protocol');
fid=fopen(fullfile(out,'online_timing_protocol.json'),'w','n','UTF-8');fwrite(fid,jsonencode(protocol,'PrettyPrint',true),'char');fclose(fid);
result=struct('table',T,'protocol',protocol);disp(T);
end

function [q,dq,acc,t,state]=sampleInputs(a,j)
d=a.data;q=d(j,2:3)';dq=d(j,4:5)';acc=d(j,6:7)';t=d(j,1);
state=struct('tau',d(j,8:9)','integral',[0;0],'previous_error',d(j,12:13)','started',j>1);
if j>1,state.integral=a.integral(j-1,:)';state.previous_error=d(j-1,12:13)';end
end

function [u,next]=controllerTick(kind,theta,q,dq,acc,t,state,cfg,phi)
dt=cfg(15);limit=cfg(17);next=state;
if kind==2
 r=sr_reference(t,cfg);e=r-q;
 if state.started,next.integral=state.integral+dt/2*(state.previous_error+e);end
 next.previous_error=e;next.started=true;
end
if kind==3
 [r,rd,rdd]=sr_reference(t,cfg);e=r-q;ed=rd-dq;s=ed+theta(1:2).*e;
 [H,h]=sr_model(q,dq,cfg);
 command=H*(rdd+theta(1:2).*ed+theta(3:4).*s+theta(5:6).*min(1,max(-1,s/theta(7))))+h;
else
 [command,~]=sr_control(kind,theta,q,dq,state.tau,next.integral,acc,t,cfg,120,phi);
end
if kind==0
 u=state.tau;next.tau=min(limit,max(-limit,state.tau+dt*command));
else
 u=min(limit,max(-limit,command));next.tau=u;
end
if kind==2 && isfinite(limit)
 bound=limit./max(theta(3:4),1e-8);next.integral=min(bound,max(-bound,next.integral));
end
end

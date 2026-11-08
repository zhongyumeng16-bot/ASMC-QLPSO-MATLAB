function result=sr_tune_balanced(task,seed,cfg,normalizers,weights,population,iterations)
% All five comparisons optimize the same frozen dimensionless objective.
% task 1/2 use identical ASMC-sat structure and initial populations per seed.
switch task
 case {1,2}
  kind=0;lo=[1,1,.1,.1,.03,.03];hi=[50,50,10,10,.8,.8];
  warm=[23.5652,24.036504,2.1023,2.0896862,.1292,.133076];phi=.5;
 case 3
  kind=1;lo=[.5,.5,.5,.5,1,1];hi=[30,30,100,100,100,100];
  warm=[2.0928,2.0928,16.4462,16.4462,60.9933,60.9933];phi=0;
 case 4
  kind=2;lo=[5,5,.1,.1,.1,.1];hi=[400,400,100,100,40,40];warm=[100,100,5,10,4,2];phi=0;
 case 5
  kind=3;lo=[.5,.5,.5,.5,1,1,.01];hi=[30,30,100,100,100,100,2];
  warm=[2.0928,2.0928,16.4462,16.4462,60.9933,60.9933,.5];phi=0;
 otherwise,error('Unknown experiment task.');
end
D=numel(lo);rng(seed,'twister');x=rand(population,D);velocity=.16*rand(population,D)-.08;
x(1,:)=(warm-lo)./(hi-lo);velocity(1,:)=0;
r1=rand(iterations,population,D);r2=rand(iterations,population,D);
explore=rand(iterations,population);randomAction=randi(4,iterations,population);
actions=[.95,2.5,.5;.6,1,2;.8,2,1.2;.5,.8,2.5];Q=zeros(4);counts=zeros(1,4);states=zeros(1,4);
fitness=zeros(population,1);failures=0;started=tic;
for i=1:population,[fitness(i),failed]=evaluate(lo+x(i,:).*(hi-lo));failures=failures+failed;end
pbest=x;personal=fitness;[best,index]=min(fitness);globalBest=x(index,:);
history=zeros(iterations+1,1);history(1)=best/(1-best);
for it=1:iterations
 oldGlobal=globalBest;
 for i=1:population
  st=stateIndex(x(i,:),oldGlobal,D);states(st)=states(st)+1;
  if task==1
   if explore(it,i)<.6,action=randomAction(it,i);
   else
    tied=find(Q(st,:)==max(Q(st,:)));action=tied(1+mod(randomAction(it,i)-1,numel(tied)));
   end
   coefficients=actions(action,:);counts(action)=counts(action)+1;
  else,action=1;coefficients=[.7298,1.49618,1.49618];end
  a=reshape(r1(it,i,:),1,D);b=reshape(r2(it,i,:),1,D);
  velocity(i,:)=coefficients(1)*velocity(i,:)+coefficients(2)*a.*(pbest(i,:)-x(i,:))+coefficients(3)*b.*(oldGlobal-x(i,:));
  velocity(i,:)=min(.08,max(-.08,velocity(i,:)));xx=min(1,max(0,x(i,:)+velocity(i,:)));
  [value,failed]=evaluate(lo+xx.*(hi-lo));failures=failures+failed;
  if task==1
   next=stateIndex(xx,oldGlobal,D);reward=fitness(i)-value;
   Q(st,action)=.99*Q(st,action)+.01*(reward+.9*max(Q(next,:)));
  end
  x(i,:)=xx;fitness(i)=value;
  if value<personal(i),personal(i)=value;pbest(i,:)=xx;end
 end
 [candidate,index]=min(personal);
 if candidate<best,best=candidate;globalBest=pbest(index,:);end
 history(it+1)=best/(1-best);
end
parameters=lo+globalBest.*(hi-lo);theta=[parameters(:);zeros(7-D,1)];
metrics=evaluate_metrics(kind,theta,cfg,120,phi);
J=weights*(metrics([1,5,3,4])./normalizers)';
result=struct('task',task,'seed',seed,'kind',kind,'parameters',parameters,'phi',phi,...
 'objective_J',J,'fitness_F',best,'history_J',history,'metrics',metrics,...
 'Q_table',Q,'action_counts',counts,'state_counts',states,'failed_evaluations',failures,...
 'population',population,'iterations',iterations,'evaluations',population*(iterations+1),...
 'tuning_seconds',toc(started),'normalizers',normalizers,'weights',weights);

 function [value,failed]=evaluate(parameters)
  theta=[parameters(:);zeros(7-D,1)];m=evaluate_metrics(kind,theta,cfg,120,phi);
  failed=m(7)>0 || any(~isfinite(m));
  if failed,value=1;return;end
  j=weights*(m([1,5,3,4])./normalizers)';value=j/(1+j);
 end
end
function st=stateIndex(x,g,D)
distance=norm(x-g)/sqrt(D);
if distance>=.75,st=1;elseif distance>=.5,st=2;elseif distance>=.25,st=3;else,st=4;end
end

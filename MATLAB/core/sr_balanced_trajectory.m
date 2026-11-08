function [data,info]=sr_balanced_trajectory(kind,theta,cfg,rho,phi)
% Native MATLAB trajectory replay for the balanced study and its extra baseline.
dt=cfg(15);N=round(cfg(14)/dt);limit=cfg(17);data=zeros(N+1,17);
state=[cfg(12:13);0;0];tau=zeros(2,1);integral=zeros(2,1);
info=struct('success',true,'completed_time',cfg(14));
for k=0:N
 t=k*dt;[r,rd,rdd]=sr_reference(t,cfg);e=r-state(1:2);a=rhs(state,tau,t,cfg);
 if kind==3
  [H,h]=sr_model(state(1:2),state(3:4),cfg);ed=rd-state(3:4);s=ed+theta(1:2).*e;
  command=H*(rdd+theta(1:2).*ed+theta(3:4).*s+theta(5:6).*min(1,max(-1,s/theta(7))))+h;
 else
  [command,s]=sr_control(kind,theta(1:6),state(1:2),state(3:4),tau,integral,a(3:4),t,cfg,rho,phi);
 end
 if kind==0,u=tau;rate=command;flags=abs(tau+dt*rate)>limit;
 else,u=min(limit,max(-limit,command));rate=zeros(2,1);flags=abs(command)>limit;a=rhs(state,u,t,cfg);end
 data(k+1,:)=[t,state',a(3:4)',u',s',e',rate',double(flags')];
 if k==N,break;end
 step=dt/cfg(16);
 for j=0:cfg(16)-1
  off=j*step;u0=min(limit,max(-limit,u+rate*off));um=min(limit,max(-limit,u+rate*(off+step/2)));ue=min(limit,max(-limit,u+rate*(off+step)));
  f1=rhs(state,u0,t+off,cfg);f2=rhs(state+step/2*f1,um,t+off+step/2,cfg);
  f3=rhs(state+step/2*f2,um,t+off+step/2,cfg);f4=rhs(state+step*f3,ue,t+off+step,cfg);
  state=state+step/6*(f1+2*f2+2*f3+f4);
 end
 if kind==0,tau=min(limit,max(-limit,u+dt*rate));else,tau=u;end
 rn=sr_reference(t+dt,cfg);integral=integral+dt/2*(e+rn-state(1:2));
 if kind==2 && isfinite(limit),bound=limit./max(theta(3:4),1e-8);integral=min(bound,max(-bound,integral));end
 if any(~isfinite([state;tau])) || max(abs(state(1:2)))>20 || max(abs(state(3:4)))>500
  data=data(1:k+1,:);info.success=false;info.completed_time=t;return;
 end
end
end
function f=rhs(state,u,t,cfg)
[H,h]=sr_model(state(1:2),state(3:4),cfg);[~,~,~,~,d]=sr_reference(t,cfg);
f=[state(3:4);H\(u-h+d)];
end

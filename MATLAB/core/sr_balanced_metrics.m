function metrics=sr_balanced_metrics(kind,theta,cfg,rho,phi)
%#codegen
% Same MATLAB model, controller and RK4 stages as sr_simulate. No trajectory
% storage is needed inside an optimizer. kind=3 adds a saturated ERLSMC.
% metrics: summed IAE, ITAE, ISU, TV, two-joint RMSE, peak actual torque,
% divergence flag, completed samples, maximum |q| and maximum |qdot|.
dt=cfg(15);N=round(cfg(14)/dt);limit=cfg(17);
state=[cfg(12:13);0;0];tau=zeros(2,1);integral=zeros(2,1);
prevE=zeros(2,1);prevU=zeros(2,1);prevSquared=zeros(2,1);
ia=zeros(2,1);ita=zeros(2,1);isu=zeros(2,1);tv=zeros(2,1);
ss=zeros(2,1);peak=0;maxq=0;maxv=0;
metrics=zeros(1,10);
for k=0:N
 t=k*dt;[r,rd,rdd]=sr_reference(t,cfg);e=r-state(1:2);
 a=brhs(state,tau,t,cfg);
 if kind==3
  [H,h]=sr_model(state(1:2),state(3:4),cfg);ed=rd-state(3:4);
  surface=ed+theta(1:2).*e;sw=min(1,max(-1,surface/theta(7)));
  command=H*(rdd+theta(1:2).*ed+theta(3:4).*surface+theta(5:6).*sw)+h;
 else
  [command,~]=sr_control(kind,theta(1:6),state(1:2),state(3:4),tau,integral,a(3:4),t,cfg,rho,phi);
 end
 if kind==0,u=tau;rate=command;else,u=min(limit,max(-limit,command));rate=zeros(2,1);end
 if k>0
  ia=ia+dt/2*(prevE+abs(e));
  ita=ita+dt/2*((t-dt)*prevE+t*abs(e));
  isu=isu+dt/2*(prevSquared+u.^2);tv=tv+abs(u-prevU);
 end
 ss=ss+e.^2;peak=max(peak,max(abs(u)));maxq=max(maxq,max(abs(state(1:2))));maxv=max(maxv,max(abs(state(3:4))));
 prevE=abs(e);prevU=u;prevSquared=u.^2;
 if k==N,break;end
 step=dt/cfg(16);
 for j=0:cfg(16)-1
  off=j*step;u0=min(limit,max(-limit,u+rate*off));um=min(limit,max(-limit,u+rate*(off+step/2)));ue=min(limit,max(-limit,u+rate*(off+step)));
  f1=brhs(state,u0,t+off,cfg);f2=brhs(state+step/2*f1,um,t+off+step/2,cfg);
  f3=brhs(state+step/2*f2,um,t+off+step/2,cfg);f4=brhs(state+step*f3,ue,t+off+step,cfg);
  state=state+step/6*(f1+2*f2+2*f3+f4);
 end
 if kind==0,tau=min(limit,max(-limit,u+dt*rate));else,tau=u;end
 rn=sr_reference(t+dt,cfg);integral=integral+dt/2*(e+rn-state(1:2));
 if kind==2 && isfinite(limit)
  bound=limit./max(theta(3:4),1e-8);integral=min(bound,max(-bound,integral));
 end
 if any(~isfinite([state;tau])) || max(abs(state(1:2)))>20 || max(abs(state(3:4)))>500
  metrics(7)=1;metrics(8)=k+1;return;
 end
end
metrics=[sum(ia),sum(ita),sum(isu),sum(tv),sqrt(sum(ss)/(2*(N+1))),peak,0,N+1,maxq,maxv];
end
function f=brhs(state,u,t,cfg)
[H,h]=sr_model(state(1:2),state(3:4),cfg);[~,~,~,~,d]=sr_reference(t,cfg);
f=[state(3:4);H\(u-h+d)];
end

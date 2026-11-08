function [command,s] = sr_control(kind,theta,q,dq,tau,integral,acc,t,cfg,rho,phi)
% kind 0: ASMC torque rate; 1: ERLSMC torque; 2: PID torque.
% ASMC acc is the actual simulated acceleration (ideal feedback assumption).
[r,rd,rdd,rddd]=sr_reference(t,cfg);e=r-q;ed=rd-dq;
theta=theta(:);
if kind==0
    alpha=theta(1:2);beta=theta(3:4);eta=theta(5:6);
    [H,h,Hdot,hq,hv]=sr_model(q,dq,cfg);
    a0=H\(tau-h);hdot0=hq*dq+hv*a0;
    s=alpha.*e+beta.*ed+eta.*(rdd-acc);
    psi=alpha.*ed+beta.*(rdd-a0)+eta.*(rddd+H\(Hdot*a0)+H\hdot0);
    if phi>0,sw=min(1,max(-1,s/phi));else,sw=sign(s);end
    command=H*((psi+rho*sw)./eta);
elseif kind==1
    [H,h]=sr_model(q,dq,cfg);s=ed+theta(1:2).*e;
    command=H*(rdd+theta(1:2).*ed+theta(3:4).*s+theta(5:6).*sign(s))+h;
else
    s=zeros(2,1);command=theta(1:2).*e+theta(3:4).*integral+theta(5:6).*ed;
end
end

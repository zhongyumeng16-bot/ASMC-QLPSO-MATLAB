function [H,h,Hdot,hq,hv] = sr_model(q,dq,cfg)
% Shared canonical Lagrange model; cfg follows experiment_protocol.json.
m1=cfg(1);m2=cfg(2);l1=cfg(3);l2=cfg(4);J1=cfg(5);J2=cfg(6);g=cfg(7);
b=m2*l1*l2;sn=sin(q(2));cs=cos(q(2));
H=[(m1+m2)*l1^2+m2*l2^2+2*b*cs+J1,m2*l2^2+b*cs;
   m2*l2^2+b*cs,m2*l2^2+J2];
A=(m1+m2)*g*l1;B=m2*g*l2;
h=[-b*sn*(2*dq(1)*dq(2)+dq(2)^2)+A*cos(q(1))+B*cos(sum(q));
    b*sn*dq(1)^2+B*cos(sum(q))];
if nargout>2
    Hdot=[-2*b*sn*dq(2),-b*sn*dq(2);-b*sn*dq(2),0];
    hq=[-A*sin(q(1))-B*sin(sum(q)),-b*cs*(2*dq(1)*dq(2)+dq(2)^2)-B*sin(sum(q));
        -B*sin(sum(q)),b*cs*dq(1)^2-B*sin(sum(q))];
    hv=[-2*b*sn*dq(2),-2*b*sn*sum(dq);2*b*sn*dq(1),0];
end
end

function [r,rd,rdd,rddd,d] = sr_reference(t,cfg)
w=cfg(8:9);r=sin(w*t);rd=w.*cos(w*t);rdd=-w.^2.*r;rddd=-w.^2.*rd;
if cfg(10)==0
    d=cfg(11)*[0.2*sin(2*t)+0.1*sin(4*t)+0.05*cos(1.5*t);
               0.02*cos(1.5*t)+0.3*sin(2*t)+0.05*cos(0.5*t)];
else
    d=cfg(11)*[0.3*cos(t)+0.2*sin(6*t)+0.05*cos(3.5*t);
               0.1*sin(0.5*t)+0.3*cos(2.4*t)+0.2*sin(t)];
end
end

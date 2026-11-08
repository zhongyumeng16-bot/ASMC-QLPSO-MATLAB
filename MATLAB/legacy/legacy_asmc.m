function result=legacy_asmc()
% Numerical core extracted from arm_SMC_ASMC.m.
% Historical arithmetic is retained, not a corrected fair-comparison model.
% Missing kaifang dependency replaced by the linear P=Q=1 surface
% and its mathematical derivative; this is a documented reconstruction.
ts=0.001;
T=20;
n=T/ts;
m1=0.3;m2=0.5;
l1=0.2;l2=0.3;
J1=0.05;J2=0.1;
g=9.81;
a1=2; a2=0.8;
dq1_1=0; dq2_1=0;
dq_1=[dq1_1;dq2_1];
ddq1_1=0; ddq2_1=0;
ddq_1=[ddq1_1;ddq2_1];
q1_1=0.5; q2_1=0.55;
q_1=[q1_1;q2_1];
tao1_1=0; tao2_1=0;
tao_1=[tao1_1;tao2_1];
rou=120;
alfa=23.5652;   kesei= 2.1023;  cita=  0.1292;
alfa=[23.5652,0;0,23.5652*1.02];   kesei=[2.1023,0;0,2.1023*0.994];  cita=[0.1292,0;0,0.1292*1.03];
P=1;Q=1;
sume1_new=0;
sume2_new=0;
for k=1:1:n
    timef(k)=ts*k;
    d1(k)=0.2*sin(2*timef(k))+0.1*sin(4*timef(k))+0.05*cos(1.5*timef(k));
    d2(k)=0.02*cos(1.5*timef(k))+0.3*sin(2*timef(k))+0.05*cos(0.5*timef(k));
    D(:,k)=[d1(k);d2(k)]*15;
    yd1(k)=sin(a1*timef(k));
    yd2(k)=sin(a2*timef(k));
    yd(:,k)=[yd1(k);yd2(k)];
    dyd1(k)=a1*cos(a1*timef(k));
    dyd2(k)=a2*cos(a2*timef(k));
    dyd(:,k)=[dyd1(k);dyd2(k)];
    ddyd1(k)=-a1^2*sin(a1*timef(k));
    ddyd2(k)=-a2^2*sin(a2*timef(k));
    ddyd(:,k)=[ddyd1(k);ddyd2(k)];
    dddyd1(k)=-a1^3*cos(a1*timef(k));
    dddyd2(k)=-a2^3*cos(a2*timef(k));
    dddyd(:,k)=[dddyd1(k);dddyd2(k)];
    h11=(m1+m2)*l1^2+m2*l2^2+2*m2*l1*l2*cos(q2_1)+J1;
    h12=m2*l2^2+m2*l1*l2*cos(q2_1);
    h21=h12;
    h22=m2*l2^2+J2;
    H=[h11,h12;
        h21,h22];
    dH=[-2*m2*l1*l2*sin(q2_1)*dq2_1,-m2*l1*l2*sin(q2_1)*dq2_1;
        -m2*l1*l2*sin(q2_1)*dq2_1,0];
    c11=-m2*l1*l2*sin(q2_1)*dq2_1;
    c12=-m2*l1*l2*sin(q2_1)*(dq1_1-dq2_1);
    c21=0;
    c22=m2*l1*l2*sin(q2_1)*dq1_1;
    C=[c11,c12;
        c21,c22];
    dC=[-m2*l1*l2*(ddq2_1*sin(q2_1)+dq2_1^2*cos(q2_1)),-m2*l1*l2*(cos(q2_1)*dq2_1*(dq1_1-dq2_1)+sin(q2_1)*(ddq1_1-ddq2_1));
        0, m2*l1*l2*(ddq1_1*sin(q2_1)+dq1_1*cos(q2_1)*dq2_1)];
    g1=(m1+m2)*g*l1*cos(q1_1)+m2*g*l2*cos(q1_1+q2_1);
    g2=m2*g*l2*cos(q1_1+q2_1);
    G=[g1;g2];
    dG=[-(m1+m2)*g*l1*sin(q1_1)*dq1_1-m2*g*l2*sin(q1_1+q2_1)*(dq1_1+dq2_1);
        -m2*g*l2*sin(q1_1+q2_1)*(dq1_1+dq2_1)];
    nn=pinv(H);
    q_new_1(:,k)=q_1+dq_1*ts;
    dq(:,k)=dq_1+nn*ddq_1*ts;
    ddq(:,k)=nn*(tao_1-G-C*dq_1+D(:,k));
    e1(:,k)=yd(:,k)-q_new_1(:,k);
    e2(:,k)=dyd(:,k)-dq(:,k);
    e3(:,k)=ddyd(:,k)-ddq(:,k);
    s(:,k)=alfa*e1(:,k)+kesei*e2(:,k)+cita*e3(:,k);
    v1=sign(s(:,k))*rou;
    A=alfa*e2(:,k);
    v(:,k)=H/cita*(A+kesei*(ddyd(:,k)-nn*(-C*dq_1-G+tao_1))+ ...
        cita*(dddyd(:,k)-nn*dH*nn*(-C*dq_1-G+tao_1)-nn*(-dC*dq_1-C*ddq_1-dG))+v1);
    tao_new_1(:,k)=tao_1+v(:,k)*ts;
    tao_1=tao_new_1(:,k);
    q_1=q_new_1(:,k);
    q1_1=q_1(1); q2_1=q_1(2);
    dq_1=dq(:,k);
    dq1_1=dq_1(1); dq2_1=dq_1(2);
    ddq_1=ddq(:,k);
    ddq1_1=ddq_1(1); ddq2_1=ddq_1(2);
    qe1_new_1(k)=sume1_new+abs(yd1(k)-q_new_1(1,k));
    sume1_new=qe1_new_1(k);
    qe2_new_1(k)=sume2_new+abs(yd2(k)-q_new_1(2,k));
    sume2_new=qe2_new_1(k);
end
result=struct();
result.timef=timef;
result.yd=yd;
result.q_new_1=q_new_1;
result.tao_new_1=tao_new_1;
result.qe1_new_1=qe1_new_1;
result.qe2_new_1=qe2_new_1;
end

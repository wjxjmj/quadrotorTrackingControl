% This source code is written to implement a controller of a quadrotor
% Author: wjxjmj
% Email: wjxjmj@126.com
% Open Source License: GPL
function omega=quadrotor_controller(s,xl,vl,psil,para,k1,k2)
s=real(s);
x=s(1);y=s(2);z=s(3);
vx=s(4);vy=s(5);vz=s(6);
phi=s(7);theta=s(8);psi=s(9);
vphi=s(10);vtheta=s(11);vpsi=s(12);
u=zeros(4,1);
tr=[para.b para.b para.b para.b;0 -para.b 0 para.b;-para.b 0 para.b 0;-para.d para.d -para.d para.d];


input_delta=k1*(xl-[x;y;z])+k2*(vl-[vx;vy;vz]);
attr=saturate(input_delta)*2+[0;0;para.g];
rot=[cos(psi) sin(psi) 0;-sin(psi) cos(psi) 0;0 0 1];
attr=rot*attr;

u(1)=para.m*norm(attr);

phi_p=asin(-attr(2)/norm(attr));
theta_p=atan(attr(1)/attr(3));
if attr(3)<0
    theta_p=theta_p-pi;
end
psi_p=psil;

u(4)=para.Iz*saturate(angleDelta(psi_p,psi)+0-vpsi);
u(2)=20*para.Ix*(u(1)+u(4))/2*saturate(angleDelta(phi_p,phi)+0-1*vphi)/para.l;
u(3)=20*para.Iy*(u(1)-u(4))/2*saturate(angleDelta(theta_p,theta)+0-1*vtheta)/para.l;

omega2=(tr\u);
omega2(omega2<0)=0;
omega=omega2.^0.5;

end

function u=saturate(input)
 u=tanh(input);
end


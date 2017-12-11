% This source code is written to implement the dynamics of a quadrotor
% Author: wjxjmj
% Email: wjxjmj@126.com
% Open Source License: GPL
function s=quadrotor_kinematics(s,omega,para,dt)

x=s(1);y=s(2);z=s(3);
vx=s(4);vy=s(5);vz=s(6);
phi=s(7);theta=s(8);psi=s(9);
vphi=s(10);vtheta=s(11);vpsi=s(12);
tr=[para.b para.b para.b para.b;0 -para.b 0 para.b;-para.b 0 para.b 0;-para.d para.d -para.d para.d];

omega2=(omega).^2;
Omega=-omega(1)+omega(2)-omega(3)+omega(4);
u=tr*omega2;

vx=vx+dt*(u(1)*(cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi))/para.m-para.k1*vx/para.m);x=x+dt*vx;
vy=vy+dt*(u(1)*(sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi))/para.m-para.k2*vy/para.m);y=y+dt*vy;
vz=vz+dt*(u(1)*(cos(theta)*cos(phi))/para.m-para.g-para.k3*vz/para.m);z=z+dt*vz;
vphi=vphi+dt*((para.Iy-para.Iz)/para.Ix*vtheta*vpsi-para.Jr*vtheta*Omega+u(2)*para.l/para.Ix-para.k4*vphi*para.l/para.Ix);phi=phi+dt*vphi;
vtheta=vtheta+dt*((para.Iz-para.Ix)/para.Iy*vphi*vpsi+para.Jr*vphi*Omega+u(3)*para.l/para.Iy-para.k5*vtheta*para.l/para.Iy);theta=theta+dt*vtheta;
vpsi=vpsi+dt*((para.Ix-para.Iy)/para.Iz*vphi*vtheta+u(4)*1/para.Iz-para.k6*vpsi/para.Iz);psi=psi+dt*vpsi;

s=[x;y;z;vx;vy;vz;phi;theta;psi;vphi;vtheta;vpsi];
end

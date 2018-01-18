% This source code is written to implement the animation of the flying
% process
% Author: wjxjmj
% Email: wjxjmj@126.com
% Open Source License: GPL

% parameters:
%  xHis:tensor in [d,n,loop] where d is the dimension and is fixed at 3 , n as
%       thenumber of quadrotors. It should be noted that n = 2 if you have 
%       only one quadrotor, because we regard the virtual leader as a 
%       craft. Let x1, x2 as the positions of quadrotor1 and quadrotor 2, 
%       then you should fill the data at loop t as xHis(:,:,loop)=[xd x1 x2] 
%       where xd is the virtual leader.
%  dt: the time interval between steps. We take fiexd-step method here.
%  tailLength: keep a trajectory for last int(tailLength) steps. Set this
%              value as -1 to show its full trajectory.
%  speed: The speed of this animation. We implement this animation by plot
%         function. If speed is setted as a, the we plot a times and the
%         data is uniformly-spaced sampled form tensor xHis according to
%         its third dimensition. If speed > size(xHis,3) then no change is
%         taken.
%  tt: total time of the simulation.
function plotHis3(xHis,dt,tailLength,speed,tt)
if nargin==2
    tailLength=-1;
    speed=-1;
    tt=-1;
elseif nargin==3
    speed=-1;
    tt=-1;
elseif nargin==4
    tt=-1;
end
[~,~,loop]=size(xHis);
ts=dt*(loop-1);
if tailLength>loop || tailLength<0
    tailLength=loop;
end
if speed>loop || speed<0
    speed=loop;
end
xyHis=[];
skip=round(loop/speed);
if skip>1
    for k=1:skip:loop
        if k==1
            xyHis=xHis(:,:,1);
        else
            xyHis(:,:,end+1)=xHis(:,:,k);
        end
    end
    if k~=loop
        xyHis(:,:,end+1)=xHis(:,:,end);
    end
else
    xyHis=xHis;
end
[~,n,loop]=size(xyHis);
amin=min(min(min(xyHis(1,:,:))));
amax=max(max(max(xyHis(1,:,:))));
bmin=min(min(min(xyHis(2,:,:))));
bmax=max(max(max(xyHis(2,:,:))));
cmin=min(min(min(xyHis(3,:,:))));
cmax=max(max(max(xyHis(3,:,:))));
aw=amax-amin;
if aw==0
    aw=1;
end
bw=bmax-bmin;
if bw==0
    bw=1;
end
amin=amin-0.05*aw;
amax=amax+0.05*aw;
bmin=bmin-0.05*bw;
bmax=bmax+0.05*bw;
xyz=reshape(xyHis,3*n,loop);%为方便画出智能体的历史轨迹，对记录智能体历史位置的矩阵进行变型
hwait=waitbar(0,'进行中>>>>>>>>>>');
for k=1:loop
    plot3(xyHis(1,:,1),xyHis(2,:,1),xyHis(3,:,1),'kx')
    hold on
    plot3(xyHis(1,1,k),xyHis(2,1,k),xyHis(3,1,k),'rp')
    plot3(xyHis(1,2:end,k),xyHis(2,2:end,k),xyHis(3,2:end,k),'bo', 'MarkerFaceColor','b','MarkerSize',5)
    
    tail=tailLength;%“尾巴的长度”
    if k<=tailLength || tailLength<0
        plot3(xyz(1,1:k),xyz(2,1:k),xyz(3,1:k),'m--')
    else
        plot3(xyz(1,k-tail:k),xyz(2,k-tail:k),xyz(3,k-tail:k),'m--')
    end
    for i=2:n
        if k<=tailLength || tailLength<0
            plot3(xyz(3*i-2,1:k),xyz(3*i-1,1:k),xyz(3*i,1:k),'c-')
        else
            plot3(xyz(3*i-2,k-tail:k),xyz(3*i-1,k-tail:k),xyz(3*i,k-tail:k),'c-')
        end
    end
    hold off%关闭“叠加”绘图模式
    title(['time=',num2str(k/loop*ts)]);
    
    axis([amin amax bmin bmax cmin cmax]);%固定绘制区域
    if tt>0
        if k/loop*ts>=tt
            break
        end
    end

      
    
%     axis equal
    %pause(0.1)
    waitbar(k/loop,hwait,'优化进行中');
end
close(hwait);
xlabel('x')
ylabel('y')
zlabel('z')

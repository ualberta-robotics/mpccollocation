state_dim = 2;

x = posdemos{1}.pos(1,:);
y = posdemos{1}.pos(2,:);
z = posdemos{1}.pos(3,:);

for i = 2:7
    %figure;
    %plot3(posdemos{i}.pos(1,:),posdemos{i}.pos(2,:),posdemos{i}.pos(3,:))
    x = [x posdemos{i}.pos(1,:)];
    y = [y posdemos{i}.pos(2,:)];
    z = [z posdemos{i}.pos(3,:)];
end

X = [x' y' z']

dx = veldemos{1}.vel(1,:);
dy = veldemos{1}.vel(2,:);
dz = veldemos{1}.vel(3,:);

for i = 2:7
    dx = [dx veldemos{i}.vel(1,:)];
    dy = [dy veldemos{i}.vel(2,:)];
    dz = [dz veldemos{i}.vel(3,:)];
end

dX = [dx' dy' dz']

polyorder = 2;
usesine = 0;

M = 3 ;
Theta = poolData(X,M,polyorder,usesine);
m = size(Theta,3);
dX = dX(1:end,:);

lambda = 0.005;
Xi = sparsifyDynamics(Theta,dX,lambda,state_dim);

out = poolDataLIST({'x','y','z'},Xi,M,polyorder,usesine);

fig = figure(1);
xlim([0 1])
ylim([-1 1])
xlabel("X")
ylabel("Y")

plot_streamline3D(fig, Xi, polyorder, usesine, 1, 1, 1);
hold on;

for i = 1:7
    plot3(posdemos{i}.pos(1,:)',posdemos{i}.pos(2,:),posdemos{i}.pos(2,:), 'Linewidth',2);
end
%Xi2d = Xi(:,1);
%plot_streamlines(fig, Xi2d, 2, usesine);

xpatch = [0.1 0 0 0.1];
ypatch = [0.1 0.1 -0.1 -0.1];

for i = 1:93
    h = line;
    direction = [0 0 1];
    rotate(h,direction,-posdemos{1}.pos(3,i));
end

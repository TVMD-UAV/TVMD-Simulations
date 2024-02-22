close all
x=linspace(0,pi/2);
y=linspace(0,pi/2);
[xx, yy] = meshgrid(x, y);
zz = -cos(xx).^2 .* sin(yy).^2 + sin(xx).^2;

surf(xx, yy, zz, 'EdgeColor', 'none'); hold on 
ty = -( cos(xx).*cos(yy)-1+sqrt(-cos(xx).^2.*sin(yy).^2+sin(xx).^2) ) ./ ...
      ( cos(xx).^2-sin(xx).^2 + 1 - 2*cos(xx).*cos(yy) );
a = cos(xx).^2 - sin(xx).^2 + 1 - 2*cos(xx).*cos(yy);
b = cos(xx) .* cos(yy) - 1;
c = 1;
% ty = (-b - sqrt(b.^2 - a.*c)) ./ a;
xlabel('x')
ylabel('y')
zlabel('z')

by = real(asin(tan(x)));
plot(x, by)
ty(tan(xx)-sin(yy) <= 0 ) = inf;
% ty(abs(real(a)) < 1e-10) = inf;
% ty(abs(real(ty)) > 10) = inf;
surf(xx, yy, ty, 'EdgeColor', 'none'); hold on 

zlim([-1 5])
caxis([-1 5]);

figure
case1 = (sin(xx) >= cos(yy)) & (cos(yy) >= cos(xx));
case2 = (sin(xx) <= cos(yy)) & (cos(yy) <= cos(xx));
cdata = zeros(size(xx));
cdata(case1) = 1;
cdata(case2) = -1;
cdata(ty>1) = 2;
heatmap(x, y, cdata)
xlabel('x')
ylabel('y')
clc
close all

data = xlsread('highway_map.xlsx');

data = data(1:50,:);

s = 140; d = 0.;


maps_x = data(:,1)
maps_y = data(:,2)
maps_s = data(:,3)


x_ = linspace(maps_x(1),maps_x(end),100);
y_ = spline(maps_x,maps_y,x_);
    
[x,y] = get_xy(s,d,maps_s, maps_x, maps_y);

[x_s,y_s] = get_xy_spline(s,d,maps_s, maps_x, maps_y);

figure
plot(data(:,1), data(:,2))
hold on
plot(x,y,'r.')
plot(x_,y_,'g')
plot(x_s,y_s,'k.')

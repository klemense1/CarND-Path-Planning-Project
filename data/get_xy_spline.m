function [x,y] = get_xy_spline(s,d,maps_s, maps_x, maps_y)
%GET_XY_SPLINE Summary of this function goes here
%   Detailed explanation goes here

    maps_dx = diff(maps_x);
    maps_dy = diff(maps_y);

    pp_x = spline(maps_s,maps_x);
    pp_y = spline(maps_s,maps_y);
    pp_dx = spline(maps_s(2:end),maps_dx);
    pp_dy = spline(maps_s(2:end),maps_dy);
    
    dx_s = ppval(pp_dx, s);
    dy_s = ppval(pp_dy, s);

    % consider s
    seg_x = ppval(pp_x, s);
    seg_y = ppval(pp_y, s);
    
    heading = atan2(dy_s,dx_s);

    perp_heading = heading-pi/2;

    % also consider d
    x = seg_x + d*cos(perp_heading);
    y = seg_y + d*sin(perp_heading);
    

end


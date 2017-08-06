function [x,y] = get_xy(s,d,maps_s, maps_x, maps_y)

    prev_wp = 0;

    while s > maps_s(prev_wp+1) && (prev_wp < size(maps_s,1)-1)
        prev_wp = prev_wp+1;
    end

    wp2 = mod(prev_wp+1,size(maps_x,1));

    heading = atan2((maps_y(wp2)-maps_y(prev_wp)),(maps_x(wp2)-maps_x(prev_wp)));

    seg_s = (s-maps_s(prev_wp));
    seg_x = maps_x(prev_wp)+seg_s*cos(heading);
    seg_y = maps_y(prev_wp)+seg_s*sin(heading);

    perp_heading = heading-pi/2;

    x = seg_x + d*cos(perp_heading);
    y = seg_y + d*sin(perp_heading);

end
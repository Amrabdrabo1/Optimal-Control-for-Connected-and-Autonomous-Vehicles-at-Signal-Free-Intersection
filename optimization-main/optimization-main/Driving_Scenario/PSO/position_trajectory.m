function vehicle_positions = position_trajectory(center, direction, L, S, dL)
xc=center(1);
yc=center(2);
n = (2*L + S)/dL + 1;
if direction == "NS"
    vehicle_positions = [[xc+(S/2)+L:-dL:xc-(S/2)-L]', repmat(yc, n, 1)];
else
    vehicle_positions = [repmat(xc, n, 1), [yc-(S/2)-L:dL:yc+(S/2)+L]'];
end
end


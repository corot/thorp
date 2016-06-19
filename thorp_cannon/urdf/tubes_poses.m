i = 0;
for a = 0:pi/3:2*pi
    i = i + 1;
    y = cos(a)*0.0044;
    z = sin(a)*0.0044;
    fprintf('  <cannon_tube parent="cannon_link" number="%d" y_loc="%f" z_loc="%f"/>\n', i, y, z);
end
function R = rpyToR(rpy)
    roll = rpy(1); pitch = rpy(2); yaw = rpy(3);
    R_z = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    R_y = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)];
    R_x = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];
    R = R_z*R_y*R_x;
end
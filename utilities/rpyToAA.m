function [axis, angle] = rpyToAA(rpy)
    R = rpyToR(rpy);
    u = [R(3,2) - R(2,3);
         R(1,3) - R(3,1);
         R(2,1) - R(1,2)];
     angle = acos((trace(R) - 1)/2);
     axis = u;
end
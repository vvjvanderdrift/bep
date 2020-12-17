function [f] = fun_angleq34_11_12(q)
    global  yaw_P roll_P pitch_P no
        f(1) = cos(pitch_P(no))*cos(yaw_P(no)) - sin(pitch_P(no))*sin(roll_P(no))*sin(yaw_P(no)) - (cos(q(1)));
        f(2) = cos(pitch_P(no))*sin(yaw_P(no)) + cos(yaw_P(no))*sin(pitch_P(no))*sin(roll_P(no)) - (sin(q(1)));
        f(3) = -cos(roll_P(no))*sin(pitch_P(no)) - (0);
        
        f(4) = - cos(roll_P(no))*sin(yaw_P(no)) - (-cos(q(2))*sin(q(1)));
        f(5) = cos(roll_P(no))*cos(yaw_P(no)) - (cos(q(1))*cos(q(2)));
        f(6) = sin(roll_P(no)) - (sin(q(2)));
      
        f(7) = cos(yaw_P(no))*sin(pitch_P(no)) + cos(pitch_P(no))*sin(roll_P(no))*sin(yaw_P(no)) - (sin(q(1))*sin(q(2)));
        f(8) = sin(pitch_P(no))*sin(yaw_P(no)) - cos(pitch_P(no))*cos(yaw_P(no))*sin(roll_P(no)) - (-cos(q(1))*sin(q(2)));
        f(9) = cos(pitch_P(no))*cos(roll_P(no)) - (cos(q(2)));
end

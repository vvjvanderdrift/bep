function [f] = fun_angleq7_11_12(q)
    global  yaw_Q roll_Q pitch_Q lambda q3_berekend q4_berekend no
        f(1) = cos(pitch_Q(no))*cos(yaw_Q(no)) - sin(pitch_Q(no))*sin(roll_Q(no))*sin(yaw_Q(no)) - (cos(lambda)*cos(q3_berekend(no))*cos(q(1)) - cos(q4_berekend(no))*sin(q3_berekend(no))*sin(q(1)) - cos(q(1))*sin(lambda)*sin(q3_berekend(no))*sin(q4_berekend(no)));
        f(2) = cos(pitch_Q(no))*sin(yaw_Q(no)) + cos(yaw_Q(no))*sin(pitch_Q(no))*sin(roll_Q(no)) - (cos(lambda)*cos(q(1))*sin(q3_berekend(no)) + cos(q3_berekend(no))*cos(q4_berekend(no))*sin(q(1)) + cos(q3_berekend(no))*cos(q(1))*sin(lambda)*sin(q4_berekend(no)));
        f(3) = -cos(roll_Q(no))*sin(pitch_Q(no)) - (sin(q4_berekend(no))*sin(q(1)) - cos(q4_berekend(no))*cos(q(1))*sin(lambda));
        
        f(4) = - cos(roll_Q(no))*sin(yaw_Q(no)) - (sin(lambda)*sin(q3_berekend(no))*sin(q4_berekend(no))*sin(q(1)) - cos(q4_berekend(no))*cos(q(1))*sin(q3_berekend(no)) - cos(lambda)*cos(q3_berekend(no))*sin(q(1)));
        f(5) = cos(roll_Q(no))*cos(yaw_Q(no)) - (cos(q3_berekend(no))*cos(q4_berekend(no))*cos(q(1)) - cos(lambda)*sin(q3_berekend(no))*sin(q(1)) - cos(q3_berekend(no))*sin(lambda)*sin(q4_berekend(no))*sin(q(1)));
        f(6) = sin(roll_Q(no)) - (cos(q(1))*sin(q4_berekend(no)) + cos(q4_berekend(no))*sin(lambda)*sin(q(1)));
      
        f(7) = cos(yaw_Q(no))*sin(pitch_Q(no)) + cos(pitch_Q(no))*sin(roll_Q(no))*sin(yaw_Q(no)) - (cos(q3_berekend(no))*sin(lambda) + cos(lambda)*sin(q3_berekend(no))*sin(q4_berekend(no)));
        f(8) = sin(pitch_Q(no))*sin(yaw_Q(no)) - cos(pitch_Q(no))*cos(yaw_Q(no))*sin(roll_Q(no)) - (sin(lambda)*sin(q3_berekend(no)) - cos(lambda)*cos(q3_berekend(no))*sin(q4_berekend(no)));
        f(9) = cos(pitch_Q(no))*cos(roll_Q(no)) - (cos(lambda)*cos(q4_berekend(no)));
end

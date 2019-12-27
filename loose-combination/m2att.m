function att=m2att(Cnb)
    att = [ asin(Cnb(3,2)); 
            atan2(-Cnb(3,1),Cnb(3,3)); %atan2返回值是[-PI,PI]atan返回值是[-PI/2,PI/2]
            atan2(-Cnb(1,2),Cnb(2,2)) ];

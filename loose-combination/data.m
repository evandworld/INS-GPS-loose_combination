function [yy_1,yy_2,yy_3,yy_4,yy_5,yy_6,yy_7] = data(y)
i=1;
    s1=0;s2=0;s3=0;s4=0;s5=0;s6=0;s7=0;
    while i<101
        s1=s1+y(13*(i-1)+1);
        s2=s2+y(13*(i-1)+2);
        s3=s3+y(13*(i-1)+3);
        s4=s4+y(13*(i-1)+4);
        s5=s5+y(13*(i-1)+5);
        s6=s6+y(13*(i-1)+6);
        s7=s7+y(13*(i-1)+7);
       i=i+1; 
    end
     s1=s1/100;
     s2=s2/100;
     s3=s3/100;
     s4=s4/100;
     s5=s5/100;
     s6=s6/100;
     s7=s7/100;
    i=1;
    while i<4000
        if y(13*(i-1)+1)-s1>0.05
        yy_1(i)=s1;
        else
        yy_1(i)=y(13*(i-1)+1);
        s1=yy_1(i);
        end
        if y(13*(i-1)+2)-s2>0.05
        yy_2(i)=s2;
        else
        yy_2(i)=y(13*(i-1)+2);
        s2=yy_2(i);
        end
        if y(13*(i-1)+3)-s3>0.05
        yy_3(i)=s3;
        else
        yy_3(i)=y(13*(i-1)+3);
        s3=yy_3(i);
        end
        if y(13*(i-1)+4)-s4>0.5
        yy_4(i)=s4;
        else
        yy_4(i)=y(13*(i-1)+4);
        s4=yy_4(i);
        end
        if y(13*(i-1)+5)-s5>0.5
        yy_5(i)=s5;
        else
        yy_5(i)=y(13*(i-1)+5);
        s5=yy_5(i);
        end
        if y(13*(i-1)+6)-s6>0.5
        yy_6(i)=s6;
        else
        yy_6(i)=y(13*(i-1)+6);
        s6=yy_6(i);
        end
        if y(13*(i-1)+7)-s7>3
        yy_7(i)=s7;
        else
        yy_7(i)=y(13*(i-1)+7);
        s7=yy_7(i);
        end   
    i=i+1;
end
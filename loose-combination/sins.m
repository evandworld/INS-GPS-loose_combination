function [qnb, vn, pos] = sins(qnb_1, vn_1, pos_1, wm, vm, ts)
    tss = ts*size(wm,2);
    [phim,dvbm] = cnscl(wm,vm);
    [wnie,wnen,rmh,rnh,gn] = earth(pos_1,vn_1);
    wnin = wnie+wnen;

    vn = vn_1 + (eye(3)+askew(-wnin*(1.0/2*tss)))*qmulv(qnb_1,dvbm) ...  
        + (gn-cross(wnie+wnin,vn_1))*tss;   

    vn1_1 = (vn+vn_1)/2;
    pos = pos_1 + tss*[vn1_1(2)/rmh;vn1_1(1)/(rnh*cos(pos_1(1)));vn1_1(3)];  

    qnb = qmul(rv2q(wnin*tss),qmul(qnb_1,rv2q(phim)));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %{
    %-----------四元数
    wbnb=wm-q2cnb(qconj(qnb_1))*wnin*tss;
    omega=[      0   -wbnb(1)  -wbnb(2)  -wbnb(3);%--------------------4阶RK
            wbnb(1)        0    wbnb(3)  -wbnb(2);
            wbnb(2)  -wbnb(3)        0    wbnb(1);
            wbnb(3)   wbnb(2)  -wbnb(1)        0  ];
    kqo0=1/2*omega*qnb_1;
    kqo1=1/2*omega*(qnb_1+tss/2*kqo0);
    kqo2=1/2*omega*(qnb_1+tss/2*kqo1);
    kqo3=1/2*omega*(qnb_1+tss*kqo2);
    
    qnb_1=qnb_1+(kqo0+2*kqo1+2*kqo2+kqo3)*tss/6;
    
    modqnb_1=sqrt(qnb_1(1)^2+qnb_1(2)^2+qnb_1(3)^2+qnb_1(4)^2);
    if modqnb_1~=0
        qnb=qnb_1/modqnb_1;
    end
    %-----------速度
    W=askew(wnie+wnin); fn=q2cnb(qnb_1)*vm/ts;
    k1=(fn-W*vn_1+gn); 
    k2=(fn-W*(vn_1+k1*tss/2)+gn);
    k3=(fn-W*(vn_1+k2*tss/2)+gn);
    k4=(fn-W*(vn_1+k3*tss)+gn);
    vn=vn_1+(k1+2*k2+2*k3+k4)*tss/6;
    %-----------位置
    pos = pos_1 + tss*[vn(2)/rmh;vn(1)/(rnh*cos(pos_1(1)));vn(3)];
    %}

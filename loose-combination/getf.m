function Ft = getf(qnb, vn, pos, fb)
    global glv
    [wnie,wnen,rmh,rnh,gn] = earth(pos,vn);
	sl = sin(pos(1)); cl = cos(pos(1)); tl = sl/cl; secl = 1/cl; secl2 = secl^2;
    f_RMh = 1/rmh; f_RNh = 1/rnh; f_RMh2 = f_RMh^2; f_RNh2 = f_RNh^2;
    %%%
    M1 = [0, 0, 0; -glv.wie*sl, 0, 0; glv.wie*cl, 0, 0];
    M2 = [0, -f_RMh, 0; f_RNh, 0, 0; f_RNh*tl, 0, 0];
    M3 = [0, 0, vn(2)*f_RMh2; 0, 0, -vn(1)*f_RNh2; vn(1)*secl2*f_RNh, 0, -vn(1)*tl*f_RNh2];
    M13 = M1+M3;
    M4 = askew(vn)*M2 - askew(2*wnie+wnen);                                                                         
    M5 = askew(vn)*(2*M1+M3);
    M6 = [0, f_RMh, 0; secl*f_RNh, 0, 0; 0, 0, 1];
    M7 = [0, 0, -vn(2)*f_RMh2; vn(1)*secl*tl*f_RNh, 0, -vn(1)*secl*f_RNh2; 0, 0, 0];
    S1 = askew(wnie+wnen); S3 = askew(qmulv(qnb,fb));
    o3 = zeros(3,3);   Cnb = q2cnb(qnb);
    %om1=-1/500*ones(3);om2=-1/500*ones(3);
    %%%%%  phi    dvn   dpos   ebÕ”¬›∆Ø“∆ŒÛ≤Ó dbº”±Ì∆Ø“∆ŒÛ≤Ó
    Ft = [-S1    M2    M13    -Cnb    o3 
           S3    M4    M5     o3      Cnb 
           o3    M6    M7     o3      o3  
           o3    o3    o3     o3      o3  
           o3    o3    o3     o3      o3]; 
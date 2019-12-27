function [Fikk_1, Qk] = kfdis(Ft, Qt, Tkf, n)
    %Fikk_1=In +Tkf*Ft +Tkf^2/2*Ft^2 +Tkf^3/6*Ft^3 +Tkf^4/24*Ft^4 +Tkf^5/120*Ft^5; 
    %M1=Qt; M2=Ft*M1+(Ft*M1)'; M3=Ft*M2+(Ft*M2)'; M4=Ft*M3+(Ft*M3)'; M5=Ft*M4+(Ft*M4)';
    %Qk=M1*Tkf +M2*Tkf^2/2 +M3*Tkf^3/6 +M4*Tkf^4/24 +M5*Tkf^5/120;
    Tkfi = Tkf;   
    facti = 1;      
    Fti = Ft;
    Mi = Qt;
    In = eye(size(Ft,1));
    Fikk_1 = In + Tkf*Ft;
    Qk = Qt*Tkf;
    for i=2:1:n
        Tkfi = Tkfi*Tkf;        
        facti = facti*i;
        Fti = Fti*Ft;
        Fikk_1 = Fikk_1 + Tkfi/facti*Fti;
        
        FtMi = Ft*Mi;
        Mi = FtMi + FtMi';
        Qk = Qk + Tkfi/facti*Mi;
    end    
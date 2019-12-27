function [Xk, Pk, Kk] = kf(Phikk_1, Qk, Xk_1, Pk_1, Hk, Rk, Zk)
    if nargin<7                         %½ö½øÐÐ×´Ì¬µÝÍÆ    
        Xk = Phikk_1*Xk_1;
        Pk = Phikk_1*Pk_1*Phikk_1'+Qk;
    else                                %ÓÐ²âÁ¿Ê±ÂË²¨   
        %---time update
        Xkk_1=Phikk_1*Xk_1;             %ÔÝÌ¬    
        Pkk_1 = Phikk_1*Pk_1*Phikk_1' + Qk; 
        %---measure update
        Pxz = Pkk_1*Hk';
        Pzz = Hk*Pxz + Rk;
        Kk = Pxz*Pzz^-1;                %ÔöÒæ
        Xk = Xkk_1 + Kk*(Zk-Hk*Xkk_1);  %Ô¤²â×´Ì¬
        Pk = Pkk_1 - Kk*Pzz*Kk';        %Ô¤²â×´Ì¬Ð­·½²î
    end

function [pitch,roll,head]=astd(att)
pitch=att(1);%¸©Ñö½Ç
roll=att(2); %ºá¹ö½Ç
head=att(3); %º½Ïò½Ç
while roll>pi
    roll=roll-2*pi;
end
while roll<-pi
    roll=roll+2*pi;
end
while head>pi
    head=head-2*pi;
end
while head<-pi
    head=head+2*pi;
end

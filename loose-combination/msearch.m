function result=msearch(matrix,option)
%ÕÒµ½×îÖµ
if nargin<2
    error('wrong input number')
end
[m n]=size(matrix);
mtemp=matrix(1,1);
if option==1
       for i=1:m   
            for j=1:n               
                if abs(mtemp)>abs(matrix(i,j))                  
                    mtemp=matrix(i,j);
                end
            end
       end
elseif option==2
       for i=1:m   
            for j=1:n               
                if abs(mtemp)<abs(matrix(i,j))                  
                    mtemp=matrix(i,j);
                end
            end
       end
else
    error('wrong option')
end
result=mtemp;
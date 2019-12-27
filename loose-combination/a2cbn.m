   
   function Cbn=a2cbn(a,b,c)
    si = sin(a); ci = cos(a); 
    sj = sin(b); cj = cos(b); 
    sk = sin(c); ck = cos(c);
    Cbn = [ cj*ck+si*sj*sk , ci*sk  ,      sj*ck-si*cj*sk    ;
            -cj*sk+si*sj*ck , ci*ck   ,      -sj*sk-si*cj*ck    ;
            -ci*sj         , si      ,      ci*cj ];
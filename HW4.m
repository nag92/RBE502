syms  J v b  m g h D

A = [   0 1 ; m*g*h/J 0]
B = [ (D*v)/(b*J); (m*v^2*h)/(b*J)];
C = [1 0];


observe = [ C;C*A] 

control = [B,A*B ]
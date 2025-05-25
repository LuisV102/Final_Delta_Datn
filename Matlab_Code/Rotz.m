function T = Rotz(q)
R=Rz(q);
T=[R(1,1) R(1,2) R(1,3) 0;
   R(2,1) R(2,2) R(2,3) 0;
   R(3,1) R(3,2) R(3,3) 0;
   0 0 0 1   ];
end
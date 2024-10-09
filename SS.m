function S_n = SS(a)
% The function returns the skew symmetrical (SS) matrix for a 3-dimensional
% (row or column) vector.

if (length(a) ~= 3), error('Input vector must have a dimension of 3!');
 end

a1 = a(1);
a2 = a(2);
a3 = a(3);

S_n = [0 -a3 a2;
      a3 0 -a1;
      -a2 a1 0];

% The definition of the SS matrix is given in Definition 2.2 of
% Fossen (2011).
% Fossen, T.I., 2011. Handbook of marine craft hydrodynamics and motion 
% control. John Wiley & Sons.
end
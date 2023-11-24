function [y,nb] = NaiveRealisation(x, seuil)

N = length(x);
y = x;
nb = 0;

for n = 1:N
    if abs(x(n)) < seuil
        y(n) = 0;
        nb = nb + 1;
    end
end
function [y] = Compression(x, thresold, reduction_gain)

N = length(x);
y = x;

max = thresold + (1-thresold)/reduction_gain;

a = (max - thresold)/(1 - thresold);
b = max - a;

for n = 1:N
    if abs(x(n)) >= thresold
        if x(n) >= 0
            y(n) = a*x(n) + b;
        else
            y(n) = a*x(n) - b;
        end
    end
end
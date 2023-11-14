clear all;

dry = 1;
wet = 1;
fb = 0.8;

b = [dry, 0, 0, 0, wet - fb*dry];
a = [1, 0, 0, 0 fb];

impz(b,a,50)
clc
clear all

syms t1 t2 t3 x y z

eqns = [150*cos(t1)*cos(t2) - (67*sin(t1))/5 + 49*cos(t1)*cos(t2)*cos(t3) - 110*cos(t1)*cos(t2)*sin(t3) - 110*cos(t1)*cos(t3)*sin(t2) - 49*cos(t1)*sin(t2)*sin(t3)==x,...
        (67*cos(t1))/5 + 150*cos(t2)*sin(t1) + 49*cos(t2)*cos(t3)*sin(t1) - 110*cos(t2)*sin(t1)*sin(t3) - 110*cos(t3)*sin(t1)*sin(t2) - 49*sin(t1)*sin(t2)*sin(t3)==y,...
        110*sin(t2)*sin(t3) - 110*cos(t2)*cos(t3) - 49*cos(t2)*sin(t3) - 49*cos(t3)*sin(t2) - 150*sin(t2)==z];

f = 150*cos(t1)*cos(t2) - (67*sin(t1))/5 + 49*cos(t1)*cos(t2)*cos(t3) - 110*cos(t1)*cos(t2)*sin(t3) - 110*cos(t1)*cos(t3)*sin(t2) - 49*cos(t1)*sin(t2)*sin(t3) - x...
    + (67*cos(t1))/5 + 150*cos(t2)*sin(t1) + 49*cos(t2)*cos(t3)*sin(t1) - 110*cos(t2)*sin(t1)*sin(t3) - 110*cos(t3)*sin(t1)*sin(t2) - 49*sin(t1)*sin(t2)*sin(t3) - y...
    +110*sin(t2)*sin(t3) - 110*cos(t2)*cos(t3) - 49*cos(t2)*sin(t3) - 49*cos(t3)*sin(t2) - 150*sin(t2) - z;

diff(eqns(3),t3)
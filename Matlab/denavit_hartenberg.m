clc
clear all

syms s1 c1 s2 c2 s3 c3 s4 c4 s5 c5 s6 c6 ca sa cb sb cg sg l1 l2 WL

T10 =   [1   0 0   l1;
        0   1  0   0;
        0   0   1   0;
        0   0   0   1];

T21 =   [1   0 0   l2;
        0   -1  0   0;
        0   0   -1   0;
        0   0   0   1];


T1 =   [c1  -s1 0   0;
        s1  c1  0   0;
        0   0   1   0;
        0   0   0   1];

R =   [ca*cb    ca*sb*sg-sa*cg  ca*sb*cg+sa*sg;
       sa*sb    sa*sb*sg+ca*cg  sa*sb*cg-ca*sg;
       -sa      cb*sg           cb*cg];

T1 =   [c1  -s1 0   0;
        s1  c1  0   0;
        0   0   1   0;
        0   0   0   1];

R1 =   [c1  -s1 0;
        s1  c1  0;
        0   0   1];

% reverse sign of s2, because sin(90-x) = -sin(x-90)
T2 =   [c2  s2  0   0;
        0   0   -1  0;
        -s2 c2  0   0;
        0   0   0   1];

R2 =   [c2  s2  0;
        0   0   -1;
        -s2 c2  0];

T3 =   [c3  s3  0   150;
        -s3 c3  0   0;
        0   0   1   -13.4;
        0   0   0   1];

R3 =   [c3  s3  0;
        -s3 c3  0;
        0   0   1];

T4 =   [c4  -s4 0   49;
        0   0   -1  -110;
        s4  c4  0   0;
        0   0   0   1];

R4 =   [c4  -s4 0;
        0   0   -1;
        s4  c4  0];

T5 =   [c5  s5  0   0;
        0   0   1  0;
        s5  -c5 0   0;
        0   0   0   1];

R5 =   [c5  s5  0;
        0   0   1;
        s5  -c5 0];

T6 =   [c6  -s6 0   0;
        0   0   -1  -WL;
        s6  c6  0   0;
        0   0   0   1];

R6 =   [c6  -s6 0;
        0   0   -1;
        s6  c6  0];

% Final rotations to match end-effector orientation to global
%RY =   [cos(90)    0   sin(90)    0;
%        0           1   0           0;
%        -sin(90)    0   cos(90)   0;
%        0           0   0           1];
RY =   [0    0   1     0;
        0    1   0      0;
        -1    0   0      0;
        0    0   0      1];

%RZ =   [cos(180)    -sin(180)   0       0;
%        sin(180)    cos(180)    0       0;
%        0           0           1       0;
%        0           0           0       1];
RZ =   [-1   0      0   0;
        0    -1     0   0;
        0    0      1   0;
        0    0      0   1];

T4_noRotation =   [ 1   0   0   49;
                    0   1   0  -110;
                    0   0   1   0;
                    0   0   0   1];

%T1*T2*T3*T4*T5*T6*RY*RZ
T1*T2*T3*T4*T5*T6
%T1*T2*T3*T4_noRotation
%T4*T5*T6
%transpose(R1*R2*R3)*R
clc
clear
close all

A = [-1.01887 0.90506 -0.00215; 0.82225 -1.07741 -0.17555; 0 0 -1];
B2 = [0; 0; 1];
B1 = [1; 0; 0];

B = [B1 , B2];
m1 = size(B1,2);
m2 = size(B2,2);
beta=5;

R = [-beta^2*eye(m1) zeros(m1,m2) ; zeros(m2,m1) eye(m2)];
C=eye(3);

[P,G,K_Hinf]=care(A,B,C'*C,R);
disp('CARE solution for H_infiinty (P):');
disp(P);

P_Q1=care(A,B2,eye(3),1);
disp('Regular solution for IRL (P):');
disp(P_Q1);
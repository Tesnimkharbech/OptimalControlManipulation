%Plot the step response of the system with the transfer function of a second-order low-pass filter. 
%This filter is used in the motor control to suppress undesirable high frequencies that can cause 
%oscillations and vibrations: H(p) = 2 / (0.15p² + p)
num= [2]
denum=[0.15 1 0]
sys = tf(num, denum)
step(sys)
%Determine the discrete transfer function for a sampling period of one second
Ts= 1 %sampling period
sysd=c2d(sys, Ts, 'zoh')
%Plot the step response of the discrete system
step(sysd)
% Put the system in state-space form
sys_ss=ss(sysd)
% Find theoretically the weighting matrices Q and R
Q=[10^(-6) 0;0 1]
R= 0.001
% Calculate the gain K and the eigenvalues for the values of Q and R
% We have to assign the results of sys_ss to variables first:
A=[1.001 -0.04072;0.03125 0] %state matrix
B=[4;0] %control matrix
C=[0.4251 2.377]
D=0
[K, S, E]= lqr(A,B,Q,R)
%Vary the value of Q (2Q, 5Q, 10Q and 12Q) and calculate each time the value of gain K, the poles of the system, and the matrix S
[K1, S1, E1]= lqr(A,B,2*Q,R)
[K2, S2, E2]= lqr(A,B,5*Q,R)
[K3, S3, E3]= lqr(A,B,10*Q,R)
[K4, S4, E4]= lqr(A,B,12*Q,R)
% Plot the variation of the poles (closed-loop) as a function of Q. Determine the influence on the behavior of the system
p1=eig(A-B*K)
p2=eig(A-B*K1)
p3=eig(A-B*K2)
p4=eig(A-B*K3)
p5=eig(A-B*K4)
%Plotting:
figure;
plot(real(p1),imag(p1), 'x')
hold on;
plot(real(p2),imag(p2), '*')
hold on;
plot(real(p3),imag(p3), '.')
hold on;
plot(real(p4),imag(p4), 'o')
hold on;
plot(real(p5),imag(p5), '+')
hold on;
xlabel('Reel');
ylabel('Imag');
legend('Q','2Q','5Q','10Q','12Q');
grid on;
%Increasing the value of the weighting matrix Q in the quadratic optimal control (LQ) 
%generally leads to a faster and more aggressive control
%(the poles move away from the imaginary axis)
% Vary the value of R (2R, 5R, 10R, and 12R) and calculate the value of the gain K, the poles of the system, and the S matrix each time.
[K5, S5, E5]= lqr(A,B,Q,2*R)
[K6, S6, E6]= lqr(A,B,Q,5*R)
[K7, S7, E7]= lqr(A,B,Q,10*R)
[K8, S8, E8]= lqr(A,B,Q,12*R)
% Plot the variation of the poles (closed-loop) as a function of Q. Determine the influence on the behavior of the system
p9=eig(A-B*K5)
p6=eig(A-B*K6)
p7=eig(A-B*K7)
p8=eig(A-B*K8)
%Plotting:
figure;
plot(real(p1),imag(p1), '+')
hold on;
plot(real(p9),imag(p9), 'o')
hold on;
plot(real(p6),imag(p6), 'x')
hold on;
plot(real(p7),imag(p7), '*')
hold on;
plot(real(p8),imag(p8), '.')
hold on;
xlabel('Reel');
ylabel('Imag');
legend('R','2R','5R','10R','12R');
grid on;
%Increasing the value of the weighting matrix R in the quadratic optimal control (LQ) generally leads to a slower 
%and smoother control. The optimization of the LQ cost function will favor minimal use of control inputs, but it 
%can lead to system instability (poles approaching the imaginary axis).

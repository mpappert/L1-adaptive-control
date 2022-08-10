%Plotten von lambda=L1NORM(G)*L für unterschiedliche Bandbreiten von C(s)
s=tf('s');
H=(s*eye(2)-Am)\b;
lambda=zeros(100,1);


for i=1:1:100
    omega = i;
    C=omega/(s+omega);
    G=H*(1-C);
    lambda(i)=L1NORM(G)*L;
end
 
grenze=ones(100,1);

I=1:1:100;
plot(I,lambda,I,grenze,'--r')
xlabel('\omega_c in rad/s');
ylabel('\lambda'); 

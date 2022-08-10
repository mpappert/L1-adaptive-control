function z = MaximiereL1NG(x)
s=tf('s');

mux=x(1); %7
km=x(2);%1.6
Masse=x(3); %24.5
epsilon1=x(4);
A =[0 1; 0 -((mux)/Masse)];
b=[0;(2*km*epsilon1)/Masse]; 
c=[1 0];
A0=tf(ss(A,b,c,0));
A0n=A0.num{1}*[s^2;s;1];
A0d=A0.den{1}*[s^2;s;1];

% L1-Regler: Referenz-Modell und Filter aus Maplefile

%L1-Regler 1: für Modell aus [TAARW13], Pole bei [-2000,-2000,-2000,-1.25,-1.25]
C=3.919739749*10^10/(3.919739749*10^10+1.200980589*10^7*s+6001.634286*s^2+s^3);
Cn=3.919739749*10^10;
Cd=(3.919739749*10^10+1.200980589*10^7*s+6001.634286*s^2+s^3);

M=1/(1.566434557*s^2+3.001699185*s+1);
Mn=1;
Md=(1.566434557*s^2+3.001699185*s+1);


% Umwandlung des Referenzmodells in die Regelungsnormalform
[M2]=canon(M,'companion');
[A_Db,b_Db,c_Db]=ssdata(M2);
Am=A_Db';
bm=c_Db';
cm=b_Db';

H=A0n*Mn*Cd/(Cn*A0n*Md+Mn*A0d*(Cd-Cn));
G=H*(1-C);

%Maximierung von L1NORM(G) (deshalb '-')
    if isstable(G)==1
       z=-L1NORM(G); %-
    else
        z=-1e12; %-
    end
end
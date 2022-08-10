function z = Timedelaymargin(x)
%File zur Minimierung der Time-Delay-Margin
s=tf('s');
omegak=689.68; %Regler1: omega_max*k=12566  Regler2: omega_max*k=689.68 
Am=[0 1; -0.6400   -1.9200];
b=[0;1];
c=[1 0];

Hq=(s*eye(2)-Am-b*[x(1) x(2)])\b;
L0=omegak/s*(1+[x(1) x(2)]*Hq);

[~,Pm,~,Wp]=margin(L0);

%Time-Delay-Margin gibt untere Schranke an für Time-Delay-Margin
%des geschlossenen adaptiven regelkreises
z=Pm/Wp;

end
function y=L1NORM(G)

[l,m]=size(G); %l Ausg�nge, m Eing�nge
Verstaerkung_Eingang=zeros(1,m);
Verstaerkung_Ausgang=zeros(l,1);

for i=1:l
   for j=1:m 
        
       %Impulsantwort
        dt=0.0001; %0.0001
        T=0:dt:50; 
        % T=50 ist bereits sehr gro�, selten muss es gr��er gew�hlt werden.
        % Dazu sollte man sich die ausgeklammerte Impulsantwort plotten
        % lassen.
        
        %impulse(G(i,j),T)
        [y,t]=impulse(G(i,j),T);
        %Berechnen des Betrages
        y2=abs(y);
        %Numerische Integralberechnung
        Z=trapz(y2);
        Integral=dt*Z;
        
        Verstaerkung_Eingang(j)=Integral;
   end %end Eing�nge
   
   Verstaerkung_Ausgang(i)=norm(Verstaerkung_Eingang,1);

end %end Ausg�nge


y=max(Verstaerkung_Ausgang);



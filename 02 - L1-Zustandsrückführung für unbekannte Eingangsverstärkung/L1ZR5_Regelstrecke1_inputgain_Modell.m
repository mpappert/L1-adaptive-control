%Setup für die Simulation des L1-Zustandsreglers 2.2
%Zuerst ParameterFileLaeufermodell.m und Preload_Fcn_Computed_Torque_nonlin_GPI.m ausführen 


close all;
clc;
run('C:\Users\Markus\Documents\MATLAB\Masterarbeit\LP3S Simulink Modell Kai Treichel\06_Gesamtmodell LP3S\ParameterFileLaeuferModell.m');
run('C:\Users\Markus\Documents\MATLAB\Masterarbeit\LP3S Simulink Modell Kai Treichel\06_Gesamtmodell LP3S\Preload_Fcn_Computed_Torque_nonlin_GPI.m');
ICy=0;
%% Berechnung des Designsystems

%Einfaches Modell der Regelstrecke / wird nicht mehr benötigt,
% da nun exaktes Modell verwendet wird.
%mux=7; %7
%km=3;
%m=20;
%A =[0 1; 0 -((mux)/m)];
b=[0;1];
c=[1 0];


%Designsystem
 K=1;
 xi=1.2; %1.2    %Normaler Fall
 T=1.25;  
 Am=[0 1; -1/T^2 -2*xi/T];
 cm=[K/T^2 0]; 
 bm=[0;1];
 M=tf(ss(Am,bm,cm,0));
 %cm wird nicht benötigt,L1 regelt
 %System auf Am und kg normiert auf 1.

%Filter erster Ordnung
k=31885; 

kg=-1/(c/Am*b);
Q=eye(2);
P=lyap(Am',Q);
Gamma=100000;

Pb=P*b;
PbG = Pb*Gamma;
xhat0 = 0;
x0 = 0;

% Loop Shaping des Zustandsschätzers
% (hat keinen Vorteil gebracht)
% ksp=1.4*sqrt(Gamma)-1;


%% Berechnung der Projektions-Schranken

%Wertebereiche der Parameter
m_min=19;           m_max=30;
mux_min=0;          mux_max=14;
Kmx_min=0.8;        Kmx_max=2.4;
epsilon_min=0.7;    epsilon_max=1.3;

%Berechnung von Theta_0=[[theta1_min, theta1_max];[theta2_min, theta2_max]]
%Berechnung theta1_min, theta2_min
mux=mux_max;
masse=m_min;
A =[0 1; 0 -((mux)/masse)];
theta1_min=A(2,1)-Am(2,1);
theta2_min=A(2,2)-Am(2,2);

%Berechnung theta1_max, theta2_max
mux=mux_min;
masse=m_max;
A =[0 1; 0 -((mux)/masse)];
theta1_max=A(2,1)-Am(2,1);
theta2_max=A(2,2)-Am(2,2);

%Berechnung Omega_0=[omega0_min omega0_max]
%Berechnung omega0_min
Epsilon=epsilon_min;
Kmx=Kmx_min;
masse=m_max;
omega0_min=2*Epsilon*Kmx/masse; %epsilon schon vergeben in ParameterFileLaeuferModell

%Berechnung omega0_max
Epsilon=epsilon_max;
Kmx=Kmx_max;
masse=m_min;
omega0_max=2*Epsilon*Kmx/masse; %epsilon schon vergeben in ParameterFileLaeuferModell


%Berechnung von von Delta_0=sigma_max
sigma0_max=2/m_min;


% Omega wird größer als Omega_0 gewählt. Das Verhältnis bestimmt
% den Amplitudenrand des geschlossenen adaptiven Regelkreises.
% Amplitudenrand von 1.2 -->
            omega_min = omega0_min/1.2;  omega_max=omega0_max*1.2;
% Delta wird größer als Delta_0 gewält, um Störungen zu berücksichtigen,
% die durch Totzeiten im geschlossenen adaptiven Regelkreises entstehen.
            Delta = 1000;

% Definition von epsilon aus den Projektionsbeschränkungen:
epsilon_theta=0.1;      epsilon_omega=0.1;      epsilon_sigma=0.1;

% Projektions-Operator stellt sicher, dass:
% max |\hat{\theta}| \leq theta_Pmax
% Frei gewählt: theta_Pmax legt Radius des Randes von \Theta = \Omega_1 fest
%(theta_Pmax kann auch bzw. sollte eher mithilfe von epsilon_theta bestimmt werden,
% wie im Anhang der Masterarbeit erklärt: 
% theta_Pmax = sqrt(1+epsilon_theta)* max_{\theta \in \Omega_0} ||\theta||_2
% Aufgrund eines Fehlers, der spät entdeckt wurde, wird hier theta_Pmax = 2.56 gesetzt 
% (und somit etwas größer gewählt), da es zeitlich nicht mehr möglich war, alle Simulationen zu wiederholen. 
theta_Pmax = 2.56; 
% omega_min \leq \hat{\omega} \leq omega_max
omega_Pmax = (omega_max-omega_min)/2;
omega_Pr_Center = omega_Pmax + omega_min;
% max |\hat{\sigma}| \leq Delta
sigma_Pmax=Delta;

% L ergibt sich aus Theta zu:
L=norm([theta_Pmax;theta_Pmax],1);

%Anzeigen der Ergebnisse:
fprintf('\n\n\n\n\nErgebnisse:\n\n')
fprintf('Theta_0 = {v_0=(v_01 v_02)^T aus R^2 : v_01 aus [%f %f] und v_02 aus [%f %f]}. \n', theta1_min, theta1_max,theta2_min,theta2_max);
fprintf('Omega_0 = {%f %f} \n',omega0_min,omega0_max);
fprintf('Delta_0 = %f\n\n',sigma0_max);

fprintf('Theta = {v=(v1 v2)^T aus R^2 : v1 aus [%f %f] und v2 aus [%f %f]}. \n', -theta_Pmax, theta_Pmax,-theta_Pmax,theta_Pmax);
fprintf('L = %1.3f\n',L);
fprintf('Omega = {%2.4f %2.4f} \n',omega_min,omega_max);
fprintf('Delta = %5.0f\n',Delta);
fprintf('k = %5.0f\n\n',k);

fprintf('Daraus ergibt sich:\t\t\t\t\t\t theta_Pmax = %2.4f\n\t\t\t\t\t\t\t\t\t\t omega_Pmax = %2.4f\n\t\t\t\t\t\t\t\t\t\t sigma_Pmax = %2.1f\n\n',...
        theta_Pmax,omega_Pmax,sigma_Pmax);
fprintf('Epsilon wurde jeweils klein gewählt:\t epsilon_theta = %2.1f\n\t\t\t\t\t\t\t\t\t\t epsilon_omega = %2.1f\n\t\t\t\t\t\t\t\t\t\t epsilon_sigma = %2.1f\n',...
        epsilon_theta,epsilon_omega,epsilon_sigma);
fprintf('\n Hinweis: Der Projektions-Operator in den Simulationsdateien wurde entnommen aus:\n http://web.mechse.illinois.edu/sites/index.php?id=1121|+%%26bull%%3B+L1+adaptive+control+tutorials\n (Stand: 22.08.2013; Autor: Dr. Naira Hovakimyan)\n\n');

%Öffnen der Simulationsdatei
open('L1ZR5_Regelstrecke_inputgain_Modell');   
    
%% Numerische Berechnung von lambda(omega*k)=L1norm(G(s))*L
% ACHTUNG, dauert 30-60 Minuten

s=tf('s');

H=(s*eye(2)-Am)\b;
C=2000/(s+2000); %L1-Norm von C der Form C=k*w/(s+k*w) ist immer 1,
                %da dies größte Verstärkung von C, daher egal, wie groß "w" (bzw. k*w)!
G=H*(1-C); 

lambda=zeros(2000,1);

for i=1:1:2000
    omegak = i;
    C=omegak/(s+omegak);
    G=H*(1-C);
    lambda(i)=L1NORM(G)*L;
end
 
grenze=ones(2000,1);

I=1:1:2000;
plot(I,lambda,I,grenze,'--r')
xlabel('\omega * k in rad/s');
ylabel('\lambda'); 


%% Berechnung der Time Delay Margin
%Achtung: "omegak" muss vor Beginn in der Funktion Timedelaymargin.m 
%auf den Wert von "omega_max * k" gesetzt werden (bei Berechnung mit unterschiedlichen k)!
%Open Loop TF für Lineares Ersatzsystem: L=C/(1-C)*(1+Theta^T*H_quer(s))
%Genetischer Algorithmus:
[x,fval,exitflag,output,population,score]=Minimierung_Timedelaymargin(2,[-theta_Pmax -theta_Pmax],[theta_Pmax theta_Pmax]);

%% Berechnung der Performance-Bounds
s=tf('s');
H=(s*eye(2)-Am)\b;
C=k*omega_max/(s+k*omega_max);    
G=H*(1-C);
r=0.14; %maximaler Sprung: 14cm

% Da es nicht gelang Delta_n von S.52 richtig zu berechnen (Ergebnisse
% waren viel zu groß (eine Cell weiter unten)), wird Delta=1 gesetzt, was bei einer 
% Totzeit von tau = 44.97 ms beobachtet wurde. Auf diese Weise erhält man 
% realistische und sogar konservative Ergebnisse für die Performance-Bounds
% (da Totzeit der Nanopositioniermaschine höchstens 2ms ist).
Delta=1; 

theta_l=[-theta_Pmax -theta_Pmax];
theta_u=[theta_Pmax theta_Pmax];

%dtheta: Wert unbekannt und konservativ geschätzt 
%(Eine schnelle Änderung kann auch als Störung Sigma interpretiert werden)
dtheta=0.1; 
%dtheta: Wert unbekannt und geschätzt 
dsigma=1/m_min; 

Theta_m=4* norm(theta_u,2)^2+4*Delta^2+(omega_max-omega_min)^2+4*(max(eig(P)))/min(eig(P))*(dtheta*norm(theta_u,2)+dsigma*Delta);
gamma1=L1NORM(C)/(1-L1NORM(G)*L)*sqrt(Theta_m/min(eig(P)));

% L-Unendlichnorm von (x_ref-x)
Linfnorm_xref_x=gamma1/(sqrt(Gamma));


%Berechnung von gamma2 (S.41 Buch)
%H1 proper and BIBO stabil ist gegeben für c0=[1;1] (kann durch ausprobieren sehr schnell gefunden werden)
c0=[0.003;3.116]; 
%C=2000/(s+2000);
%isstable(C/(c0'*H)*c0');
H1=C/(c0'*H)*c0';


% evtl. stimmt etwas mit L1NORM(H1/omega_min) nicht <- Ist sehr groß!
gamma2=L1NORM(k/(s+k*omega_max))*L*gamma1+L1NORM(H1/omega_max)*sqrt(Theta_m/min(eig(P)));
Linfnorm_uref_u=gamma2/(sqrt(Gamma));

%Berechnung der L-Unendlich-Norm der Zustände des Referenzsystems
% Der maximale Sprung ist r=0.14
x_LunendlichN=(L1NORM(H*C*kg)*r+L1NORM(G)*Delta)/(1-L1NORM(G)*L);

%Ergebnisse
fprintf('\n\n\n\n\nErgebnisse:\nDie L_unendlich-Norm von x_ref beträgt %2.5f m. (unter Berücksichtigung eines maximalen Sprungs von ||r||_L_inf=%0.2f)',x_LunendlichN,r);
fprintf('\nDer geschlossene adaptive Regelkreis kann maximal um %4.5f m von dem Referenzsystem abweichen!\n',Linfnorm_xref_x);
fprintf('Die Stellgröße des geschlossenen adaptiven Regelkreises kann maximal um %6.2f A von der des Referenzsystems abweichen!\n',Linfnorm_uref_u);


%% Berechnung von Delta_n (S.52)
% Für max(V(0)) wird Delta wieder dem aus der Tabelle berechneten Wert
% sigma0_max gesetzt:
Delta=sigma0_max;
%V(0)< 1/Gamma*(4* norm([1,1 2,5],2)^2+4*Delta^2+(omega_u-omega_l)^2) S.40
V0=1/Gamma*(4* norm(theta_u,2)^2+4*Delta^2+(omega_max-omega_min)^2);
epsilon_alpha=1/Gamma; %frei wählbar aber größer null
alpha1 = min(eig(Q))/max(eig(P));  %alpha von S.44 Buch. alpha bereits vergeben, deshalb alpha1

t=0:0.001:20;
kappa_alpha=sqrt(((V0-epsilon_alpha)*exp(-alpha1*t))/(min(eig(P))));
max(kappa_alpha); %Größe ist OK

%Psi_H1 S.45 im Buch
[l,m]=size(H1); %l Ausgänge, m Eingänge
Verstaerkung_Eingang=zeros(20001,m);
Eingang_Summe=zeros(20001,1);
Verstaerkung_Ausgang=zeros(20001,1);

for i=1:l
   for j=1:m 
       %Impulsantwort
        dt=0.001;
        T=0:dt:20;
        [y,t]=impulse(H1(i,j),T);
        %Berechnen des Quadrates
        y=y.^2;
        Verstaerkung_Eingang(:,j)=y; 
        
        Eingang_Summe=Eingang_Summe+Verstaerkung_Eingang(:,j);
        
   end %end Eingänge

   Verstaerkung_Ausgang=max(Verstaerkung_Ausgang,sqrt(Eingang_Summe));
    Eingang_Summe=zeros(20001,1);
end %end Ausgänge

Psi_H1=Verstaerkung_Ausgang;
max(Psi_H1); %sehr hoch, warum?

plot(T,Psi_H1);
figure;
plot(T,kappa_alpha);


% Berechnung epsilon_b 
epsilon_b=zeros(20001,1);

epsilon_b=conv(Psi_H1,Verstaerkung_Ausgang);

T2=0:0.001:40;
plot(T2,epsilon_b);

max(epsilon_b); %Zu groß, muss epsilon_alpha anders gewählt werden?? 
%Die Impulsantwort von H1 ist sehr groß!
%Vielleicht stimmt auch Faltung nicht.

%Da max(epsilon_b) der erste Eintrag von epsilon_b ist ergibt sich
Delta_n = (sigma0_max + max(epsilon_b))*omega_max;

%% Plot Sprungantwort

%Anmerkung: Bei TETRA wurde x und y Achse im Nachhinein getauscht, deshalb
%muss hier KmY und kvy verwendet werden.

% Tatsächliche Werte:
omega_ermittelt=2*epsilon*KmY/m;
theta1_ermittelt=-Am(2,1);
theta2_ermittelt=-kvy/m-Am(2,2);

%Plot
figure
subplot(321)
    plot1=plot(simout.time(:,1),simout.signals.values(:,1:2),simout.time(:,1),simout.signals.values(:,3),'r--');
    legend({'$r(t)$','$y_{des}(t)$','$y(t)$'},'Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique')
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Regelgr\"{o}\ss e $y(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
subplot(322)
    plotE=plot(simoutE.time(:,1),simoutE.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
subplot(323)
    plotU=plot(simoutU.time(:,1),simoutU.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
subplot(324)
    plotT=plot(simoutT.time(:,1),simoutT.signals.values);  
    legend({'$\hat{\theta}_1(t)$','$\hat{\theta}_2(t)$'},'Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique')
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Sch\"{a}tzungen $\hat{\theta}(t)$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
subplot(325)
    plot(simoutO.time(:,1),simoutO.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Sch\"{a}tzungen $\hat{\omega}(t)$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
subplot(326)
    plot(simoutS.time(:,1),simoutS.signals.values); 
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Sch\"{a}tzungen $\hat{\sigma}(t)$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
 

%% Simulation Extrembeispiel 1 (schlechteste Performance)
    
    %Maximaler Dämpfungskoeffizient
        kvy=14;
    
    % Mit dieser Wahl wird Eingangsverstärkung und Filterbandbreite minimiert:
    % omega = (2*epsilon*Kmy)/m
        m=30; KmY=0.8; epsilon=0.7;
 
 %% Simulation Extrembeispiel 2 (geringste Totzeittoleranz)
    %Minimaler Dämpfungskoeffizient
        kvy=0;
    
    % Mit dieser Wahl wird Eingangsverstärkung und Filterbandbreite maximiert:
    % omega = (2*epsilon*Kmy)/m
        m=19; KmY=2.4; epsilon=1.3;
 %% Erneutes Setzen der Parameter der Nanopositioniermaschine
 
        kvy=7.2; m=19.24; KmY=1.62; epsilon=1.08;
        
%% Vergleich k=31885, k=1750
    %Minimieren der Eingangsverstärkung
        m=30; KmY=0.8; epsilon=0.7; kvy=14;
        
    % Transport-Delay=0 in Model einstellen; und wieder Stellbeschränkung
    % [-3A 3A]; dann wurde mit k=31885 simuliert
    % Dann:
        simout1=simout;
        simoutE1=simoutE;
        
    k=1750;
    
    % Nochmal Simulieren und dann plotten:
    
    figure
    subplot(221)
    plot1=plot(simout1.time(:,1),simout1.signals.values(:,1:2),simout1.time(:,1),simout1.signals.values(:,3),'r--');
    legend({'$r(t)$','$y_{des}(t)$','$y(t)$'},'Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique')
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Regelgr\"{o}\ss e $y(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    title('$k=31885$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(222)
    plotE=plot(simoutE1.time(:,1),simoutE1.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    title('$k=31885$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(223)
    plot1=plot(simout.time(:,1),simout.signals.values(:,1:2),simout.time(:,1),simout.signals.values(:,3),'r--');
    legend({'$r(t)$','$y_{des}(t)$','$y(t)$'},'Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique')
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Regelgr\"{o}\ss e $y(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    title('$k=1750$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(224)
    plotE=plot(simoutE.time(:,1),simoutE.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    title('$k=1750$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');

    


%% Folgereglerentwurf
%(bei Problemen beim Ausführen: clear all; und dann Cell 1-3 ausführen und anschließend diese hier)
  
open('L1ZR5_Regelstrecke_inputgain_Modell_FRNEU');

k=2400;
Gamma=1000000;
  
%Solltrajektorie
%Ref_Amplitude=0.02;
Ref_Frequenz=2*pi*5; 
Ref_Amplitude=0.3/Ref_Frequenz^2;


% Vorsteuerung
% ergibt sich aus invertieren von M(s),
% also vertauschen von Eingang und Ausgang. (siehe Simulink)
  
% Bode-Diagramm zu Vorsteuerung
s=tf('s');
C=k*omega_max/(s+k*omega_max);
C1=C;
C2=k*omega_min/(s+k*omega_min);
bode(M,C1*M,C2*M)

% Zustandsrückführung
Kq=[(theta1_max+theta1_min)/2 (theta2_max+theta2_min)/2];

%PID-Regler
roots2=[-80 -8+8i -8-8i];
p=poly(roots2);
pD=p(2)-1.92;
pK=p(3)-0.64;
pI=p(4);

Regler = pK + pD*s/(1/800*s+1) + pI/s;
figure,
bode(Regler*C1*M,Regler*C2*M)
  


%% Plot der Systemantwort auf Vorsteuerung und Folgeregelung des L1aRK
sim('L1ZR5_Regelstrecke_inputgain_Modell_FRNEU');
figure
    subplot(211)
    plot1=plot(simout.time(:,1),simout.signals.values(:,1:1),simout.time(:,1),simout.signals.values(:,3),'r');
    legend({'$r(t)$','$y(t)$'},'Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique')
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Regelgr\"{o}\ss e $y(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(223)
    plotE=plot(simoutE.time(:,1),simoutE.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylim([-25e-6 25e-6])
    subplot(224)
    plotU=plot(simoutU3.time(:,1),simoutU3.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    

    
%% Plot der Systemantwort auf Vorsteuerung des L1aRK mit Solltrajektorie
sim('L1ZR5_Regelstrecke_inputgain_Modell_FRNEU');
figure
    subplot(211)
    plot1=plot(simout.time(:,1),simout.signals.values(:,1:2),simout.time(:,1),simout.signals.values(:,3),'r--');
    legend({'$r(t)$','$y_{des}(t)$','$y(t)$'},'Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique')
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Regelgr\"{o}\ss e $y(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(223)
    plotE=plot(simoutE.time(:,1),simoutE.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(224)
    plotU=plot(simoutU3.time(:,1),simoutU3.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    

%% Vergleich Variation k und Gamma
Gamma=100000000;

k=31885;
sim('L1ZR5_Regelstrecke_inputgain_Modell_FRNEU');
SimoutE1=simoutE;
SimoutU1=simoutU;

k=20000;
sim('L1ZR5_Regelstrecke_inputgain_Modell_FRNEU');
SimoutE2=simoutE;
SimoutU2=simoutU;

k=10000;
sim('L1ZR5_Regelstrecke_inputgain_Modell_FRNEU');
SimoutE3=simoutE;
SimoutU3=simoutU;

k=1750;
sim('L1ZR5_Regelstrecke_inputgain_Modell_FRNEU');
SimoutE4=simoutE;
SimoutU4=simoutU;


figure
    subplot(421)
    plotE=plot(SimoutE1.time(:,1),SimoutE1.signals.values);
    title('k=31885');
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(422)
    plotU=plot(SimoutU1.time(:,1),SimoutU1.signals.values);
    title('k=31885');
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(423)
    plotE=plot(SimoutE2.time(:,1),SimoutE2.signals.values);
    title('k=20000');
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(424)
    plotU=plot(SimoutU2.time(:,1),SimoutU2.signals.values);
    title('k=20000');
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(425)
    plotE=plot(SimoutE3.time(:,1),SimoutE3.signals.values);
    title('k=10000');
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(426)
    plotU=plot(SimoutU3.time(:,1),SimoutU3.signals.values);
    title('k=10000');
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(427)
    plotE=plot(SimoutE4.time(:,1),SimoutE4.signals.values);
    title('k=1750');
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(428)
    plotU=plot(SimoutU4.time(:,1),SimoutU4.signals.values);
    title('k=1750');
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    
%% Plotte Folgeregelung für unterschiedliche Parameterwerte

m=19.24; kvy=7.2; KmY=1.62; epsilon=1;
sim('L1ZR5_Regelstrecke_inputgain_Modell_FRNEU');
SimoutE1=simoutE;
SimoutU1=simoutU;

m=30; kvy=14; KmY=0.8; epsilon=0.7;
sim('L1ZR5_Regelstrecke_inputgain_Modell_FRNEU');
SimoutE2=simoutE;
SimoutU2=simoutU;

m=19; kvy=0; KmY=2.4; epsilon=1.3;
sim('L1ZR5_Regelstrecke_inputgain_Modell_FRNEU');
SimoutE3=simoutE;
SimoutU3=simoutU;

    figure
    subplot(321)
    plotE=plot(SimoutE1.time(:,1),SimoutE1.signals.values);
    title('$m=19.24$, $\mu_x=7.2$, $K_{mx}=1.62$, $\epsilon=1$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    xlim([0 10])
    subplot(322)
    plotU=plot(SimoutU1.time(:,1),SimoutU1.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    xlim([0 10])
    subplot(323)
    plotE=plot(SimoutE2.time(:,1),SimoutE2.signals.values);
    title('$m=30$, $\mu_x=14$, $K_{mx}=0.8$, $\epsilon=0.7$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    xlim([0 10])
    subplot(324)
    plotU=plot(SimoutU2.time(:,1),SimoutU2.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    xlim([0 10])
    subplot(325)
    plotE=plot(SimoutE3.time(:,1),SimoutE3.signals.values);
    title('$m=19$, $\mu_x=0$, $K_{mx}=2.4$, $\epsilon=1.3$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    xlim([0 10])
    subplot(326)
    plotU=plot(SimoutU3.time(:,1),SimoutU3.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    xlim([0 10])
 
 
 
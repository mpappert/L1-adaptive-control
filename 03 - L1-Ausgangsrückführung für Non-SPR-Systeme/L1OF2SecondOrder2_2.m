%% Setup für die Simulation des L1-Output Feedback Reglers 
%mit dem nominellen LP3S Modell als Regelstrecke
%Zuerst ParameterFileLaeufermodell.m und Preload_Fcn_Computed_Torque_nonlin_GPI.m ausführen 

close all;
clc;

run('C:\Users\Markus\Documents\MATLAB\Masterarbeit\LP3S Simulink Modell Kai Treichel\06_Gesamtmodell LP3S\ParameterFileLaeuferModell.m');
run('C:\Users\Markus\Documents\MATLAB\Masterarbeit\LP3S Simulink Modell Kai Treichel\06_Gesamtmodell LP3S\Preload_Fcn_Computed_Torque_nonlin_GPI.m');
ICy=0;
%% Nominelle Regelstrecke
%durchschnittliche Werte
s=tf('s');
mux=7; %7           [0    14]
km=1.6;%1.6         [0.8  2.4]
Masse=24.5; %24.5     [19   30]
epsilon1=1; %1    [0.7  1.3]
A =[0 1; 0 -((mux)/Masse)];
b=[0;(2*km*epsilon1)/Masse]; 
c=[1 0];
A0=tf(ss(A,b,c,0));
A0n=A0.num{1}*[s^2;s;1];
A0d=A0.den{1}*[s^2;s;1];

% Nominelle Parameter für das Regelstreckenmodell 
 kvy=7; KmY=1.6; m=24.5; epsilon=1; %#ok<*NASGU>
 %mux=kvy;km=KmY;Masse=m;epsilon1=epsilon;
 %kvy=mux;KmY=km;m=Masse;epsilon=epsilon1;
%% L1-Regler: Referenz-Modell und Filter aus Maplefile

Reglerauswahl = 5;

switch Reglerauswahl
    case 1
            %L1-Regler 1: für Modell aus [TAARW13], Pole bei [-2000,-2000,-2000,-1.4907,-0.4293]
            C=3.919739749*10^10/(3.919739749*10^10+1.200980589*10^7*s+6001.634286*s^2+s^3);
            Cn=3.919739749*10^10;
            Cd=(3.919739749*10^10+1.200980589*10^7*s+6001.634286*s^2+s^3);
            M=1/(1.566434557*s^2+3.001699185*s+1);
            Mn=1;
            Md=(1.566434557*s^2+3.001699185*s+1);

    case 2
            %L1-Regler 2: für Modell aus [TAARW13], Pole bei [-1000,-1000,-1000,-1.4907,-0.4293]
            C=4.899674686*10^9/(4.899674686*10^9+3.004903030*10^6*s+3001.634286*s^2+s^3);
            Cn=4.899674686*10^9;
            Cd=(4.899674686*10^9+3.004903030*10^6*s+3001.634286*s^2+s^3);
            M=1/(1.570265776*s^2+3.003199184*s+1);
            Mn=1;
            Md=(1.570265776*s^2+3.003199184*s+1);
            
    case 3       
            %L1-Regler 3: für Modell aus [TAARW13] und für Folgeregler, H=M; Pole bei [-1000,-1000,-1000,-1.2,-0.1084]
            % Time-Delay-margin im Worst-Case: 2ms
            C=9.959250000*10^8/(9.959250000*10^8+3.003067895*10^6*s+3001.022686*s^2+s^3);
            Cn=9.959250000*10^8;
            Cd=(9.959250000*10^8+3.003067895*10^6*s+3001.022686*s^2+s^3);
            M=1/(7.711159061*s^2+10.06142558*s+1);
            Mn=1;
            Md=(7.711159061*s^2+10.06142558*s+1);
            
    case 4       
            %L1-Regler 4: für Modell aus [TAARW13] und für Folgeregler, C*H=M; Pole bei [-1500, -1000+4000I,-1000-4000I, -0.1,-0.1] 
            C=1.952343750*10^9/(1.952343750*10^9+1.999970003*10^7*s+3499.914286*s^2+s^3);
            Cn=1.952343750*10^9;
            Cd=(1.952343750*10^9+1.999970003*10^7*s+3499.914286*s^2+s^3);
            M=1/(99.99327780*s^2+20.00078431*s+1);
            Mn=1;
            Md=(99.99327780*s^2+20.00078431*s+1);
       
   case 5       
            %L1-Regler 5: für Modell aus [TAARW13] und für Folgeregler, C*H=M; Pole bei [-900, -900,-900, -1.2,-0.10841] 
            % Time-Delay-margin im Worst-Case: 2.2ms
            C=7.260293250*10^8/(7.260293250*10^8+2.432761089*10^6*s+2701.022686*s^2+s^3);
            Cn=7.260293250*10^8;
            Cd=(7.260293250*10^8+2.432761089*10^6*s+2701.022686*s^2+s^3);
            M=1/(7.713778844*s^2+10.06175892*s+1);
            Mn=1;
            Md=(7.713778844*s^2+10.06175892*s+1);         
            
end

% Umwandlung des Referenzmodells in die Regelungsnormalform
[M2]=canon(M,'companion');
[A_Db,b_Db,c_Db]=ssdata(M2);
Am=A_Db';
bm=c_Db';
cm=b_Db';

%% Stückweise stetiges Adaption Law 
%(muss nach Änderung von M oder C in der vorherigen Cell neu ausgeführt werden)
Q=eye(size(Am));
P=lyap(Am',Q);
ST=0.0005; %Sampling Time 0.0005
D1=(cm/sqrtm(P))';
D=null(D1','r')';
Lambda=[cm;D*sqrtm(P)];
einsVektor=zeros(ndims(Am),1);
einsVektor(1,1)=1;
mu1=expm(Lambda*Am/Lambda*ST)*einsVektor; %expm
%syms tau1
% Phi=int(expm(Lambda*Am/Lambda*(ST-tau1))*Lambda,tau1,0,ST); %expm
% Alternativ:
 Phifun = @(tau1) expm(Lambda*Am/Lambda*(ST-tau1))*Lambda;
 Phi = integral(Phifun,0,ST,'ArrayValued',true)
Phi=double(Phi); % Zurückwandeln von symbolic zu Numerischer Form
Gain=-Phi\mu1;
C2=C/M*(cm*(inv(s*eye(size(Am))-Am)));


%% H(s) und G(s) für nominelle Regelstrecke
H=A0n*Mn*Cd/(Cn*A0n*Md+Mn*A0d*(Cd-Cn));
% isstable(H); % Lemma 1
G=H*(1-C);
L1NORM(G)

%Störsensitivität eingangsseitig
Di=(Cd-Cn)*A0n*Mn/((Cd-Cn)*A0d*Mn+A0n*Md*Cn);
%bode(Di);

open('L1OF2_SecondOrder_M2_2'); 
%% Stabilitätscheck mit Karitonov

%Wertebereiche der Parameter
m_min=19;           m_max=30;
mux_min=0;          mux_max=14;
Kmx_min=0.8;        Kmx_max=2.4;
epsilon_min=0.7;    epsilon_max=1.3;
 
s=tf('s');

%Berechnung der minimalen Koeffizienten von Hd
An_min=2*Kmx_min*epsilon_min;
Ad_min=s^2*m_min+mux_min*s;
Hdmin=Cn*An_min*Md+Mn*Ad_min*(Cd-Cn);

HdminV=Hdmin.num{1}; %HdminV(6-i) spricht Xi an

%Berechnung der maximalen Koeffizienten von Hd
An_max=2*Kmx_max*epsilon_max;
Ad_max=s^2*m_max+mux_max*s;
Hdmax=Cn*An_max*Md+Mn*Ad_max*(Cd-Cn);

HdmaxV=Hdmax.num{1}; %HdmaxV(6-i) spricht Yi an

%Koeffizient delta \in [X0, Y0]
%K1 = X0 + X1 s + Y2 s^2 + Y3 s^3 + X4 s^4 + X5 s^5
K1 = HdminV(6-0)+HdminV(6-1)*s+HdmaxV(6-2)*s^2+HdmaxV(6-3)*s^3+HdminV(6-4)*s^4+HdminV(6-5)*s^5;

%K2 = X0 + Y1 s + Y2 s^2 + X3 s^3 + X4 s^4 + Y5 s^5
K2 = HdminV(6-0)+HdmaxV(6-1)*s+HdmaxV(6-2)*s^2+HdminV(6-3)*s^3+HdminV(6-4)*s^4+HdmaxV(6-5)*s^5;

%K3 = Y0 + X1 s + X2 s^2 + Y3 s^3 + Y4 s^4 +X5 s^5 
K3 = HdmaxV(6-0)+HdminV(6-1)*s+HdminV(6-2)*s^2+HdmaxV(6-3)*s^3+HdmaxV(6-4)*s^4+HdminV(6-5)*s^5;

%K4 = Y0 + Y1 s + X2 s^2 + X3 s^3 + Y4 s^4 + Y5 s^5
K4 = HdmaxV(6-0)+HdmaxV(6-1)*s+HdminV(6-2)*s^2+HdminV(6-3)*s^3+HdmaxV(6-4)*s^4+HdmaxV(6-5)*s^5;

if (isstable(1/K1)*isstable(1/K2)*isstable(1/K3)*isstable(1/K4)) == 1
    fprintf('\n\nDas Karitonov Theorem wurde erfüllt und Hd ist stabil für alle Parameter aus den angegeben Wertebereichen!\n\n');

else
    fprintf('\n\nDas Karitonov Theorem wurde NICHT erfüllt! Hd kann für gewisse Parameter instabil werden! M(s) und C(s) müssen neu entworfen werden!\n\n');
end


%% Berechnung der Performance-Schranken
% Kann nur genau berechnet werden, wenn A(s) bekannt ist.
% Eine Berechnung mit A0(s) (durchschnittliche Parameter) bietet jedoch
% gute Annäherung, wenn Bandbreite von C(s) groß ist (im Vergleich zu A(s) und M(s)).
% Funktioniert jedoch nicht ganz! Normalerweise soll gamma0quer festgelegt werden,
% anschließend soll Abtastzeit Ts so gewählt werden, dass gamma0 < gamma0quer.
% Wie geht man jedoch vor, wenn Abtastzeit schon festliegt: 500 us
% Es findet sich kein gamma0quer, für das gamma0 < gamma0quer erfüllt ist.
% Grund: Definition von gamma0 ist rekursiv und beinhaltet Delta, das
% wesentlich durch gamma0quer bestimmt wird.


Hd=(Cn*A0n*Md+Mn*A0d*(Cd-Cn));
Hn=A0n*Mn*Cd;
H=Hn/Hd;
G=H*(1-C);

r_LU=0.01; %L_unendlichNorm von r(t)
L=350;
L0=0.3;
gamma0quer=0.000001;

H0=Cd*A0n*Md/Hd;
H1=(Cn*A0n*Md-Cn*A0d*Mn)/Hd;
H2=(Hn*Cn*Md)/(Mn*Hd*Cd);

rho_r=(L1NORM(H*C)*r_LU+L1NORM(G)*L0)/((1-L1NORM(G)*L));

Delta=L1NORM(H1)*r_LU+L1NORM(H0)*(L*rho_r+L0)+(L1NORM(H1/M)+L1NORM(H0)*L1NORM(H2)*L/(1-L1NORM(G)*L))*gamma0quer;

beta1=1;
beta2=[1 0]*expm(Lambda*Am/Lambda*ST)*[0;1];
beta3=int(abs(einsVektor'*expm(Lambda*Am/Lambda*(ST-tau1))*Lambda/Phi*expm(Lambda*Am/Lambda*ST)*einsVektor),tau1,0,ST);
beta3=double(beta3);
beta4=int(abs(einsVektor'*expm(Lambda*Am/Lambda*(ST-tau1))*Lambda*bm),tau1,0,ST);
beta4=double(beta4);
kappa=beta4;

P2=[0 1]*(sqrtm(P)/Lambda)'*(sqrtm(P)/Lambda)*[0;1];
alpha1=max(eig(Lambda'\P/Lambda))*((2*Delta*norm(Lambda'\P*bm,2))/(min(eig(Lambda'\Q/Lambda)))).^2;
sigma1=norm([1 0]*expm(Lambda*Am/Lambda*ST)*[0;1],2)*sqrt(alpha1/(max(eig(P2))))+kappa*Delta;
gamma0=beta1*sigma1+beta2*sqrt(alpha1/(max(eig(P2))))+beta3*sigma1+beta4*Delta;

gamma1 = (L1NORM(H2))/(1-L1NORM(G)*L)*gamma0;
%gamma2 = L1NORM(H2)*L*gamma1+L1NORM(H3/M)*gamma0quer;

fprintf('\n\nDer maximale Abstand zwischen y_ref und y ist kleiner als %f m\n\n',gamma1);


%% Berechnung maximale L1Norm(G)
[x,fval,exitflag,output,population,score] = MaximiereL1NG_Opt(4,[0; 0.8; 19; 0.7],[14; 2.4; 30; 1.3]);


%% Simulationsergebnisse

% Nominelle Parameter für das Regelstreckenmodell 
 kvy=7; KmY=1.6; m=24.5; epsilon=1;
 
% 1)Regler 1 aktivieren (dazu Cell 3 (mit Reglerauswahl=1) und Cell 4 ausführen);
% 2)Totzeit tau=0 einstellen
% --> Simulieren
 open('L1OF2_SecondOrder_M2_2');

figure
    subplot(221)
    plot1=plot(simout.time(:,1),simout.signals.values(:,1:2),simout.time(:,1),simout.signals.values(:,3),'r--');
    legend({'$r(t)$','$y_{ref}(t)$','$y(t)$'},'Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique')
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Regelgr\"{o}\ss e $y(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    title('Regler 1 mit $A(s)=A_0(s)$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(222)
    plotE=plot(simoutE.time(:,1),simoutE.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(223)
    plotU=plot(simoutU.time(:,1),simoutU.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(224)
    plotS=plot(simoutS.time(:,1),simoutS.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Sch\"{a}tzung $\hat\sigma(t)$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    legend({'$\sigma_1(t)$','$\sigma_2(t)$'},'Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique')
   
% Regler 2 aktivieren (Cell 3 und 4 ausführen);
% --> Simulieren
figure
    subplot(221)
    plot1=plot(simout.time(:,1),simout.signals.values(:,1:2),simout.time(:,1),simout.signals.values(:,3),'r--');
    legend({'$r(t)$','$y_{ref}(t)$','$y(t)$'},'Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique')
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Regelgr\"{o}\ss e $y(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    title('Regler 2 mit $A(s)=A_0(s)$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(222)
    plotE=plot(simoutE.time(:,1),simoutE.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(223)
    plotU=plot(simoutU.time(:,1),simoutU.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(224)
    plotS=plot(simoutS.time(:,1),simoutS.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Sch\"{a}tzung $\hat\sigma(t)$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    legend({'$\sigma_1(t)$','$\sigma_2(t)$'},'Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique')
    
    
    % Regler 1: tau= 2ms; 
    kvy=14; KmY=2.4; m=19; epsilon=1.3;
    
figure
    subplot(221)
    plot1=plot(simout.time(:,1),simout.signals.values(:,1:2),simout.time(:,1),simout.signals.values(:,3),'r--');
    legend({'$r(t)$','$y_{ref}(t)$','$y(t)$'},'Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique')
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Regelgr\"{o}\ss e $y(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    title('Regler 1 mit $\tau=4.1ms$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(222)
    plotE=plot(simoutE.time(:,1),simoutE.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(223)
    plotU=plot(simoutU.time(:,1),simoutU.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(224)
    plotS=plot(simoutS.time(:,1),simoutS.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Sch\"{a}tzung $\hat\sigma(t)$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    legend({'$\sigma_1(t)$','$\sigma_2(t)$'},'Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique')
   
    
    % Regler 2: tau= 2ms; kvy=14; KmY=2.4; m=19; epsilon=1.3;
    figure
    subplot(221)
    plot1=plot(simout.time(:,1),simout.signals.values(:,1:2),simout.time(:,1),simout.signals.values(:,3),'r--');
    legend({'$r(t)$','$y_{ref}(t)$','$y(t)$'},'Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique')
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Regelgr\"{o}\ss e $y(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    title('Regler 2 mit $\tau=4.1ms$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(222)
    plotE=plot(simoutE.time(:,1),simoutE.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(223)
    plotU=plot(simoutU.time(:,1),simoutU.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(224)
    plotS=plot(simoutS.time(:,1),simoutS.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Sch\"{a}tzung $\hat\sigma(t)$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    legend({'$\sigma_1(t)$','$\sigma_2(t)$'},'Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique')
    
    
    
%% Folgeregler

    Ref_Frequenz=2*pi*0.33; 
    Ref_Amplitude=0.3/Ref_Frequenz^2;

    %PID-Regler
    
    a2=M.den{1}(1);
    a1=M.den{1}(2);
    a0=M.den{1}(3);
    
    roots2=[-100 -10+0i -12-0i]; 
    p=poly(roots2);
    pD=p(2)-a1/a2;
    pK=p(3)-a0/a2;
    pI=p(4);
    
    Ti=pK/pI;
    Td=pD/pK;
    Tt=0.0667;

    Regler = pK + pD*s/(1/800*s+1) + pI/s;

    invCden=(s+12556)^3; %12556rad/s = 2000Hz    
    C_inv=((C.den{1}/C.den{1}(4))*[s^3;s^2;s;1]) / ((invCden.num{1}/invCden.num{1}(4))*[s^3;s^2;s;1]);
    
   open('L1OF2_SecondOrder_M2_2_FRNEU');
   
 
 %Bode-Plot von C*Regler*H_min und C*Regler*H_max
 mux=14; Masse=30; km=0.8; epsilon1=0.7;
 A0n=2*km*epsilon1/Masse;
 A0d=s*(s+mux/Masse);
 H_min=A0n*Mn*Cd/(Cn*A0n*Md+Mn*A0d*(Cd-Cn)); 
 
 mux=0; Masse=19; km=2.4; epsilon1=1.3;
 A0n=2*km*epsilon1/Masse;
 A0d=s*(s+mux/Masse);
 H_max=A0n*Mn*Cd/(Cn*A0n*Md+Mn*A0d*(Cd-Cn)); 

 bode(C*Regler*H_min,C*Regler*H_max)

   
%% Plot der Systemantwort auf Vorsteuerung des L1aRK mit Solltrajektorie

figure
    subplot(211)
    plot1=plot(simout.time(:,1),simout.signals.values(:,1),simout.time(:,1),simout.signals.values(:,2),'r--');
    legend({'$r(t)$','$y(t)$'},'Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique')
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Regelgr\"{o}\ss e $y(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(223)
    plotE=plot(simoutE.time(:,1),simoutE.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(224)
    plotU=plot(simoutU.time(:,1),simoutU.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    
%%
kvy=14; m=30; KmY=0.8; epsilon=0.7;
% 1) Rampe aktivieren und simulieren
simoutHminRE=simoutE;
simoutHminRU=simoutU;
% 2) Cosinus aktivieren und simulieren
simoutHminCE=simoutE;
simoutHminCU=simoutU;

kvy=0; m=19; KmY=2.4; epsilon=1.3;
% 3) Rampe aktivieren und simulieren
simoutHmaxRE=simoutE;
simoutHmaxRU=simoutU;
% 4) Cosinus aktivieren und simulieren
simoutHmaxCE=simoutE;
simoutHmaxCU=simoutU;

%5) Plot erzeugen
    figure
    subplot(421)
    plotE=plot(simoutHminRE.time(:,1),simoutHminRE.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(422)
    plotU=plot(simoutHminRU.time(:,1),simoutHminRU.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
     subplot(423)
    plotE=plot(simoutHminCE.time(:,1),simoutHminCE.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(424)
    plotU=plot(simoutHminCU.time(:,1),simoutHminCU.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
     subplot(425)
    plotE=plot(simoutHmaxRE.time(:,1),simoutHmaxRE.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(426)
    plotU=plot(simoutHmaxRU.time(:,1),simoutHmaxRU.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
     subplot(427)
    plotE=plot(simoutHmaxCE.time(:,1),simoutHmaxCE.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Fehler $e(t)$ in $[m]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    subplot(428)
    plotU=plot(simoutHmaxCU.time(:,1),simoutHmaxCU.signals.values);
    xlabel('Zeit $t$ in $[s]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    ylabel('Stellgr\"{o}\ss e $u(t)$ in $[A]$','Interpreter','LaTeX','FontSize',14,'fontWeight','light','fontAngle','oblique');
    



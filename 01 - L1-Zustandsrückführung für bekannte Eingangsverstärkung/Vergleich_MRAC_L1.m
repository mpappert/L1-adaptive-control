%Vergleich zwischen MRAC und L1-Regler

%Regelstrecke
%Pole bei s1=-0.1 ; s2=-0.4
A=[0,1;-0.4,-0.5];
b=[0;1];
c=[1,0];

%Regelstreckenmodellunsicherheiten sind theta_1,2=+-1
% Es wird konservativ gewählt, um den L1-Regler zu testen: \theta_Pmax=2
% Daraus ergibt sich:
L=4;
%Unsicherheiten theta (hier ändern, für Simulation unterschiedlicher Unsicherheiten)
theta=[-1,-1];

% Referenzmodell
Am=[0,1;-2,-2];
bm=[0;1];
cm=[1,0];
M=ss(Am,bm,cm,0);

%Zustandsrückführung
km=[1.6,1.5];
kg=-1/(c*inv(Am)*b);


%Lyapunovgleichung
P=lyap(Am',eye(2));
Pb=P*b;

%L1 adaptiver Regler
Gamma1 = 100000;
omega = 40; %Lambda ungefähr 0.214

%MRAC
Gamma2 = 10;  
%Pole der unbekannten Dynamik (2x): -50 +0I -> Gamma = 81 und höher -> instabil


%% Berechnung von Lambda (Achtung dauert 1-2 min)
s=tf('s');
C=omega/(s+omega);
H=inv(s*eye(2)-Am)*b;
G=H*(1-C);
lambda=L1NORM(G)*L;
run('lambda_plot')


%% Plot der Parameterverläufe MRAC

theta=[-1,-1];
sim('Vergleich_MRAC')
X1=simout.time;
YMatrix1=squeeze(simout.signals.values);

theta=[1,1];
sim('Vergleich_MRAC')
X2=simout.time;
YMatrix2=squeeze(simout.signals.values);

theta=[-1,1];
sim('Vergleich_MRAC')
X3=simout.time;
YMatrix3=squeeze(simout.signals.values);

theta=[1,-1];
sim('Vergleich_MRAC')
X4=simout.time;
YMatrix4=squeeze(simout.signals.values);

createfigure1(X1, YMatrix1, X2, YMatrix2, X3, YMatrix3, X4, YMatrix4)

%% Plot der Parameterverläufe L1

theta=[-1,-1];
sim('Vergleich_L1')
X1=simout.time;
YMatrix1=simout.signals.values;

theta=[1,1];
sim('Vergleich_L1')
X2=simout.time;
YMatrix2=simout.signals.values;

theta=[-1,1];
sim('Vergleich_L1')
X3=simout.time;
YMatrix3=simout.signals.values;

theta=[1,-1];
sim('Vergleich_L1')
X4=simout.time;
YMatrix4=simout.signals.values;

createfigure1(X1, YMatrix1, X2, YMatrix2, X3, YMatrix3, X4, YMatrix4)







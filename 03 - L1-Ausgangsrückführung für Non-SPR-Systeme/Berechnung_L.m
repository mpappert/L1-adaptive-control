%% Laden der Simulationsdaten aus Berechnung_L_Model.mdl und Vorberechnungen
clear all
load('BerechnungL.mat')

Fx_plus =     Fx_plus_S.signals.values;                     %FpyLookUp1;
x1_plus =     Fx_plus_S.time;                               %yLookUp1;

Fx_minus =    Fx_minus_S.signals.values;                     %FpyLookUp2;
x1_minus =    Fx_minus_S.time;                                %yLookUp2;

figure
plot(x1_plus,Fx_plus,x1_minus,Fx_minus)

dFx_plus=diff(Fx_plus)./diff(x1_plus);
dFx_plus=[dFx_plus; dFx_plus(end)];
dFx_minus=diff(Fx_minus)./diff(x1_minus);
dFx_minus=[dFx_minus; dFx_minus(end)];

figure
plot(x1_plus,dFx_plus,x1_minus,dFx_minus)

% Geschwindigkeit x2
x2=linspace(-1,1,length(x1_plus))';

% [DFx_plus,DFx_minus]=meshgrid(dFx_plus,dFx_minus);
% [FFx_plus,FFx_minus]=meshgrid(Fx_plus,Fx_minus);
%% Berechnung der Jacobimatrix von Fx und Berechnung der Norm für dx/dt=0 (um Maximum zu erhalten)
%x1 = x
%x2 = dx/dt

%J1 = dF/dx1
J1=1/2*(tanh(1000*x2)+1).*dFx_plus-1/2*(tanh(1000*x2)-1).*dFx_minus;

%J2=dF/dx2
%J2=1/2*(1-tanh(1000*x2).^2)*1000.*Fx_plus-1/2*(1-tanh(1000*x2).^2)*1000.*Fx_minus;

%J2=dF/dx2 für x2 = 0, da dort Maximum von d(tanh(x))/dx
J2=1/2*ones(length(x2),1)*1000.*Fx_plus-1/2*ones(length(x2),1)*1000.*Fx_minus; 


Norm1J=abs(J1)+abs(J2);

figure,plot(x2,abs(J1),x2,abs(J2),x2,abs(J1)+abs(J2))
leg1=legend('J1 = dF/dx1','J2=dF/dx2','abs(J1)+abs(J2)','Location','NorthWest');

MaximumI=find(Norm1J==max(Norm1J));
hold on
plot(x2(MaximumI),Norm1J(MaximumI),'k+')
text(x2(MaximumI),Norm1J(MaximumI),'317.1441 \rightarrow',...
     'HorizontalAlignment','right')



plot(xref,yref,'r',x_real,y_real,'b--')
legend('saída de referência','saída real medida')
figure
plot(t,ep(:,1),'r',t,ep(:,2),'b--',t,ep(:,3),'k--')
legend('Erro x','Erro y','Erro teta')
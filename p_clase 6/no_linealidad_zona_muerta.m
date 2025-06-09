% Ejemplo de no-linealidad-zona muerta
ue=-2.5:.1:2.5; N=length(ue); uo=zeros(1,N);
ZM=.5;
for ii=1:N
    if abs(ue(ii))>ZM
    uo(ii)=ue(ii)-ZM*sign(ue(ii));
    end
end
plot(ue,uo,'k');xlabel('Tensión de entrada');
ylabel('V_o','Rotation',0);grid on;
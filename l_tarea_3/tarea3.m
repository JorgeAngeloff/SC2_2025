clc;clear all; close all;
m=2; 
b=0.3;%coeficiente de rozamiento
l=1;
G=10;
delta=135;%angulo de referencia en grados
p=-4; %Triple

[A,B,C,D]=linmod('pendulo_mod_tarea',delta*pi/180)
eig(A)
rank(ctrb(A,B))
%%
Aa=[[A;C] zeros(3,1)]
Ba=[B;0]
eig(Aa)
rank(ctrb(Aa,Ba))
%%
K=acker(Aa,Ba,[p p p])
k1=K(1)
k2=K(2)
k3=K(3)
eig(Aa-Ba*K) % polos lazo cerrado
tscalc=7.5/(-p) % tiempo de respuesta calculado

%%

sim('pendulo_pid_tarea')
figure(1), plot(tout,yout)
grid on, title('Salida')
figure(2), plot(yout,velocidad) %plano de fase
grid on, title('Plano de fases')
figure(3), plot(tout,torque) % torque total
grid on, title('Torque')
figure(4), plot(tout,-accint) % acción integral
grid on, title('Accion integral')
ymax=max(yout) % máximo valor de salida
S=(ymax-delta)/delta*100 % sobrepaso en %
erel=(delta-yout)/delta; %error relativo
efinal=erel(end) % error final, debe ser cero
ind=find(abs(erel)>.02); % índice elementos con error relativo absoluto menor a 2%
tss=tout(ind(end)) % tiempo de establecimiento (ultimo valor del vector)
yte=yout(ind(end)) % salida al tiempo ts
uf=torque(end) % torque final
Intf=-accint(end) % acción integral final
%%
m_nom = 2;
variaciones_masa = [0.9, 1.0, 1.1];
colores = ['r', 'g', 'b']; % colores para las 3 curvas
labels = {'0.9 m', '1.0 m', '1.1 m'};

figure(10); hold on; title('Salida \theta(t)')
figure(11); hold on; title('Plano de fases ')
figure(12); hold on; title('Torque aplicado')
figure(13); hold on; title('Acción integral')

for i = 1:3
    m = variaciones_masa(i) * m_nom;
    sim('pendulo_pid_tarea');
    
    figure(10)
    plot(tout, yout, 'DisplayName', labels{i})
    
    figure(11)
    plot(yout, velocidad, 'DisplayName', labels{i})
    
    figure(12)
    plot(tout, torque, 'DisplayName', labels{i})
    
    figure(13)
    plot(tout, -accint, 'DisplayName', labels{i})
end

for f = 10:13
    figure(f)
    legend
    xlabel('Tiempo [s]')
    grid on
end

%%
% para analizar robustez
% Guardar resultados para diferentes masas
m_nom = 2;
variaciones_masa = [0.9, 1.0, 1.1];
resultados = zeros(3, 4); % Columnas: sobrepaso, tss, uf, Intf

for i = 1:3
    m = variaciones_masa(i) * m_nom;
    sim('pendulo_pid_tarea')

    ymax = max(yout);
    S = (ymax - delta)/delta * 100;
    erel = (delta - yout)/delta;
    ind = find(abs(erel) > 0.02);
    tss = tout(ind(end));
    uf = torque(end);
    Intf = -accint(end);

    resultados(i,:) = [S, tss, uf, Intf];
end

% Mostrar tabla
fprintf('\nTabla de robustez (masa variable):\n');
fprintf('Masa\t\tSobrepaso (%%)\tTs (s)\t\tUf\t\tIntf\n');
for i = 1:3
    fprintf('%.2f\t\t%.2f\t\t\t%.2f\t\t%.2f\t\t%.2f\n', ...
        variaciones_masa(i)*m_nom, resultados(i,1), resultados(i,2), resultados(i,3), resultados(i,4));
end
%%
m_nom = 2;
variaciones_masa = [0.9, 1.0, 1.1];
colores = ['r', 'g', 'b']; % colores para las 3 curvas
labels = {'0.9 m', '1.0 m', '1.1 m'};

figure(10); hold on; title('Respuesta angular \theta(t)')
figure(11); hold on; title('Plano de fases (\theta, \dot{\theta})')
figure(12); hold on; title('Torque aplicado')
figure(13); hold on; title('Acción integral acumulada')

for i = 1:3
    m = variaciones_masa(i) * m_nom;
    sim('pendulo_pid_tarea');
    
    figure(10)
    plot(tout, yout, 'DisplayName', labels{i})
    
    figure(11)
    plot(yout, velocidad, 'DisplayName', labels{i})
    
    figure(12)
    plot(tout, torque, 'DisplayName', labels{i})
    
    figure(13)
    plot(tout, -accint, 'DisplayName', labels{i})
end

for f = 10:13
    figure(f)
    legend
    xlabel('Tiempo [s]')
    grid on
end

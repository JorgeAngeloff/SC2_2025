clear all; clc;
syms fi fi_p fi_pp p p_p p_pp M m u long Fricc g;

disp('Para el equilibrio ESTABLE')
ang_inic = pi;  % Punto de equilibrio en φ = π


p_pp = (1/(M+m))*(u + m*long*fi_pp + m*long*fi_p^2*(pi-fi) - Fricc*p_p); %(π-fi) en lugar de fi

% Con φ ≈ π
fi_pp = solve(fi_pp == (1/long)*(g*(pi-fi) + p_pp), fi_pp); %g(π-φ) y signo +p_pp
disp('fi_pp='); pretty(simplify(fi_pp));

% Sustitución
p_pp = subs(p_pp, 'fi_pp', fi_pp);
disp('p_pp='); pretty(simplify(p_pp));

% Construcción de la matriz A (misma estructura pero evaluada en φ = π)
Mat_A = [[0 1 0 0];
    [subs(subs(subs(subs(diff(p_pp, p), p,0), p_p,0), fi, ang_inic), fi_p,0), ...
     subs(subs(subs(subs(diff(p_pp, p_p), p,0), p_p,0), fi, ang_inic), fi_p,0), ...
     subs(subs(subs(subs(diff(p_pp, fi), p,0), p_p,0), fi, ang_inic), fi_p,0), ...
     subs(subs(subs(subs(diff(p_pp, fi_p), p,0), p_p,0), fi, ang_inic), fi_p,0)];
    [0 0 0 1];
    [subs(subs(subs(subs(diff(fi_pp, p), p,0), p_p,0), fi, ang_inic), fi_p,0), ...
     subs(subs(subs(subs(diff(fi_pp, p_p), p,0), p_p,0), fi, ang_inic), fi_p,0), ...
     subs(subs(subs(subs(diff(fi_pp, fi), p,0), p_p,0), fi, ang_inic), fi_p,0), ...
     subs(subs(subs(subs(diff(fi_pp, fi_p), p,0), p_p,0), fi, ang_inic), fi_p,0)]];

% Construcción de la matriz B
Mat_B = [0;
    subs(subs(subs(subs(diff(p_pp, u), p,0), p_p,0), fi, ang_inic), fi_p,0);
    0;
    subs(subs(subs(subs(diff(fi_pp, u), p,0), p_p,0), fi, ang_inic), fi_p,0)];

% Mostrar resultados
disp('Matriz A para equilibrio estable:');
pretty(simplify(Mat_A))
disp('Matriz B para equilibrio estable:');
pretty(simplify(Mat_B))
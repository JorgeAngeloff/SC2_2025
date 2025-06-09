function Xn = modmotor(t_etapa, Xant, accion)
    % Simula el modelo del motor con integración explícita (Euler)
    % Xant: [ia; wr; theta]
    % accion: [Va; TL]

    % Parámetros reales
    Ra=2.27;
    Laa=0.0047;
    Ki=0.25;
    Kb=0.25;
    Bm=0.00131;
    Jm=0.00233;

    h = 0.0004247701268049;  % Paso de integración

    ia = Xant(1); 
    wr = Xant(2); 
    tita = Xant(3);

    Va = accion(1); 
    TL = accion(2);

    pasos = round(t_etapa / h);

    for i = 1:pasos
        ia_p = -(Ra / Laa) * ia - (Kb / Laa) * wr + (1 / Laa) * Va;
        wr_p = (Ki / Jm) * ia - (Bm / Jm) * wr - (1 / Jm) * TL;

        ia = ia + ia_p * h;
        wr = wr + wr_p * h;
        tita = tita + wr * h;
    end

    Xn = [ia; wr; tita];
end


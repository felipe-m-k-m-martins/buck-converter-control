% buck_cascade_design.m
% Controle em cascata de um conversor buck (modelo medio):
% malha interna de corrente e malha externa de tensao, ambas com PI.
% Analises de Bode, margens, Nyquist e lugar das raizes, respostas ao degrau
% e simulacao no tempo (degrau e rampa) com saturacao do duty e anti-windup.

clear; close all; clc;

% Parametros do conversor e da simulacao
Vin_nom   = 24;
Vout_ref  = 12;
Rload     = 1.0;
L         = 220e-6;
C         = 470e-6;
ESR       = 0.05;
fsw       = 20e3;
dt        = 1e-4;
tfinal    = 0.12;
duty_min  = 0.0;
duty_max  = 0.95;
% Cruzamento da malha de corrente em fsw/10 e da malha de tensao 5 vezes abaixo
inner_crossover_factor = 1/10;
outer_crossover_factor = inner_crossover_factor / 5;
ramp_time = 0.03;
simulate_step = true;
simulate_ramp = true;
freqRange = {2*pi*1, 2*pi*1e6};

% Grandezas derivadas
w0 = 1/sqrt(L*C);
Q  = Rload * sqrt(C/L);
s = tf('s');

% Plantas do modelo medio: duty -> tensao (2a ordem) e duty -> corrente (aprox. integrador)
Gvd = Vin_nom * (1/(L*C)) / ( s^2 + (1/(Rload*C))*s + 1/(L*C) );

Gid = (Vin_nom)/(L*s);

fprintf('Plantas criadas: Gvd e Gid\n');

% Projeto dos PI com pidtune; ganhos fixos se pidtune falhar
wc_i = 2*pi * (fsw * inner_crossover_factor);
wc_v = 2*pi * (fsw * outer_crossover_factor);

use_pidtune = true;
try
    Ci = pidtune(Gid,'PI',wc_i);
    fprintf('Ci projetado com pidtune.\n');
catch
    use_pidtune = false;
    fprintf('pidtune falhou para Ci. Usando gains manuais.\n');
end

try
    Cv = pidtune(Gvd,'PI',wc_v);
    fprintf('Cv projetado com pidtune.\n');
catch
    use_pidtune = false;
    fprintf('pidtune falhou para Cv. Usando gains manuais.\n');
end

if ~use_pidtune
    Kp_i = 5;    Ki_i = 1000;
    Ci = Kp_i + Ki_i/s;
    Kp_v = 0.05; Ki_v = 10;
    Cv = Kp_v + Ki_v/s;
else
    if isa(Ci,'pid')
        Kp_i = Ci.Kp;
        Ki_i = Ci.Ki;
    else
        Kp_i = 5; Ki_i = 1000;
    end
    if isa(Cv,'pid')
        Kp_v = Cv.Kp;
        Ki_v = Cv.Ki;
    else
        Kp_v = 0.05; Ki_v = 10;
    end
end

fprintf('Gains obtidos: Inner: Kp_i=%.4g, Ki_i=%.4g | Outer: Kp_v=%.4g, Ki_v=%.4g\n',Kp_i,Ki_i,Kp_v,Ki_v);

% Malhas aberta e fechada
L_inner = Ci * Gid;
L_outer = Cv * Gvd;
CL_inner = feedback(L_inner,1);
CL_outer = feedback(L_outer,1);

% Analise em frequencia
figure('Name','Bode: Plants and Loops','NumberTitle','off','Position',[50 50 1200 700]);
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
nexttile; bode(Gid, freqRange); grid on; title('G_{id} (duty -> i_L)');
nexttile; bode(Gvd, freqRange); grid on; title('G_{vd} (duty -> V_{out})');
nexttile; bode(L_inner, freqRange); grid on; title('L_{inner} = C_i * G_{id}');
nexttile; bode(L_outer, freqRange); grid on; title('L_{outer} = C_v * G_{vd}');
nexttile; bode(CL_inner, freqRange); grid on; title('Closed-loop inner (i)');
nexttile; bode(CL_outer*Vout_ref, freqRange); grid on; title('Closed-loop outer (v)');

[GM_i,PM_i,Wcg_i,Wcp_i] = margin(L_inner);
[GM_v,PM_v,Wcg_v,Wcp_v] = margin(L_outer);
fprintf('\nMargins:\nInner loop -> PM = %.2f deg, GM = %.3g (%.2f dB), Wcp = %.2g rad/s\n',PM_i,GM_i,20*log10(GM_i),Wcp_i);
fprintf('Outer loop -> PM = %.2f deg, GM = %.3g (%.2f dB), Wcp = %.2g rad/s\n',PM_v,GM_v,20*log10(GM_v),Wcp_v);

figure('Name','Nyquist: Inner and Outer','NumberTitle','off','Position',[200 100 1100 450]);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile; nyquist(L_inner); grid on; title('Nyquist - Inner Loop (Current)');
nexttile; nyquist(L_outer); grid on; title('Nyquist - Outer Loop (Voltage)');

figure('Name','Root Locus - Inner','NumberTitle','off');
rlocus(L_inner); title('Root Locus - Inner Loop (C_i * G_{id})'); grid on;

figure('Name','Root Locus - Outer','NumberTitle','off');
rlocus(L_outer); title('Root Locus - Outer Loop (C_v * G_{vd})'); grid on;

% Respostas ao degrau do modelo linear
figure('Name','Step Responses LTI','NumberTitle','off','Position',[100 100 900 500]);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile; step(CL_inner*1, 0:1e-4:0.02); title('LTI Step - Inner Closed-loop (i)'); grid on;
nexttile; step(CL_outer*Vout_ref, 0:1e-4:0.05); title('LTI Step - Outer Closed-loop (v)'); grid on;

Plant_inner_closed = feedback(Gid*Ci, 1);
CL_total = feedback(Cv * Gvd * feedback(Ci*Gid,1), 1);
figure('Name','Step - System Cascaded LTI','NumberTitle','off');
step(CL_total * Vout_ref, 0:1e-4:0.05); grid on; title('LTI Step - Sistema Cascata (approx)');

% Simulacao no tempo com o modelo medio
t = 0:dt:tfinal;
N = length(t);

if simulate_step
    vref_step = Vout_ref * ones(size(t));
end
if simulate_ramp
    vref_ramp = zeros(size(t));
    for k = 1:N
        if t(k) <= ramp_time
            vref_ramp(k) = (Vout_ref / ramp_time) * t(k);
        else
            vref_ramp(k) = Vout_ref;
        end
    end
end

modes = {};
if simulate_step, modes{end+1} = 'step'; end
if simulate_ramp, modes{end+1} = 'ramp'; end

sim = struct();

for m=1:length(modes)
    mode = modes{m};
    % Estados iniciais e historico
    iL = 0.0; vC = 0.0;
    int_ev = 0.0; int_ei = 0.0;
    duty = 0.0;
    iL_hist = zeros(N,1); vC_hist = zeros(N,1); duty_hist = zeros(N,1);
    iref_hist = zeros(N,1); ev_hist = zeros(N,1); ei_hist = zeros(N,1);
    if strcmp(mode,'step'), vref = vref_step; else vref = vref_ramp; end

    for k = 1:N
        ev = vref(k) - vC;
        int_ev = int_ev + ev * dt;
        iref = Kp_v * ev + Ki_v * int_ev;

        ei = iref - iL;
        int_ei = int_ei + ei * dt;
        duty_unsat = Kp_i * ei + Ki_i * int_ei;

        % Saturacao do duty; o integrador da corrente para de acumular enquanto satura
        duty_sat = min(max(duty_unsat, duty_min), duty_max);
        if duty_sat ~= duty_unsat
            int_ei = int_ei - ei * dt;
            duty = duty_sat;
        else
            duty = duty_unsat;
        end

        % Dinamica media do conversor
        diL = (duty * Vin_nom - vC)/L;
        dvC = (iL - vC/Rload)/C;

        iL = iL + diL * dt;
        vC = vC + dvC * dt;

        iL_hist(k) = iL;
        vC_hist(k) = vC;
        duty_hist(k) = duty;
        iref_hist(k) = iref;
        ev_hist(k) = ev;
        ei_hist(k) = ei;
    end

    sim.(mode).t = t;
    sim.(mode).iL = iL_hist;
    sim.(mode).vC = vC_hist;
    sim.(mode).duty = duty_hist;
    sim.(mode).iref = iref_hist;
    sim.(mode).ev = ev_hist;
    sim.(mode).ei = ei_hist;
end

for m=1:length(modes)
    mode = modes{m};
    % Graficos no tempo
    data = sim.(mode);

    figure('Name',['Time Response - ' upper(mode)],'NumberTitle','off','Position',[150 120 1000 700]);
    tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

    nexttile;
    plot(data.t, data.vC,'LineWidth',1.5); hold on; yline(Vout_ref,'--k','Reference');
    xlabel('Tempo (s)'); ylabel('V_{out} (V)'); title(['V_{out} - ' mode]); grid on;

    nexttile;
    plot(data.t, data.iL,'LineWidth',1.2); hold on; plot(data.t, data.iref,'--','LineWidth',1.2);
    legend('i_L','i_{ref}'); xlabel('Tempo (s)'); ylabel('Corrente (A)'); title('Corrente do indutor'); grid on;

    nexttile;
    plot(data.t, data.duty,'LineWidth',1.2); ylim([-0.05 1.05]);
    xlabel('Tempo (s)'); ylabel('Duty'); title('Ação de controle (duty)'); grid on;

    nexttile;
    plot(data.t, data.ev,'LineWidth',1.1); xlabel('Tempo (s)'); ylabel('Erro V (V)'); title('Erro de tensão'); grid on;

    nexttile;
    plot(data.t, data.ei,'LineWidth',1.1); xlabel('Tempo (s)'); ylabel('Erro I (A)'); title('Erro de corrente'); grid on;

    nexttile;
    plot(data.t, data.vC,'LineWidth',1.2); xlim([0 min(0.05,tfinal)]); hold on; yline(Vout_ref,'--k');
    xlabel('Tempo (s)'); ylabel('Vout (V)'); title('Transient zoom'); grid on;
end

% Resumo
fprintf('\n--- RESUMO FINAL ---\n');
fprintf('L = %.3e H, C = %.3e F, Rload = %.3f Ohm, fsw = %.1f Hz\n', L, C, Rload, fsw);
fprintf('Inner PI: Kp_i = %.4g , Ki_i = %.4g\n', Kp_i, Ki_i);
fprintf('Outer PI: Kp_v = %.4g , Ki_v = %.4g\n', Kp_v, Ki_v);
fprintf('Inner margins: PM = %.2f deg, GM = %.3g (%.2f dB)\n', PM_i, GM_i, 20*log10(GM_i));
fprintf('Outer margins: PM = %.2f deg, GM = %.3g (%.2f dB)\n', PM_v, GM_v, 20*log10(GM_v));

fprintf('Script finalizado.\n');

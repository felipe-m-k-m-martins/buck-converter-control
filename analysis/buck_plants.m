% buck_plants.m
% Plantas do conversor buck no modelo medio: duty -> tensao de saida (Gvd)
% e duty -> corrente no indutor (Gid), com a versao aproximada Vin/(L s).

clear; clc; close all;

% Parametros
Vin = 24;
L   = 330e-6;
C   = 470e-6;
R   = 1.0;

s = tf('s');

% Plantas
Gvd = (Vin/(L*C)) / ( s^2 + (1/(R*C))*s + 1/(L*C) );

Gvc_il = 1/(C*s + 1/R);
Gid = (Vin/(L*s)) - (1/(L*s)) * Gvd;

Gid_approx = Vin/(L*s);

disp('Gvd (duty -> Vout):');
Gvd

disp('Gid (duty -> iL) - exata:');
Gid

disp('Gid approx (Vin/(L s)):');
Gid_approx

% Polos e zeros
poles_Gvd = pole(Gvd);
zeros_Gvd = zero(Gvd);
poles_Gid = pole(Gid);
zeros_Gid = zero(Gid);

fprintf('\nPolos Gvd:\n'); disp(poles_Gvd);
fprintf('Zeros Gvd:\n'); disp(zeros_Gvd);
fprintf('\nPolos Gid:\n'); disp(poles_Gid);
fprintf('Zeros Gid:\n'); disp(zeros_Gid);

% Gid * Gvc_il deve reproduzir Gvd
Gcascade = Gid * Gvc_il;
fprintf('\nNorma do erro entre Gcascade e Gvd (deve ser ~0): %.3e\n', norm(freqresp(Gcascade - Gvd, 2*pi*100)));

% Graficos
figure('Name','Bode - plantas','NumberTitle','off','Position',[100 100 900 700]);
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
nexttile; bode(Gid, {1,1e6}); grid on; title('Bode: G_{id}(s) - duty \rightarrow i_L');
nexttile; bode(Gvd, {1,1e6}); grid on; title('Bode: G_{vd}(s) - duty \rightarrow v_{out}');

figure('Name','Nyquist - plantas','NumberTitle','off','Position',[200 200 900 400]);
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile; nyquist(Gid); title('Nyquist: G_{id}');
nexttile; nyquist(Gvd); title('Nyquist: G_{vd}');

figure('Name','Comparação Gcascade vs Gvd','NumberTitle','off');
bode(Gcascade, Gvd, {1,1e6}); grid on;
legend('G_{cascade} = G_{id}*G_{vc,iL}','G_{vd} (direct)');

if license('test','Symbolic_Toolbox')
    syms s_sym real
    Gvd_sym = (Vin/(L*C)) / (s_sym^2 + (1/(R*C))*s_sym + 1/(L*C));
    pretty(Gvd_sym)
else
    disp('Symbolic Toolbox not available, skipping symbolic pretty print.');
end

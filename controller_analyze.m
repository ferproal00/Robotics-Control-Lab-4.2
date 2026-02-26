clear; clc; close all;

%% ===== CARGAR CSV =====
data = readtable('parsed_data.csv');

pos_x = data.pos_x;
pos_y = data.pos_y;
pos_z = data.pos_z;

des_x = data.des_x;
des_y = data.des_y;
des_z = data.des_z;

err_x = data.err_x;
err_y = data.err_y;
err_z = data.err_z;

v_norm = data.v_norm;

N = length(pos_x);
t = (0:N-1)';   % muestras (si quieres tiempo real, usa dt=0.02)

%% ===== METRICAS =====
rmse_x = sqrt(mean(err_x.^2));
rmse_y = sqrt(mean(err_y.^2));
rmse_z = sqrt(mean(err_z.^2));

rmse_total = sqrt(mean(err_x.^2 + err_y.^2 + err_z.^2));

max_err_total = max(sqrt(err_x.^2 + err_y.^2 + err_z.^2));

fprintf('\n===== RESULTADOS =====\n');
fprintf('RMSE X: %.5f\n', rmse_x);
fprintf('RMSE Y: %.5f\n', rmse_y);
fprintf('RMSE Z: %.5f\n', rmse_z);
fprintf('RMSE TOTAL: %.5f\n', rmse_total);
fprintf('MAX ABSOLUTE TOTAL ERROR TOTAL: %.5f\n', max_err_total);

%% ===== GRAFICA 1: Desired vs Actual =====
figure;
plot(t, pos_x, 'b'); hold on;
plot(t, des_x, 'r--');
title('Posicion X: Real vs Deseada');
legend('Real','Deseada');
grid on;

figure;
plot(t, pos_y, 'b'); hold on;
plot(t, des_y, 'r--');
title('Posicion Y: Real vs Deseada');
legend('Real','Deseada');
grid on;

figure;
plot(t, pos_z, 'b'); hold on;
plot(t, des_z, 'r--');
title('Posicion Z: Real vs Deseada');
legend('Real','Deseada');
grid on;

%% ===== GRAFICA 2: Error =====
figure;
plot(t, err_x); hold on;
plot(t, err_y);
plot(t, err_z);
title('Error por eje');
legend('X','Y','Z');
grid on;

%% ===== GRAFICA 3: Velocidad Comandada =====
figure;
plot(t, v_norm);
title('Magnitud velocidad comandada');
grid on;
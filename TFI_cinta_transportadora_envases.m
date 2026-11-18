%%% -------------- 2.7 Estabilidad absoluta ------------------

close all; clear all; clc

pkg load control

s = tf('s');

ftla = (4.90*10^5)/(s^2+15813*s+487664);

polos = pole(ftla);

pzmap(ftla);

%%%----------------- 2.9 Respuesta temporal-----------------

step(ftla,0.5); grid
title('Respuesta al escalón unitario del sistema sin compensar')
xlabel('Tiempo [s]'); ylabel('Amplitud [V]')

%%%--------------- 4 - Diseño del controlador ------------------

%% Lugar de raices de la FTLA compensada
close all, clear all, clc
pkg load control

s = tf('s');

%% FTLA sin compensar, forma factorizada
FTLA = 4.90e5 / ((s + 15782)*(s + 30.9));

%% PI por cancelacion del polo dominante
Ti = 1/30.9;
PI = (s + 1/Ti)/s;
FTLAc = minreal(PI*FTLA)          % queda 4.90e5 / (s*(s + 15782))

%% Lugar de raices
rlocus(FTLAc)
title('Lugar de raíces de FTLA Compensada')
xlabel('Eje real'); ylabel('Eje imaginario')

%% Punto de diseno y ganancia Kc
te = 0.1; % te <= 100 ms (criterio 2 %)
alpha = 4/te % te = 4/alpha  ->  alpha = 40
s1 = -alpha;

%% Zoom con el punto de diseño
figure
rlocus(FTLAc); sgrid(0, alpha)     % circunferencia wn = 40: cruza el eje real en -40
axis([-100 10 -20 20])
title('Lugar de raíces de FTLA compensada')
xlabel('Eje real'); ylabel('Eje imaginario')

%% Por condición de módulo |G(s)H(s)|=1/K
invKp = abs((s1 + 1/Ti)/s1 * 4.90e5/((s1 + 15782)*(s1 + 30.9)));
Kp = 1/invKp

%% Verificacion de polos de la funcion de transferencia de lazo cerrado
FTLC = 6.30e5 / ((s + 15742)*(s + 40));
pole(FTLC)

%%% --------------- 5 - Simulación ------------------
close all, clear all, clc
pkg load control
s = tf('s');

%% Sistema compensado
FTLA = 4.90e5 / ((s + 15782)*(s + 30.9));
Ti = 1/30.9;
Kp = 1.285;
PI = Kp*(s + 1/Ti)/s;
FTLC = minreal(feedback(PI*FTLA, 1))   % 6.30e5 / ((s + 40)*(s + 15742))
pole(FTLC)% -40 y -15742

%% Escalon de referencia vref = 0,2 m/s
step(0.2*FTLC, 0.3); grid
title('Respuesta al escalón del sistema compensado')
xlabel('Tiempo [s]'); ylabel('Velocidad de la cinta [m/s]')

%% Para obtener el punto exacto del gráfico en donde se llegue al 98% del valor de ref
step(0.2*FTLC, 0.3); grid
hold on
plot(0.098, 0.196, 'r*')                   % t_e (banda del 2 %)
text(0.105, 0.185, 't_e = 98 ms')
plot(0.025, 0.126, 'r*')                   % 63 % del valor final
text(0.032, 0.115, '\tau = 25 ms')
plot([0 0.3], [0.196 0.196], 'r--')        % limite del 2 %
hold off

title('Respuesta al escalón del sistema compensado')
xlabel('Tiempo [s]'); ylabel('Velocidad de la cinta [m/s]')
legend off

%% Error en estado estable
step(0.2*(1 - FTLC), 0.3); grid
title('Error de seguimiento e(t) = v_{ref} - v')
xlabel('Tiempo [s]'); ylabel('Error [m/s]')

%% Accion de control: tension de armadura Va
Gu = feedback(PI, FTLA);          % Uc/Vref = PI/(1 + PI*FTLA)
Va = 2.4*12.3*0.2*Gu;             % Va = Kd*Kref*vref*Gu, escalon de 0,2 m/s
step(Va, 0.3); grid
hold on
plot([0 0.3], [12 12], 'r--')            % limite del driver
text(0.15, 11, 'Límite del driver: 12 V')
plot([0 0.3], [5.88 5.88], 'k:')         % valor de regimen
text(0.15, 5.3, 'Régimen: 5,9 V')
plot(0, 7.59, 'r*')                      % pico inicial
text(0.01, 8.2, 'Pico: 7,6 V')
hold off
axis([0 0.3 0 13])
title('Acción de control: tensión de armadura V_a')
xlabel('Tiempo [s]'); ylabel('Tensión [V]')
legend off

%% Perturbacion: torque de carga del envase (vidrio 1 L)
Ra=5.74; La=3.63e-4; Km=0.0109; Bm=4.67e-7; Jeff=6.86e-7; Kd=2.4; Ks=4.66e-3;
s=tf('s');
D = La*Jeff*s^2+(La*Bm+Ra*Jeff)*s+(Ra*Bm+Km^2);
G = Km/D;
Gd = -(Ra+La*s)/D;
FTLA = Kd*G*Ks;
Kp=1.285; Ti=1/30.9;
PI = Kp*(s+1/Ti)/s;
FTLC = minreal(feedback(PI*FTLA,1));
Gd_lc = minreal(Gd/(1+PI*FTLA));

dt = 0.001; t = 0:dt:0.6;
n0 = 200;                                        % el envase ingresa en t0 = n0*dt = 0,2 s
vr = step(0.2*FTLC, t);                          % arranque: 0 -> 0,2 m/s
vd = step(3.79e-4*0.947e-3*Gd_lc, t(1:end-n0));  % efecto del envase
v  = vr;
v(n0+1:end) = vr(n0+1:end) + vd;                 % superposicion

plot(t, v); grid
hold on
plot([0 0.6], [0.2 0.2], 'k:')
text(0.4, 0.208, 'Referencia: 0,2 m/s')
plot([0.2 0.2], [0 0.22], 'k--')
text(0.21, 0.05, 'Ingreso del envase (t = 0,2 s)')
plot(0.228, 0.1945, 'r*')
text(0.24, 0.18, 'Mínimo: 0,1945 m/s')
hold off
axis([0 0.6 0 0.22])
title('Respuesta ante perturbación de torque (envase de vidrio 1 L)')
xlabel('Tiempo [s]'); ylabel('Velocidad de la cinta [m/s]')

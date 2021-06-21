% Datos del vehículo
m=1831+60;
mu1=223;
Cx=0.23;
Fimp1=15400;
h=0.1;

% Condiciones iniciales
Position_Car_i=[0;0;0];
V_i=0;
betta_i=pi;
theta_i=0;
ts = 0:h:100;
y = zeros(1,length(ts));
y(1) = V_i;
x = zeros(3,length(ts));
x(:,1)=Position_Car_i;

for i=1:(length(ts)-1)       
    k_1 = Fcar(Fimp1, mu1,y(i), m, Cx);
    k_2 = Fcar(Fimp1, mu1,y(i)+0.5*h*k_1, m, Cx);
    k_3 = Fcar(Fimp1, mu1,y(i)+0.5*h*k_2, m, Cx);
    k_4 = Fcar(Fimp1, mu1,y(i)+h*k_3, m, Cx);

    y(i+1) = y(i) + (1/6)*(k_1+2*k_2+2*k_3+k_4)*h;% main equation
    if y(i+1)<-6
        y(i+1)=-6;
    end
    
    theta_i1=theta_i;
    if theta_i1>pi*0.24
        theta_i1=pi*0.24;
    elseif theta_i1<-pi*0.24
        theta_i1=-pi*0.24;
    end
    
    if (betta_i^2-pi^2)<0.001
        Velocity_Car1=[1 0 0;0 1 0;0 0 1]*[0;0;y(i)];
    else
        Velocity_Car1=[cos(betta_i) 0 sin(betta_i);0 1 0;-sin(betta_i) 0 cos(betta_i)]*[0;0;y(i)];
    end
    
    x(:,i+1)=x(:,i)+h.*Velocity_Car1;
    betta_i1=betta_i+0.05*theta_i1*y(i)+0.9*(theta_i1-theta_i)*y(i);
    
    betta_i=betta_i1;
    theta_i=theta_i1;
end
ykm=y*3.6;

%Se elaboran las gráficas con los datos del movimiento
figure()
plot(ts(1:200),ykm(1:200), 'LineWidth',3,'Color',[50,100,255]/255)
hold on
plot([0,4.3],[100,100],'--','LineWidth',2,'Color',[255,154,32]/255)
plot([4.3,4.3],[0,100],'--','LineWidth',2,'Color',[255,154,32]/255)
xlabel('Tiempo desde arranque (s)','FontSize', 20)
ylabel('Velocidad (km/h)','FontSize', 20)
title('Prueba de aceleración corta', 'FontSize', 20)

figure()
plot(ts,ykm, 'LineWidth',3,'Color',[50,100,255]/255)
hold on
plot([0,100],[233,233],'--r','LineWidth',2)
plot([0,4.4],[100,100],'--k','LineWidth',2)
plot([4.4,4.4],[0,100],'--k','LineWidth',2)
xlabel('Tiempo desde arranque (s)','FontSize', 20)
ylabel('Velocidad (km/h)','FontSize', 20)
legend('Evolución temporal de la velocidad', 'Velocidad máxima','Location','southeast', 'FontSize', 18)
title('Prueba de aceleración larga', 'FontSize', 20)

figure()
plot(ts(1:151),x(3,1:151), 'LineWidth',3,'Color',[50,100,255]/255)
hold on
plot([0,12.5],[402.34,402.34],'--','LineWidth',2,'Color',[255,154,32]/255)
plot([12.5,12.5],[0,402.34],'--','LineWidth',2,'Color',[255,154,32]/255)
xlabel('Tiempo desde arranque (s)','FontSize', 20)
ylabel('Distancia recorrida (m)','FontSize', 20)
title('Prueba del cuarto de milla', 'FontSize', 20)
legend('Distancia recorrida en función del tiempo', 'Marca del cuarto de milla','Location','northwest', 'FontSize', 18)

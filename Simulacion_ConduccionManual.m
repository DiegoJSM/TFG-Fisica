%Se inicia el mundo virtual
world = vrworld('Car3.WRL'); 
open(world);
fig = view(world, '-internal'); 

% Parámetros del modelo
m=1831+60;
mu1=223;
Cx=0.23;
Fimp1=15400;
delta_t=0.1;
h=delta_t;
 
% Condiciones iniciales
Position_Car_i=[0;0;0];
V_i=0;
betta_i=pi;
F_i=0;
theta_i=0;

% Se introduce el joystick
joy = vrjoystick(1);

for t=0:0.1:100
    %Comando pause para que la simulación no vaya demasiado rápido
    pause(.05);
    
    % La interacción con el joystick entra a través de un vector de dos componentes: 
    %    a(1) para girar el volante, con a(1)~-1 para izquierda y a(1)~1
    %    para derecha.
    %    a(2) para acelerar y frenar, con a(1)~-1 si joystick hacia arriba y a(1)~1
    %    hacia abajo.

    a = axis(joy, [1 2]);
 
    theta_i1=theta_i-0.01*round(a(1));
    
    if theta_i1>pi*0.24
        theta_i1=pi*0.24;
    elseif theta_i1<-pi*0.24
        theta_i1=-pi*0.24;
    end

    F_i1=-Fimp1*round(a(2));
    x=Position_Car_i(1);
    
    %Ajuste del coeficiente de rozamiento si el vehículo sale de la
    %carretera
    if x^2>16
        mu1=2000;
    else
        mu1=223;
    end
    
    %Solución de la ecuación mediante Runge-Kutta
    k_1 = Fcar(F_i1, mu1,V_i, m, Cx);
    k_2 = Fcar(F_i1, mu1,V_i+0.5*h*k_1, m, Cx);
    k_3 = Fcar(F_i1, mu1,V_i+0.5*h*k_2, m, Cx);
    k_4 = Fcar(F_i1, mu1,V_i+h*k_3, m, Cx);
    
    V_i1 = V_i + (1/6)*(k_1+2*k_2+2*k_3+k_4)*h;% main equation
    if V_i1<-6
        V_i1=-6;
    end
    
    Velocity_Car1=[cos(betta_i) 0 sin(betta_i);0 1 0;-sin(betta_i) 0 cos(betta_i)]*[0;0;V_i1];
    Position_Car_i1=Position_Car_i+delta_t.*Velocity_Car1;
    betta_i1=betta_i+0.05*theta_i1*V_i1+0.9*(theta_i1-theta_i)*V_i1;
        
    %Transformaciones introducidas en la simulación
    world.transform_car.translation=(Position_Car_i1)';
    world.transform_car.rotation=[0 1 0 betta_i1];
    world.front_right_wheel.rotation=[1 0 0 theta_i1];
    world.front_left_wheel.rotation=[1 0 0 theta_i1];
       
    %Nuevos parámetros iniciales para la siguiente iteración
    Position_Car_i=Position_Car_i1;
    V_i=V_i1;
    betta_i=betta_i1;
    theta_i=theta_i1;
    F_i=F_i1;
    
    %Se dibuja en la simulación
    vrdrawnow;
end

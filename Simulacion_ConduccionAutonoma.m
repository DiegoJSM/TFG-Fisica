%Se inicia el mundo virtual
world = vrworld('car3_cone1.WRL'); 
open(world);
fig = view(world, '-internal'); 

% Parámetros del modelo
m=1831+60;
mu1=223;
Cx=0.23;
Fimp1=15400;
C=0;
p=0;
q=0;
v_obj=0;
delta_t=0.1;
h=delta_t;

%Condiciones iniciales
Position_Car_i=[3;0;0];
V_i=0;
betta_i=pi;
F_i=0;
theta_i=0;
a=[0 0];

for t=0:0.1:300
    theta_i1=theta_i-0.04*round(a(1));
    if theta_i1>pi*0.24
        theta_i1=pi*0.24;
    elseif theta_i1<-pi*0.24
        theta_i1=-pi*0.24;
    end
 
    F_i1=Fimp1*round(a(2));
    x=Position_Car_i(1);
    
    if x^2>16
        mu1=2000;
    else
        mu1=223;
    end
    
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
    
    %Inicialmente se sitúan los conos debajo de la carretera
    if t==0
        world.Cone1.translation=[2 -1 -10];
        world.Cone2.translation=[-2 -1 -10];
    end
    
    if C==0 & t>30
        v_obj=(50/3.6)/30;
        p=randn;
    elseif C==1
        v_obj=(50/3.6)/80;
    end
    
    %Se pisa o no el acelerador en función de si se ha llegado a la
    %velocidad objetivo
    if V_i1<v_obj
        a(2)=1;
    else
        a(2)=0;
    end
    
    %Aparición del primer cono
    if p>2.3
        world.Cone1.translation=[3 0.5 Position_Car_i1(3)-20];
        a(1)=-1;
        a(2)=-1;
        t_paso=t+50;
        C=1;
        p=0;
    end
    
    %Ajuste del giro del coche para sortear el cono
    if C==1 & x<0
        a(1)=1;
        if (betta_i1-pi)^2<0.01
            a(1)=0;
            betta_i1=pi;
            theta_i1=0;
            C=2;
            v_obj=(50/3.6)/30;
        end
    end
    
    if C==2 & t>t_paso
        q=randn;
    elseif C==3
        v_obj=(50/3.6)/80;
    end
    
    %Aparición del segundo cono
    if q>2
        world.Cone2.translation=[-3 0.5 Position_Car_i1(3)-20];
        a(1)=1;
        a(2)=-1;
        C=3;
        q=0;
    end
    
    if C==3 & x>0
        a(1)=-1;
        if (betta_i1-pi)^2<0.01
            a(1)=0;
            betta_i1=pi;
            theta_i1=0;
            v_obj=(50/3.6)/20;
            C=4;
        end
    end
          
    %Nuevos parámetros iniciales para la siguiente iteración
    Position_Car_i=Position_Car_i1;
    V_i=V_i1;
    betta_i=betta_i1;
    theta_i=theta_i1;
    F_i=F_i1;
    
    %Se dibuja en la simulación
    vrdrawnow;

end



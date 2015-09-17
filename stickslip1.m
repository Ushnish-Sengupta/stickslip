%loswald (>.<)
runtime=16;
% Defining time displacement and velocity arrays
t=[0:0.0005:runtime];
theta=zeros(1,size(t,2)); % initializing x and v as zero arrays of the same dimension as t
w=zeros(1,size(t,2));
% Initializing variables
theta0=0;
w0=0;
t0=0;
theta(1)=0;
w(1)=0; % loops start from the second element of array, first elements are known at t=0 from initial conditions
ri=0.05;
ro=0.08; % inner and outer radii
m=2; % mass of disk
wdisk=2; % angular velocity of driving disk
mk=0.6;
ms=0.7; % coefficients of friction
mukro=mk*(ro+ri)/2;
musro=ms*(ro+ri)/2; % effective rotational coefficient of friction for flat clutch uniform wear such that frictionaltorque=muro*nf
nf=10; % normal force
kspring=45; % spring constant
kro=kspring*ro*ro; % effective rotational spring constant such that springtorque=kro*theta
jdisk=0.5*m*(ro^2+ri^2); % moment of inertia
i=2; %index, starts at 2 because (i-1) is used in the while loop
while(t(i)<runtime)
    % this loop is slip phase
    % loop ends when condition for starting of stick is satisfied
    % solving the differential equation jdisk*theta''= muk*N-k*theta
    while(w(i-1)<=wdisk) && (t(i)<runtime)
        %alpha1 alpha2 obtained by putting in initial condition v0 t0 x0
        alpha1=(cos(sqrt(kro/jdisk)*t0)*theta0-cos(sqrt(kro/jdisk)*t0)*mukro*nf/kro-sin(sqrt(kro/jdisk)*t0)*w0/sqrt(kro/jdisk));
        alpha2=(sin(sqrt(kro/jdisk)*t0)*theta0-sin(sqrt(kro/jdisk)*t0)*mukro*nf/kro+cos(sqrt(kro/jdisk)*t0)*w0/sqrt(kro/jdisk));
        theta(i)=mukro*nf/kro+alpha1*cos(sqrt(kro/jdisk)*t(i))+alpha2*sin(sqrt(kro/jdisk)*t(i)); % general solution
        w(i)=-sqrt(kro/jdisk)*sin(sqrt(kro/jdisk)*t(i))*alpha1+sqrt(kro/jdisk)*cos(sqrt(kro/jdisk)*t(i))*alpha2;
        i=i+1; 
    end
    % this loop is stick phase
    % loop ends when condition for starting of slip is satisfied
    % constant velocity
    while (theta(i-1)<(musro*nf/kro)) && (t(i)<runtime)
        w(i)=wdisk;
        theta(i)=theta(i-1)+ w(i)*(t(i)-t(i-1));
        i=i+1;
    end
    % reinitializing variables for the differential equations of slip phase
    theta0=theta(i-1);
    w0=w(i-1);
    t0=t(i-1);
end
plot(t,theta);
hold on
plot(t,w);
t=t.'; w=w.'; theta=theta.';
tabledata=table(t,theta,w);


    
        
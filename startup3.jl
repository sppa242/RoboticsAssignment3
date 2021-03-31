import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate();

using LinearAlgebra, StaticArrays
using RigidBodyDynamics, RigidBodySim
using MeshCat, MeshCatMechanisms
vis = Visualizer();open(vis)
#import PandaRobot # for visualizing Panda


qstart = [0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1]
qend = [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01]

T = 10

function traj(t)

   q_t = qstart + ((3*t^2)/(T^2) - (2*t^3/T^3) )*(qstart - qend)
   q_d_t = ((6*t)/(T^2) - (6*t^2/T^3) )*(qstart - qend)
   q_d_d_t = (6/(T^2) - (12*t)/(T^3) )*(qstart - qend)
   q_d_t_max = (3/(2*T))*(qstart - qend)
   q_d_d_t_min= -1*((6/(T^2))*(qstart - qend))
   q_d_d_t_max = ((6/(T^2))*(qstart - qend))


   print("q_t:")
   print(q_t)
   print("\n")
   print("q_d_t:")
   print(q_d_t)
   print("\n")
   print("q_d_d_t: ")
   print(q_d_d_t)
   print("\n")
   print("q_d_d_t_max: ")
   print(q_d_d_t_max)
   print("\n")
   print("q_d_d_t_min: ")
   print(q_d_d_t_min)
end

traj(10)

################################################ PID Control ###################################

function control!(τ, t, state)
    # Do some PD
    #τ .= -20 .* velocity(state) - 100*(configuration(state) - [0.0;0.0;0.0;-0.0;0.0;pi;0.01;0.01;0.01])
    τ .= -diagm([20,20,20,20,20,20,20,20,20])*velocity(state) - diagm([100,100,100,100,100,100,100,100,100])*(configuration(state) - [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01])
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end


###################### Control_CTC##################################

function Control_CTC!(τ, t, state)
  
    q0 = configuration(state)
  

    k_p = 50
    k_d = 20

    q_des_pos = [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01]

    M = mass_matrix(state)

    
        q1 = q0 + ((3*t^2)/(T^2) - (2*t^3/T^3) )*(q_des_pos-q0)
        q2 = ((6*t)/(T^2) - (6*t^2/T^3) )*(q_des_pos-q0)
        q3 = (6/(T^2) - (12*t)/(T^3) )*(q_des_pos-q0)


        e = q1 - configuration(state)
        
        edt = q2 - velocity(state)

        τ .= M*(q3 + k_p * e + k_d * edt) + dynamics_bias(state)

        act_sat = 50; # Actuator limits
        τ .= map( x -> x > act_sat ? act_sat : x,τ)
        τ .= map( x -> x < -act_sat ? -act_sat : x,τ)

end


####################################################################################

urdfPath = "panda.urdf"
mechanism = parse_urdf(Float64,urdfPath)

state = MechanismState(mechanism)
set_configuration!(state,[0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1])
#zero_configuration!(state);
#set_configuration!(state,[pi/6;pi/6;pi/6]);
zero_velocity!(state)
mvis = MechanismVisualizer(mechanism, URDFVisuals(urdfPath),vis)
#for bd in bodies(mechanism)
#    setelement!(mvis,default_frame(bd),0.5,"$bd")
#end
#setelement!(mvis,default_frame(bodies(mechanism)[12]),0.5,"$bodies(mechanism)[11]")
manipulate!(state) do x
    set_configuration!(mvis, configuration(x))
end

#problem = ODEProblem(Dynamics(mechanism,control!), state, (0., 10.));
problem = ODEProblem(Dynamics(mechanism,Control_CTC!), state, (0., 10.));
sol = solve(problem, Tsit5(),reltol=1e-8,abstol=1e-8);
mvis = MechanismVisualizer(mechanism, URDFVisuals(urdfPath),vis)
setanimation!(mvis, sol; realtime_rate = 1.0);

q_des = [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01]
error = q_des - sol[end][1:9]
print("\n")
print("Norm of Error :")
print(norm(error,2))

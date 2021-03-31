# RoboticsAssignment3
Robotics Assignment 3 ME699

<p><strong>Running Git Clone on Windows Environment</strong></p>
<p><span style="font-weight: 400;">CD C:/user/&hellip;./ desired directory name</span></p>
<p><span style="font-weight: 400;">git clone</span></p>
<h3><strong>Run Main Program</strong></h3>
<p><span style="font-weight: 400;">cd project_folder/</span></p>
<p><span style="font-weight: 400;">julia</span></p>
<p><span style="font-weight: 400;">include("startup3.jl")</span></p>
<h3><strong>Program Usage</strong></h3>
<ul>
<li aria-level="1">
<h4><strong>Generating Trajectory</strong></h4>
</li>
</ul>
<p><span style="font-weight: 400;">This is the first function which uses the polynomial time scaling. The function generates vectors such as position , velocity ( min and max) and acceleration by inputting time t in seconds .</span><strong>[1]</strong></p>
<p><span style="font-weight: 400;">We can run the following command in terminal : traj(t) where t we can add as time is seconds say t= 10 so traj(10)</span></p>
<p><span style="font-weight: 400;">traj(t)&nbsp;</span></p>
<ul>
<li aria-level="1">
<h4><strong>PID Control and CTC Control&nbsp;</strong></h4>
</li>
</ul>
<p><span style="font-weight: 400;">The program can be modified to use two kinds of control methods viz PID Control and CTC Control for&nbsp; controlling the Panda Manipulator.&nbsp; Once can see in action the controllers by changing the code in appropriate place please refer below</span></p>
<p><span style="font-weight: 400;">problem = ODEProblem(Dynamics(mechanism,control!), state, (0., 10.));</span><strong>[2]</strong></p>
<p><span style="font-weight: 400;">problem = ODEProblem(Dynamics(mechanism,Control_CTC!), state, (0., 10.));</span><strong> [1]</strong></p>
<p><strong>References</strong></p>
<ol>
<li style="font-weight: 400;" aria-level="1"><span style="font-weight: 400;">Frank, C. (2017). </span><em><span style="font-weight: 400;">Modern Robotics-Mechanics, Planning, and Control</span></em><span style="font-weight: 400;">. Cambridge University Press.</span></li>
<li style="font-weight: 400;" aria-level="1"><a href="https://github.com/hpoonawala/rmc-s21/blob/master/julia/odes/panda.jl"><span style="font-weight: 400;">https://github.com/hpoonawala/rmc-s21/blob/master/julia/odes/panda.jl</span></a></li>
</ol>
<p><strong>Acknowledgements</strong></p>
<p><span style="font-weight: 400;">I would like to thank Dr. Hassan Poonawala for helping us to learn this course and making the resources available for the assignment on his github page. Also I would like to thank Mr. Keith Russell for his extensive guidance on the topic</span></p>

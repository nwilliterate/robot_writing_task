# robot manipulator writing task

> Authors:	Seonghyeon Jo(cpsc.seonghyeon@gmail.com)
> 
> Date:		 Jan, 1, 2022
>


With the recent advances in robotics, robots are needed not only in the manufacturing field but also in various service fields. In order to realize a robot in the service fields, it is necessary to perform complex tasks and various work environments. Therefore traditional robots must be capable of easily performing physical interaction tasks. In this respect, learning from demonstration (LfD), a method in which robots acquire mimic skills by learning to imitate an expert, is an alternative way. LfD method learns the dynamics of the trajectory by optimizing it with a mathematical linear model. However, complex or overlapping patterns are difficult to learn and require the use of mixed models when learning position and force. In this paper we approach the problem of learning time-series data by predicting the output for the next state with the input for the current and previous states. Also, in terms of work, it is important to utilize the information of position/force so that the robot can perform its role well. In LfD, the imitation performance varies depending on the position/force controller. In this paper, we implicitly learn the policy for the task condition from the expert's demonstration and perform position/force-based hybrid impedance control that is not restricted from motion in the trajectory derived from the policy. In addition, it uses a position/force-based hybrid impedance controller to simultaneously learn position and force through kinematic teaching from experts to perform free movement. The proposed LfD method was experimentally verified by applying a writing task to a 7-DoF manipulator. The proposed method can be applied quickly and easily to position/force-based tasks without constraints.


### video 
https://www.youtube.com/embed/IUuEeZEsJNY?autoplay=1&mute=1&loop=1

![Hnet com-image](https://user-images.githubusercontent.com/46432171/152459719-ac93ed3d-8249-4858-ba72-df84a7166081.gif)

![Hnet com-image (1)](https://user-images.githubusercontent.com/46432171/152459741-0f8b139c-3758-4f62-aaa2-2572e800af53.gif)

### 0. Raw data 
<img src="/fig/teaching_result2.png" alt="1"  width="375" /> 
<img src="/fig/teaching_result1.png" alt="1"  width="375" /> 

### 1. preprocessing data

<img src="/fig/preprocessing_result1.png" alt="1"  width="375" /> 
<img src="/fig/preprocessing_result2.png" alt="1"  width="375" /> 

### 2. learning from demonstartion using LSTM network 

<img src="/fig/learning_result3.png" alt="1"  width="375" /> 
<img src="/fig/learning_result4.png" alt="1"  width="375" /> 

### 3. trajectory interpolation
<img src="/fig/traj_inter_result1.png" alt="1"  width="375" /> 
<img src="/fig/traj_inter_result2.png" alt="1"  width="375" /> 
<img src="/fig/traj_inter_result3.png" alt="1"  width="375" /> 

### 4. simulation result 
<img src="/fig/simulation_result1.png" alt="1"  width="375" /> 
<img src="/fig/simulation_result2.png" alt="1"  width="375" /> 

### 5. experiment result
<img src="/fig/experiment_result1.png" alt="1"  width="375" /> 
<img src="/fig/experiment_result2.png" alt="1"  width="375" /> 

In learning from demonstation, the output is the next state of the robot taught by the expert.


<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5CX_%7Bk%2B1%7D+%3D+%5Cf%28%5CX_%7Bk%7D%2C+%5CX_%7Bk-1%7D%2C%5Ccdots+%2C+%5CX_%7Bk-l%7D%29%0A" 
alt="\X_{k+1} = \f(\X_{k}, \X_{k-1},\cdots , \X_{k-l})
">


<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5CX_%7Bk%7D+%3D+%5BP_x%2C+P_y%2C+P_z%2C+R_x%2C+R_y%2C+R_z%2C+R_w%2C+F_x%5D" 
alt="\X_{k} = [P_x, P_y, P_z, R_x, R_y, R_z, R_w, F_x]">

<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cbegin%7Balign%7D%0A%7B%7D+%5E%7Bi-1%7D+T_%7Bi%7D++%3D%26+%7Brot%7D_%7Bx%2C%5Calpha_%7Bi-1%7D%7D+%7Btrans%7D_%7Bx%2Ca_%7Bi-1%7D%7D%7Brot%7D_%7Bz%2C%5Cthata_%7Bi%7D%7D+%7Btrans%7D_%7Bz%2Cd_%7Bi%7D%7D+%5C%5C%0A%3D%26+%5Cleft%5B%7B%5Cbegin%7Barray%7D%7Bccc%7Cc%7D%5Ccos+%5Ctheta+_%7Bi%7D+%26+-%5Csin+%5Ctheta+_%7Bi%7D+%26+0+%26+a_%7Bi-1%7D%5C%5C+%5Csin+%5Ctheta+_%7Bi%7D%5Ccos+%5Calpha+_%7Bi-1%7D+%26+%5Ccos+%5Ctheta+_%7Bi%7D%5Ccos+%5Calpha+_%7Bi-1%7D+%26+-%5Csin+%5Calpha+_%7Bi-1%7D+%26+-d_%7Bi%7D%5Csin+%5Calpha+_%7Bi-1%7D%5C%5C%5Csin+%5Ctheta+_%7Bi%7D%5Csin+%5Calpha+_%7Bi-1%7D+%26+%5Ccos+%5Ctheta+_%7Bi%7D%5Csin+%5Calpha+_%7Bi-1%7D+%26+%5Ccos+%5Calpha+_%7Bi-1%7D+%26+d_%7Bi%7D%5Ccos+%5Calpha+_%7Bi-1%7D%5C%5C%5Chline+0+%26+0+%26+0+%26+1%5Cend%7Barray%7D%7D%5Cright%5D%0A%5Cend%7Balign%7D+" 
alt="\begin{align}
{} ^{i-1} T_{i}  =& {rot}_{x,\alpha_{i-1}} {trans}_{x,a_{i-1}}{rot}_{z,\thata_{i}} {trans}_{z,d_{i}} \\
=& \left[{\begin{array}{ccc|c}\cos \theta _{i} & -\sin \theta _{i} & 0 & a_{i-1}\\ \sin \theta _{i}\cos \alpha _{i-1} & \cos \theta _{i}\cos \alpha _{i-1} & -\sin \alpha _{i-1} & -d_{i}\sin \alpha _{i-1}\\\sin \theta _{i}\sin \alpha _{i-1} & \cos \theta _{i}\sin \alpha _{i-1} & \cos \alpha _{i-1} & d_{i}\cos \alpha _{i-1}\\\hline 0 & 0 & 0 & 1\end{array}}\right]
\end{align} ">

<img src=
"https://render.githubusercontent.com/render/math?math=%5Cdisplaystyle+%5Cbegin%7Balign%7D%0A%7B%7D+%5E%7B0%7D+T_%7Bi%7D+%26%3D+%7B%7D+%5E%7B0%7DT_%7B1%7D%5C+%7B%7D+%5E%7B1%7DT_%7B2%7D%5C+%7B%7D+%5E%7B2%7DT_%7B3%7D%5C+%5Ccdots%5C+%7B%7D+%5E%7Bi-1%7DT_%7Bi%7D+%5Cnonumber%5C%5C%0A%26%3D%5Cleft%5B%7B%5Cbegin%7Barray%7D%7Bccc%7Cc%7D%26+%26+%26+%5C%5C%26+%7B%7D+%5E%7B0%7DR_%7Bi%7D+%26+%26+%7B%7D+%5E%7B0%7DP_%7Bi%7D%5C%5C%26+%26+%26+%5C%5C%5Chline+0%26+0%26+0%26+1%5Cend%7Barray%7D%7D%5Cright%5D%0A%5Cend%7Balign%7D+%0A" 
alt="\begin{align}
{} ^{0} T_{i} &= {} ^{0}T_{1}\ {} ^{1}T_{2}\ {} ^{2}T_{3}\ \cdots\ {} ^{i-1}T_{i} \nonumber\\
&=\left[{\begin{array}{ccc|c}& & & \\& {} ^{0}R_{i} & & {} ^{0}P_{i}\\& & & \\\hline 0& 0& 0& 1\end{array}}\right]
\end{align} 
">

## 


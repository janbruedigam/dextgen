Search.setIndex({docnames:["envs/barrett_hand","envs/envs","envs/parallel_jaw","envs/rotations","envs/seaclear","envs/shadow_hand","envs/utils","getting_started/setup","getting_started/testing","getting_started/training","index","mp_rl/core","mp_rl/core.actor","mp_rl/core.critic","mp_rl/core.ddpg","mp_rl/core.noise","mp_rl/core.normalizer","mp_rl/core.replay_buffer","mp_rl/core.utils","notes/acknowledgements","optim/constraints","optim/control","optim/core","optim/core.interior_point","optim/core.optimizer","optim/geometry","optim/grippers","optim/grippers.kinematics","optim/objective","optim/optim","optim/utils"],envversion:{"sphinx.domains.c":2,"sphinx.domains.changeset":1,"sphinx.domains.citation":1,"sphinx.domains.cpp":5,"sphinx.domains.index":1,"sphinx.domains.javascript":2,"sphinx.domains.math":2,"sphinx.domains.python":3,"sphinx.domains.rst":2,"sphinx.domains.std":2,"sphinx.ext.todo":2,sphinx:56},filenames:["envs/barrett_hand.rst","envs/envs.rst","envs/parallel_jaw.rst","envs/rotations.rst","envs/seaclear.rst","envs/shadow_hand.rst","envs/utils.rst","getting_started/setup.rst","getting_started/testing.rst","getting_started/training.rst","index.rst","mp_rl/core.rst","mp_rl/core.actor.rst","mp_rl/core.critic.rst","mp_rl/core.ddpg.rst","mp_rl/core.noise.rst","mp_rl/core.normalizer.rst","mp_rl/core.replay_buffer.rst","mp_rl/core.utils.rst","notes/acknowledgements.rst","optim/constraints.rst","optim/control.rst","optim/core.rst","optim/core.interior_point.rst","optim/core.optimizer.rst","optim/geometry.rst","optim/grippers.rst","optim/grippers.kinematics.rst","optim/objective.rst","optim/optim.rst","optim/utils.rst"],objects:{"envs.rotations":[[3,1,1,"","axisangle2quat"],[3,1,1,"","embedding2mat"],[3,1,1,"","embedding2quat"],[3,1,1,"","euler2quat"],[3,1,1,"","fastembedding2quat"],[3,1,1,"","fastmat2quat"],[3,1,1,"","map2pi"],[3,1,1,"","mat2embedding"],[3,1,1,"","mat2euler"],[3,1,1,"","mat2quat"],[3,1,1,"","quat2embedding"],[3,1,1,"","quat2mat"],[3,1,1,"","quat_conjugate"],[3,1,1,"","quat_mul"],[3,1,1,"","vec2quat"]],"envs.utils":[[6,1,1,"","ctrl_set_action"],[6,1,1,"","goal_distance"],[6,1,1,"","map_sh2mujoco"],[6,1,1,"","mocap_set_action"],[6,1,1,"","reset_mocap2body_xpos"],[6,1,1,"","reset_mocap_welds"],[6,1,1,"","robot_get_obs"]],"mp_rl.core":[[12,0,0,"-","actor"],[13,0,0,"-","critic"],[14,0,0,"-","ddpg"],[15,0,0,"-","noise"],[16,0,0,"-","normalizer"],[17,0,0,"-","replay_buffer"],[18,0,0,"-","utils"]],"mp_rl.core.actor":[[12,2,1,"","Actor"],[12,2,1,"","DDP"],[12,2,1,"","PosePolicyNet"]],"mp_rl.core.actor.Actor":[[12,3,1,"","__call__"],[12,3,1,"","__init__"],[12,4,1,"","__weakref__"],[12,3,1,"","backward_step"],[12,3,1,"","eval"],[12,3,1,"","init_dist"],[12,3,1,"","load"],[12,3,1,"","select_action"],[12,3,1,"","target"],[12,3,1,"","train"],[12,3,1,"","update_target"]],"mp_rl.core.actor.DDP":[[12,3,1,"","__init__"],[12,3,1,"","forward"]],"mp_rl.core.actor.PosePolicyNet":[[12,3,1,"","__init__"],[12,3,1,"","embedding2flatmat"],[12,3,1,"","embedding2flatmat_safe"],[12,3,1,"","forward"],[12,3,1,"","forward_include_network_output"]],"mp_rl.core.critic":[[13,2,1,"","Critic"],[13,2,1,"","CriticNetwork"]],"mp_rl.core.critic.Critic":[[13,3,1,"","__call__"],[13,3,1,"","__init__"],[13,4,1,"","__weakref__"],[13,3,1,"","backward_step"],[13,3,1,"","init_dist"],[13,3,1,"","load"],[13,3,1,"","target"],[13,3,1,"","update_target"]],"mp_rl.core.critic.CriticNetwork":[[13,3,1,"","__init__"],[13,3,1,"","forward"]],"mp_rl.core.ddpg":[[14,2,1,"","DDPG"]],"mp_rl.core.ddpg.DDPG":[[14,3,1,"","__init__"],[14,4,1,"","__weakref__"],[14,3,1,"","_train_agent"],[14,3,1,"","_update_norm"],[14,3,1,"","eval_agent"],[14,3,1,"","init_dist"],[14,3,1,"","load_pretrained"],[14,3,1,"","save_models"],[14,3,1,"","save_plots"],[14,3,1,"","save_stats"],[14,3,1,"","train"],[14,3,1,"","wrap_obs"]],"mp_rl.core.noise":[[15,2,1,"","GaussianNoise"],[15,2,1,"","NoiseProcess"],[15,2,1,"","OrnsteinUhlenbeckNoise"],[15,2,1,"","UniformNoise"]],"mp_rl.core.noise.GaussianNoise":[[15,3,1,"","__init__"],[15,3,1,"","reset"],[15,3,1,"","sample"]],"mp_rl.core.noise.NoiseProcess":[[15,3,1,"","__init__"],[15,4,1,"","__weakref__"],[15,3,1,"","reset"],[15,3,1,"","sample"]],"mp_rl.core.noise.OrnsteinUhlenbeckNoise":[[15,3,1,"","__init__"],[15,3,1,"","reset"],[15,3,1,"","sample"]],"mp_rl.core.noise.UniformNoise":[[15,3,1,"","__init__"],[15,3,1,"","reset"],[15,3,1,"","sample"]],"mp_rl.core.normalizer":[[16,2,1,"","Normalizer"]],"mp_rl.core.normalizer.Normalizer":[[16,3,1,"","__call__"],[16,3,1,"","__init__"],[16,4,1,"","__weakref__"],[16,3,1,"","_transfer_buffers"],[16,3,1,"","init_dist"],[16,3,1,"","load"],[16,3,1,"","normalize"],[16,3,1,"","save"],[16,3,1,"","update"]],"mp_rl.core.replay_buffer":[[17,2,1,"","HERBuffer"],[17,2,1,"","MemoryBuffer"],[17,2,1,"","ReplayBuffer"],[17,2,1,"","TrajectoryBuffer"],[17,1,1,"","default_sampling"],[17,1,1,"","her_sampling"]],"mp_rl.core.replay_buffer.HERBuffer":[[17,3,1,"","__init__"],[17,3,1,"","__len__"],[17,3,1,"","append"],[17,3,1,"","get_trajectory_buffer"],[17,3,1,"","sample"]],"mp_rl.core.replay_buffer.MemoryBuffer":[[17,3,1,"","__init__"],[17,3,1,"","__len__"],[17,3,1,"","append"],[17,3,1,"","clear"],[17,3,1,"","sample"]],"mp_rl.core.replay_buffer.ReplayBuffer":[[17,3,1,"","__init__"],[17,3,1,"","__len__"],[17,4,1,"","__weakref__"],[17,3,1,"","append"],[17,3,1,"","sample"]],"mp_rl.core.replay_buffer.TrajectoryBuffer":[[17,3,1,"","__getitem__"],[17,3,1,"","__init__"],[17,3,1,"","__iter__"],[17,3,1,"","__len__"],[17,4,1,"","__weakref__"],[17,3,1,"","append"],[17,3,1,"","clear"],[17,3,1,"","items"],[17,3,1,"","keys"],[17,3,1,"","values"]],"mp_rl.core.utils":[[18,1,1,"","running_average"],[18,1,1,"","soft_update"],[18,1,1,"","sync_grads"],[18,1,1,"","sync_networks"],[18,1,1,"","unwrap_obs"]],"optim.constraints":[[20,1,1,"","create_disk_constraints"],[20,1,1,"","create_distance_constraints"],[20,1,1,"","create_force_constraints"],[20,1,1,"","create_max_angle_constraints"],[20,1,1,"","create_moments_constraints"],[20,1,1,"","create_plane_constraints"],[20,1,1,"","create_sphere_constraint"],[20,1,1,"","quaternion_cnst"]],"optim.control":[[21,2,1,"","Controller"]],"optim.control.Controller":[[21,3,1,"","__call__"],[21,3,1,"","__init__"],[21,4,1,"","__weakref__"],[21,3,1,"","_check_geom"],[21,3,1,"","_check_reached"],[21,3,1,"","_compute_ctrl"],[21,3,1,"","optimize_grasp"],[21,3,1,"","reset"],[21,3,1,"","set_geom"],[21,3,1,"","set_xopt"]],"optim.core":[[23,0,0,"-","interior_point"],[24,0,0,"-","optimizer"]],"optim.core.interior_point":[[23,1,1,"","linesearch"],[23,1,1,"","solve"]],"optim.core.optimizer":[[24,2,1,"","Optimizer"]],"optim.core.optimizer.Optimizer":[[24,3,1,"","_compile_equality_constraints"],[24,3,1,"","_compile_inequality_constraints"],[24,3,1,"","add_equality_constraint"],[24,3,1,"","add_equality_mconstraint"],[24,3,1,"","add_inequality_constraint"],[24,3,1,"","add_inequality_mconstraint"],[24,3,1,"","optimize"],[24,3,1,"","reset"],[24,3,1,"","set_lower_bounds"],[24,3,1,"","set_maxeval"],[24,3,1,"","set_min_objective"],[24,3,1,"","set_upper_bounds"]],"optim.geometry":[[25,0,0,"-","base_geometry"],[25,0,0,"-","cube"]],"optim.geometry.base_geometry":[[25,2,1,"","Geometry"]],"optim.geometry.base_geometry.Geometry":[[25,3,1,"","create_constraints"],[25,3,1,"","create_surface_constraints"]],"optim.geometry.cube":[[25,2,1,"","Cube"]],"optim.geometry.cube.Cube":[[25,3,1,"","_contact_mapping"],[25,3,1,"","create_surface_constraints"]],"optim.grippers":[[26,0,0,"-","barrett_hand"],[26,0,0,"-","base_gripper"],[26,0,0,"-","parallel_jaw"],[26,0,0,"-","shadow_hand"]],"optim.grippers.barrett_hand":[[26,2,1,"","BarrettHand"]],"optim.grippers.barrett_hand.BarrettHand":[[26,3,1,"","create_grasp_wrench"],[26,3,1,"","create_kinematics"],[26,5,1,"","joint_limits"]],"optim.grippers.base_gripper":[[26,2,1,"","Gripper"]],"optim.grippers.base_gripper.Gripper":[[26,3,1,"","create_constraints"],[26,3,1,"","create_gripper_constraints"],[26,3,1,"","create_kinematics"],[26,5,1,"","joint_limits"]],"optim.grippers.kinematics":[[27,0,0,"-","barrett_hand"],[27,0,0,"-","parallel_jaw"],[27,0,0,"-","shadow_hand"],[27,0,0,"-","tf"]],"optim.grippers.kinematics.barrett_hand":[[27,1,1,"","kin_1dist_p"],[27,1,1,"","kin_2dist_p"],[27,1,1,"","kin_3dist_p"],[27,1,1,"","kin_bh_full"]],"optim.grippers.kinematics.parallel_jaw":[[27,1,1,"","kin_pj_full"],[27,1,1,"","kin_pj_left"],[27,1,1,"","kin_pj_right"]],"optim.grippers.kinematics.shadow_hand":[[27,1,1,"","kin_sh_full"]],"optim.grippers.kinematics.tf":[[27,1,1,"","tf_matrix"],[27,1,1,"","tf_matrix_q"],[27,1,1,"","zrot_matrix"]],"optim.grippers.parallel_jaw":[[26,2,1,"","ParallelJaw"]],"optim.grippers.parallel_jaw.ParallelJaw":[[26,3,1,"","create_grasp_wrench"],[26,3,1,"","create_gripper_constraints"],[26,3,1,"","create_kinematics"],[26,5,1,"","joint_limits"]],"optim.grippers.shadow_hand":[[26,2,1,"","ShadowHand"]],"optim.grippers.shadow_hand.ShadowHand":[[26,3,1,"","create_grasp_wrench"],[26,3,1,"","create_kinematics"],[26,5,1,"","joint_limits"]],"optim.objective":[[28,1,1,"","create_cube_objective"]],"optim.utils":[[30,0,0,"-","rotations"],[30,0,0,"-","utils"]],"optim.utils.rotations":[[30,1,1,"","quat2mat"]],"optim.utils.utils":[[30,1,1,"","check_grasp"],[30,1,1,"","filter_info"],[30,1,1,"","import_guard"]],"optim.visualization":[[30,1,1,"","visualize_contacts"],[30,1,1,"","visualize_geometry"],[30,1,1,"","visualize_gripper"]],envs:[[3,0,0,"-","rotations"],[4,0,0,"-","seaclear"],[6,0,0,"-","utils"]],optim:[[20,0,0,"-","constraints"],[21,0,0,"-","control"],[28,0,0,"-","objective"],[30,0,0,"-","visualization"]]},objnames:{"0":["py","module","Python module"],"1":["py","function","Python function"],"2":["py","class","Python class"],"3":["py","method","Python method"],"4":["py","attribute","Python attribute"],"5":["py","property","Python property"]},objtypes:{"0":"py:module","1":"py:function","2":"py:class","3":"py:method","4":"py:attribute","5":"py:property"},terms:{"0":[4,9,14,15,16,18,20,24,27],"01":16,"03":27,"07035":12,"1":[7,12,14,18,20],"1000":23,"10000":17,"15m":4,"16":[7,9],"1812":12,"2":[4,7,17],"2022":27,"3":[12,20],"3d":16,"4":17,"50":18,"6":12,"6d":[3,12],"7":20,"777777":30,"9":12,"abstract":[15,17,25,26],"case":6,"class":[1,11,12,13,14,15,16,17,21,24,25,26],"default":[9,14,17,30],"do":[9,27,29,30],"export":7,"final":9,"float":[3,12,13,14,15,16,17,18,20,23],"function":[1,3,6,7,8,10,11,12,13,17,18,22,23,24,26,29,30],"import":1,"int":[12,13,14,15,16,17,18,23,24],"new":[7,16,23,26,27],"return":[3,6,12,13,14,15,16,17,18,20,21,23,24,25,26,27,28,30],"static":[12,17],"switch":9,"true":[3,21,30],"try":30,"while":29,A:[6,9,12,14,15,17,18,26,30],As:9,At:[25,26],Be:9,For:[6,8,9,16,29],If:[3,7,9,12,13,14,16,17,24,26],In:[4,6,8,9,18,23,24,28,29],It:[7,10,12,14,17,20,23,24],Its:[11,25,29],NOT:24,No:21,On:[1,12],The:[0,1,2,3,4,5,6,7,8,9,11,12,13,16,17,18,20,21,22,23,24,25,26,27,28,29,30],These:9,To:[9,15,20],_:[17,26],__call__:[12,13,16,21],__getitem__:17,__init__:[8,9,12,13,14,15,16,17,21],__iter__:17,__len__:17,__weakref__:[12,13,14,15,16,17,21],_check_geom:21,_check_reach:21,_compile_equality_constraint:24,_compile_inequality_constraint:24,_compute_ctrl:21,_contact_map:25,_src:[20,27,28,30],_train_ag:14,_transfer_buff:16,_update_norm:14,_variablefunctionsclass:13,a11:3,a12:3,a13:3,a21:3,a22:3,a23:3,abc:17,about:20,accept:23,access:9,accord:[6,12,29],achiev:[10,17,18,21],acknowledg:10,across:[9,12,13,14,16,18],act:[1,12,13],action:[6,12,13,17],action_clip:12,activ:[7,9,14,16,30],actor:[9,10,11,14,15],actor_net_typ:12,actual:[12,13,17],actuat:6,ad:16,adapt:27,add:[9,16,24,26],add_equality_constraint:24,add_equality_mconstraint:24,add_inequality_constraint:24,add_inequality_mconstraint:24,addit:[4,5,9,14,17,22,25],addition:[6,26],adopt:26,after:[7,8,9,17,23,29],ag:17,agent:[5,9,12,14,21],agoal:17,agument:9,albeit:29,algorithm:[11,14],alia:16,all:[0,1,2,5,6,7,9,14,16,18,20,21,23,24,25,27,29],allow:[20,22,23,24,29,30],allreduc:[16,18],alpha:23,also:[9,12],although:[7,27],amount:[8,24,29],an:[3,5,7,12,13,14,15,16,17,18,20,23,24,28,29,30],angl:[3,20,27],ani:[4,15],api:[24,29],append:17,appli:9,applic:15,appropri:[9,17],apt:7,ar:[0,1,2,3,5,6,7,9,12,14,16,17,18,20,21,24,25,27,28,29],arg:[14,17],argpars:14,argument:[8,9,17,20,29],arm:1,around:[1,3,12,13],arrai:[3,6,12,14,15,16,17,18,24,27,30],arxiv:12,assert:17,assertionerror:[16,17,21,30],asset:[1,27],associ:[6,23],assum:[0,3,5,7,12,14,20],auto:11,autodiff:[23,24],automat:[29,30],avail:[0,1,2,5,8,9,24],available_env:8,averag:[9,12,13,14,16,18],axi:3,axisangle2quat:3,b1:12,b2:12,back:12,backend:9,backup:[9,14],backward:[12,13],backward_step:[12,13],barrett_hand:[1,10],barretthand:[0,9,26,27],base:[1,15,16,17,18,25,26,29],base_geometri:30,base_gripp:25,baselin:[16,18],bashrc:7,batch:17,batch_siz:17,becaus:23,becom:1,been:[3,7,9,17,21,27,30],befor:[3,9,12,16,24,29],begin:24,behind:16,being:16,best:23,between:[6,9,14,20],bin:7,binari:7,blob:3,bodi:6,bool:[3,14,17,21,24,30],border:20,both:29,bound:24,broadcast:18,buffer:[14,16,17],build:[10,30],built:9,c:[20,24],cach:24,calcul:[9,20,23,25,26],call:[1,14,17,20],callabl:[17,20,23,24,26,28],can:[7,8,9,12,14,15,23,24,29],candid:30,cannot:[8,29],care:[9,12,13,14],categor:9,ce:23,center:[6,20,27,28],chang:[9,17,27,29],check:[7,16,17,21,30],check_grasp:30,checkpoint:[9,11,14],child:25,choic:12,choos:12,chosen:28,ci:[7,23],circumv:23,clear:17,clip:[12,13,15,16],clone:29,closer:4,closest:25,collect:[17,20,26],color:30,com:[3,6,16,18,20,27,28],combin:12,command:9,comment:16,common:[17,25],commun:16,compat:[8,12,23,24,27,29,30],compil:[23,24,29],complet:12,complex:15,compon:[3,21,22],comput:[6,9,12,13,14,18,21,27,29],con_pt:[26,30],concurr:9,conda:[7,9],condit:[28,29],config:[8,9,14,20,29],configur:[8,14,20,21,26,27,28,29,30],conjug:3,consist:[8,12,13,16,17,20,22,29],constitut:[28,30],constrain:20,constraint:[6,10,21,22,23,24,25,26,29],constrict:20,construct:[11,20,22,27],contact:[20,21,25,26,29,30],contain:[0,1,2,5,9,11,12,13,14,17,20,23,26,27,29,30],content:27,conting:14,continu:12,control:[1,6,8,9,10,12,18,23,26,29],conveni:[9,17,30],converg:[21,24,29],convers:[1,3,30],convert:[3,30],convolut:18,coordin:6,copi:[6,12,13,18],core:[9,10,25,26,29],correct:[12,27],correl:15,correspond:16,coupl:[26,29],cp_normal:20,cpu:9,creat:[7,12,13,14,17,20,21,25,26,27,28],create_constraint:[25,26],create_cube_object:28,create_disk_constraint:20,create_distance_constraint:20,create_force_constraint:20,create_grasp_wrench:26,create_gripper_constraint:26,create_kinemat:26,create_max_angle_constraint:20,create_moments_constraint:20,create_plane_constraint:20,create_sphere_constraint:20,create_surface_constraint:25,criteria:23,critic:[9,10,11,14],criticnetwork:13,cross:5,ctrl:6,ctrl_set_act:6,cube:[8,21,28,29],current:[6,8,9,14,16,17,20,21,24,25,26,29,30],cymj:6,d:23,dai:9,data:16,date:[9,14],ddp:[9,12],ddpg:[10,11,12,13],debug:29,decis:9,deep:[10,12,13,14],deeper:9,default_sampl:17,defin:[1,12,13,14,15,16,17,20,21,23,24,25,26,29],definit:[24,26],delta:6,denot:27,depend:[7,9],deriv:[24,26],descend:23,describ:7,descript:29,desir:[6,9,18,21,27],detail:[12,29],detect:29,determinist:[12,14,29],dev:7,develop:9,deviat:[15,20],dext:7,dexter:10,dict:[12,13,17,18,21,25,26,30],dictionari:[14,18,21,25,29,30],differ:[1,6,9,10],differenti:[20,27,28,29,30],difficulti:9,dim:15,dimens:[15,16,17,24],dimension:15,direct:[18,23],directli:[9,12],directori:[8,9,14,29],disabl:[8,29],disable_nan_check:24,disk:20,dispatch:30,dist:14,distanc:[6,20],distribut:[12,13,14,15,16],distributeddataparallel:9,diver:12,divers:[0,5],divid:1,doc:[20,30],docker:7,dockerfil:7,doe:[12,13],don:[17,21],done:17,dure:[9,12,14,17,23],e501:27,each:[9,15,16,20,26],earli:23,easi:9,easili:29,echo:7,either:[12,17,30],els:[21,30],embed:[3,12],embedding2flatmat:12,embedding2flatmat_saf:12,embedding2mat:3,embedding2quat:3,emphas:20,emploi:9,enabl:[9,14,16,24,25,29],encapsul:[12,13,14],encout:14,end:24,enough:24,ensur:[16,23],enter:4,entri:[16,17],env:[8,9,10,14,29,30],env_nam:[1,8,14],environ:[0,1,2,4,5,6,8,9,12,14,17,21,25,27,29],ep:[12,16],ep_buff:[14,17],ep_success:14,ep_tim:14,episod:[4,14,16,17,29],equal:[6,12,13,14,20,22,23,24],error:17,especi:24,estim:16,etc:22,euclideanspac:3,euler2quat:3,euler:[3,27],eval:[12,14],eval_ag:14,evalu:[12,14,24],everi:14,exact:[12,13],exactli:20,exampl:[6,8,9],except:20,exclud:14,execut:[8,9,21],exist:9,exp:17,expect:[8,9,16,17,24,29],experi:[1,7,8,14,17,26,29],experiment:[24,29],experiment_config:[8,9,29],explicit:9,expos:17,extend:[3,29,30],extract:26,extrem:24,f:[23,24],facilit:29,factor:[15,17,18],factori:[20,28],fail:[4,7,8,16,17,21],failur:29,fall:9,fals:[3,14,21,24,30],familiar:24,farama:[1,6],fashion:17,fast:24,fastembedding2quat:3,faster:3,fastmat2quat:3,featur:[0,2,5],few:[9,29],ff:6,field:6,fig:30,figur:30,file:[1,6,7,8,9,14,27,29],fill:17,filter:30,filter_info:30,find:29,finger:[26,27],finish:9,finit:20,first:[3,12,21,22,24,27],fix:17,flat:[0,5],flatbarrettal:0,flatbarrettcub:0,flatbarrettcylind:0,flatbarrettmesh:0,flatbarrettspher:0,flatpjal:2,flatpjcub:[2,8,29],flatpjcylind:2,flatpjmesh:2,flatpjori:2,flatpjorienteul:2,flatpjspher:2,flatshal:5,flatshcub:5,flatshcylind:5,flatshmesh:[5,9],flatshori:5,flatshspher:5,flatten:21,flow:23,folder:[1,7,9],follow:[7,9,17,24],forbidden:[4,5],forc:[20,21,26],form:[3,24],forward:[12,13],forward_include_network_output:12,found:9,foundat:[1,6],four:[9,20],fraction:[12,13],frame:27,from:[3,6,7,8,9,12,14,15,16,17,18,20,25,26,27,29],full:[2,5,9,17],full_kinemat:20,further:[1,9,14],furthermor:9,fuse:[14,24],g:17,gain:9,gaussian:15,gaussiannois:15,gen:7,gener:[9,14,15,17,26,29,30],geom:[21,30],geometri:[3,21,30],geq:20,get:[12,17,18],get_trajectory_buff:17,getitem:17,github:[3,6,7,16,18],given:[12,21,22,26,29,30],global:16,gloo:9,glx:7,go:9,goal:[2,4,5,6,9,12,13,14,17,18,21,29],goal_a:6,goal_b:6,goal_dist:6,goal_norm:9,good:29,gpu:9,grad_clip:[12,13],gradiend:18,gradient:[9,12,13,14,18,29],grasp:[9,10,20,21,23,26,28,29,30],grasp_forc:20,gripper:[1,2,4,6,9,10,12,20,21,25,29,30],gripper_nam:9,ground:[0,5],group:[14,16],guess:29,gym:[1,6,10,12,14,18],ha:[4,7,9,16,17,21,24,26,27,30],hand:[9,27],hardcod:27,hardwar:[7,9],have:[3,7,8,9,20,21,23,26,27,29],head:12,her:[14,17],her_sampl:17,herbuff:[14,17],hessian:23,highli:29,hindsight:[10,17],hold:17,home:7,homogen:[21,27],horizon:17,hour:9,how:[7,18],howev:9,http:[3,6,12,16,18,27],hwthread:9,hyperparamet:9,idx:16,ignor:9,implement:[0,1,2,5,11,12,13,16,17,24,25,26,27,29],implicitli:12,import_guard:30,improv:[21,29],includ:[4,5,6,7,8,9,12,24,29],independ:1,index:[10,16,24],indexerror:17,indic:[16,20],indirectli:29,individu:[9,16,20],inequ:[20,22,23,24],inf:[12,13,16],infer:14,influenc:[12,13],info:[21,25,26,30],inform:[20,21,25,29,30],init:17,init_dist:[12,13,14,16],initi:[0,1,5,9,12,13,14,15,16,17,21,23,24,25,28,29,30],inner:27,input:[12,13,14,16,20,21],insid:24,inspect:[8,30],instal:[7,9],instead:[9,12,14,29],integr:[21,24],interfac:[11,15,17,20,24,25,26],interior:[23,24,29],interior_point:[10,22,24],intermedi:12,intern:[12,15,17],interv:3,invok:[24,29,30],item:17,itemsview:17,iter:[17,23,24],its:[1,4,9,12,13,20,21,29],itself:[17,20],jaw:[2,8,27,29],jax:[20,23,24,27,28,29,30],jit:[23,24],jite:24,joint:[6,26,27],joint_limit:26,json:[9,14],just:[24,29],k:17,keep:[9,16,25],kei:[9,17],keyerror:30,keysview:17,kin_1dist_p:27,kin_2dist_p:27,kin_3dist_p:27,kin_bh_ful:27,kin_pj_ful:27,kin_pj_left:27,kin_pj_right:27,kin_sh_ful:27,kinemat:[10,20,25,26,29],kwarg:9,lagrangian:[23,24],larg:29,larger:16,latest:[8,9,29],launch:9,layer:[12,13],layer_width:[12,13],ld_library_path:7,lead:[4,12],learn:[1,11,12,13],least:7,left:27,length:17,less:[12,17],level:9,lf:6,lib:7,libgl1:7,libglfw3:7,libosmesa6:7,libosmesa:7,librari:[7,29],lie:[20,21],lift:[21,29],like:21,likewis:29,limit:26,linear:24,linesearch:23,link:[26,27],linux:7,list:[8,9,12,13,14,15,16,17,18,21,30],load:[8,9,11,12,13,14,16,29],load_pretrain:[9,14],local:16,locat:[1,8,14,27,29],logic:1,loss:[12,13],lower:24,lr:[12,13],lwoer:24,made:7,main:[9,11,26],maintain:16,make:[1,7,9,24,27,30],manag:[9,14,24],mani:27,manual:[9,30],map2pi:3,map:[3,6,12,25],map_sh2mujoco:6,mass:[20,28],master:3,mat2embed:3,mat2eul:3,mat2quat:3,mat:3,match:[9,17],math:3,matplotlib:30,matric:[3,12,27],matrix:[3,12,21,27,30],matrixtoquaternion:3,max_angl:20,max_sampl:17,maximum:[12,17,20,24],maxlen:17,mean:[15,16,29,30],memorybuff:17,merg:14,mesa:7,mesh:[1,9],method:[14,17,23,24,29],metric:[28,29],mf:6,might:7,mimic:24,min_dist:20,miniconda:7,minim:[20,22],minimum:16,minut:24,miss:7,mix:20,mjcf:1,mjsim:6,mocap:6,mocap_set_act:6,mode:[12,14,16,17,18],modifi:6,modul:[0,2,5,9,10,11,12,13,14,15,16,17,18,20,21,22,23,24,25,26,27,28],moment:[20,25,26],more:[0,5,15],move:9,mp_rl:[10,12,13,14,15,16,17,18],mpi:[7,9,12,13,14,16,18],mpirun:9,mu:[15,23],much:18,mujoco210:7,mujoco:[1,3,6,10,26,29],mujoco_pi:6,mujoco_worldgen:3,multipl:[14,16,24,30],multipli:3,multiprocess:11,must:[9,17,24],n:[8,9,15,24,29],name:[4,7,8,9,26,27],namespac:14,nccl:9,ndarrai:[3,6,12,14,15,16,17,18,20,21,23,24,26,27,28,30],necessari:[7,9,28],need:[27,28,29],net:[9,12,13],network:[9,11,12,13,14,18],neural:[9,12],newli:20,next:[12,26],next_stat:17,niter:[23,24],nlayer:[12,13],nlopt:[24,29],nn:[9,18],node:[9,12,13,14,16],nois:[10,12,14],noise_process:12,noiseprocess:[12,15],noisi:12,non:24,none:[14,15,16,24,30],noqa:27,norm:[20,23],normal:[3,9,10,11,14,20],note:[3,8],np:3,ntest:[8,29],numba:23,number:[9,12,13,17,23,24,27],numer:16,numpi:[3,6,12,14,15,16,17,18,20,21,23,24,26,27,28,29,30],nvidia:7,ob:18,object:[0,2,4,5,8,10,12,13,14,15,16,17,20,21,22,23,24,29,30],observ:[17,18,29],obstacl:5,obstacleshcub:5,obtain:29,offici:7,offset:20,onc:9,one:[12,17,20,23],onli:[7,9,12,14,16,17,24,25,26,27,29,30],op:[15,16],open:7,openai:[1,3,6,10,14,16,18],openmpi:7,oper:[18,27],oppos:21,opt:[9,25,26],optim:[6,12,13,23,25,26,27],optimize_grasp:21,option:[8,9,14,15,16,24,29,30],order:[6,7,8,9,12,23,24,28,29],orderli:17,org:12,orient:[2,5,6,12,20,21,24],origin:12,ornstein:15,ornsteinuhlenbecknois:15,other:[9,29],otherwis:[9,14,21],our:[3,6,12],output:[12,13],over:[14,16,18],overwrit:9,p:26,p_her:17,packag:[7,9,26,29],panda:1,paper:14,parallel:[2,8,9,12,27,29],parallel_jaw:[1,10],paralleljaw:[9,26,27],paramet:[3,9,12,13,14,15,16,17,18,20,21,23,24,25,26,27,28,30],parameter:[12,20],part:[25,29],particular:[9,29],pass:[9,12,13],patchelf:7,path:[12,13,14,16],pathlib:[12,13,14,16],pdf:[12,27],per:[12,13],perform:[9,12,13,14,16,18],persist:24,physic:9,pi:3,pickl:16,pip:7,pkl:9,place:[0,5,18],plai:14,plane:[20,25],pleas:9,plot:[9,14,30],plt:30,point:[4,20,23,24,25,26,29,30],polici:[12,14,21],pose:[0,2,5,12,21,25,29],posepolicynet:12,posit:[4,6,12,20,21,25,26,27],possibl:[9,15,23,24],pre:14,preprocess:16,pretrain:[9,14],previou:[15,17],previous:9,prior:7,probabl:[12,17],problem:[22,23,24,28,29],procedur:15,process:[7,8,11,12,13,14,15,16,18,21,29],progress:9,proper:[3,12,30],properti:26,prototyp:21,provid:[8,12,13,20,24,29],pt:9,purpos:7,py:[3,8,9,29],pytest:8,python:[1,8,9,24,29],pytorch:18,q0:3,q1:3,q:[3,13,30],qpo:6,qualiti:[28,29],quat2embed:3,quat2mat:[3,30],quat:3,quat_conjug:3,quat_mul:3,quaterion:27,quaternion:[3,6,20,27,30],quaternion_cnst:20,radian:3,radiu:20,rais:[16,17,21,30],random:[12,17],rank:14,rate:[12,13,14,21],re:6,reach:[24,29],read:25,reason:[9,16,24],recalcul:[17,27],recent:7,recommend:[7,9],recreas:23,rectangl:20,rectangular:20,reduc:29,reduct:15,refer:[12,13,14,15,16,17,20,21],region:[4,5],regist:[1,20,21,24,25],regular:[3,12],reimplement:30,reinforc:[10,11,29],rel:6,relat:1,reload:[12,13],render:[8,29],replac:[7,8,12],replai:[14,17],replay_buff:[10,11,14],replaybuff:17,replic:[27,29],repositori:[1,3,7],represent:[1,12],reproduc:9,request:17,requir:[1,7,9,11,14,23,24,27,28,29],resampl:[14,17],reset:[6,15,16,21,24,29],reset_mocap2body_xpo:6,reset_mocap_weld:6,reshap:12,respect:[27,29],rest:16,restrict:[20,25],result:[4,30],revisit:9,reward:[4,14,17],reward_fun:17,rf:6,right:27,rl:15,robot:[1,4,6,20,27,29],robot_get_ob:6,root:[7,8,9,29],rotat:[1,10,12,21,27],routin:[24,29],run:[7,8,12,13,14,18,24,29],running_averag:18,runtim:23,runtimeerror:[17,21,30],s:[1,3,6,9,10,17,18,26,27,29],safe:12,same:6,sampl:[12,14,15,17],sample_mod:17,satur:12,save:[8,9,12,13,14,16,24,29],save_model:14,save_plot:14,save_stat:14,savefil:16,saw:9,scalar:24,scenario:9,scheme:29,scoop:4,scratch:9,script:[21,29],seaclear:10,search:23,second:[3,12,22,27,29],section:[7,29],see:[3,6,9,12,16,18,27],seed:9,select:[12,16],select_act:12,self:[12,16],separ:[1,16,18,30],set:[6,9,12,14,21,24],set_geom:21,set_lower_bound:24,set_maxev:24,set_min_object:24,set_upper_bound:24,set_xopt:21,setup:[9,10,30],sever:9,sh:6,shadow_dexterous_hand_e_technical_specif:27,shadow_hand:[1,10],shadowhand:[5,6,9,26,27],shadowrobot:27,shape:[12,16],shift:18,should:[8,9,16,17,20],show:30,side:[20,21],sigma:15,signific:[9,21,24],significantli:29,sim:6,simpl:17,simul:[1,4,7,9,26,29],simultan:29,sinc:[9,21,24,29,30],singl:[3,9,14,17,20,24,30],singledispatch:30,size:[9,12,13,14,16,17,18,23],size_:[12,13,17],size_a:[12,13,17],size_g:17,slack:23,slow:24,small:9,smaller:20,smooth:18,smoothli:12,so:[0,5,9,12],soft:[12,13,18],soft_upd:18,solut:[21,29],solv:[9,14,23,24],some:[9,20],sourc:7,spars:14,special:[12,17],specif:[6,20,25,26],specifi:24,speed:[9,24,29],sphere:20,sphinx:30,split:1,stabil:16,standard:[4,15,23],start:[9,14,15,23],stat:[9,14],state:[9,12,13,14,15,17,21],state_norm:9,stateless:15,statist:[9,14],statu:23,step:[12,13,15,17,23,24],still:29,stop:23,store:17,str:[12,17,18,26,30],strategi:29,string:30,style:[18,24],submodul:[1,20,27,29],success:[8,14,21,29],successfulli:9,sudo:7,suit:8,suitabl:[23,28],sum:[12,13,16,20],summari:8,support:[12,13,14,15,16,17,24,29,30],sure:[7,9],surfac:[0,5,20,25,27],sync_grad:18,sync_network:18,synchron:[12,13,14,16,18],syncron:[12,13],system:7,t:[17,21,23],take:[9,12,13,14,20,21,24],tanh:12,target:[4,6,8,10,12,13,18,20,21,23,29],task:[9,14],tau:[12,13,18],tensor:[3,12,13,14],test:[7,10,14,27,29],tf_matrix:27,tf_matrix_q:27,than:[4,16,17,20],thei:6,them:27,themselv:[1,20],theori:9,therefor:[7,24,29],theta:[3,27],thi:[0,1,2,5,6,7,8,9,12,15,17,21,23,24,29],third:[12,27],thread:[7,9],three:12,through:29,thumb:6,time:[9,14,17,24,29],timestamp:14,timestep:[16,17],tip:[26,27],toggl:14,toler:21,top:9,torch:[9,12,13,14,18],torqu:6,total:9,tpu:24,track:16,train:[1,7,10,11,12,13,14,17,29],trajectori:[14,17],trajectorybuff:[14,17],transfer:[12,13],transform:[12,21,27],transit:17,translat:[6,12,29],transpos:12,tri:[6,29],tune:21,tupl:[6,12,17,18,20,21,23,27,28,30],two:[6,20,22],type:[1,10,12,17,30],typeerror:30,uhlenbeck:15,unchang:16,uncorrel:15,under:[8,9,14,29],underli:20,underwat:4,uneven:[0,5],unevenbarrettcub:0,unevenbarrettmesh:0,unevenshcub:5,unevenshmesh:5,uniform:15,uniformli:15,uniformnois:15,union:[15,17,27,30],unit:[16,20],unitquaternion:3,unnorm:21,unsupport:30,until:29,unus:26,unwrap:18,unwrap_ob:18,up:24,updat:[12,13,14,16,18,23],update_target:[12,13],upload:27,upon:1,upper:24,us:[1,6,7,8,9,10,12,14,15,16,23,24,25,26,27,29,30],use_single_precis:24,user:[7,14],usr:7,util:[1,3,10,12,13],v0:[0,2,4,5,8,9,29],v:27,valid:[18,21,29,30],valu:[6,9,12,13,16,17,18,20],valuesview:17,vanilla:[12,13],vari:[9,12],variabl:[7,9,20,23,24,26,28],varianc:[15,16],variou:[1,17,30],vastli:9,vec2quat:3,vector:[3,12,20,21,22,23,24],veloc:6,version:[12,29],via:[1,7,16],view:17,violat:21,visual:8,visualize_contact:30,visualize_geometri:30,visualize_gripp:30,w_t_d:21,w_t_r:21,wa:[6,9,16],we:[6,7,9,12,23,24,30],weak:[12,13,14,15,16,17,21],weight:[12,13,18],weld:6,well:[1,9,12,13,14,17,28],what:14,when:8,whenev:4,where:8,which:[11,12,15,16,18,21,22],window:18,within:[20,21],without:[8,12,14,17,29],work:[0,5],worker:[13,18],world:[6,14],world_siz:[14,16],worldgen:3,worth:20,wp:27,wrap:14,wrap_ob:14,wrapper:[1,12,13],wrench:26,wrist:6,written:24,www:[3,27],x0:23,x:[3,12,16,20,23,24,27],xinit:[24,28],xopt:21,y:[3,12],yaml:[7,8,9,29],yield:12,you:[7,8,9,29],your:[7,9],z:[3,12,27],zero:[12,16,20],zone:4,zrot_matrix:27},titles:["envs.barrett_hand","envs","envs.parallel_jaw","envs.rotations","envs.seaclear","envs.shadow_hand","envs.utils","Setup","Testing","Training","Dext-Gen documentation","mp_rl.core","core.actor","core.critic","core.ddpg","core.noise","core.normalizer","core.replay_buffer","core.utils","Acknowledgements","Constraint functions","Controller","optim.core","core.interior_point","core.optimizer","Grasp target objects","Grippers","Gripper kinematics","Objective functions","optim","Utility modules"],titleterms:{"function":[20,28],acknowledg:19,actor:12,agent:[8,29],an:9,barrett_hand:[0,26,27],base_geometri:25,base_gripp:26,common:7,configur:9,constraint:20,control:21,core:[11,12,13,14,15,16,17,18,22,23,24],credit:1,critic:13,cube:25,ddpg:14,design:29,dext:10,distribut:9,document:10,doe:29,env:[0,1,2,3,4,5,6],environ:[7,10],experi:9,gen:10,geometri:25,get:[10,29],grasp:25,gripper:[26,27],how:29,indic:10,interfac:29,interior_point:23,issu:7,kinemat:27,learn:[10,29],modul:[1,29,30],mp_rl:11,mujoco:7,nois:15,normal:16,note:[9,10],object:[25,28],optim:[10,20,21,22,24,28,29,30],packag:8,parallel_jaw:[2,26,27],pre:9,python:7,replay_buff:17,result:9,rotat:[3,30],run:9,seaclear:4,setup:7,shadow_hand:[5,26,27],start:[10,29],structur:1,tabl:10,target:25,test:8,tf:27,train:[8,9],util:[6,18,30],visual:30,work:29}})
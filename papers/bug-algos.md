## A Comparative Study of Bug Algorithms for Robot Navigation 

K. N. McGuire[1] _[∗]_ , G.C.H.E. de Croon[1] and K. Tuyls[2] 

> 1 _Delft University of Technology, The Netherlands_ 

> 2 _University of Liverpool, United Kingdom *Corresponding Author: k.n.mcguire@tudelft.nl_ 

## **Abstract** 

This paper presents a literature survey and a comparative study of Bug Algorithms, with the goal of investigating their potential for robotic navigation. At first sight, these methods seem to provide an efficient navigation paradigm, ideal for implementations on tiny robots with limited resources. Closer inspection, however, shows that many of these Bug Algorithms assume perfect global position estimate of the robot which in GPS-denied environments implies considerable expenses of computation and memory – relying on accurate Simultaneous Localization And Mapping (SLAM) or Visual Odometry (VO) methods. We compare a selection of Bug Algorithms in a simulated robot and environment where they endure different types noise and failure-cases of their on-board sensors. From the simulation results, we conclude that the implemented Bug Algorithms’ performances are sensitive to many types of sensor-noise, which was most noticeable for odometry-drift. This raises the question if Bug Algorithms are suitable for real-world, on-board, robotic navigation as is. Variations that use multiple sensors to keep track of their progress towards the goal, were more adept in completing their task in the presence of sensor-failures. This shows that Bug Algorithms must spread their risk, by relying on the readings of multiple sensors, to be suitable for real-world deployment. 

## _Keywords:_ 

Bug Algorithms, Robotic Navigation, Comparative Study, Limited Sensing, Indoor Navigation 

## **1. Introduction** 

Robotic indoor navigation of robots has been a soughtafter topic for the last few decades within the robotic community. An important stimulus for this interest is its potential for a wide range of scenarios, e.g. searchand-rescue, greenhouse observation, industrial inspection. Indoor navigation also comes with a wide range of issues, such as the absence of a reliable GPS-signal and wall interference in long-range communication. An indoor robot should preferably be autonomous and able to navigate based on its on-board sensors and computational capacity. 

There has been tremendous progress in autonomous robotic navigation, up to a point that some researchers believe this to be an already solved problem. With the emerging autonomous cars, simultaneous localization and mapping (SLAM) has reached high maturity in development (see Bresson et al. (2017) for a review). SLAM is a notoriously complex and expensive algorithm, consuming much of the robot’s on-board progressing power. To strive towards computationally efficient methods is advantageous for any robot, but it becomes vital when the application requires the use of tiny, light-weight robots. For instance, small Micro Aerial Vehicles (MAVs), in the order of 50 grams and 15 cm diameter, could be ideal for exploring small and confined spaces. However, their on-board 

**==> picture [131 x 143] intentionally omitted <==**

**----- Start of picture text -----**<br>
Target<br>Agent<br>Start<br>**----- End of picture text -----**<br>


Figure 1: An example of an agent performing a Bug Algorithm-like behavior, while navigation in a indoor environment. From a starting position (bottom-right), it moves towards the target (top-left), where it tries to move towards the target whenever it can, and follows the obstacles’ boundary when it hits an obstacle. Its trajectory is given in green. 

computational resources are so limited that currently they cannot make use of SLAM methods. 

Given these strict computational requirement for tiny robotic platforms, an important question is raised: does the actual simple principle of navigation, _going from point A to point B_ , need the computational and memory requirements for constructing and maintaining high-resolution metric maps? Should the complexity of the strategy not 

_Preprint submitted to Robotics and Autonomous Systems_ 

_August 20, 2018_ 

be proportional to the simplicity of the task? 

There are several light-weight alternatives to SLAM to consider, such as Topological SLAM (see Boal et al. (2014) for a review). Biologically inspired techniques like the Snapshot Model (Cartwright and Collett (1983)) and the Average Landmark Vector (Lambrinos et al. (2000)) can also be considered. These efficient methods, however, still have the tendency to scale up the memory requirements, when navigating in a more complex and large environments. 

In this article, we will look at a navigation method of a different kind: _Bug Algorithms_ . Although the name suggests a biological origin, it is a path-planning technique that evolved from maze-solving algorithms. The main principle of Bug Algorithms is that they do not know the obstacles in their environment and only know their target’s relative position. They will locally react only upon contact with obstacles and walls, in a way that lets the agents progress towards their goal, by following the obstacle’s boundaries (”wall-following”), as illustrated in Fig. 1. The nature of Bug Algorithms is ideal for indoor navigation on tiny, resource-limited, robotic systems, as their potential memory and processing requirements are low, therefore expected to take up little space on the on-board computer. This will free up resources for other tasks/behaviors. 

In this paper we will delve into Bug Algorithms in more detail, by providing an overview of the techniques existing today. There have been two comparative studies on Bug Algorithms before (Noborio et al. (2000), Ng and Br¨aunl (2007)), however the biggest difference is that we will evaluate how suitable Bug Algorithms are in becoming a new navigation standard within robotics. Here we will look into the assumptions made about the environment and we will evaluate whether they are realistic. An important conclusion of our study is that Bug Algorithms tend to heavily rely on a perfect position estimation, which cannot be taken for granted in a GPS-deprived indoor environment. Global positioning systems could be set up beforehand, such as a motion capture or Ultra-Wide-Band (UWB) localization system (like in Mueller et al. (2015)). However, in cases like search-and-rescue scenarios, it is undesirable to have humans prepare the robot’s environment. The robots would need to rely on their estimated position, obtained by their own, noisy, on-board sensors. 

We will compare a representative subset of Bug Algorithms in the ARGoS simulator, which is capable of modeling realistic physical interactions with objects in the environment. Although we will not implement as many Bug Algorithms as Ng and Br¨aunl (2007) did, we will test them in more realistic real-world conditions, containing elements such as odometry-drift or recognition-failures. We investigate their behaviors on hundreds of procedurally generated indoor environments, to compare their performance statistically. Here it is shown that the increased measurement noise on the on-board sensors causes a dramatic drop in overall performances of the Bug Algorithms. We will discuss how this affects the potential of Bug Algorithms in 

robotic navigation and what type of assumptions we can make about the environment, which can point us to the variations that are the most suitable. 

An overview of Bug Algorithms is given in section 2, starting from their ”maze-solver” origins, to the fundamental contact-based Bug Algorithms, to the more recent extended range-based versions and hybrid solutions. This is followed by a sum-up of the methods already used in robotic navigation in section 3. Subsequently, we perform a quantitative comparison of the Bug Algorithms performances, of which the setup is explained in section 4. The experiments themselves are discussed in section 5, and involve various degrees of sensor-noise and -failures. The findings of this paper will be discussed in section 6, from which we will present our conclusions in section 7. 

## **2. Theory and Variants of Bug Algorithms** 

The late 80s is when the term _Bug Algorithms_ first came into existence, evolving from the existing path planning algorithms like Dijkstra (Dijkstra (1959)) and A* (Hart et al. (1968)). However, the latter methods need to know their environment in advance, which includes start and goal positions, all obstacles and their position along the way. With this information, they need to find the quickest path from A to B within a predefined scenario[1] . But yet, what if the location, size, shape and the amount of those obstacles are not known? 

## _2.1. Origins: Maze solving algorithms_ 

Maze-solving algorithms first explored the navigational problem without knowledge about the environment, for enclosed spaces with walls and only one entrance and exit. The _random-walker_ algorithm is the simplest technique to solve a maze (Evans (2017)). It moves in a straight line until it encounters an obstacle. At that point, it will choose an arbitrary and oblique direction to go to next. Luck determines the random walker’s success and it could take a significant amount of time before the exit is reached. 

If the target is reachable through a series of interconnected walls, a _wall-follower_ would guarantee a quicker solution than the random mouse (Mishra and Bande (2008)). Its left or right side must be in contact with the boundary of the obstacle or wall while it moves towards the exit. However, if the environment is not an interconnected maze and contains disjoint obstacles between the start- and endlocation, the wall-follower might get stuck in an endless loop. 

The _Pledge_ algorithm can handle a maze with disjoint walls (Abelson and DiSessa (1986))[2] . The Pledge-agent will first commit (”pledge”) to a fixed oblique direction in heading and moves there in a straight line. If it hits 

> 1Also called the ”piano movers problem” 

> 2The Pledge algorithm was originally intended as a mathematical educational tool 

2 

**==> picture [263 x 86] intentionally omitted <==**

**----- Start of picture text -----**<br>
T T T<br>S S S<br>a) Com b) Bug1 c) Bug2<br>**----- End of picture text -----**<br>


Figure 2: The behavior of simple Bug Algorithms: a) Com, b) Bug1 and c) Bug2. The _S_ and _T_ depicts the start and target position respectively. 

an obstacle, it will adapt a wall-following behavior, while monitoring the changes in heading. If the angular sum of its heading, with respects to its initially committed heading, returns to 0 _[◦]_ (here not equivalent to 360 _[◦]_ ), the Pledge algorithm will leave the obstacle at that point and continue to follow the original direction it started out with. This enables the Pledge agent to also handle mazes with disjoint walls, which is an improvement from the simple wall-follower. However, this algorithm will by itself not move directly towards the exit, as it does not have any knowledge of where it is. If, for instance, its final goal is a fixed position located in an wide open space, the Pledgealgorithm could miss it entirely. 

## _2.2. Contact Bug algorithms_ 

Typical indoor environments have corridors, rooms and disjoint obstacles, where Bug Algorithms (BAs) should be able to solve the path planning problem. Lumelsky and Stepanov (1986) are the pioneers in developing this new technique. At first, they described a very simplistic BA, called the ”common sense algorithm” which can be abbreviated as _Com_ . Fig. 3(a) shows a state machine of the BA, where it will move towards the target whenever it can. This results in the behavior illustrated in Fig. 2(a). The position where a BA hits the obstacle for the first time is called a _hit-point_ , and it has a _leave-point_ as soon as the direction to the target is free. Intuitively, Com could solve many situations; however, Lumelsky and Stepanov (1986) pointed out that there are scenarios in which it cannot reach the goal. This happens when introduced to a more complex environment as, for instance, the one illustrated in Fig. 4(a). 

In the same paper of Lumelsky and Stepanov (1986), the _Bug1_ algorithm was introduced, following a different strategy to overcome the issues that Com is facing (see Fig. 3(b) for its state machine). Every obstacle Bug1 comes across, it first has to ”explore” the obstacle by following its entire border, while simultaneously keeping track of which position is the closest to the target, as shown in the simple environment in Fig. 2(b). After it encounters its original hit-point, Bug1 will continue and move towards the position closest to the target, from which it will leave the obstacle. The path length will therefore never exceed the limit: _P_ = _d_ ( _S, T_ ) + 1 _._ 5 _·_[�] _pi_ , where P is the total 

**==> picture [262 x 596] intentionally omitted <==**

**----- Start of picture text -----**<br>
Start<br>Move<br>toward T<br>Reached T<br>Way<br>towardsis free T Stop If Oi is hit<br>Follow Start<br>boundary<br>(CW) Move<br>towards T<br>Reached<br>a) Com Li Reached T If Oi is hit<br>Oi Go closestto Li onto Stop Store Hi<br>T<br>Reach Hi of Follow<br>Determine M-Line Oi Boundary(CW)<br>from S to T<br>Start Store closest Li to T<br>Move toward<br>T along M-line b) Bug1<br>Reached T<br>Hit M-line &<br>closerthan to HiT Stop If Oi is hit<br>Follow<br>boundary<br>(CW)<br>Start<br>Move<br>toward T<br>c) Bug2<br>Reached T<br>Way<br>towardsis free & T Stop If Oi is hit<br>d ( Hi, T  )><br>d ( x, T  )<br>BoundaryFollow Stored ( Hi, T  )<br>(CW)<br>Determine M-Line<br>from S to T<br>Start<br>d) Com1<br>Move toward<br>T along M-line<br>Reached T<br>Hit M-line<br>& T closerthan Hi to Stop If Oi is hit<br>Save Hi<br>Follow<br>boundary<br>CW CCW<br>Hi is Start<br>hit<br>Move<br>toward T<br>e) Alg1<br>Reached T<br>towardsWay T Stop If Oi is hit<br>is free &<br>d ( Hi, T  ) > Store<br>d ( x, T  ) d ( Hi, T  )<br>Follow and Hi<br>boundary<br>CW CCW<br>Hi is<br>hit<br>f) Alg2<br>**----- End of picture text -----**<br>


Figure 3: Bug algorithm state machines. The _S_ and _T_ represent the start and target position respectively. _Oi_ is the i-th obstacle that the bug hits and _Li_ and _Hi_ is the i-th leave- and hit-point, respectively. 

3 

**==> picture [251 x 253] intentionally omitted <==**

**----- Start of picture text -----**<br>
T T<br>T<br>S S<br>S<br>a) Com b) Bug1 c) Bug2<br>T T T<br>H 3 H 3 H 3<br>H 2 H 2 H 2<br>H 1 H 1 H 1<br>SEES S S S<br>d) Com1 e) Alg1 f) Alg2<br>T T T<br>S S S<br>g) DistBug h) Rev1 i) Rev2<br>**----- End of picture text -----**<br>


Figure 4: Generated paths by the Bug Algorithms (a) Com, (b) Bug1, (c) Bug2, (d) Com1, (e) Alg1, (f) Alg2, (g) DistBug, (h) Rev1 and (i) Rev2 in a more challenging environment. The _S_ and _T_ depicts the start and target position respectively and _Hi_ means the i _[th]_ hit-point. x is the current position of the agent and CW and CCW stand for Clock Wise and Counter Clock Wise respectively. 

path length, _d_ ( _S, T_ ) the direct distance between the start (S) and target (T) location and _pi_ is the length of the boundary of the i _[th]_ obstacle. Bug1 is able to handle environments where Com failed (as seen in Fig. 4(b)); however, it is a less intuitive approach. As it needs to know the entire border of the obstacle, this will naturally create unnecessary long paths. 

Lumelsky and Stepanov (1987) recognized the nonoptimal path-lengths of Bug1, and therefore proposed an alternative: _Bug2_ . Between the beginning and end position, an imaginative line is drawn, called the _M-line_ (see Fig. 3(c) for Bug2’s state-machine). In the simple scenario of Fig. 2(c), this means that the bug will follow the obstacles border until it hits the same M-line at the other side. As long as that point is closer towards the target than the hit-point’s position, it will depart from the obstacle. This reduces the maximum possible BA’s path length to _P_ = _d_ ( _S, T_ )+1 _· pi_ , which is also illustrated by Fig. 4(c). 

Sankaranarayanan and Vidyasagar (1990) still found scenarios in which Bug2 would still produce an unnecessary long path. According to them, it is because of its incapability to store and compare previous visited points along the obstacle’s boundary. They extended the Bug2 algorithm with the following principle: to change its wallfollowing direction if it comes across a previously visited hit-point along the border of the obstacle. It has been 

dubbed as _Alg1_ , which can be seen in Fig. 3(d). It is true that in some situations a shorter path will be generated, however Alg1’s maximum _possible_ path length is longer: _P_ = _d_ ( _S, T_ ) + 2 _·_ } _pi_ . Fig. 4(e) shows an example of its behavior in a complex environment. 

Sankaranarayanan and Vidyasagar (1990) also expressed interest for the intuitive method Com, as it does not exploit the restrictive M-line, but leaves the boundary as soon as there is a free space between the BA and the target. They suggested an extended version of Com, _Com1_[3] , which remembers the previous hit-point’s distance to the target. Com1 will utilize this as an extra argument in his state-machine (Fig. 3(d)), to initiate the departure from the obstacle boundary, as seen in Fig. 4(d). Based on Com1, _Alg2_ was created in the same paper of Sankaranarayanan and Vidyasagar (1990) as well, where it, similar to Alg1, reverses the wall-following direction if it encounters a previous saved hit-point (Fig. 3). Alg2 therefore needs to keep track of all previous hit-points on its way for the reverse local direction condition, as it this will occasionally occur (Fig. 4(f)).[4] 

Kamon and Rivlin (1997) created a BA quite similar to Alg2, _DistBug_[5] . The only difference is that it will not remember the positions of all the previous hit-points, but solely the last one, therefore making it more memory efficient. Another intriguing aspect of DistBug, is that there is no fixed initial local wall-following direction along the boundary of the obstacle, as it depends on the orientation on which the BA touches the hit-point. Most times, this will naturally lead it to the target and result in a shorter path, which is noticeable in the environment illustrated in Fig. 4(g). However, there are situations where such a policy will fail, as in Fig. 5(a). At the first hit point, it would be better to follow the wall in the other direction. 

An extension to both Alg1 and Alg2 was proposed by Horiuchi and Noborio (2001), named _Rev1_ and _Rev2_ respectively. Both BAs will alternate their local direction at each (new) hit-point, which is a good strategy for environments like in Fig. 4(h) and (i). Rev1&2 save the chosen local direction and its associated hit-point in a list. If these locations are revisited again, the bug algorithm will chose the opposite local direction than the one stored. However, the ”best” choice for the local wall-following direction is not trivial. Fig. 5(b) and (c) show a situation where alternating the local wall-following direction is not the best policy. One may argue that the shown case is disadvantageous to Rev1 and Rev2, as they do not encounter any previous hit-points on their path. However, the examples does show that the best choice of local direction de- 

> 3This is also being referred to as _Class1_ in the studies of Noborio et al. (2000) and Ng and Br¨aunl (2007) 

> 4 The statemachine of Com, Com1, Bug2, Alg1 and Alg2 are also available as pseudo code in appendix A, as they will be implemented later in this paper for the comparative study. 

> 5Here we are revering to the extended DistBug algorithm of the same paper, with the search manager and local-direction choice based on the slope of the wall. 

4 

**==> picture [238 x 107] intentionally omitted <==**

**----- Start of picture text -----**<br>
S S S<br>T T T<br>a) Distbug b) Rev1 c) Rev2<br>**----- End of picture text -----**<br>


Figure 5: An alternative complex environment to show a case that would produce a long path-length for a) DistBug, b) Rev1 and c) Rev2. 

pends on the environment. It is, therefore, difficult to find a generic strategy for determining the best wall-following direction. 

## _2.3. Bug Algorithms with a Range Sensor_ 

What if the robot is able to sense obstacles already at a certain range and therefore act before touching the obstacles physically? Lumelsky and Stepanov (1986) already mentioned this idea in their first paper, which has been materialized in Lumelsky and Skewis (1988) and Lumelsky and Skewis (1990) as _VisBug 21 & 22_ . Both are based on Bug2, but are also equipped with a range sensor able to sense up to a given maximum range. The BA will still follow the M-line but they can detect ”short-cuts” to the next obstacle which should reduce the total path traveled,[6] as can be seen in Fig. 6(a). 

Kamon et al. (1996) introduced a successful version of the range-based Bug Algorithms, called _TangentBug_ . Within the maximum range of its sensor, a local tangent graph (LTG) is constructed, as illustrated in Fig. 6(b). The LTG represents the discontinuities/borders of the detectable obstacle field around the robot. It starts out by moving towards the target while traversing the LTG edge that results in the quickest path to the target ( _T_ ) from its current position ( _x_ ). This goes as follows: 

**==> picture [226 x 11] intentionally omitted <==**

**==> picture [232 x 11] intentionally omitted <==**

**==> picture [187 x 11] intentionally omitted <==**

, where _Di_ is the distance of the agent towards the left or right obstacle _Oi_ ( _d_ ( _x, Oi_ )) plus the remaining distance from that obstacle to the target _d_ ( _Oi, T_ ). TangentBug will always follow the LTG edge which is expected to result in the smallest path towards the target. However, if _D_ of that edge increases, it will save the current range to the target as a local minimum ( _dmin_ ) and will continue following the remaining boundary of that obstacle. If the robot senses a 

> 6No indication of the path length is given here, however, of the complexity, this will no longer be mentioned from here on in this report. 

**==> picture [254 x 129] intentionally omitted <==**

**----- Start of picture text -----**<br>
T T<br>d ( OL, T  ) LeavePoint T<br>OL dmin<br>LTG ORd ( OR, T  )<br>r<br>S<br>S S<br>a) VisBug b) TangentBug c) Special Scenario<br>Tangentbug<br>**----- End of picture text -----**<br>


Figure 6: The Bug Algorithms developed with obstacle detection with range sensors: (a) VisBug and (b) TangentBug. The _S_ and _T_ depicts the start and target position respectively. _r_ stands for radius of the range sensor. LTG stands for local tangent graph and _OL_ and _OR_ stand for the left and right border of the detected obstacle within the range sensor respectively. ( _d_ ( _OR, T_ ) and ( _d_ ( _OL, T_ ) stand for the distance between the left and right obstacle boundary to the target, respectively. (c) A close up of a scenario in which Tangent bug is able to handle local minima. 

node on the boundary of the obstacle that is smaller than _dmin_ , it trigger a leave-condition and, if possible, moves directly to the target (see Fig. 6(c)). Kamon et al. (1999) extended TangentBug to operate in 3D-environments as well ( _3DBug_ ). 

TangentBug is probably the most referred work in the field of BAsand many variants of it exists. The 360 _[◦]_ range sensor assumption is changed to a sensor with a limited field of view with _WedgeBug_ (Laubach and Burdick (1999)), for instance,to represent a stereo camera. Magid and Rivlin (2004) developed a BA which will actively search for the right local wall-following direction, to prevent a long-path length. Their _CautiousBug_ will not choose a direction based on the angle of attack on the hitpoint, as DistBug, but will first do a spiral search along the border, with the hit-point in the center. A disadvantage of this method seems that the spiral search by itself will also produce a long path, therefore it has less of an advantage over Tangentbug. A newer variation is _InsertBug_ by Xu and Tang (2013), which navigates by means of way-points, placed on a safe distance from the obstacle’s boundary. This could be seen as a version of TangentBug that adds a safety margin to each obstacle detected. 

## _2.4. Special Bug Algorithms_ 

Some BAs either take a special approach or are combined with other existing methods ( _HybridBugs_ ). Lee et al. (1997) used fuzzy logic with an adjusted Com method, a.k.a _FuzzyBug_ . Assuming the BA is equipped with two single-beam sensors, pointed forward on both sides, it can detect if an obstacle is closer to its right or left. Based on a fuzzy membership function, FuzzyBug decides to follow the obstacle’s boundary on its right or left, which a similar approach to DistBug’s strategy. 

Noborio et al. (1999) developed HB-I, which is another HybridBug. After each hit-point of the obstacle, HB-I 

5 

moves along the border in both directions until it hits a corner. It will then select the best direction first, based on the best-first search of a decision tree. Xu (2014) used a different approach with _RandomBug_ . Once it detects an obstacle, it generates random points within the range of its sensor. From these points, RandomBug selects the optimal one, dependent on how close the point is to the target, and generates a motion vector towards it. This produces a path quite similar to InsertBug, but the process is highly related to rapidly-exploring random trees (LaValle and James J. Kuffner (2001)). 

Taylor and LaValle (2009) developed _IBug_ , which is short for Intensity-bug. Its only information about its target is a wireless beacon on the specified location, of which it will navigate towards by means of the signal strength. Since they assume that IBug can make use of a ”towerorientation” sensor, the agent will move towards the beacon location. When it does, IBug will temporarily save the value of the intensity ( _iH_ ) at that very moment. Here, a high intensity (signal strength) means a short distance to the target and a low intensity a large distance. While the robot follows the obstacle’s boundary (always CW or CCW), it compares the current intensity level with _iH_ , as well as the current intensity and of time-steps back. If the signal strength decreases after increasing, the agent will have detected a local minima and a leave-condition is triggered, but only if the current measured intensity is larger than _iH_ . Although the leave condition is different, the latter comparison of intensity levels at the hit- and leave-points is quite similar in approach to Com1, with intensities substituted for distances. 

## _2.5. Overview Bug algorithms_ 

The BAs discussed in the previous sections are visualized in the overview of Fig. 7, where they are connected based on their development and features.We subdivide the algorithms in a few major categories. The main division already started in the paper of Lumelsky and Stepanov (1986), where they presented Com, Bug1 and Bug2. Com led to a series of Bug Algorithms that navigated in an azimuth angle towards the goal whenever it had the chance to do so. Hence, here we categorize them as _Angle-Bugs_ . Lumelsky and Stepanov (1986) realized that their next creation, Bug1, would create long trajectories by default. The community seems to have agreed as no extension or variation of Bug1 was developed here after, so therefore no similar Bug Algorithms has emerged since. Lumelsky and Stepanov (1986)’s alternative solution, Bug2, did have more potential, so new variations of _M-line-Bugs_ have been presented, leading to a separate category of BAs. 

Com is arguably the most simplest BAs, as it uses no memory, nor determines any M-line. Although for some simple environments this proved to be sufficient, Com has a chance of never reaching its final destination in more complex scenarios. With Com1, Sankaranarayanan and Vidyasagar (1990) added a distance-based leave condition, 

**==> picture [237 x 340] intentionally omitted <==**

**----- Start of picture text -----**<br>
Bug<br>Angle-bugs Algorithms<br>COM<br>StepanovLumelsky(1986)and Lumelsky Bug1 and M-line-bugs<br>Stepanov (1986)<br>Bug2<br>Lumelsky and<br>Closest Stepanov (1986)<br>COM1 distancegoal to<br>Sankaranarayanan<br>and Vidyasagar<br>(1990) Ibug<br>Taylor and<br>LaValle (2009)<br>Alg1<br>Sankaranarayanan<br>Position and Vidyasagar<br>Alg2 hit-points (1990)<br>Sankaranarayanan<br>and Vidyasagar<br>(1990) Local<br>directions at<br>hit-point Rev1<br>Horiuchi and<br>DistBug Kamon and Rev2 Noborio (2001)<br>Rivlin (1997) Horiuchi and<br>Noborio (2001)<br>VisBug 21<br>& 22<br>HB-I & Ave Lumelsky and<br>Noborio et al. Skewis (1988)<br>FuzzyBug (1999) Noborio<br>Lee et al. (1997) et al. (2004)<br>TangentBug Kamon et al. Cautious<br>InsertBug (1996) Magid Bug and Rivlin<br>Xu and Tang (2004)<br>(2013) Random Wedge<br>Bug Bug<br>Xu (2014) 3DBug Kamon and<br>Kamon et al. Rivlin (1997)<br>Range-bugs (1999)<br>Complexity Bug<br>Algorithm<br>**----- End of picture text -----**<br>


Figure 7: An overview of all the discussed Bug Algorithms (BAs) in section 2. These BAs are presented in a development tree of increasing complexity. It makes a distinction between Angle-Bugs, BAs that move to the target’s azimuth direction, M-line-Bugs, BAs that use an M-line to navigate, and Range-Bugs, which use a range sensor to detect obstacles. The BAs noted in a dotted circle are special/ hybrid-bugs. The gray blobs indicate the type of memory and leave-condition added to the method. The latter is only shown until Rev1&2. 

where it will only leave the obstacle if it reached a position closer as it has been before. This requires Com1 to remember what its latest closest distance was to the goal and use it in the techniques’s leave-condition, which has been adopted by the following BAs ever since. 

Sankaranarayanan and Vidyasagar (1990)’s Alg1 and 2 are given additional memorization tasks. Not only do they remember the previous minimal distance to the goal, but all the hit-points’ locations in between as well. If Alg1&2 encounter one of those saved hit-points, they will reverse the local direction of their wall/boundary-following. Horiuchi and Noborio (2001) went one step further and made the Rev1&2 remember their last local wall-following direction, together with the local direction chosen at each hit-point, and alternates at each revisit. However, DistBug uses a more memory-friendly approach to determine 

6 

his local position, which is purely based on the detected slope of the approached obstacle, which sets it apart from Rev2. 

Fig. 7 shows that BAs started to use range-sensors at one point, creating the _Range-Bugs_ category. Visbug21&22 were able to find shortcuts from the M-line to the obstacle’s boundary. Both FuzzyBug and TangentBug used their range-sensor to choose the expected best local direction and guide the obstacle-following behavior and the same holds for the many variants of TangentBug. 

We make some general observations about the overview in Figure 7. Firstly, there are more Angle-bug-based BAs than M-line bugs. This is likely thanks to their more intuitive and less restrictive navigation strategy towards the target. Secondly, more and more features are added to the BAs as time progresses. Each new BA builds on top of an other, adding new leaving conditions and memory capabilities, therefore increasing the bug’s complexity in the hope to find more efficient paths. The sole exception is the more recent Ibug, which is a more recent variation, but is only one step away from Com1 in complexity. 

## **3. Bug Algorithms for Robotic Navigation** 

The BAs presented in the last section are considered as a potential new robotic path planning paradigm, because of their simplicity and low memory footprint. We first will discuss how the principle of BAs translates to realistic operating conditions. Afterwards, existing BA robotic implementations will be presented, discussing how well these studies represent real-world scenarios. 

## _3.1. Bug Algorithms in Real-World Conditions_ 

In the earlier literature overview in section 2, it seems to be the case that BAs heavily rely on perfect localization. They almost all assume that the BA does not know the exact location and shape of the obstacles, however they almost all need to know the exact coordinates of their goal position and their own position. The latter is used for more aspects of BAs than first meets the eye: 

- Angle Bugs (i.e. Com, Com1, Bug2, Alg2) need to know the distance and azimuth angle to the target at any point. 

- M-line-Bugs (i.e. Bug2, Alg1, Rev1) both remember the exact line (and direction) between the starting position and the goal, and recognize if they have reached it. 

- Hit-point memorizing BAs (i.e Alg1&2) need to match their current position estimate with previously hitpoint coordinates. 

In a typical indoor GPS-deprived environment, obtaining and maintaining a world position is a significant challenge.. An external global localization system can be set up (e.g. motion capture, UWB triangulation); however, 

in many scenarios (e.g., in a search-and-rescue scenario) there might not be the possibility or time to do this. Realworld robots will need to rely on odometry, which is prone to errors and has the tendency to drift in time from the ground truth. With ground-bound robots, wheel slippage (Borenstein and Feng (1996)) can cause an increasing error between the real and estimated position. The same goes for visual odometry (Scaramuzza and Fraundorfer (2011)), used by MAVs or hovercraft-like vehicles, where the error of the noisy velocity estimate will get accumulated over time. This is especially the case in a texture-poor environment. 

Some BAs (i.e. Alg1&2 and Rev1&2) have to remember the exact coordinates of where they have been, which ensures a convergence to the target. From a practical perspective, this means that the robot needs to recognize where it has been before. As stated earlier, this could be done by odometry. On the other hand, a BA can recognize its current position with the features of its surroundings. An omni-directional camera can observe the scene which will be memorized with local feature descriptions as SIFT (Goedem´e et al. (2007)) or global scene descriptors as Bag-of-Words (Fraundorfer et al. (2007)). It then has to evaluate that template of features the entire time while it is traversing along the border of an obstacle. As with visual odometry, the descriptor’s performance depends on the texture of the environment. Practically, this will take up extra capacity of the on-board computer. On a limited platform, this is something that is best avoided. Moreover, this principle tends to move towards the definition of map-based navigation. 

Most BAs use a Distance-to-Target (DT) measurement in their leave-condition. Next to using the drift-susceptible odometry, they could also retrieve the DT in ways such as received signal strength intensity (RSSI) of BlueTooth (Bargh and de Groote (2008)) or Ultra-Wide Band (UWB, Guo et al. (2017)). This does of course require the placement of a wireless transmitter at the target location. Moreover, none of these sensors are perfect. DT estimation by BlueTooth RSSI could get an error up to 2 meters and can not practically determine a range from 5 meters on[7] (Coppola et al. (2018)), which is influenced by the amount of environment clutter. UWB has better statistics, with a standard deviation of 0.2 meters and a maximum limit up to 100 meters[8] , with less interference from walls and obstacles in between. Even if the distance measurements are very good, the higher energy expenditure of the latter could be a valid reason to prefer the more energy efficient BlueTooth dongle. 

## _3.2. Existing Implemented Bug Algorithms for Robotic Navigation_ 

This section will look at current robotic BA implementation, either in a real world environment or a simulated 

> 7This is based on a Bled112 Bluegiga Bluetooth module 

> 8This is based on a DecaWave UWB module in ranging mode 

7 

Table 1: Robotic implementations of various Bug Algorithms (BAs). These are evaluated on the type of platform used, whether the environment was real or simulated and which BA type was used. Moreover, it shows the used local sensors for obstacle detection and the used global sensor for a position estimate. 

|Paper|Platform|Environment|Bug<br>algo-<br>rithm|Local sensors|Global sensors|
|---|---|---|---|---|---|
|Kamon and Rivlin (1997)|Wheeled robot|Real|DistBug|Range sensors|Global localization<br>(system not given)|
|Laubach and Burdick (2000)|Microrover|Real|RoverBug<br>(wedgebug<br>extended)|Stereo images|Guiding<br>operator<br>(First person view)|
|Mastrogiovanni et al. (2009)|Wheeled robot<br>Hexapod robot|Real|_µ_NAV|Ultrasound range sensor<br>Wheel odometry|Azimuth<br>angle<br>by<br>photo diodes<br>(only for hexapod)|
|Zhu et al. (2010)|Wheeled robot|Real|Bug2<br>and<br>a<br>DistBug vari-<br>ant|Laser scanner (180 deg)|Global localization<br>(system not given)|
|Kim et al. (2013)|Wheeled robot|Real|Tangentbug<br>(adjusted)|Ultrasound range sensor<br>Wheel odometry|Global localization<br>(system not given)|
|Taylor and LaValle (2014)|Wheeled robot|Real|Ibug|Touch sensors|IR Beacon|
|Ebrahimi et al. (2014)|Quadcopter|Simulation|UavisBug|Camera|Motion Capture Sys-<br>tem|
|Gulzar et al. (2015)|Wheeled robot|Real|Not Given|Ultrasound|Motion Capture Sys-<br>tem|
|Marino et al. (2016)|Quadcopter|Simulation|Bug2|Laser scanner (180 deg)|UWB localization|



scenario. An overview of these methods is presented in Tab. 1, which lists the platform they used and shows the sensors the robot was equipped with for local obstacle sensing and global position estimations. 

Kamon and Rivlin (1997) were one of the first to consider more realistic sensors for the agents in BAs. With DistBug, they showed, as one of the first, a BA implemented on an actual wheeled robot, a Nomad200. In their paper they mention that the robot, while moving to the target, only responds to local measurements by the contact sensors. However, the robot always moves towards the target after boundary following, therefore, it must also know its own and the targets position in global coordinates. Although the paper of Kamon and Rivlin (1997) has not specified this, their BA would need to use a global localization system. 

Laubach and Burdick (2000) extended their earlier developed WedgeBug to _RoverBug_ , for implementation on a real-world micro-rover. It used a stereo camera to detect and follow the obstacles. However, the initiative to leave the obstacle to move towards the target is controlled by a tele-operator, which monitors the rover through a first-person-view image feed. Zhu et al. (2010), Kim et al. (2013) and Gulzar et al. (2015) have implemented a BA on autonomous real-world wheeled robots without a teleoperator. In all cases, they were using single beam range sensors and/or a laser scanner. Again, the exact location of the robots is needed in order for the BA to move towards the target. Unfortunately, the papers do not specify which type of global localization system was used in their experiments. 

Mastrogiovanni et al. (2009) acknowledged that a robot would not be able to know its exact position, but would need to infer it from its noisy on-board sensors. They implemented _µNav_ on a real-world wheeled robot, AmigoBot 

and a hexapod, Sistino. The first platform used ultrasonic sensors for obstacle detection and wheel-odometry for global localization. Since the wheeled robot combined its wheel-odometry with the azimuth angle toward the target, it could reach the target location from one room to another, even if the orientation was manually perturbed by the researchers. However, the operation area only spanned across a few rooms and no notion was given of what the navigational limit was, based on accumulated errors of odometry drift. Their second robot, the hexapod, was not able to use odometry, so the azimuth angle had to be given by an external source through photo diodes. 

Taylor and LaValle (2014) implemented IBug on a small wheeled robot for several small-scale environments. In their previous work (Taylor and LaValle (2009)), they described the BA to be suitable for navigating towards a single wireless beacon. Nevertheless, for the test on a real robot, a Lego-Mindstorm-based platform, Taylor and LaValle (2014) used an infra-red (IR) beacon instead. It proved to be challenging for their tests to use the signal strength of i.e. a WiFi beacon at a large range. The use of the IR beacon did necessitate a constant line of sight, which required the obstacles and walls to be lower than the robot itself. Moreover, the IR sensor could detect a low-resolution bearing towards the beacon, but not the distance towards it. This means is that the minimal-distancebased leave-condition from IBug could not be used. Although the tested environments did not require this extra argument, it will be essential once loop-detection is required in more complex environments. 

Marino et al. (2016), from the same group as Mastrogiovanni et al. (2009), created a simulation of a MAV to navigate through multiple floors. Bug2, enhanced with a potential-field-based boundary following, is implemented on the simulated quadcopter. The model was equipped 

8 

with a 360 _[◦]_ laser scanner and a salient cue sensor, which is used to detect the target. For simulation it was assumed that its exact location is known, referring to recent UWB localization systems. Moreover, if the agent believes it is at the right goal position but on a different floor, it will use the Dijkstra method to compute the shortest path. This is an interesting choice, as the original Dijkstra algorithm does need to know the grid map of the environment and its obstacles, which is opposite to the problem that BAs intend to solve. 

Another simulated MAV implementation by Ebrahimi et al. (2014) assumes exact localization, mention a motion capture system. They developed _UavisBug_ for a simulated MAV for visual guided navigation. The navigation strategy exists in a 2D horizontal plane only and is quite similar to Bug2. However, they combined the BA with SLAM for the obstacle detection and boundary following, from which they used a potential force field to navigate around the obstacle. Even though, Ebrahimi et al. (2014) and Marino et al. (2016) acknowledged the limited sensing, computing and energy capability of MAVs, they still combine the efficient BAs with computationally-heavy navigation techniques. 

If we look at the existing implementations of BAs in real or simulated robots, they all assume or need explicit global localization, either by a UWB localization system, a motion capture system or a guiding navigator, for the exception of IBug, which used a visual beacon. Mastrogiovanni et al. (2009) is actually the only one that used the odometry of a (bigger) wheeled robot to recover its own position and to update the azimuth angle towards the target; however, the real-life test was too small to draw any conclusions about the suitability of BAs for robotic navigation. In the comparative study, presented in the next sections, we will test various BAs with varying amounts of odometry drift, recognition failures and distance noise. This will show that these real-world conditions will have significant effect on the BAs’ performances. 

## **4. Experimental Set-up Comparative Study Bug Algorithms** 

In this paper, we study whether BAs could be used for real-world robotic navigation. Most indoor environments have more complex obstacle configurations than an open environment with a few convex obstacles. There are many situations where the robots could get stuck on their way, particularly in rooms. In this section, we will present our motivation for this study and the chosen set of BAs to be evaluated. We will then provide the details of the simulation used and the procedural environment generator for typical indoor environments. Afterwards, the implementation specifics of the BAs will be presented, by explaining a wall-following paradigm, which is the foundation for all BAs to be implemented. 

## _4.1. Motivation and Choice Bug Algorithms_ 

There have been previous comparisons between the different BAs. In the paper of Noborio et al. (2000), Class1, Bug2, Alg1, Alg2 and HB-I, of which the latter is of their own making, were compared and evaluated on their generated path-length within a complex maze. Evaluating four different starting positions, they concluded that Class1 and Bug2 had the longest path-length and usually could not complete the task within the required amount of time. Alg1, Alg2 and in particular HD-I, had shorter path lengths and all finished in time. However, they only based their observations on just one indoor environment. 

A newer comparative study was performed by Ng and Br¨aunl (2007), on: Bug1, Bug2, Alg1, Alg2, DistBug, TangentBug, OneBug, LeaveBug, Rev1, Rev2 and Class1. They presented the BAs with four types of environments and recorded the total path length for each run. They concluded that in 3 of the 4 environments, Bug1 is the one with the longest trajectory and Tangentbug is the fastest out of the 4. However, for the other BAs, their performance could not be adequately compared due to the inconsistent results. 

Here, we test the BAs in hundreds of procedurally generated environments, so we can statistically evaluate their performances. Moreover, we also want to select a set of BAs, from the literature review in Sect. 2, to be implemented in a more realistic simulation environment. The selection will not be as large as the selection of Ng and Br¨aunl (2007) and Noborio et al. (2000), as we believe that these will have similar effects on BAs that stem from the same groups in the overview shown in Fig. 7. In the overview of the BA-methods existing today (section 2.5), it can be seen that many of the methods are natural increments of one another with increasing complexity. If the fundamental BAs can be tested with these real-world conditions, we would automatically find the issues that their descendants are facing as well. 

Specifically, we have selected Com, Com1, Bug2 and Alg1&2, based on the overview in Fig. 7. Range-bugs will not be considered as these BAs are the base of those more complex versions. Moreover, the selected BAs presents a mix of different types of strategies (Angle-Bugs and M- line-Bugs) and memory-use (distance and/or hit-points). We will exclude bugs that determine a local wall-following direction, as the policy for this choice is heavily influenced by the structure of the environment, as previously discussed in section 2.2. Moreover, any special bugs will not be considered as well, since they contain aspects and enhancements that no other BA-related research followed up on. 

## _4.2. Simulation and Procedurally Environment Generator_ 

It is our ambition to test the earlier mentioned BAs in a simulator with realistic and swift physics calculations. ARGoS, a multi-physics robot simulator developed by Pinciroli et al. (2012), is used for our comparative study. Its 

9 

**==> picture [462 x 182] intentionally omitted <==**

**----- Start of picture text -----**<br>
a) Start corridor b) Corridor c) Final result<br>agents generation corridors<br>W es es] PRS<br>d) Create corridor e) Divide rooms f) Create doors<br>walls (a) Generated environment in ArGos<br>fe d . ies<br>**----- End of picture text -----**<br>


**==> picture [150 x 182] intentionally omitted <==**

**----- Start of picture text -----**<br>
PRS<br>(a) Generated environment in ArGos<br>ies<br>**----- End of picture text -----**<br>


Figure 8: The steps of the procedural generated environment method will be explained here. The corridor-generating random agents (blue circles) start in (a) at the same positions as the start and target locations of the experiment. These will move forward in (b), while occasionally turning left and right, while leaving a corridor trace (red blocks). Once it reaches the corridor-density threshold in (c), the corridors-cells are tested for interconnectivity, such that the target position can be reached from the starting position (green circles). The corridor walls are created in (d) and then, in (e), remaining non-corridor spaces are then divided into rooms (purple stripes) and random door-openings (gray blocks) are created along the border of the corridors in (f). 

**==> picture [109 x 9] intentionally omitted <==**

**----- Start of picture text -----**<br>
(b) Modified ArGos Foot-bot<br>**----- End of picture text -----**<br>


main trait is its efficiency, which enables the simulator to run many times faster than real-time, which will be essential if the BAs are evaluated in many environments. Although ARGoS does have the capability to incorporate its own, C++ based, controller for the robots, the ROS framework is used to enable Python-based controllers. The ROS messaging system is also ideal to modulate whether a new environment needs to be generated, to vary the measurement noise and select the right bug algorithm. 

Since the BAs will be evaluated in many indoor environments, it would be unfeasible to design these by hand. Therefore, a procedural indoor environment generator will automatically generate a new arena for the bugs to navigate in. This process is depicted in Fig. 8. First, in a coarse grid world (a), two entities are initialized on the exact position of the start and target position to be in the eventual task. They will perform a simple 4-connected path generation, where they will have a certain chance of going straight ( _pstr_ ). The chance of either going left or right is equal to 1 _− pstr_ . Each agent will leave a corridor trace, as can be seen in (b), until, in (c), the amount of corridors hit a density threshold ( _tcor_ = 0.4), which is the number of grid-cells occupied with a corridor divided by the total number of existing grid cells in the environment. 

A connectivity check is performed, to check if the initial position of the robots are connected by these corridors, which will re-initiate the process in case it fails. This is to ensure that the BA is always able to reach its final desti- 

Figure 9: (a) Th resulting environment from Fig. 8(f) generated within the ARGoS simulator and (b) a modified Foot-bot simulated robot with range sensors (red lines) used for wall following. 

nation. Next, walls will be added to these corridors (d). The remaining areas will act as rooms and are divided when they are too large in (e). Finally, in (f), random openings are added along the border of the corridors to create passages these areas. Rooms are especially challenging, as they can lead to agents getting stuck in loops, which will showcase the strengths and weaknesses of the evaluated BAs. The resulting environment in the ARGoS simulation is shown in Fig. 9(a). 

The ARGoS FootBot is used for our experiments, which is a simulated wheeled mobile platform (see Pinciroli et al. (2012) for specifications). The FootBot contains many options to attach various types of sensors, however for our experiments we will only use the proximity sensors. We adapted FootBot to turn the proximity sensors into single beam range sensors with a maximum measurable distance of two meters, placed in the configuration shown in Fig. 9(b). The robot has two separate single beam range sensors located on each side and 20 range sensors pointing to the front in a wedge shape. This is to simulate a depth sensor/stereo-camera for obstacle detection with a few additional range sensors on the side. Since the robot must move towards the range-wedge configuration, its movement will be non-holonomic. 

10 

**==> picture [516 x 203] intentionally omitted <==**

**----- Start of picture text -----**<br>
(a) Calculate α<br>Start withAlignwall<br>d ( x, O⊥ ) r α rs rf rs β rf (b)<br>rf d ( <x, Odref⊥ ) R aroundTurnwall If rfrs cos ≈ β<br>rs = OR ω (e)<br>x v d =( x, Od ( x, O⊥ ) ⊥R ) C d ( x, O⊥ ) C d ( x, O⊥r ) sR (d) adjustGo rf forwardheading= OR andto (c)<br>a)wallFoot-botand calculatesapproaches α b)alignUseswith α towallturn and c) d atFollowsa preferedwall distancewhile keeping r OR f = align with wall<br>d)turnsDetectsto aligncornerwalland g) State machine wall-following<br>Legend<br>x Current position of robot d ( x, O⊥ ) R Distance to wall (RANSAC) rs rf<br>rs Value range sensor on the side d ( x, O⊥ ) C Distance to wall (2 ranges)<br>rαf ValueAngle rangeof the sensorapproachedin wedge*wall vω ForwardTurn ratevelocity e)start rf tois ORturnsoaroundwill corner<br>β Angle between rf and rs<br>Object not within range Object detected f) Once around corner,<br>align again with wall<br>**----- End of picture text -----**<br>


*closest to the side range sensor 

Figure 10: (a-f)Schematics to explain the wall-following paradigm for a right-sided local direction with (g) the corresponding state machine . OR stand for out of range. 

distance _d_ ( _x, O⊥⊥_ ) _R_ can be estimated from the robot to the wall.[[10]] If this distance becomes smaller than _drefref_ , the preferred distance from the wall, it will keep turning either CW or CCW until it is aligned with the wall. Fig. 10(b) and (c) shows this alignment for a right-side local direction. This will be the case if the measurement of the side range sensor _rss_ is equal _rff ·_ cos _β_ , where _rff_ is the element from the range wedge that is the closest to _rss_ and _β_ is the angle between _rss_ and _rff_ . 

**==> picture [518 x 282] intentionally omitted <==**

**----- Start of picture text -----**<br>
S S S distance  d ( x, O⊥⊥ ) R can be estimated from the robot to the<br>wall. [[10]] If this distance becomes smaller than  drefref , the pre-<br>ferred distance from the wall, it will keep turning either<br>CW or CCW until it is aligned with the wall. Fig. 10(b)<br>T T and (c) shows this alignment for a right-side local direc-<br>tion. This will be the case if the measurement of the side<br>c]<br>a) WF (200.10 sec) b) Com (200.10 sec) c) Com1 (137.10 sec) range sensor rss is equal rff ·  cos  β , where rff is the element<br>S S S from the range wedge that is the closest to rss and β is the<br>angle between rss and rff .<br>After the robot is aligned, it will need to follow the<br>wall, as in Fig. 10(c). Now the true distance to the wall<br>T T T ( d ( x, O⊥ ) C ) [11] will be calculated as follows:<br> ja d ( x, O⊥ ) C d = rs · rf  sin β (4)<br>d) Bug2 (150.90 sec) e) Alg1 (178.00 sec) f) Alg2 (170.20 sec) re rs [2] +  rf [2] [−] [2] [ ·][ r][s][ ·][ r][f] [ cos] [ β]<br>Figure 11: The results of (a) the wall-following only (WF) and the The derivation of the latter equation can be found in<br>implemented bug algorithms that use the same WF (b-e) as part of<br>their navigation strategy. The time limit is 200 sec. appendix B.1. The FootBot will maintain  d ( x, O⊥ ) C to be<br>near dref , and to keep being aligned in the process. How-<br>ever, since the robot’s heading and the measurements of<br>4.3. Implementation Details Bug Algorithms rs and rf are coupled, therefore a separate control heuris-<br>The most important element of any BA, is the ability tic is developed to make the wall alignment possible. The<br>follow a boundary of an obstacle or wall. Based on details of this wall-alignment method can be found in ap-<br>robot configuration in Fig. 9(b), we developed a wall pendix B.2, Alg. B.2.<br>**----- End of picture text -----**<br>


Figure 11: The results of (a) the wall-following only (WF) and the implemented bug algorithms that use the same WF (b-e) as part of their navigation strategy. The time limit is 200 sec. 

## _4.3. Implementation Details Bug Algorithms_ 

The most important element of any BA, is the ability to follow a boundary of an obstacle or wall. Based on the robot configuration in Fig. 9(b), we developed a wall following principle as illustrated in Fig. 10. Fig. 10(a-f) shows the wall following of the footbot for a right-sided local direction and Fig. 10(g) shows the state machine, which can also be found as pseudo code in appendix B.2, Alg. B.1. If the robot moves forward and hits a wall, like in Fig. 10a, the angle of the wall can be estimated by using a RANSAC line-fit method (Fischler and Bolles (1981)) in the wedge of range sensors.[9] This is done so that the true 

When the FootBot hits another wall during its forward motion, as in the corner in Fig. 10(d), while in its wallfollowing state, it will turn away from the wall until it is 

> 10This only goes with the assumption that the robot will always encounter walls and no single objects, such plants. The later will not be simulated in ARGoS; however, a classifier able to distinguish walls from small obstacles, must be added if this principle is implemented on a real robot. 

> 11The _R_ and _C_ subscript of _d_ ( _x, O⊥_ ) enables separation of the two methods (RANSAC or only 2 ranges) of retrieving the true distance to the walls. 

> 9Since RANSAC uses random samples to determine the slope of the plane, some stochastic is expected in the wall-following behavior. 

11 

aligned with the wall (similar condition as with Fig. 10(b)). If during a forward motion, the front-range sensor is out of range, as in Fig. 10(e), the foot-bot will initiate a wide turn, to find the wall on the other side as in Fig. 10(f). The state macine for the wall-following behavior can be found in Fig 10 (b), of which the pseudo code can be found in appendix B.2, Alg. B.1. 

This control heuristic should result in a robust wallfollowing behavior, in particular for indoor environments with straight walls. The resulting wall-following behavior is shown in Fig. 11(a). Here it can be seen that the wallfollowing produces a smooth path all along the walls of the mirrowed ”G”. All the implemented bug algorithms, from which the pseudo-code can be found in appendix A, will make use of this exact same wall following behavior in their state machine. The resulting trajectories in the ARGos simulated environment are shown in Fig. 11(b-f). 

## **5. Experimental Results of Bug Algorithms in Real-World Conditions** 

In this section, the BAs will be compared against each other on a wider range and variety of environments than in previous studies. Moreover, we will investigate how sensitive these algorithms are to real-world conditions, subjecting them to the experimental setup explained in section 4. First the selected BAs, which are Com, Com1, Bug2, Alg1 and Alg2 (see subsection 4.1 for the choice’s motivation), will be evaluated with perfect localization. After that, the BAs will be subjected to increasing severity of odometry drift. Next, we will experiment with varying hit-point recognition failures and Distance-to-Target (DT) noise. The results of this section will be discussed in the following part of this paper. 

## _5.1. Experiments with Perfect Localization_ 

The implemented BAs’ performances are now evaluated in 200 procedurally generated environments, with a constant size of 14 by 14 meters. Each BA will have one chance to navigate through the same environment with a time limit of 300 seconds. This should be a sufficient amount of time to reach the target, while preventing the simulation to run endlessly, if one of the BAs gets stuck in a loop. Each BA’s success percentage is recorded, which is the percentage of when the target is reached out of the 200 environments. Fig. 12(a) shows the percentage of BAs that made it to the goal within the required 300 seconds, where the goal is considered reached if the BA is able to get within one meter radius. 

The BA’s total trajectory is recorded as well, which is normalized by dividing by an optimal path length as calculated by the A* path planning algorithm. A* will get an occupancy grid, identical to the procedurally generated environment, which is not available to the bug algorithms by any means, and can visit all the 8 neighboring cells 

**==> picture [221 x 299] intentionally omitted <==**

**----- Start of picture text -----**<br>
100<br>80<br>60<br>40<br>20<br>0<br>WF Com Com1 Bug2 Alg1 Alg2<br>(a) Success percentage.<br>8<br>6<br>4<br>2<br>0<br>WF Com Com1 Bug2 Alg1 Alg2<br>(b) Normalized trajectory length.<br>[%]<br>Target<br>reached<br>BA<br>A*<br>Trajectory<br>/<br>BA<br>Trajectory<br>**----- End of picture text -----**<br>


Figure 12: (a) The percentage of the wall-follower (WF), and the Bug Algorithms (BAs) Com, Com1, Bug2, Alg1 and Alg2, which made it to the goal in an ideal situation with perfect localization, and (b) the trajectory length normalized by the optimal trajectory length calculated by A*. 

at each step.[12] The normalization is applied in order to compare the performances adequately across the generated environments, as the optimal path will be different at each iteration. Fig. 12(b) shows a box-plot of the length of the BAs’ trajectories. For all BAs, all path-lengths are taken into account, including the ones that did not reach the goal. Although this can skew the statistics, a time limitation of 300 seconds will be held constant throughout the experiments to ensure consistency. 

In the simulation set-up, the goal is not located near a wall, so the BA would need to leave the walls physically to reach it. Therefore, the wall-follower (WF) can should not be able to reach the goal position at all. However, there are still a few moments that the environment generator creates a situation where the WF does reaches the goal within one meter, so there is still a slim chance it is reaching the goal, as seen in Fig. 12(a). However, as WF is not moving actively towards the target, this number is marginal small. Com has more freedom in movement as it can leave the wall whenever it is free; however, is only capable to reach 

> 12An 8-connection A* will cause the path to go through a corner of an obstacle. The grid that is available A*, will include padded wall and obstacles compared to the actual environment where the Bug Algorithms will navigate in. 

12 

a) WF (300.10 sec) b) Com (300.10 sec) c) Com1 (173.70 sec) 

**==> picture [232 x 64] intentionally omitted <==**

**----- Start of picture text -----**<br>
S S S<br>T T T<br>**----- End of picture text -----**<br>


d) Bug2 (226.60 sec) e) Alg1 (282.90 sec) f) Alg2 (172.10 sec) 

**==> picture [232 x 64] intentionally omitted <==**

**----- Start of picture text -----**<br>
S S S<br>T T T<br>**----- End of picture text -----**<br>


Figure 13: Behaviors of the implemented (a) wall-follower (WF) and Bug Algorithms (BAs): (b) Com, (c) Com1, (d) Bug2, (e) Alg1 and (f) Alg2 in one generated environment. The BA starts in the top left corner at Start (S) and ends withing 1 meter radius from the Target (T), with a time-limit of 300 seconds. 

the goal about 60 % of the time. Being the simplest of all the BAs, Com does not use memory, therefore, can not recognize where it has been before. Consequently, it quickly get stuck in loops, as shown in Fig. 13(b). The last four BAs, Com1, Bug2, Alg1 and 2, have a success percentage around the 90% in Fig. 12(a). Still the latter two have a much shorter trajectory in comparison, which is around 2.5 times A* instead of 3.3 (Fig. 12(b)). 

In Fig. 13(d) and (e), it can be seen that Alg1 and its ancestor Bug2 need to find the M-line first before it can leave the wall. However, this restriction seem to result in longer trajectories. The M-line-Bugs will even navigate behind the target before finding the M-line again. Com1 and Alg2, on the other hand, will move towards the target if the chance arises, hence have more leave-opportunities along their path (Fig. 13(c) and (f)). The outcome is that in the 200 generated environments, Com1 and Alg2 have a shorter path-length than Bug2 and Alg1 in average.[13] 

## _5.2. Experiments with Odometry Drift_ 

In this paper, we test BAs’ potential for real-world navigation purposes. Therefore, we have added more realistic elements to the simulation, based on our discussion in section 3.1. In the absence of an exact global position, BAs will need to rely on odometry. Therefore, this section will investigate the effects of odometry drift. We assume that the BA will know its own and the target’s position at the 

> 13A bootstrapping based statistical similarity analysis of both the success rate and the trajectory length can be found in appendix C.1. 

**==> picture [250 x 356] intentionally omitted <==**

**----- Start of picture text -----**<br>
100<br>σ =0.00<br>σ =0.05<br>80 σ =0.1<br>σ =0.15<br>σ =0.20<br>60<br>40<br>20<br>0 bhi<br>Com Com1 Bug2 Alg1 Alg2<br>(a) Success percentage.<br>10<br>8<br>6<br>4<br>2<br>garry<br>0<br>Com Com1 Bug2 Alg1 Alg2<br>(b) Normalized trajectory length<br>[%]<br>Target<br>reached<br>BA<br>A*<br>Trajectory<br>/<br>BA<br>Trajectory<br>**----- End of picture text -----**<br>


Figure 14: The (a) percentage of the Bug Algorithms Com, Com1, Bug2, Alg1 and Alg2, which made it to the goal of increasing velocity measurement noise ( _σ_ ) which causes odometry drift, and (b) the trajectory length normalized by the optimal path calculated by A*. 

start of the experiments, but it has to keep them up-todate with its own, noisy, velocity measurements. For these experiments, we assume that the position estimate is acquired by the latter assumption, namely: 

**==> picture [164 x 13] intentionally omitted <==**

, where **˜xt** is the x- and y-position estimate at a given time. **˙˜x** _t−_ 1 is assumed to be _N_ ( **u** _t−_ 1 _, σ_ ), where **u** is the actual velocity, from which the outcome on the system consists of noise with a standard deviation of _σ_ . 

Fig. 14 shows the impact on the performances of the BAs when exposed to odometry drift due to noisy velocity estimates, with a _σ_ of 0.05, 0.10, 0.15 and 0.2. In Fig. 14(a) indicates a significant drop in all the BAs’ success percentage with an increasing _σ_ . In Fig. 14(b) we see that it has a large effect on the trajectory length overall, although there is a less significant degeneration of the Angle-Bugs’ performance (Com, Com1 and Alg2). Bug2’s and Alg2’s performances took the deepest dive with a relatively small increment of the odometry drift, whereas Com’s performance only gradually decreased. As Com does not save 

13 

**==> picture [194 x 202] intentionally omitted <==**

**----- Start of picture text -----**<br>
a) σ =0.05 (300.10 sec) b) σ =0.10 (59.70 sec)<br>S S<br>T T<br>c) σ =0.15 (70.50 sec) d) σ =0.20 (300.10 sec)<br>S S<br>T T<br>**----- End of picture text -----**<br>


Figure 15: Example of Alg2 in environment # 123 of the experimental testing, with increasing noise variance of _σ_ = (a) 0.05, (b) 0.10, (c) 0.15 and (d) 0.20. The BA starts in the top left corner at Start (S) and ends withing 1 meter radius from the Target (T), with a time-limit of 300 seconds. In (d) Alg2 suddenly turns 180 degrees on the left side of the environment, without having seen that hit-point before. 

any position or distance-to-goal at hit-points, only its bearing estimate towards the goal is effected by faulty velocity estimates, resulting in the simplest BA outperforming the rest with _σ >_ 0 _._ 05. Alg2 already lost its advantage to recognize previously visited places, as its success-rate is similar, if not lower, than Com1 at a _σ_ of 0.2. 

However, both Alg1 and Alg2 show signs of stagnation from _σ_ = 0.15 and on, as their performances does not seem to decrease any further and even seem to improve slightly. At that point, it could be that it would accidentally recognize a previously hit-point at a location where it has not been before due to the odometry drift. Although seemingly unwanted, this randomness could have helped the BA to get out of difficult situations, as in Fig. 15 with Alg2. Although it is still successful at a _σ_ = 0.05 (Fig. 15(a)), at a velocity measurement noise of _σ >_ 0 _._ 05, Alg2 is already unable to go straight towards the goal in Fig 15(bc), prematurely hitting a wall and navigating backwards. Fig. 15(d) shows that at a _σ_ = 0.2, Alg2 suddenly encounters a place that it thinks it has been before and turns around during wall following. Although the BA’s observation is false, it did put Alg2 back into a situation where it could reach the goal once again, even though it needed a longer trajectory than without odometry drift.[14] 

**==> picture [233 x 412] intentionally omitted <==**

**----- Start of picture text -----**<br>
10 100<br>p=0.000<br>p=0.005<br>p=0.010<br>p=0.015<br>8 80<br>p=0.020<br>p=0.025<br>6 60<br>4 40<br>2 20<br>0 0<br>Alg1 Alg2 Alg1 Alg2<br>(a) Trajectory length (FP) (b) Goal reached (FP)<br>10 100<br>p=0.0<br>p=0.2<br>p=0.4<br>p=0.6<br>8 80<br>p=0.8<br>p=0.10<br>6 60<br>4 40<br>2 20<br>0 0<br>Alg1 Alg2 Alg1 Alg2<br>(c) Trajectory length (FN) (d) Goal reached (FN)<br>A*<br>Trajectory [%]Target<br>/<br>BA<br>reached<br>BA<br>Trajectory<br>A*<br>Trajectory [%]Target<br>/<br>BA<br>reached<br>BA<br>Trajectory<br>**----- End of picture text -----**<br>


Figure 16: The (a&b) measured trajectory length and (c&d) percentage of Alg1 and Alg2 reaching the goal, with a varying chance ( _p_ ) of a false-positive (FP) or a false-negative (FN) of a previous recognized point to occur. 

## _5.3. Experiments with False Positive and False Negative Recognition Rate_ 

BAs can also recognize previous hit-points based on scene recognitions. In this paper, we will not use the techniques and descriptors discussed in subsection 3.1, but will simulate their performance through false-negative (FN) and false-positive (FP) recall rates. With an increasing probability ( _p_ ) of a uniform distribution, the chances of a previously visited hit-point being falsely recognized at a different location (FP) or not being recognized at the right position (FN) will increase. 

> 14Statistical correlation analysis of the effect of the increasing odometery noise both the success rate and trajectory length can be found in appendix C.2. 

14 

a) p=0.00 (110.70 sec)b) p=0.20 (254.00 sec)c) p=0.40 (116.70 sec) 

**==> picture [254 x 262] intentionally omitted <==**

**----- Start of picture text -----**<br>
S S S<br>T T T<br>d) p=0.60 (300.10 sec)e) p=0.80 (233.70 sec) f) p=1.00 (300.10 sec)<br>S S S<br>T T T<br>AES<br>Figure 17: An example environment with the trajectories of Alg1<br>with increasing chance( p ) of False Positives (FP) to occur, with<br>p (FP) = (a) 0.0, (b) 0.2, (c) 0.4, (d) 0.6, (e) 0.8 and (f) 1.0.The<br>BA starts in the top left corner at Start (S) and ends withing 1<br>meter radius from the Target (T), with a time-limit of 300 seconds.<br>**----- End of picture text -----**<br>


Of the implemented BA, only Alg1 and Alg2 specifically use previously visited locations to change their local wall-following direction from right- to left- sided. In Fig. 16, they are being evaluated with an increasing _p_ (FP) in Fig. 16(a&b) or _p_ (FN) in Fig. 16(c&d) over 100 generated environments. At a _p_ (FP)=0.005, there is a chance of FP occurring 1-2 times (0.5 %) during the run-time of 300 second and at _p_ (FP)=0.025 a chance of 7-8 times (2.5 %). At _p_ (FN) = 0.2, every time the BA encounters a previous hit-point, there is a 20 % chance that it will not recognize it and at _p_ (FN)=1.0, the hit-point will never be recalled. 

Fig. 16(a) and (b) shows that increasing the _p_ (FP) has more effect on the performance of Alg1 than Alg2. An example of Alg1’s behavior in an environment with an increasing _p_ (FP) is shown in Fig. 17(a). From _p_ (FP)=0.2 and on, Alg1 misses the sparse and crucial places on the M-line where it needs to leave the wall, at the moments it prematurely detects a hit-point (17(b-f)). Alg2 has fewer leave-restrictions and can move towards the target whenever its path is clear. Thanks to this flexible behavior, it seems to be less sensitive to more frequent occurrences of FP. 

In Fig. 16(c) and (d), the effects of a higher FN rate is shown; however, both Alg1 as Alg2 seemed to be hardly effected by it. The only trend that could be noticed is for Alg2 as the variance of the trajectory length slowly creeps up with an increasing _p_ (FN) in Fig. 16(c). When _p_ (FN) = 1.0, then Alg2 is completely identical to the implemented Com1, as it only remembers the range measurements at hit-points as a leave-condition. The same goes for Alg1, which transforms into its ancestor Bug2 with _p_ (FN). As 

**==> picture [230 x 220] intentionally omitted <==**

**----- Start of picture text -----**<br>
10 100<br>σ =0<br>σ =1<br>σ =2<br>σ =3<br>8 σ =4 80<br>σ =5<br>6 60<br>4 40<br>2 20<br>0 0<br>Com1 Alg2 Com1 Alg2<br>(a) Trajectory length (b) Target reached<br>Figure 18: da The performance of Alg2 and IBug with varying<br>A*<br>Trajectory [%]Target<br>/<br>BA<br>reached<br>BA<br>Trajectory<br>**----- End of picture text -----**<br>


Figure 18: The performance of Alg2 and IBug with varying distance measurement noise ( _σ_ ) in meters, in the (a) normalized trajectory length and percentage (b) of bugs who made it to the goal. 

both Bug2 and Com1 have the ability to get out of a loop, almost no difference can be noticed in the success rate of Fig. 16(d) with _p_ (FN) = 0.0 and 1.0 for both Alg1 and Alg2.[15] 

## _5.4. Experiments with Distance Measurement Noise_ 

BAs could also use a Distance-to-Target (DT) measurement, so here we assume that the agents are carrying a sensor able to determine this. Com1 and Alg2 both save previous DT measurements to prevent getting stuck in a loop in some situations. In Fig. 18, we are showing the (a) trajectory length and (b) success rate of the increasing standard deviation of the DT noise, while keeping both the velocity measurement noise (odometry drift) and the FP & FN rate at 0.0. The noisy DT measurements ( _d_[˜] ( _x, T_ ) _t_ ) at time _t_ are modeled by _d_[˜] ( _x, T_ ) _t_ = _N_ ( _d_ ( _x, T_ ) _t, σ_ ), where _d_ ( _x, T_ ) _t_ is a scalar that stands for the true DT at time _t_ and _σ_ is the standard deviation of the noise. The degrading performance in both trajectory length and success percentage for increasing _σ_ is more noticeable for Com1 than for Alg2. Com1’s only mechanism to get out of a potential loop is to compare its current DT with a saved one to decide when to leave the wall. Once it is gradually losing this capability with the noisier DT measurements, its behavior will become more and more similar to Com’s, as observed in Fig. 19. Moreover, Com1’s success percentages in Fig. 19(b) drops to around 60 percent at a _σ_ =6 meters, which is equivalent to Com’s score in Fig. 12(b). Alg2 is less affected by the increasing DT noise, which is likely be- 

> 15Statistical correlation analysis of the effect of the increasing recognition failure rate on both the success rate and trajectory length can be found in appendix C.3. 

15 

a) _σ_ =0 (267.40 sec) b) _σ_ =1 (266.80 sec) c) _σ_ =2 (300.10 sec) 

**==> picture [232 x 64] intentionally omitted <==**

**----- Start of picture text -----**<br>
S S S<br>T T T<br>**----- End of picture text -----**<br>


**==> picture [241 x 89] intentionally omitted <==**

**----- Start of picture text -----**<br>
d) σ =3 (300.10 sec) e) σ =4 (300.10 sec) f) σ =5 (300.10 sec)<br>S S S<br>T T T<br>**----- End of picture text -----**<br>


Figure 19: An example environment with the trajectories of Com1 with increasing distance measurement noise variance ( _σ_ ) in meters of (a) 0, (b) 1, (c) 2, (d) 3, (e) 4 and (f) 5 meters. The BA starts in the top left corner at Start (S) and ends within 1 meter radius from the Target (T), with a time-limit of 300 seconds. 

cause it can rely on memorized position as an additional leave condition.[16] 

## **6. Discussion** 

This section will reflect on both the experimental setup and results. The modeled real-world conditions will be discussed first, including the implementation details of the simulation and the chosen noise-models and Bug Algorithms (BAs). Here we will give some suggestions for future development in this topic. Afterwards, we will discuss the results from our experiments, from which we will determine which BA aspects work or do not work for realworld scenarios. This discussion will be concluded on in the last section of this comparative study. _6.1. Modeling Real-World Conditions_ BAs are a seemingly ideal indoor navigation paradigm for tiny robotic platforms with limited recourses. Potentially, they could only take up a small fraction of the onboard computer’s capacity, which opens up space for other computations and tasks. Although the paths generated are sub-optimal compared to path-planning algorithms as A*, no map is needed or needs to be generated. With the target’s location in mind, the BA reacts locally on obstacles and only saves small bits of information in order 

> 16Statistical correlation analysis of the effect of the increasing DT measurement noise on both the success rate and trajectory length can be found in appendix C.4. 

to converge. Nevertheless, we established that the BAs, presented in section 2, tend to over-rely on a perfect localization, which can not be guaranteed for indoor environments. 

If no global localization scheme can be set-up, the BA needs to rely on its noisy on-board sensors to know where it is and integrate this knowledge with the target’s position. In section 3, we reflected on several issues that a robotic implementation of a BA will come across. This includes: an increasing odometry drift, a mis-match between its measured position and the ground truth; recognition failures, i.e. when it fails to recognize a previous location or falsely detects one; and noisy distance to target measurements, which could interfere with the suitable leaving-condition. There are other sensor-noise and failures to consider, such as the noise in the laser-range sensors or (stereo-)cameras for (local) boundary/wall following. However, in this paper, we focused on the global position estimation instead, as this is an issue that all bug algorithms have to deal with and is less dependent on the implemented platform. Moreover, we aimed to keep the wall-following behavior identical among the implemented BAs. 

In the experimental setup, section 4, we selected a set of suitable BAs to experiment on and motivated that choice. We believe that this selection represented most issues of real-world implementation well enough to draw generic conclusions, applicable to the more current BA variants, such as TangentBug (Kamon et al. (1998)). However, for future work, we could also look at newer BAs, where we could include the earlier mentioned local sensor noise. Next to this, the ARGoS simulator and environment generator was very useful for this paper’s experiments, as it was able to generate new environments at a high pace and run the experiments quicker than real-time. This enabled us to test the BAs on hundreds of environments, leading to more reliable results. Nevertheless, further development of these experiments must be performed in a more realistic simulation, with more types of obstacles and visual representations, to induce more challenges of a typical indoor navigation task. 

For the experiments themselves in, section 5, we used simple noise models, i.e. using a Gaussian probability distribution for the odometry drift and noisy range measurements, and a pseudo-random number generator for FNs and FPs occurrences. Future work could look at more realistic noise characteristics. For ground-bound robots, for instance, wheel slippage is determined by the materials used and the friction with the floor. If visual-odometry is used, Gaussian noise could very well be applied, however the texture of the environment is crucial to the variance. The FP & FN occurrences are also very much determined by the features of the environment, as aliasing could occur at areas that are very similar. There is no equal probability of these failures to happen throughout the trajectory of the bug. Moreover, distance measurements by radio beacons not only suffer from regular noise around the mean, 

16 

but have to endure a whole range of disturbances. This includes uneven directional propagation noise, the reflection off the walls, interference of other signals. For the experiments in this paper, we wanted to have more control over the noisy measurements to find a clear correlation between its severity and the performance of the BAs, so we restricted ourselves to use the basic versions of the noise models. However, these considerations be included for an even more realistic simulation in future work. 

## _6.2. Experimental Results_ 

Generally, our experiments showed that all BAs performed worse with a higher odometry drift, noisier range measurements and increasing failure cases. The most noticeable feature, is that the BAs did not all have a similar drop in performances, which is especially noticeable with increasing odometry drift in Fig. 14. Some had a more severe response than others, namely those using memory. Com, being the simplest of all BAs, started out as the worst one of the six, to the best performing with only standard deviation of 0.1 m/s in the velocity estimation. As it only uses the odometry to get a range and bearing to the goal and nothing else, there are less ”bad” decisions it could make based on it. Since odometry is likely to be noisy on very small robots, such simplicity may be the better strategy. Nevertheless, although Com is less influenced by odometry drift, it success rate still drops to 40%, which is still a low score. In general, it is ill-advised to have BAs solely rely on odometry alone. 

In section 5.3 and 5.4, we also assumed that the BAs will also have access to measurements other than odometry. Although a decrease in performance was noticed in all the tested BAs with these specific features (Com1, Alg1 and Alg2), it became evident that Alg2 is the most resilient algorithm. With increasing FN & FP occurrences, Alg1’s performance was noticeably decreasing but Alg2 was hardly affected. This indicates that the M-line-Bugs, as Alg1, seem to have a disadvantage over Angle-Bugs, as Alg1, due to their restrictive leave-condition. This is also noticeable in section 5.2, as M-line-Bugs suffered the most from increasing odometry noise. If real-world conditions apply, BAs should rather be able to leave the wall/obstacle whenever there is the possibility to do so. 

The same goes for noisy distance-to-target measurements (section 5.4), where Com1 is performing worse than Alg2. The reasoning behind this observation is simple: Alg2 is using more mechanisms to get out of complex situations, namely remembering range measurements and locations of previous hit-points. If one of these mechanisms perform badly, then Alg2 can fall back on the other one. Now these measures are operating separately and have a different behavioral outcome; however, it could be more beneficial to a BA if they were fused together or used for cross validation and checking if the bug is stuck in a loop. Nevertheless, it is of great interest to have multiple types of measurements to rely on, either concerning position of the robot itself or the relative position of the goal. 

## **7. Conclusion** 

This paper investigates the potential of Bug Algorithms as a computationally efficient method for robotic navigation. Although the general idea behind the methods seems ideal for implementation of light-weight robots, the literature survey shows that many of their variants rely on either a global localization system or perfect on-board sensors. Our simulation experiments evaluated several implemented Bug Algorithms with varying noisy measurements and failure cases, which showed a significant performance degradation of all algorithms. This indicates that Bug Algorithms can not simply be implemented as they are on a navigating robot, which has to rely on only on-board sensors without any external help. The experimental results did, however, shed some light on how these techniques can be enhanced. Simplicity is a key element, as the most basic Bug Algorithm, Com, was also the one that was the most resilient to odometry drift. Another crucial element is a robust loop detection system, where the robot should not just rely on one but on multiple measured variables, especially in realistic, noise-inducing, environments. Considering these observations in the design of new Bug Algorithms, will make them suitable for the autonomous navigation of tiny robotic platform with limited computational recourses. 

## **Acknowledgements** 

This work has been funded by the NWO grant of Natural Intelligence. The research has been conducted at the Delft University of Technology, Faculty of AeroSpace Engineering, Department of Control and Simulation and Liverpool University at the Faculty of Computer Science, SmartLab. I would like to thank James Butterworth for helping me with setting up the ARGoS simulation and brainstorming about Bug-Algorithms in general. 

## **References** 

## **References** 

- Abelson, H., DiSessa, A. A., 1986. Turtle geometry: The computer as a medium for exploring mathematics. MIT press. 

- Bargh, M. S., de Groote, R., 2008. Indoor localization based on response rate of bluetooth inquiries. In: Proceedings of the First ACM International Workshop on Mobile Entity Localization and Tracking in GPS-less Environments. MELT ’08. ACM, New York, NY, USA, pp. 49–54. 

- URL `http://doi.acm.org/10.1145/1410012.1410024` 

- Boal, J., S´anchez-Miralles, A., Arranz, A., 2014. Topological simultaneous localization and mapping: a survey. Robotica 32 (05), 803–821. 

- URL `https://doi.org/10.1017/S026357471300107` 

- Borenstein, J., Feng, L., Dec 1996. Measurement and correction of systematic odometry errors in mobile robots. IEEE Transactions on Robotics and Automation 12 (6), 869–880. URL `https://doi.org/10.1109/70.544770` 

17 

- Bresson, G., Alsayed, Z., Yu, L., Glaser, S., Sept 2017. Simultaneous localization and mapping: A survey of current trends in autonomous driving. IEEE Transactions on Intelligent Vehicles 2 (3), 194–220. URL `https://doi.org/10.1109/TIV.2017.2749181` 

- Cartwright, B. A., Collett, T. S., Dec 1983. Landmark learning in bees. Journal of comparative physiology 151 (4), 521–543. URL `https://doi.org/10.1007/BF00605469` 

- Coppola, M., McGuire, K. N., Scheper, K. Y. W., de Croon, G. C. H. E., May 2018. On-board communication-based relative localization for collision avoidance in micro air vehicle teams. Autonomous Robots. URL `https://doi.org/10.1007/s10514-018-9760-3` 

- Dijkstra, E. W., Dec 1959. A note on two problems in connexion with graphs. Numerische Mathematik 1 (1), 269–271. URL `https://doi.org/10.1007/BF01386390` 

- Ebrahimi, A., Janabi-Sharifi, F., Ghanbari, A., 2014. Uavisbug: vision-based 3d motion planning and obstacle avoidance for a mini-uav in an unknown indoor environment. Canadian Aeronautics and Space Journal 60 (01), 9–21. URL `https://doi.org/10.5589/q14-005` 

- Evans, J., 2017. Optimization algorithms for networks and graphs. Routledge. 

- Fischler, M. A., Bolles, R. C., Jun. 1981. Random sample consensus: A paradigm for model fitting with applications to image analysis and automated cartography. Vol. 24. ACM, New York, NY, USA, pp. 381–395. URL `http://doi.acm.org/10.1145/358669.358692` 

- Fraundorfer, F., Engels, C., Nister, D., Oct 2007. Topological mapping, localization and navigation using image collections. In: 2007 IEEE/RSJ International Conference on Intelligent Robots and Systems. pp. 3872–3877. URL `https://doi.org/10.1109/IROS.2007.4399123` 

- Goedem´e, T., Nuttin, M., Tuytelaars, T., Van Gool, L., Sep 2007. Omnidirectional vision based topological navigation. International Journal of Computer Vision 74 (3), 219–236. URL `https://doi.org/10.1007/s11263-006-0025-9` 

- Gulzar, M. M., Ling, Q., Yaqoob, M., Iqbal, S., Oct 2015. Realization of an improved path planning strategy. In: 2015 International Conference on Control, Automation and Information Sciences (ICCAIS). pp. 384–389. URL `https://doi.org/10.1109/ICCAIS.2015.7338698` 

- Guo, K., Qiu, Z., Meng, W., Xie, L., Teo, R., 2017. Ultra-wideband based cooperative relative localization algorithm and experiments for multiple unmanned aerial vehicles in gps denied environments. International Journal of Micro Air Vehicles 9 (3), 169–186. URL `https://doi.org/10.1177/1756829317695564` 

- Hart, P. E., Nilsson, N. J., Raphael, B., July 1968. A formal basis for the heuristic determination of minimum cost paths. IEEE Transactions on Systems Science and Cybernetics 4 (2), 100–107. URL `https://doi.org/10.1109/TSSC.1968.300136` 

- Horiuchi, Y., Noborio, H., May 2001. Evaluation of path length made in sensor-based path-planning with the alternative following. In: Proceedings 2001 ICRA. IEEE International Conference on Robotics and Automation (Cat. No.01CH37164). Vol. 2. pp. 1728–1735 vol.2. URL `https://doi.org/10.1109/ROBOT.2001.932860` 

- Kamon, I., Rimon, E., Rivlin, E., 1998. Tangentbug: A range-sensorbased navigation algorithm. The International Journal of Robotics Research 17 (9), 934–953. URL `https://doi.org/10.1177/027836499801700903` 

- Kamon, I., Rimon, E., Rivlin, E., May 1999. Range-sensor based navigation in three dimensions. In: Proceedings 1999 IEEE International Conference on Robotics and Automation (. Vol. 1. pp. 163–169 vol.1. URL `https://doi.org/10.1109/ROBOT.1999.769955` 

- Kamon, I., Rivlin, E., Dec. 1997. Sensory-based motion planning with global proofs. IEEE Transactions on Robotics and Automation 13 (6), 814–822. URL `https://doi.org/10.1109/70.650160` 

- Kamon, I., Rivlin, E., Rimon, E., April 1996. A new range-sensor 

based globally convergent navigation algorithm for mobile robots. In: Proceedings of IEEE International Conference on Robotics and Automation. Vol. 1. pp. 429–435 vol.1. URL `https://10.1109/ROBOT.1996.503814` 

- Kim, D.-H., Shin, K., Han, C.-S., Lee, J. Y., 2013. Sensor-based navigation of a car-like robot based on bug family algorithms. Proceedings of the Institution of Mechanical Engineers, Part C: Journal of Mechanical Engineering Science 227 (6), 1224–1241. URL `https://doi.org/10.1177/0954406212458202` 

- Lambrinos, D., Mller, R., Labhart, T., Pfeifer, R., Wehner, R., 2000. A mobile robot employing insect strategies for navigation. Robotics and Autonomous Systems 30 (1), 39 – 64. URL `https://doi.org/10.1016/S0921-8890(99)00064-0` 

- Laubach, S., Burdick, J., Jan. 2000. Roverbug: Long range navigation for mars rovers. Experimental Robotics VI. URL `https://doi.org/10.1007/BFb0119412` 

- Laubach, S. L., Burdick, J. W., May 1999. An autonomous sensorbased path-planner for planetary microrovers. In: Proceedings 1999 IEEE International Conference on Robotics and Automation (Cat. No.99CH36288C). Vol. 1. pp. 347–354 vol.1. URL `https://doi.org/10.1109/ROBOT.1999.770003` 

- LaValle, S. M., James J. Kuffner, J., 2001. Randomized kinodynamic planning. The International Journal of Robotics Research 20 (5), 378–400. 

- URL `https://doi.org/10.1177/02783640122067453` 

- Lee, S., Adams, T. M., yeol Ryoo, B., 1997. A fuzzy navigation system for mobile construction robots. Automation in Construction 6 (2), 97 – 107. 

- URL `https://doi.org/10.1016/S0926-5805(96)00185-9` 

- Lumelsky, V., Skewis, T., April 1988. A paradigm for incorporating vision in the robot navigation function. In: Proceedings. 1988 IEEE International Conference on Robotics and Automation. pp. 734–739 vol.2. 

- URL `https://doi.org/10.1109/ROBOT.1988.12146` 

- Lumelsky, V., Stepanov, A., nov 1986. Dynamic path planning for a mobile automaton with limited information on the environment. IEEE Transactions on Automatic Control 31 (11), 1058–1063. URL `https://doi.org/10.1109/TAC.1986.1104175` 

- Lumelsky, V. J., Skewis, T., Sep. 1990. Incorporating range sensing in the robot navigation function. and Cybernetics IEEE Transactions on Systems, Man 20 (5), 1058–1069. URL `https://doi.org/10.1109/21.59969` 

- Lumelsky, V. J., Stepanov, A. A., Nov 1987. Path-planning strategies for a point mobile automaton moving amidst unknown obstacles of arbitrary shape. Algorithmica 2 (1), 403–430. URL `https://doi.org/10.1007/BF01840369` 

- Magid, E., Rivlin, E., Sep. 2004. Cautiousbug: a competitive algorithm for sensory-based robot navigation. In: Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS) (IEEE Cat. No.04CH37566). Vol. 3. pp. 2757–2762 vol.3. URL `https://doi.org/10.1109/IROS.2004.1389826` 

- Marino, R., Mastrogiovanni, F., Sgorbissa, A., Zaccaria, R., 2016. A minimalistic quadrotor navigation strategy for indoor multi-floor scenarios. In: Menegatti, E., Michael, N., Berns, K., Yamaguchi, H. (Eds.), Intelligent Autonomous Systems 13. Springer International Publishing, Cham, pp. 1561–1570. URL `https://doi.org/10.1007/978-3-319-08338-4_112` 

- Mastrogiovanni, F., Sgorbissa, A., Zaccaria, R., Feb 2009. Robust navigation in an unknown environment with minimal sensing and representation. IEEE Transactions on Systems, Man, and Cybernetics, Part B (Cybernetics) 39 (1), 212–229. URL `https://doi.org/10.1109/TSMCB.2008.2004505` 

- Mishra, S., Bande, P., Nov 2008. Maze solving algorithms for micro mouse. In: 2008 IEEE International Conference on Signal Image Technology and Internet Based Systems. pp. 86–93. URL `https://doi.org/10.1109/SITIS.2008.104` 

- Mueller, M. W., Hamer, M., D’Andrea, R., May 2015. Fusing ultrawideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation. In: 2015 IEEE International Conference on Robotics and Automation (ICRA). pp. 1730–1736. 

18 

## URL `https://doi.org/10.1109/ICRA.2015.7139421` 

- Ng, J., Br¨aunl, T., Sep 2007. Performance comparison of bug navigation algorithms. Journal of Intelligent and Robotic Systems 50 (1), 73–84. 

- URL `https://doi.org/10.1007/s10846-007-9157-6` 

- Noborio, H., Fujimura, K., Horiuchi, Y., Oct 2000. A comparative study of sensor-based path-planning algorithms in an unknown maze. In: Proceedings. 2000 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2000) (Cat. No.00CH37113). Vol. 2. pp. 909–916 vol.2. URL `https://doi.org/10.1109/IROS.2000.893135` 

- Noborio, H., Maeda, Y., Urakawa, K., Oct 1999. Three or more dimensional sensor-based path-planning algorithm hd-i. In: Proceedings 1999 IEEE/RSJ International Conference on Intelligent Robots and Systems. Human and Environment Friendly Robots with High Intelligence and Emotional Quotients (Cat. No.99CH36289). Vol. 3. pp. 1699–1706 vol.3. URL `https://doi.org/10.1109/IROS.1999.811723` 

- Noborio, H., Nogami, R., Hirao, S., April 2004. A new sensor-based path-planning algorithm whose path length is shorter on the average. In: IEEE International Conference on Robotics and Automation, 2004. Proceedings. ICRA ’04. 2004. Vol. 3. pp. 2832–2839 Vol.3. 

- URL `https://doi.org/10.1109/ROBOT.2004.1307490` 

- Pinciroli, C., Trianni, V., O’Grady, R., Pini, G., Brutschy, A., Brambilla, M., Mathews, N., Ferrante, E., Di Caro, G., Ducatelle, F., Birattari, M., Gambardella, L. M., Dorigo, M., 2012. ARGoS: A modular, parallel, multi-engine simulator for multi-robot systems. Swarm Intelligence 6 (4), 271–295. URL `https://doi.org/10.1007/s11721-012-0072-5` 

- Sankaranarayanan, A., Vidyasagar, M., May 1990. A new path planning algorithm for moving a point object amidst unknown obstacles in a plane. In: Proceedings., IEEE International Conference on Robotics and Automation. pp. 1930–1936 vol.3. URL `https://doi.org/10.1109/ROBOT.1990.126290` 

- Scaramuzza, D., Fraundorfer, F., Dec 2011. Visual odometry [tutorial]. IEEE Robotics Automation Magazine 18 (4), 80–92. URL `https://doi.org/10.1109/MRA.2011.943233` 

_xglobal_ stands for the global position estimate of the Bug Algorithm. _d_ ( _H, T_ ) _prev_ stands for the previous distance of the hit-point to target and _d_ ( _xglobal, T_ ) stands for the current distance from BA to target. _listhp_ stand for a list of previously encountered hit-points. _v_ is the control output for the forward velocity of the robot and _cv_ is the fixed velocity constant. _ω_ is the control output for the heading of the robot in rad/s and _cω_ is a fixed rate constant, to control the speed of the robot’s turns. 

## **Algorithm A.1** The pseudo-code for the state-machine of Com. 

Init: _state_ = ”forward”, _sW F_ 

- **Require:** _cv_ , _cω, xglobalrlocal, listhp_ **function** Com 

**if** _state_ is ”forward” **then** 

- _v ← cv_ 

_ω ←_ 0 **if** Obstacle is hit **then** _state ←_ ”wall ~~f~~ ollowing” **else if** _state_ is ”wall ~~f~~ ollowing” **then** [ _v, ω_ ] _←_ `Wall` ~~`F`~~ `ollowing(` _cv, cω, sW F , rlocal_ `)` _▷_ See B.2 **if** Way towards T is free **then** _▷_ Based on _rlocal state ←_ ”rotate ~~t~~ o target” **else if** _state_ is ”rotate ~~t~~ o ~~t~~ arget” **then** _v ←_ 0 

_ω ← cω_ **if** Heading BA same as direction T **then** _state ←_ ”forward” **return** _v, ω_ 

- Taylor, K., LaValle, S. M., May 2009. I-bug: An intensity-based bug algorithm. In: Proc. IEEE Int. Conf. Robotics and Automation. pp. 3981–3986. URL `https://doi.org/10.1109/ROBOT.2009.5152728` 

- Taylor, K., LaValle, S. M., Apr. 2014. Intensity-based navigation with global guarantees. Autonomous Robots 36 (4), 349. URL `http://dx.doi.org/10.1007/s10514-013-9356-x` 

- Xu, Q.-l., 2014. Randombug: Novel path planning algorithm in unknown environment. Open Electrical & Electronic Engineering Journal 8, 252–257. URL `https://doi.org/10.2174/1874129001408010252` 

- Xu, Q.-L., Tang, G.-Y., dec 2013. Vectorization path planning for autonomous mobile agent in unknown environment. Neural Computing and Applications 23 (7-8), 2129. URL `https://doi.org/10.1007/s00521-012-1163-3` 

- Zhu, Y., Zhang, T., Song, J., Li, X., Dec. 2010. A new bug-type navigation algorithm considering practical implementation issues for mobile robots. In: Proc. IEEE Int. Conf. Robotics and Biomimetics. pp. 531–536. 

- URL `https://doi.org/10.1109/ROBIO.2010.5723382` 

## **Appendices** 

## **A. Pseudo-Code Bug Algorithms** 

The pseudo-code for Com, Com1, Bug2, Alg1 and Alg2, is listed in Algorithm A.1, A.2, A.3, A.4 and A.5 respectively. T stands for target and _sW F_ is a variable that determines if the wall-following is right- ( _sW F_ =1) or left-sided ( _sW F_ =-1). _rlocal_ stand for local sensor measurements, which can be either contact- or range-sensors. 

**Algorithm A.2** The pseudo-code for the state-machine of Com1. 

Init: _state_ = ”forward”, _sW F_ = 1 

- **Require:** _cv_ , _cω, rlocal_ **function** Com 

**if** _state_ is ”forward” **then** 

_v ← cv ω ←_ 0 

**if** Obstacle is hit **then** _d_ ( _H, T_ ) _← d_ ( _xglobal, T_ ) _state ←_ ”wall ~~f~~ ollowing” **else if** _state_ is ”wall ~~f~~ ollowing” **then** [ _v, ω_ ] _←_ `Wall` ~~`F`~~ `ollowing(` _cv, cω, sW F , rlocal_ `)` _▷_ See B.2 **if** Way towards T is free and _d_ ( _xglobal, T_ )¡ _d_ ( _H, T_ ) **then** _state ←_ ”rotate ~~t~~ o target” 

**else if** _state_ is ”rotate ~~t~~ o ~~t~~ arget” **then** _v ←_ 0 

_ω ← cω_ **if** Heading BA same as direction T **then** _state ←_ ”forward” **return** _v, ω_ 

19 

**Algorithm A.3** The pseudo-code for the state-machine of Bug2. 

**Algorithm A.5** The pseudo-code for the state-machine of Alg2. 

Init: _state_ = ”forward”, _sW F_ = 1 Init: _state_ = ”forward”, _sW F_ = 1, _listHP_ =[ ] **Require:** _M − line_ , _cv_ , _cω, xglobalrlocal_ **Require:** _cv_ , _cω, rlocal_ **function** Com **function** Com **if** _state_ is ”forward” **then if** _state_ is ”forward” **then** _v ← cv v ← cv ω ←_ 0 _ω ←_ 0 **if** Obstacle is hit **then if** Obstacle is hit **then** _state ←_ ”wall ~~f~~ ollowing” _sW F_ = 1 **else if** _state_ is ”wall ~~f~~ ollowing” **then** _d_ ( _H, T_ ) _← d_ ( _xglobal, T_ ) _for_ [ _v, ω_ ] _←_ `Wall` ~~`F`~~ `ollowing(` _cv, cω, sW F , rlocal_ `)` _▷_ See B.2 _listHP ←_ [ _listHP , xglobal_ ] **if** _M − line_ is hit and BA is closer to T **then** _state ←_ ”wall ~~f~~ ollowing” _state ←_ ”rotate ~~t~~ o ~~t~~ arget” **else if** _state_ is ”wall ~~f~~ ollowing” **then else if** _state_ is ”rotate ~~t~~ o ~~t~~ arget” **then** [ _v, ω_ ] _←_ `Wall` ~~`F`~~ `ollowing(` _cv, cω, sW F , rlocal_ `)` _▷_ See B.2 _v ←_ 0 **if** _xglobal_ is in _listHP_ **then** _ω ← cω state_ is ”change ~~l~~ ocal ~~d~~ irection” **if** Heading BA same as direction T **then if** Way towards T is free and _d_ ( _xglobal, T_ ) _< d_ ( _H, T_ ) **then** _state ←_ ”forward” _state ←_ ”rotate ~~t~~ o target” **return** _v, ω_ **else if** _state_ is ”rotate ~~t~~ o ~~t~~ arget” **then** _v ←_ 0 _ω ← cω_ **if** Heading BA same as direction T **then** _state ←_ ”forward” **else if** _state_ is ”change ~~l~~ ocal ~~d~~ irection” **then** _v ←_ 0 

_ω ← cω_ 

_sW F_ = _−_ 1 **if** BA has rotated 18 _[o]_ **then** _state ←_ ”wall ~~f~~ ollowing” **return** _v, ω_ 

## **B. Wall Following** 

_B.1. Calculation Real Distance from Wall_ 

**Algorithm A.4** The pseudo-code for the state-machine of Alg1. 

Init: _state_ = ”forward”, _sW F_ = 1, , _listHP_ =[ ] **Require:** _M − line_ , _cv_ , _cω, xglobalrlocal_ **function** Com **if** _state_ is ”forward” **then** _v ← cv ω ←_ 0 **if** Obstacle is hit **then** _listHP ←_ [ _listHP , xglobal_ ] _state ←_ ”wall ~~f~~ ollowing” **else if** _state_ is ”wall ~~f~~ ollowing” **then** [ _v, ω_ ] _←_ `Wall` ~~`F`~~ `ollowing(` _cv, cω, sW F , rlocal_ `)` _▷_ See B.2 **if** _xglobal_ is in _listHP_ **then** _state_ is ”change ~~l~~ ocal ~~d~~ irection” **if** _M − line_ is hit and BA is closer to T **then** _state ←_ ”rotate ~~t~~ o ~~t~~ arget” **else if** _state_ is ”rotate ~~t~~ o ~~t~~ arget” **then** _v ←_ 0 _ω ← cω_ **if** Heading BA same as direction T **then** _state ←_ ”forward” **else if** _state_ is ”change ~~l~~ ocal ~~d~~ irection” **then** _v ←_ 0 

_ω ← cω sW F_ = _−_ 1 **if** BA has rotated 18 _[o]_ **then** _state ←_ ”wall ~~f~~ ollowing” **return** _v, ω_ 

**==> picture [127 x 56] intentionally omitted <==**

**----- Start of picture text -----**<br>
c<br>h<br>β<br>a b<br>**----- End of picture text -----**<br>


Figure B.20: Visualization of the triangle configuration for the derivation. 

In Fig. B.20, the configurations of the solved triangle is solved, where we want to calculate _h_ (height triangle) with the triangle sides of _a_ and _b_ and the angle _β_ . _c_ is the triangle side that will be unknown, so a formula will be derived that will only use _a_ , _b_ and _β_ . 

The geometrical equations used to achieve the ranges are the triangle area formula: 

**==> picture [160 x 72] intentionally omitted <==**

, the SAS triangle rule: 

, the cosine-rule: 

**==> picture [187 x 13] intentionally omitted <==**

20 

with _A_ is the area of the triangle. 

Substitute _A_ in Eq. B.2 for the right side of Eq. B.1, and solve for _h_ : 

**==> picture [159 x 21] intentionally omitted <==**

Now substitute _c_ in Eq. B.4, for the right side of Eq. B.3, which results in the following equation: 

**==> picture [189 x 26] intentionally omitted <==**

**Algorithm B.2** The procedure of keeping the heading of the FootBot aligned with the wall during wall following. 

|**Require:** _sW F , d_(_x, O⊥_)_, dref, rs, rf, _|**Require:** _sW F , d_(_x, O⊥_)_, dref, rs, rf, _|**Require:** _sW F , d_(_x, O⊥_)_, dref, rs, rf, _|_cw, β_|
|---|---|---|---|
|**function** Wall<br>Following|~~A~~nd|~~A~~ligning||
|**if** _|dref −d_(_x, O⊥_)_| > −td_ ) **then**<br>**if** _dref −d_(_x, O⊥_)_> td_ **then**<br>_ω_ =_sW F · cω_|||_▷_If too far from _dref_<br>_▷_If too far from wall<br>_▷_Turn towards the wall|
|**else**|||_▷_If too close to wall|
|_ω_ =_−sW F · cω_|||_▷_Turn from the wall|
|**else if** _|dref −d_(_x, O⊥_)_|  _|_< td_|**then**|_▷_If close to _dref_|
|**if** _rs > rf ·_cos_β_ **then**|||_▷_Fine tune alignment|
|_ω_ =_sW F · cω_<br>**else**<br>_ω_ =_−sW F · cω_<br>**else**|||_▷_Turn towards the wall<br>_▷_Turn from the wall<br>_▷_Do not adjust the turn|
|_ω_ = 0||||
|**return** _ω_||||



## _B.2. Pseudo Code Wall Following_ 

## **C. Statistical Tests** 

The procedure of the wall-following behavior is listed in this appendix in Algorithm B.1 and B.2. T _sW F_ is a variable that determines if the wall-following is right( _sW F_ =1) or left-sided ( _sW F_ =-1), _d_ ( _x, O⊥_ ) is the current distance to the robot calculated perpendicular from the wall and _dref_ is the preferred distance from the wall in meters and _td_ is the threshold to determine if the robot near _dref_ . _rs_ and _rf_ are the side and range sensor’s measurement in meters and _β_ is the angle between them. If _sW F_ =1, then _rs_ is the right range sensor and if _sW F_ =-1, then _rs_ is the left range sensor. _v_ is the control output for the forward velocity of the robot and _cv_ is the fixed velocity constant. _ω_ is the control output for the heading of the robot in rad/s and _cω_ is a fixed rate constant, to control the speed of the robot’s turns. 

**Algorithm B.1** The procedure of the wall-following behavior. 

Init: _state_ = ”rotate ~~t~~ o ~~a~~ lign ~~w~~ all” **Require:** _sW F , d_ ( _x, O⊥_ ) _, dref , rs, rf , cv, cω, β β_ = 60 deg **function** Wall ~~F~~ ollowing **if** _state_ is ”rotate ~~t~~ o ~~a~~ lign ~~w~~ all” **then** _v ←_ 0 _ω ←−_ 1 _· sW F · cω ▷_ Turn away from the wall **if** _rs ≈ rf ·_ cos( _β_ ) **then** _state ←_ ”wall ~~f~~ ollowing ~~a~~ nd ~~a~~ ligning” **if** _rf_ = OR **then** _state ←_ ”rotate ~~a~~ round ~~c~~ orner” **else if** _sW F_ is ”wall ~~f~~ ollowing ~~a~~ nd ~~a~~ ligning” **then** _v ← cv ω ←_ `Wall` ~~`F`~~ `ollowing` ~~`a`~~ `nd` ~~`A`~~ `ligning()` _▷_ See B.2 **if** _d_ ( _x, Omin_ ) _< dref_ **then** _state ←_ ”rotate ~~t~~ o ~~a~~ lign ~~w~~ all” **if** _rf_ is OR **then** _state ←_ ”rotate ~~a~~ round ~~c~~ orner” **else if** _state_ is ”rotate ~~a~~ round ~~c~~ orner” **then** _v ← cv ω ← sW F · v/dref ▷_ Wide turn, radius = _dref_ **if** _rs ≈ rf ·_ cos( _β_ ) **then** _state ←_ ”wall ~~f~~ ollowing ~~a~~ nd ~~a~~ ligning” **if** _d_ ( _x, Omin_ ) _< dref_ **then** _state ←_ ”rotate ~~t~~ o ~~a~~ lign ~~w~~ all” **return** _ω_ 

## _C.1. Bootstrapping Bug Algorithms_ 

In Fig. 12, the resulting performance values per bug algorithm was shown. Here, both the success rate and the trajectory length are subjected to a bootstrapping test, to evaluate whether the bug algorithms belong to the same distribution (null-hypothesis). Table C.2 contains the boostrapping tests from the data presented in Fig. 12(a) and Table C.3 for Fig. 12(b). 

||Com|Com1|Bug2|Alg1|Alg2|
|---|---|---|---|---|---|
|Com|1|1|0|0|1|
|Com1|1|1|0|0|1|
|Bug2|0|0|1|1|0|
|Alg1|0|0|1|1|0|
|Alg2|1|1|0|0|1|



Table C.2: Bootstrapping results on the trajectory length of the evaluated bug algorithms with a sample size 10000. The value ”1” means that the null-hypothesis (the evaluated data comes from the same distribution) holds, while ”0” means it is rejected. 

||Com|Com1|Bug2|Alg1|Alg2|
|---|---|---|---|---|---|
|Com|1|0|0|0|0|
|Com1|0|1|1|1|0|
|Bug2|0|1|1|1|0|
|Alg1|0|1|1|1|0|
|Alg2|0|0|0|0|1|



Table C.3: Bootstrapping results on the success rate of the evaluated bug algorithms with a sample size 10000. The value ”1” means that the null-hypothesis (the evaluated data comes from the same distribution) holds, while ”0” means it is rejected. 

## _C.2. Correlation Analysis Odometry Noise_ 

In order to evaluate whether an relationship exists between the increasing odometry noise and the degeneration of the performances of the bug algorithms, the data presented in Fig. 14 are subjected to regression analysis. Table C.5 contains the logistic regression analysis with a R2 value, from the trajectory length data presented in 

21 

Fig. 14(a) and Table C.4 contains the logistic regression analysis with a pseudo-R2 value, from the success rate data presented in Fig. 14(b). 

||Com|Com1|Bug2|Alg1|Alg2|
|---|---|---|---|---|---|
|Slope|8.081|13.642|13.561|11.857|12.523|
|Intercept|2.752|2.724|3.152|3.522|2.654|
|R2|0.076|0.189|0.217|0.173|0.161|



Table C.4: Linear regression evaluation of the trajectory lengths against the measurement noise, including the intercept, slope and R2 value per bug algorithm. 

||Com|Com1|Bug2|Alg1|Alg2|
|---|---|---|---|---|---|
|Slope|-1.240|-3.100|-3.860|-3.480|-3.110|
|Intercept|0.587|0.800|0.779|0.706|0.847|
|R2|0.035|0.189|0.343|0.323|0.198|



Table C.5: Logistic regression evaluation of the success rate against the measurement noise, including the intercept, slope and (psuedo) R2 value per bug algorithm. 

## _C.3. Correlation Analysis Recognition Failures_ 

In order to evaluate whether an relationship exists between the increasing failing recognition rate and the degeneration of the performances of the bug algorithms Alg1 and Alg2, the data presented in Fig. 16 are subjected to regression analysis. Table C.7 contains the logistic regression analysis with a R2 value, from the trajectory length data presented in Fig. 16(a) and Table C.6 contains the logistic regression analysis with a pseudo-R2 value, from the success rate data presented in Fig. 16(b). Table C.9 contains the logistic regression analysis with a R2 value, from the trajectory length data presented in Fig. 16(c) and Table C.8 contains the logistic regression analysis with a pseudo-R2 value, from the success rate data presented in Fig. 16(d). 

|Slope<br>Intercept<br>R2|Alg1|Alg2|
|---|---|---|
||0.5472|0.1722|
||2.4302|1.8843|
||0.0112|0.0020|



||Alg1|Alg2|
|---|---|---|
|Slope|0.1873|1.0584|
|Intercept|3.2478|2.2733|
|R2|0.0000|0.0006|



Table C.8: Linear regression evaluation of the trajectory lengths against the False Negative recognition rate, including the intercept, slope and R2 value per bug algorithm. 

||Alg1|Alg2|
|---|---|---|
|Slope|0.0577|0.1010|
|Intercept|0.8018|0.9192|
|R2|0.3668|0.7171|



Table C.9: Logistic regression evaluation of the success rate against the False Negative recognition rate, including the intercept, slope and (psuedo) R2 value per bug algorithm. 

## _C.4. Correlation Analysis Distance Sensor Noise_ 

In order to evaluate whether an relationship exists between the increasing distance measurement noise and the degeneration of the performances of the bug algorithms Alg1 and Alg2, the data presented in Fig. 18 are subjected to regression analysis. Table C.11 contains the logistic regression analysis with a R2 value, from the trajectory length data presented in Fig. 18(a) and Table C.10 contains the logistic regression analysis with a pseudo-R2 value, from the success rate data presented in Fig. 18(b). 

||Com1|Alg2|
|---|---|---|
|Slope|0.0501|-0.0075|
|Intercept|2.5783|2.4204|
|R2|0.0019|0.0001|



Table C.10: Linear regression evaluation of the trajectory lengths against the distance measurement noise, including the intercept, slope and R2 value per bug algorithm. 

||Com1|Alg2|
|---|---|---|
|Slope|-0.0557|-0.0225|
|Intercept|0.8682|0.9283|
|R2|0.2412|0.5583|



Table C.11: Logistic regression evaluation of the success rate against the distance measurement noise, including the intercept, slope and (psuedo) R2 value per bug algorithm. 

Table C.6: Linear regression evaluation of the trajectory lengths against the False Positive recognition rate, including the intercept, slope and R2 value per bug algorithm. 

||Alg1|Alg2|
|---|---|---|
|Slope|-0.1443|-0.0014|
|Intercept|0.9105|0.9418|
|R2|0.4652|0.7773|



Table C.7: Logistic regression evaluation of the success rate against the False Positive recognition rate, including the intercept, slope and (psuedo) R2 value per bug algorithm. 

22 


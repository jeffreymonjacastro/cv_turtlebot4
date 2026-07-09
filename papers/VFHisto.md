© 1991 IEEE. Reprinted with permission, from _IEEE Journal of Robotics and Automation_ Vol 7, No 3, June 1991, pp. 278-288. 

# **THE VECTOR FIELD HISTOGRAM - FAST OBSTACLE AVOIDANCE FOR MOBILE ROBOTS** 

by 

**J. Borenstein** , Member, IEEE and **Y. Koren** , Senior Member, IEEE The University of Michigan, Ann Arbor Advanced Technology Laboratories 1101 Beal Avenue, Ann Arbor, MI 48109 

## **ABSTRACT** 

A new real-time obstacle avoidance method for mobile robots has been developed and implemented. This method, named the _vector field histogram_ (VFH), permits the detection of unknown obstacles and avoids collisions while simultaneously steering the mobile robot toward the target. 

The VFH method uses a two-dimensional Cartesian _histogram grid_ as a world model. This world model is updated continuously with range data sampled by on-board range sensors. The VFH method subsequently employs a _two-stage_ data-reduction process in order to compute the desired control commands for the vehicle. In the first stage the _histogram grid_ is reduced to a onedimensional _polar histogram_ that is constructed around the robot's momentary location. Each sector in the _polar histogram_ contains a value representing the _polar obstacle density_ in that direction. In the second stage, the algorithm selects the most suitable sector from among all _polar histogram_ sectors with a low _polar obstacle density_ , and the steering of the robot is aligned with that direction. 

Experimental results from a mobile robot traversing densely cluttered obstacle courses in smooth and continuous motion and at an average speed of 0.6 0.7m/sec demonstrate the power of the VFH method. 

This work was sponsored by the Department of Energy Grant DE-FG02-86NE37969 

## **1. INTRODUCTION** 

Obstacle avoidance is one of the key issues to successful applications of mobile robot systems. All mobile robots feature some kind of collision avoidance, ranging from primitive algorithms that detect an obstacle and stop the robot short of it in order to avoid a collision, through sophisticated algorithms, that enable the robot to detour obstacles. The latter algorithms are much more complex, since they involve not only the detection of an obstacle, but also some kind of quantitative measurements concerning the dimensions of the obstacle. Once these have been determined, the obstacle avoidance algorithm needs to steer the robot around the obstacle and proceed toward the original target. Usually,  this procedure requires the robot to stop in front of the obstacle, take the measurements, and only then resume motion. Obstacle avoidance (also called reflexive obstacle avoidance or local path planning) may result in non-optimal paths [5], since no prior knowledge about the environment is used. 

A brief survey of relevant earlier obstacle avoidance methods is presented in Section 2, while Section 3 summarizes the _virtual force field_ (VFF), an obstacle avoidance method developed earlier by our group at the University of Michigan [5]). While the VFF method provides superior real-time obstacle avoidance for fast mobile robots, some limitations concerning fast travel among densely cluttered obstacles were identified in the course of our experimental work [18]. To overcome these limitations, we developed a new method, named _vector field histogram_ (VFH), which is introduced in Section 4. The VFH method eliminates the shortcomings of the VFF method, yet retains all advantages of its predecessor (as will be shown in Section 4). A comparison of the VFH method to earlier methods is given in Section 5, and Section 6 presents experimental results obtained with our VFH-controlled mobile robot. 

## **2. SURVEY OF EARLIER OBSTACLE AVOIDANCE METHODS** 

This section summarizes relevant obstacle avoidance methods, namely _edge-detection, certainty grids_ , and _potential field methods_ . 

## **2.1 Edge-Detection Methods** 

One popular obstacle avoidance method is based on _edge-detection_ . In this method, an algorithm tries to determine the position of the vertical edges of the obstacle and then steer the robot around either one of the "visible" edges. The line connecting two visible edges is considered to represent one of the boundaries of the obstacle. This method was used in our very early research [4], as well as in several other works [11,21,22,28], all using ultrasonic sensors for obstacle detection. A disadvantage with current implementations of this method is that the robot stops in front of obstacles to gather sensor information. However, this is not an 

```
Page 2
```

inherent limitation of _edge-detection_ methods; it may be possible to overcome this problem with faster computers in future implementations. 

In another _edge-detection_ approach (using ultrasonic sensors), the robot remains stationary while taking a panoramic scan of its environment [13,14]. After the application of certain line-fitting algorithms, an edge-based _global path planner_ is instituted to plan the robot's subsequent path. 

A common drawback of both _edge-detection_ approaches is their sensitivity to sensor accuracy. Ultrasonic sensors present many shortcomings in this respect: 

**Poor directionality** limits the accuracy in determining the spatial position of an edge to 1050 cm, depending on the distance to the obstacle and the angle between the obstacle surface and the acoustic axis. 

**Frequent misreadings** are caused by either ultrasonic noise from external sources or stray reflections from neighboring sensors (i.e., crosstalk). Misreadings cannot always be filtered out and they cause the algorithm to falsely detect edges. 

**Specular reflections** occur when the angle between the wavefront and the normal to a smooth surface is too large. In this case the surface reflects the  incoming ultra-sound waves away from the sensor, and the obstacle is either not detected, or "seen" as much smaller than it is in reality (since only part of the surface is detected). 

Any one of these errors can cause the algorithm to determine the existence of an edge at a completely wrong location, oftentimes resulting in highly unlikely paths. 

## **2.2 The Certainty Grid for Obstacle Representation** 

A method for probabilistic representation of obstacles in a grid-type world model has been developed at Carnegie-Mellon University (CMU) [13,23,24]. This world model, called _certainty grid_ ,  is especially suited to the accommodation of inaccurate sensor data such as range measurements from ultrasonic sensors. 

In the _certainty grid_ , the robot's work area is represented by a two-dimensional array of square elements, denoted as cells. Each cell contains a _certainty value_ (CV) that indicates the measure of confidence that an obstacle exists within the cell area. With the CMU method, CVs are updated by a probability function that takes into account the characteristics of a given sensor. Ultrasonic sensors, for example, have a conical field of view. A typical ultrasonic sensor [25] returns a radial measure of the distance to the nearest object within the cone, yet does not specify the _angular_ location of the object. (Fig. 1 shows the area A in which an object must be located in order to result in a distance measurement _d_ ). If an object is detected 

```
Page 3
```

by an ultrasonic sensor, it is _more likely_ that this object is _closer_ to the acoustic axis of the sensor than to the periphery of the conical field of view [4]. For this reason, CMU's probabilistic function _Cx_ increases CVs in cells close to the acoustic axis more than CVs in cells at the periphery. 

In CMU's applications of this method [23,24], the mobile robot remains _stationary_ while it takes a panoramic scan with its 24 ultrasonic sensors. Next, the probabilistic function _Cx_ is applied to each of the 24 range readings, updating the _certainty grid_ . Finally, the robot moves to a new location, stops, and repeats the procedure. After the robot traverses a room in this manner, the resulting _certainty grid_ represents a fairly accurate map of the room. A global path-planning method is then employed for off-line calculations of subsequent robot paths. 

**==> picture [147 x 39] intentionally omitted <==**

**----- Start of picture text -----**<br>
NUE Object  EN<br>4]a Wanye|)<br>**----- End of picture text -----**<br>


## **2.3 Potential Field Methods** 

The idea of imaginary forces acting on a robot has been suggested by Khatib [16]. In this method, obstacles exert repulsive forces, while the target applies an attractive force to the robot. A resultant force vector _**R**_ , comprising the sum of a target-directed attractive force and repulsive forces from obstacles, is 

vfh05.ds4, p18fig2.wmf 

**Figure. 1:** Two-dimensional projection of the conical field of view of an ultrasonic sensor. A range reading _d_ indicates the existence of an object somewhere within the shaded region _A_ (Carnegie Mellon's method). 

calculated for a given robot position. With _**R**_ as the _accelerating force_ acting on the robot, the robot's new position for a given time interval is calculated, and the algorithm is repeated. 

Krogh [19] has enhanced this concept further by taking into consideration the robot's velocity in the vicinity of obstacles.  Thorpe [27] has applied the potential field method to off-line path planning and Krogh and Thorpe [20] suggest a combined method for global and local path planning, which uses a "Generalized Potential Field" approach. Newman and Hogan [15] introduce the construction of potential functions through combining individual obstacle functions with logical operations. 

Common to these methods is the assumption of a _known and prescribed_ world model, in which simple, predefined geometric shapes represent obstacles and the robot's path is generated _off-line_ . 

`Page 4` 

While each of the above methods features valuable refinements, none have been implemented on a mobile robot with real sensory data. By contrast, Brooks [8,9] and Arkin [1]  use a potential field method on experimental mobile robots (equipped with a ring of ultrasonic sensors).  Brooks' implementation treats each ultrasonic range reading as a repulsive force vector. If the magnitude of the sum of the repulsive forces exceeds a certain threshold, the robot stops, turns into the direction of the resultant force vector, and moves on. In this implementation, however, only one set of range readings is considered, while previous readings are lost. Arkin's robot employs a similar method; his robot was able to traverse an obstacle course at 0.12 cm/sec (0.4 feet/sec). 

## **3. THE VIRTUAL FORCE FIELD (VFF) METHOD** 

The _Virtual Force Field_ (VFF) method is our _earlier_ real-time obstacle avoidance method for fast-running vehicles [5]. Unlike the methods reviewed above, the VFF method allows for fast, continuous, and smooth motion of the controlled vehicle among unexpected obstacles, and does not require the vehicle to stop in front of obstacles. 

## **3.1 The VFF Concept** 

The individual components of the VFF method are presented below. 

- a. The VFF method uses a two-dimensional Cartesian _histogram grid_ **C** for obstacle representation. Like in CMU's _certainty grid_ concept, each cell ( _i,j_ ) in the _histogram grid_ holds a certainty value, _ci,j_ , that represents the confidence of the algorithm in the existence of an obstacle at that location. 

The _histogram grid_ differs from the _certainty grid_ in the way it is built and updated. CMU's method projects a probability profile onto those cells that are affected by a range reading; this procedure is computationally intensive and would impose a heavy timepenalty if real-time execution on an on-board computer was attempted.  Our method, on the other hand, increments only one cell in the _histogram grid_ for each range reading, creating a _"probability" distribution_ 1 with only small computational overhead. For ultrasonic sensors, this cell corresponds to the measured distance _d_ (see Fig. 2a) and lies on the acoustic axis of the sensor. While this approach may seem to be an oversimplification, a _probabilistic_ distribution is actually obtained by _continuously_ and _rapidly_ sampling each sensor while the vehicle is moving. Thus, the same cell and its neighboring cells are repeatedly incremented, as shown in Fig. 2b. This results in a _histogramic probability distribution_ , in which high _certainty values_ are obtained in cells close to the actual location of the obstacle. 

> 1 We use the term "probability" in the literal sense of " _likelyhood_ ." 

```
Page 5
```

b. Next, we apply the potential field idea to the _histogram grid_ , so the _probabilistic_ sensor information can be used efficiently to control the vehicle. Fig. 3 shows how this algorithm works: 

**==> picture [303 x 222] intentionally omitted <==**

**----- Start of picture text -----**<br>
Histogram Ob ject<br>Object<br>‘0.‘0  00.010.0'9 o\0 oof 0.00000.0 o\0 o!o~<br>‘oop0\0 ooosto00.0.0.0. h0'0/09 | ‘oooloo0K0oo M23doo  440Ko<br>Mod0. d!d\0'0'00 O100 O10'0'0)a0!0'0/0 | 101010010'0'0)/f'0'9'0'0'0;0094) 0'0.0;0. |<br>| Certainty<br>| values<br>| :<br>_-30°cone, |<br>Measured | |<br>distance d |<br>:<br>|<br>:<br>| \I<br>'! | Dofirectmot Dofirectmotofirectmotirectmotrectmotmot i i o onnn ii/i/ / Dofirectmotofirectmotirectmotrectmotmot<br>— —* eS<br>vfh10.ds4, p18fig3.wmf Sonar pre v io us current<br>readingingng reading<br>**----- End of picture text -----**<br>


As the vehicle moves, a Measured | | distance d | window of _ws_ x _ws_ cells : | accompanies it, : overlying a square | \I region of _**C**_ . We call this '! | Dofirectmot Dofirectmotofirectmotirectmotrectmotmot ~~i~~ i ~~o~~ onnn ii/i/ / Dofirectmotofirectmotirectmotrectmotmot — ~~—* eS~~ region the " _active_ vfh10.ds4, p18fig3.wmf Sonar pre ~~v~~ io ~~us~~ current _region_ " (denoted as _**C**_ *), readingingng reading Figure 2: and cells that **a.** Only one cell is incremented for each range reading. With ultrasonic momentarily belong to sensors, this is the cell that lies on the acoustic axis and corresponds to the _active region_ are the measured distance _d_ . called " _active cells_ " **b.** A _histogramic probability distribution_ is obtained by continuous and (denoted as _c_ * ). In our _i,j_ rapid sampling of the sensors while the vehicle is moving current implementation, the size of the window is 33x33 cells (with a cell size of 10cmx10cm), and the window is always centered about the robot's position. Note that a _circular_ window would be geometrically more appropriate, but is computationally more expensive to handle than a _square_ one. 

**a.** Only one cell is incremented for each range reading. With ultrasonic sensors, this is the cell that lies on the acoustic axis and corresponds to the measured distance _d_ . 

**b.** A _histogramic probability distribution_ is obtained by continuous and rapid sampling of the sensors while the vehicle is moving 

Each _active cell_ exerts a _virtual repulsive force_ _**F** i,j_ toward the robot. The magnitude of this force is proportional to the certainty value _c_ *  and inversely proportional to _i,j d_ x , where _d_ is the distance between the cell and the center of the vehicle, and _x_ is a positive real number (we assume _x_ =2 in the following discussion). At each iteration, all virtual repulsive forces are added up to yield the resultant repulsive force _**F** r_ . Simultaneously, a virtual attractive force _**F** t_ of constant magnitude is applied to the vehicle, "pulling" it toward the target.  The summation of _**F** r_ and _**F** t_ yields the resulting force vector _**R**_ . In order to compute _**R**_ , up to 33x33=1089 individual repulsive force vectors _**F** i,j_ must be computed and accumulated. The computational heart of the VFF algorithm is therefore a specially developed algorithm for the fast computation and summation of the repulsive force vectors. 

`Page 6` 

**==> picture [282 x 181] intentionally omitted <==**

**----- Start of picture text -----**<br>
Ri Vs| J /2 Object<br>Obje ct<br>Certainty ffyf<br>ABABA values LM<br>SAX\\ iy<br>\ ; \<br>’p<br>Robot<br>**----- End of picture text -----**<br>


**Figure 3:** The Virtual Force Field (VFF) concept: Occupied cells exert repulsive forces onto the robot; the 2 magnitude is proportional to the certainty value _ci,j_ of the cell and inversely proportional to _d_ . 

`Page 7` 

- c. Combining the above two concepts (a.) and (b.) in real-time enables sensor data to influence the steering control _immediately_ . In practice, each range reading is recorded into the _histogram grid_ as soon as it becomes available, and the subsequent calculation of _**R**_ takes this data-point into account. This feature gives the vehicle fast response to obstacles that appear suddenly, resulting in fast reflexive behavior imperative at high speeds. 

## **3.2 Shortcomings of the VFF Method** 

The VFF method has been implemented and extensively tested on-board a mobile robot equipped with a ring of 24 ultrasonic sensors (see Section 6). Under most conditions, the VFF-controlled robot performed very well. Typically, it traversed an obstacle course at an average speed of 0.5m/sec, provided the obstacles were placed at least 1.8m apart (the robot diameter is 0.8m). With less clearance between two obstacles (e.g., a doorway), some problems were encountered. Sometimes, the robot would not pass through a doorway, because the repulsive forces from both sides of the doorway resulted in a force that pushed the robot away. 

Another problem arose out of the discrete nature of the _histogram grid_ . In order to efficiently calculate repulsive forces in real-time, the robot's momentary position is mapped onto the _histogram grid_ . Whenever this position changes from one cell to another, drastic changes in the resultant _**R**_ may be encountered. The following numeric example explains this point. Consider a repulsive force generated by a cell ( _m_ , _n_ ) and applied to the robot's momentary position at ( _m_ , _n_ +6), which is six cells away (i.e., 0.6 m, with a cell size of 10x10cm). The magnitude of this particular force vector is | _**F** m,n_ |= _k_ /0.62=2.8 _k_ . As the robot advances by one cell, and its position corresponds to cell ( _m_ , _n_ +5), the new force vector is | _**F** 'm,n_ |= _k_ /0.52=4 _k_ . The change is 42%. Changes of this magnitude cause considerable fluctuations in the steering control. The situation is even worse when the magnitude of the target-directed constant attractive force _**F** t_ lies between the directions of the two successive forces | _**F**_ | and | _**F**_ '| (this condition is in fact most likely, because it corresponds to the "steady state" condition when the robot travels alongside an obstacle). In this situation, the direction of the resultant _**R**_ may fluctuate by up to 180 . For this reason it is necessary to smooth the control signal to the steer-o ing motor by adding a low-pass filter to the VFF control loop [5]. This filter, however, introduces a delay that adversely affects the robot's steering response to unexpected obstacles. 

Finally, we also identified a problem that occurs when the robot travels in narrow corridors. When traveling along the center-line between the two corridor walls, the robot's motion is stable. If, however, the robot strays slightly to either side of the center-line, it experiences a strong virtual repulsive force from the closer wall. This force usually "pushes" the robot across the center-line, and the process repeats with the other wall. Under certain conditions, this process results in oscillatory and unstable motion [17,18]. 

```
Page 8
```

## **4. THE VECTOR FIELD HISTOGRAM (VFH) METHOD** 

Careful analysis of the shortcomings of the VFF method reveals its inherent problem: excessively drastic data reduction that occurs when the individual repulsive forces from _histogram grid_ cells are added up to calculate the resultant force vector _**F** r_ . Hundreds of data points are reduced in one step to only two items: direction and magnitude of _**F** r_ . Consequently, _detailed information about the local obstacle distribution is lost_ . 

To remedy this shortcoming, we have developed a new method called the _vector field histogram_ (VFH).  The VFH method employs a _two-stage data reduction_ technique, rather than the single-step technique used by the VFF method. Thus, three levels of data representation exist: 

- a. The highest level holds the detailed description of the robot's environment. In this level, the two-dimensional Cartesian _histogram grid_ _**C**_ is continuously updated in real-time with range data sampled by the on-board range sensors. This process is identical to the one described in Section 3 for the VFF method. 

- b. At the intermediate level, a one-dimensional _polar histogram_ _**H**_ is constructed around the robot's momentary location. _**H**_ comprises _n_ angular sectors of width α . A transformation (described in Section 4.1, below) maps the _active region_ _**C**_ * onto _**H**_ , resulting in each sector _k_ holding a value _hk_ that represents the _polar obstacle density_ in the direction that corresponds to sector _k_ . 

- c. The lowest level of data representation is the output of the VFH algorithm: the reference values for the drive and steer controllers of the vehicle. 

The following sections describe the two data reduction stages in more detail. 

## **4.1 First Data Reduction and Creation of the** _**Polar Histogram**_ 

The first data-reduction stage maps the _active region_ _**C**_ * of the _histogram grid_ _**C**_ onto the _polar histogram_ _**H**_ , as follows: As with our earlier VFF method, a _window_ moves with the vehicle, overlying a square region of _ws_ x _ws_ cells in the _histogram grid_ (see Fig. 4). The contents of each _active cell_ in the _histogram grid_ are now treated as an _obstacle vector_ , the direction of which is determined by the direction β from the cell to the _Vehicle Center Point_ (VCP)[2] 

> 2 For symmetrically shaped vehicles, the VCP is easily defined as the geometric center of the vehicle. For rectangular vehicles, it is possible to chose two VCPs, e.g., one each at the center-point of the front and rear axles. 

```
Page 9
```

**Figure 4:** Mapping of _active cells_ onto the _polar histogram_ . 

**==> picture [101 x 24] intentionally omitted <==**

**==> picture [16 x 12] intentionally omitted <==**

and the magnitude is given by 

**==> picture [441 x 15] intentionally omitted <==**

where 

_a,b_ Positive constants. _c*i,j_ Certainty value of _active cell_ ( _i,j_ ). _di,j_ Distance between _active cell_ ( _i,j_ ) and the VCP. _mi,j_ Magnitude of the _obstacle vector_ at cell ( _i,j_ ). _x_ 0 , _y_ 0 Present coordinates of the VCP. _xi_ , _yj_ Coordinates of _active cell_ ( _i,j_ ). β _i,j_ Direction from _active cell_ ( _i,j_ ) to the VCP. 

Notice that 

`Page 10` 

- a. _c*i,j_ is squared. This expresses our confidence that _recurring_ range readings represent actual obstacles, as opposed to single occurrences of range readings, which may be caused by noise. 

- b. _mi,j_ is proportional to - _d_ . Therefore, occupied cells produce large vector magnitudes when they are in the immediate vicinity of the robot, and smaller ones when they are further 

away. Specifically, _a_ and _b_ are chosen such that _a-bdmax_ =0, where _dmax_ = 2 ( _ws_ -1)/2 is the distance between the farthest _active cell_ and the VCP. This way _mi,j_ =0 for the farthest _active cell_ and increases linearly for closer cells. 

_**H**_ has an arbitrary angular resolution α such that _n_ =360/ α is an integer (e.g., α =5o and _n_ =72). Each sector _k_ corresponds to a discrete angle quantized to multiples of α , such that = _k_ α , where _k_ = 0,1,2,..., _n_ -1. Correspondence between _c*i,j_ and sector _k_ is established through 

**==> picture [441 x 14] intentionally omitted <==**

For each sector _k_ , the _polar obstacle densityhk_ is calculated by 

**==> picture [441 x 16] intentionally omitted <==**

Each _active cell_ is related to a certain sector by equations (1) and (3). In Fig. 4, which shows the mapping from _**C**_ * into _**H**_ , all _active cells_ related to sector _k_ have been highlighted. Note that the sector width in Fig. 4 is α =10o (not α =5o, as in the actual algorithm) to clarify the drawing. 

Because of the discrete nature of the _histogram grid_ , the result of this mapping may appear ragged and cause errors in the selection of the steering direction (as explained in Section 4.2). Therefore, a smoothing function is applied to _**H**_ , which is defined by 

**==> picture [441 x 24] intentionally omitted <==**

where _h'k_ is the _smoothed polar obstacle density_ (POD). 

In our current implementation, _l_ =5 yields satisfactory smoothing results. 

```
Page 11
```

Fig. 5a shows a typical obstacle setup in our lab. Note that the gap between obstacles B and C is only 1.2m and that A is a thin pole of 3/4" diameter. The _histogram grid_ obtained after partially traversing this obstacle course is shown in Fig. 5b. The (smoothed) _polar histogram_ corresponding to the momentary position of the robot O is shown in Fig. 6a. The directions 

(in degrees) in the _polar histogram_ correspond to directions measured counterclockwise from the positive x-axis of the _histogram grid_ . The peaks A, B, and C in the _polar histogram_ result from obstacle clusters A, B, and C in the _histogram grid_ . Fig. 6b shows the _polar_ form of the exact same _polar histogram_ as Fig. 6a, overlaying part of the _histogram grid_ of Fig. 5b. 

**Figure 6:** 

**a.** _Polar obstacle density_ represented in the _smoothed polar histogram_ _**H**_ '( _k_ ) ) relative to the robot's position at _O_ (in Fig. 5b). 

**Figure 5: a.** Example of an obstacle course. **b.** The corresponding _Histogram grid_ representation. 

**b.** The same _polar histogram_ as in a, shown in _polar_ form and overlaying part of the _histogram grid_ of Fig. 5b. 

`Page 12` 

## **4.2 Second Data Reduction and Steering Control** 

The second data-reduction stage computes the required steering direction θ . This section explains how θ is computed. 

As can be seen in Fig. 6, a _smoothed polar histogram_ typically has "peaks," i.e., sectors with high PODs, and "valleys," i.e., sectors with low PODs. Any _valley_ comprised of sectors with PODs below a certain threshold (see discussion in Sec. 4.3) is called a _candidate valley_ .  Figure 7 visualizes the match between _candidate valleys_ and the actual environment:  Based on the threshold and the _polar histogram_ of Fig. 6, _candidate valleys_ are shown as lightly shaded sectors in Fig. 7, while unsafe directions (i.e., those with PODs above the threshold) are shown in darker shades. 

**Figure 7:** A threshold on the _polar histogram_ determines the _candidate directions_ for subsequent travel. 

Usually there are two or more _candidate_ travel. _valleys_ and the VFH algorithm selects the one that most closely matches the direction to the target _ktarg_ (an exception to this rule is discussed in Section 4.5). Once a _valley_ is selected, it is further necessary to choose a suitable sector _within_ that _valley_ . The following discussion explains how the algorithm finds this sector and thus the required steering direction. 

At first, the algorithm measures the _size_ of the _selected valley_ (i.e., the number of consecutive sectors with PODs below the threshold). Here, two types of _valleys_ are distinguished, namely, _wide_ and _narrow_ ones. A _valley_ is considered _wide_ if more than _smax_ consecutive sectors fall below the threshold (in our system _smax_ =18). _Wide valleys_ result from wide gaps between obstacles or from situations where only one obstacle is near the vehicle. Fig. 8 shows a typical obstacle configuration that produces a _wide valley_ . The sector that is nearest to _ktarg_ and below the threshold is denoted _kn_ and represents the _near border_ of the _valley_ . The _far border_ is denoted as _kf_ and is defined as _kf_ = _kn_ + _smax_ . The desired steering direction θ is then defined as θ =( _kn_ + _kf_ )/2. Figure 8 demonstrates why this method results in a stable path when traveling alongside an obstacle: If the robot approaches the obstacle too closely (Fig. 8a), θ points away from the obstacle. If the robot is further away from the obstacle, θ allows the robot to approach the obstacle more closely (Fig. 8b). Finally, when traveling at the proper distance _ds_ from the obstacle (Fig. 8c), θ is parallel to the obstacle boundary and small disturbances from this parallel path are corrected as described above.  Note that the distance _ds_ is mostly determined by _smax_ .  The larger _smax_ , the further the robot will keep from an obstacle, under 

`Page 13` 

**==> picture [422 x 349] intentionally omitted <==**

**----- Start of picture text -----**<br>
| Target kf=s   =18 \ max<br>kn k<br>targ<br>? ktarg<br>kf=s   =18max kn<br>Robot<br>Robot<br>?J V 5 ,<br>a d<ds b<br>d>ds<br>7] teer<br>kf=s   =18max<br>kn<br>k<br>targ<br>Robot<br>Robot<br>‘ el<br>c d=ds<br>**----- End of picture text -----**<br>


**==> picture [38 x 15] intentionally omitted <==**

**----- Start of picture text -----**<br>
p16fig8.dfx  7/11/90<br>vfh90.ds4    7/15/93<br>p16fig8.wmf  2/5/95<br>**----- End of picture text -----**<br>


**Figure 8:** Obtaining a stable path when traveling alongside an obstacle: **a.** θ points away from the obstacle when the robot is too close. **b.** θ points toward the obstacle when the robot is further away. 

**c.** Robot runs alongside the obstacle when at the proper distance _ds_ . 

steady state conditions. 

The second case, a _narrow valley_ , is created when the mobile robot travels between closely spaced obstacles, as shown in Fig. 9. Here the _far border kf_ is less than _smax_ sectors apart from _kn_ . However, the direction of travel is again chosen as θ = ( _kn_ + _kf_ )/2 and the robot maintains a course centered between obstacles. 

Note that the robot's ability to pass through narrow passages and doorways results from the ability to identify a _narrow valley_ and to choose a centered path through that _valley_ . This feature is made possible through the intermediate data representation in the _polar histogram_ . Our earlier VFF method and other potential field methods, by contrast, lack this ability [18]. 

`Page 14` 

Another important benefit from this method is the elimination of the vivacious fluctuations in the steering control (a problem associated with the VFF method). With the averaging effect of the _polar histogram_ and the additional smoothing by Eq. (5), _kn_ and _kf_ (and consequently θ ) vary only mildly between sampling intervals. Thus, the VFH method does not require a lowpass filter in the steering control loop and is therefore able to react much faster to unexpected obstacles. Similarly, a VFH-controlled robot does not oscillate when traveling in narrow corridors (as is the case with potential field methods, under certain circumstances [18]). 

## **4.3 The Threshold** 

As mentioned above, a threshold is used to determine the _candidate valleys_ .  While choosing the proper threshold is a critical issue for many sensor-based systems, the performance of the VFH method is largely insensitive to a fine-tuned threshold.  This becomes apparent when considering Fig. 6:  Lowering or raising the threshold even by a factor of 3 or 4 only affects the width of the _candidate valleys_ . This, in turn, has only a small effect on _narrow valleys_ , since the steering direction is chosen in the center of the _valley_ .  In _wide valleys_ , only the distance _ds_ from the obstacle is affected. 

**==> picture [277 x 268] intentionally omitted <==**

**----- Start of picture text -----**<br>
A<br>A<br>kf  [(last free sector)] kn  [(first free sector)]<br>Target<br>k<br>B targ<br>C<br>p16fig9.dwg  7/11/90<br>Robot \vfh\vfh80.drw 7/15/93<br>p16fig7.wmf 2/5/95<br>**----- End of picture text -----**<br>


**Figure 9:** Finding the steering reference direction θ when _ktarg_ is obstructed by an obstacle. 

_Severe_ maladjustments of the threshold have the following effect on the system performance: 

a. If the threshold is much too large (e.g., higher than peak `A' in Fig. 6a), the robot is not "aware" of that obstacle and approaches it on a collision course.  However, during the approach additional sensor readings further increase the CVs representing that obstacle, while the distance _d_ to the affected cells decreases.  As is evident from Eq. (2), this results in higher PODs and consequently in a higher "peak" that _eventually_ exceeds the threshold. However, in this case robot may approach the obstacle too closely (especially when traveling at high speed) and collide with the object. 

`Page 15` 

- b. If, on the other hand, the threshold is much too low, some potential _candidate valleys_ will be precluded and the robot will not pass through narrow passages. 

In summary, it can be concluded that the VFH-system needs a fine-tuned threshold only for the most challenging applications (e.g., travel at high speed _and_ in densely cluttered environments); under less demanding conditions the system performs well even with an unprecisely set threshold. 

One way to optimize performance is to set an _adaptive threshold_ from a higher hierarchical level, e.g., as a function of a "global" plan. For example, during normal travel the threshold is set to a very safe, low level. If the global plan calls for passing through a narrow passage (e.g., a doorway), the threshold is temporarily raised while the travel speed is lowered. 

## **4.4 Speed Control** 

The robot's maximum speed _Vmax_ can be set at the beginning of a run. The robot tries to maintain this speed during the run unless forced by the VFH algorithm to a lower instantaneous speed _V_ , which is determined in each sampling interval as follows: 

The _smoothed polar obstacle density_ in the _current_ direction of travel is denoted as _hc_ '. _h_ '>0 _c_ indicates that an obstacle lies ahead of the robot. Large values of _h_ ' mean a large obstacle lies _c_ ahead or an obstacle is very close to the robot. Either case is likely to require a drastic change in direction, and a reduction in speed is necessary to allow the steering wheels to turn into the new direction. This reduction in speed is implemented by the following function: 

**==> picture [441 x 12] intentionally omitted <==**

where 

**==> picture [441 x 12] intentionally omitted <==**

and _hm_ is an empirically determined constant that causes a sufficient reduction in speed. Note that Eq. (9) guarantees _V_ ' 0, since _h''c hm_ . 

While Eqs. (8) and (9) reduce the speed of the robot in _anticipation_ of a steering maneuver, speed can be further reduced proportionally to the actual steering rate Ω : 

**==> picture [440 x 12] intentionally omitted <==**

where Ω _max_ is the maximal allowable steering rate for the mobile robot (in our system Ω _max_ =120o/sec). 

Note that _V_ is prevented from going to zero by setting a lower limit for _V_ , namely _V Vmin_ ; in our implementation _Vmin_ =4cm/sec. 

```
Page 16
```

## **4.5 Limitations and Remedies** 

The VFH method is a _local path planner_ , i.e., it does not attempt to find an _optimal path_ (an optimal path can only be found if complete environmental information is given). Furthermore, a VFH controlled robot may get "trapped" in dead-end situations (as is the case with other _local path planners_ ). When trapped, mobile robots usually exhibit what has been called " _cyclic behavior_ ," i.e., going around in circles or cycling between _multiple traps_ (typical examples for _cyclic behavior_ are discussed in [5]). While it is possible to devise a set of heuristic rules that would guide the robot out of _trap-situations_ and resolve _cyclic behavior_ [5], the resulting path is still not optimal. 

To resolve these problems, we have introduced a _path monitor_ that works as follows: If the robot diverts from the target direction _ktarg_ (see Fig. 9) the _path monitor_ records this as either left (as is the case in Fig. 9) or right _diversion mode_ . Subsequently, when looking for the _near_ border of the closest _candidate valley_ , _kn_ (see Section 4.2), the VFH algorithm searches to the _left_ or _right_ of _ktarg_ , according to the original _diversion mode_ . If _kn_ cannot be found within _n_ =180o/ α =36 sectors, the _path monitor_ flags a _trap-situation_ . Once a certain _diversion mode_ has been set, it is only cleared if the robot faces again into the target direction. 

When a _trap-situation_ is flagged, the robot slows down (and may come to a complete halt), while the VFH algorithm is temporarily suspended. A _global path planner_ (GPP) algorithm is then invoked to plan a new path based on the available information in the _histogram grid_ [29]. The new path comprises a set of _via-points_ that are then presented as intermediate target locations to the VFH algorithm. 

The maximum travel speed of a VFH-controlled robot is limited by the sampling rate of the ultrasonic sensors, and not by the computation time of the algorithm. In our system, it takes 160 msec to have all 24 ultrasonic sensors sampled and processed once. We estimate that with our current computing hardware (see Section 6) a travel speed of up to 1.5m/sec is possible if the sampling rate of the sensors could be doubled.  The relationship between sampling time, robot travel speed, signal-to-noise ratio, and the resulting certainty values is rather complicated and cannot be treated here because of space limitations (a thorough discussion of this problem is given in [6] and [7]). 

## **5. COMPARISON TO EARLIER METHODS** 

During our research in obstacle avoidance for mobile robots [2,3,4,5] we implemented and evaluated some of the methods discussed in Section 2. This section compares the performance of our new VFH method to these earlier methods. 

```
Page 17
```

## **5.1 Comparison to Edge-Detection Methods** 

The blurry and inaccurate data produced by ultrasonic sensors does not provide the sharply defined contours required by _edge-detection_ methods. Consequently, misreadings or inaccurate range measurements may be interpreted as part of an obstacle, thereby distorting the perceived shape of the obstacle. 

The VFH method, on the other hand, reacts to clusters of range readings. As soon as a range reading has been sampled, it becomes available to the steering controller (via the _histogram grid_ ) and can influence the path of the vehicle immediately. A single range reading will have only minor influence on the path, while repeated range readings in a confined area (cluster) will cause a more drastic change of direction for the vehicle. 

The force field method developed by Brooks [8,9] and the similar method developed by Arkin [1], _do_ function in _experimental_ real-time systems, using actual sensory data [8,9]. However, these methods are somewhat oversimplified, since a threshold determines if an object is at a safe distance or too close. In the latter case, and because of the binary character of the threshold, the robot must stop and rotate away from the object before resuming motion. An additional shortcoming of these methods is their susceptibility to misreadings (due to ultrasonic noise, crosstalk, etc.) since they take into account only one set of range readings (one reading from each ultrasonic sensor). Consequently, misreadings and correct readings (i.e., those produced by actual obstacles) have the same weight. Therefore, a single misreading can cause the resultant force to exceed the threshold level and "scare" the robot away from a possibly safe, free path. Our method, on the other hand, also takes into account _past_ measurements by means of the _histogram grid_ , thereby increasing the weight of recurring measurements, while minimizing the weight of randomly occurring misreadings. In addition, the _smoothing function_ (Eq. 5) reduces the weight of false readings. Thus, the VFH method results in much more robust and error-resistant control. An additional advantage of the VFH method is the permanent map information contained in the _histogram grid_ after a run. Brooks' and Arkin's methods, on the other hand, do not produce a permanent record. 

A critical discussion of both _simulated_ and _experimental_ potential field methods is given in [18].  Also, based on a rigorous mathematical analysis, [18] discusses six inherent shortcomings of potential field methods. 

## **5.4 Reflexive vs. Reactive Control** 

On a more abstract level, researchers are beginning to distinguish between two fundamentally different approaches to mobile robot obstacle avoidance.  The "conventional" approach, _reactive_ control, is based on the traditional artificial intelligence model of human _cognition_ . _Reactive_ control algorithms reason about the robot's _perception_ (sensor data) while building elaborate world models (memory) and subsequently plan the robot's _actions_ .  This approach requires large amounts of computation and decision making, resulting in a relatively slow 

```
Page 18
```

response of the system. _Reflexive_ control (with Brooks as one of its foremost proponents), eliminates _cognition_ altogether.  In _reflexive_ control there is no planning and reasoning; nor are there world models.  Simple _reflexes_ tie _actions_ to _perceptions_ , resulting in faster response to outside stimuli. 

At first glance it may seem that our VFH method is a typical example of _reactive control_ , considering the _histogram grid_ world model and even a second model, the _polar histogram_ . However, some distinctions should be made.  Our world model, the _histogram grid_ , has two different functional properties, namely a _short term_ effect and a _long term_ effect. The _long term effect_ is provided by the whole _histogram grid_ , as described in Section 3. The information stored in the _histogram grid_ may serve for map building purposes and for the _global path planner_ (see Section 4.5). A large _histogram grid_ , however, is not necessary for our algorithm to work properly. It is the _short term effect_ of the _histogram grid_ that is important for the VFH algorithm. As explained in Section 4.1, only cells within the _active window_ influence the VFH computations. Since the _active window_ travels with the robot, cells are only briefly inside the window and have thus only a _temporary (short term)_ effect. Also, since the ultrasonic sensors are limited to only 2m look-ahead (about the size of the _active window_ ), only cells _inside_ the window are updated with sensor information. Therefore, the VFH algorithm would work equally well if _all information was lost_ from cells that are outside of the _active window_ . Through the concept of the _active window_ , the _histogram grid_ becomes sort of a "short term memory," where readings are retained briefly (while the _active window_ sweeps through the area) to enhance the accuracy by accumulating multiple sensor readings. In a way, this process is similar to the short term memory associated with human hearing: Without this mechanism, people would hear but not necessarily comprehend all speech. 

## **6. EXPERIMENTAL RESULTS** 

We implemented and tested the VFH method on our mobile robot CARMEL ( _C_ omputer- _A_ ided _R_ obotics for _M_ aintenance, _E_ mergency, and _L_ ife support). CARMEL is based on a commercially available mobile platform [12], as seen in Fig. 10. This platform has a maximum travel speed of _Vmax_ = 0.78 m/sec, a maximum steering rate of Ω = 120 deg/sec, and weighs (in its current configuration) about 125 kg. The platform has a hexagonal structure and a unique three-wheel drive (synchro-drive) that permits omnidirectional steering. A Z)80 onboard computer serves as the low-level controller of the vehicle. Two computers were added: a PC-compatible single-board computer to control the sensors, and a 20 Mhz, 80386-based AT-compatible that runs the VFH algorithm. 

```
Page 19
```

CARMEL is equipped with a ring of 24 ultrasonic sensors [25]. The sensor ring has a diameter of 0.8m, and objects must be at least 0.27m away from the sensors to be detected. Therefore, the theoretical minimum width for safe travel in a passage-way is _Wmin_ =0.8 + 2x0.27 = 1.34 m. 

In extensive tests, we ran the VFHcontrolled CARMEL through difficult obstacle courses. The obstacles were unmarked, commonplace objects such as chairs, partitions, and bookshelves. In most experiments, CARMEL ran at its maximum speed 

**Figure 10:** CARMEL, The University of Michigan's Cybermotion K2A robot, dashes through an obstacle course at 0.8 m/sec. 

_Vmax_ =0.78m/sec. This speed was 

only reduced when an obstacle was approached head-on (see discussion of speed control in Section 4.4). 

Fig. 11 shows the _histogram grid_ after a run through a particularly challenging obstacle course of 3/4"-diameter vertical poles spaced at a distance of about 1.4m from each other. The actual location of the rods is indicated by (+) symbols in Fig. 11. It should be noted that none of the obstacle locations were known to the robot in advance: the CV-clusters in Fig. 11 gradually appeared on the operator's screen while CARMEL was moving. 

To test the performance limits of our system, we performed a variety of experiments with other slender obstacles. For example, 1/2" diameter poles were still detected, but not reliably so when approached at CARMEL's maximum speed. Unreliable detection would also result when 1"x1" square rods were used. Larger objects, such as 2" diameter cylinders, square shaped cardboard boxes, furniture, and even slowly walking people are reliably detected and avoided. These obstacles are easier to detect than the 3/4" poles in the experiment described here. 

Each blob in Fig. 11 represents one cell in the _histogram grid_ . In our current implementation, _certainty values_ (CVs) range from 0 to 15 and are indicated in Fig. 11 by blobs of varying sizes. CV = 0 means no sensor reading has been projected onto the cell during the run (i.e., no blob), while CVs > 0 indicate the increasing confidence in the existance of an object at that location.  CARMEL traversed this obstacle course at an average speed of 0.58 m/sec without stopping for obstacles.  Note that this is a _typical_ experimental run, and similar performance 

`Page 20` 

has been routinely obtained in countless experiments and demonstrations, using different kinds of obstacles at random layouts. 

An indication of the real-time performance of the VFH algorithm is the sampling time _T_ (i.e., the rate at which the steer and speed commands for the low-level controller are issued). The following steps occur during _T_ : 

- a. Obtain sonar information from the sensor controller. 

- b. Update the _histogram grid_ . 

- c. Create the _polar histogram_ . 

- d. Determine the free sector and steering direction. 

- e. Calculate the speed command. 

- f. Communicate with the low-level motion controller (send speed and steer commands and receive position update). 

On an Intel 80386-based PC-compatible computer running at 20 Mhz, _T_ = 27 msec. 

## **7. CONCLUSIONS** 

**Figure 11:** _Histogram grid_ representation of a run through a field of densely spaced, thin vertical poles. This paper presents a new obstacle The average speed in this run was _Vavrg_ = 0.58m/sec. avoidance method for fast-running vehicles. 

This approach, called the _vector field histogram_ (VFH) method, has been developed and successfully tested on our experimental mobile robot CARMEL. The VFH algorithm is computationally efficient, very robust and insensitive to misreadings, and it allows continuous and fast motion of the mobile robot without stopping for obstacles. The VFH-controlled mobile robot traverses very densely cluttered obstacle courses at high average speeds and is able to pass through narrow openings (e.g., doorways) or negotiate narrow corridors without oscillations. 

The VFH method is based on the following principles: 

- a. A two-dimensional Cartesian _histogram grid_ is continuously updated in real-time with range data sampled by the on-board range sensors. 

- b. The _histogram grid_ is reduced to a one-dimensional _polar histogram_ that is constructed around the momentary location of the robot. The _polar histogram_ is the most significant 

`Page 21` 

distinction between the VFF and the VFH method as it allows a spatial interpretation (called _polar obstacle density_ ) of the robot's instantaneous environment. 

- c. Consecutive sectors with a _polar obstacle density_ below threshold are called " _candidate valleys_ ."  The _candidate valley_ closest to the target direction is selected for further processing. 

- d. The direction of the center of the selected _candidate direction_ is determined and the steering of the robot is aligned with that direction. 

- e. The speed of the robot is reduced when approaching obstacles head-on. 

The characteristic behavior of a VFH-controlled mobile robot is best described as follows: The response of the vehicle is dependent on the _likelihood for the existence of an obstacle_ . In the _histogram grid_ , this likelihood is expressed in terms of size and _certainty values_ of a cluster. This information is transformed into height and width of an elevation in the _polar histogram_ . The strength of the VFH method lies thus in its ability to maintain a statistical obstacle representation at both the _histogram grid_ level as well as at the intermediate data level, the _polar histogram_ . This feature makes the VFH method especially suited to the accommodation of inaccurate sensor data, such as that produced by ultrasonic sensors, as well as sensor fusion. 

```
Page 22
```

## **8. REFERENCES** 

1. Arkin, R. C., "Motor Schema-Based Mobile Robot Navigation." _The International Journal of Robotics Research_ , August 1989, pp. 92-112. 

2. Borenstein, J. and Koren, Y., "A Mobile Platform For Nursing Robots." _IEEE Transactions on Industrial Electronics_ , Vol. 32, No. 2, 1985, pp. 158-165. 

3. Borenstein, J. and Koren, Y., "Motion Control Analysis of a Mobile Robot." _Transactions of ASME, Journal of Dynamics, Measurement and Control_ , Vol. 109, No. 2, 1987, pp. 73-79. 

4. Borenstein, J. and Koren, Y., "Obstacle Avoidance With Ultrasonic Sensors." _IEEE Journal of Robotics and Automation_ , Vol. RA-4, No. 2, 1988, pp. 213-218. 

5. Borenstein, J. and Koren, Y., "Real-time Obstacle Avoidance for Fast Mobile Robots." _IEEE Transactions on Systems, Man, and Cybernetics_ , Vol. 19, No. 5, Sept/Oct 1989, pp. 1179-1187. 

6. Borenstein, J. and Koren, Y., "Histogramic In-motion Mapping for Mobile Robot Obstacle Avoidance." _IEEE Journal of Robotics and Automation_ , Vol. 7, No. 4, 1991, pp. 535-539. 

7. Borenstein, J. and Koren, Y., "Real-time Map-building for Fast Mobile Robot Obstacle Avoidance." _SPIE Symposium on Advances in Intelligent Systems, Mobile Robots V_ , Boston, MA, Nov. 4-9, 1990. 

8. Brooks, R. A., "A Robust Layered Control System for a Mobile Robot." _IEEE Journal of Robotics and Automation_ , Vol. RA-2, No. 1, 1986, pp. 14-23. 

9. Brooks, R. A. and Connell, J. H., "Asynchronous Distributed Control System for a Mobile Robot." _Proceedings of the SPIE, Mobile Robots_ , Vol. 727, 1987, pp. 77-84. 

10. Crowley, J. L., "Dynamic World Modeling for an Intelligent Mobile Robot." _IEEE Seventh International Conference on Pattern Recognition, Proceedings_ , Montreal, Canada, July 30-August 2, 1984, pp. 207-210. 

11. Crowley, J. L., "World Modeling and Position Estimation for a Mobile Robot Using Ultrasonic Ranging." _Proceedings of the 1989 IEEE International Conference on Robotics and Automation_ . Scottsdale, Arizona, May 14-19, 1989, pp. 674-680. 

12. Cybermation, "K2A Mobile Platform." _Commercial Offer_ , 5457 JAE Valley Road, Roanoke, VA 24014, 1987. 

```
Page 23
```

13. Elfes, A., "Sonar-based Real-World Mapping and Navigation." _IEEE Journal of Robotics and Automation_ , Vol. RA-3, No 3, 1987, pp. 249-265. 

14. Flynn, A. M., "Combining Sonar and Infrared Sensors for Mobile Robot Navigation." _The International Journal of Robotics Research_ , Vol. 7, No. 6, December 1988, pp. 5- 14. 

15. Newman, W. S. and Hogan, N., "High Speed Robot Control and Obstacle Avoidance Using Dynamic Potential Functions." _Proceedings of the 1987 IEEE International Conference on Robotics and Automation,_ Raleigh, North Carolina, March 31-April 3, 1987, pp. 14-24. 

16. Khatib, O., "Real-Time Obstacle Avoidance for Manipulators and Mobile Robots." _1985 IEEE International Conference on Robotics and Automation_ , St. Louis, Missouri, March 25-28, 1985, pp. 500-505. 

17. Koren, Y. and Borenstein, J., "Analysis of Control Methods for Mobile Robot Obstacle Avoidance." _Proceedings of the IEEE International Workshop on Intelligent Motion Control_ , Istanbul, Turkey, August 20-22, 1990, pp. 457-462. 

18. Koren, Y. and Borenstein, J., 1991, "Potential Field Methods and Their Inherent Limitations for Mobile Robot Navigation." _Proceedings of the IEEE International Conference on Robotics and Automation_ Sacramento, California, April 7-12, 1991, pp. 1398-1404. 

19. Krogh, B. H., "A Generalized Potential Field Approach to Obstacle Avoidance Control." _International Robotics Research Conference_ , Bethlehem, Pennsylvania, August, 1984. 

20. Krogh, B. H. and Thorpe, C. E., "Integrated Path Planning and Dynamic Steering Control for Autonomous Vehicles." _Proceedings of the 1986 IEEE International Conference on Robotics and Automation_ , San Francisco, California, April 7-10, 1986, pp. 16641669. 

21. Kuc, R. and Barshan, B., "Navigating Vehicles Through an Unstructured Environment With Sonar." _Proceedings of the 1989 IEEE International Conference on Robotics and Automation_ , Scottsdale, Arizona, May 14-19, 1989, pp. 1422-1426. 

22. Lumelsky V. and Skewis, T., "A Paradigm for Incorporating Vision in the Robot Navigation Function." _Proceedings of the 1988 IEEE Conference on Robotics and Automation_ , Philadelphia, April 25, 1988, pp. 734-739. 

23. Moravec, H. P. and Elfes, A., "High Resolution Maps from Wide Angle Sonar." _IEEE Conference on Robotics and Automation_ , Washington D.C., 1985, pp. 116-121. 

```
Page 24
```

24. Moravec, H. P., "Sensor Fusion in Certainty Grids for Mobile Robots." _AI Magazine_ , Summer 1988, pp. 61-74. 

25. POLAROID Corporation, Ultrasonic Components Group, 119 Windsor Street, Cambridge, Massachusetts 02139, 1990. 

26. Raschke, U. and Borenstein, J., "A Comparisson of Grid-type Map-building Techniques by Index of Performance." _Proceedings of the 1990 IEEE International Conference on Robotics and Automation_ , Cincinnati, Ohio, May 13-18, 1990. 

27. Thorpe, C. F., "Path Relaxation: Path Planning for a Mobile Robot." _Carnegie-Mellon University, The Robotics Institute, Mobile Robots Laboratory, Autonomous Mobile Robots, Annual Report 1985_ , pp. 39-42. 

28. Weisbin, C. R., de Saussure, G., and Kammer, D., "SELF-CONTROLLED. A RealTime Expert System for an Autonomous Mobile Robot." _Computers in Mechanical Engineering_ , September, 1986, pp. 12-19. 

29. Zhao, Y., BeMent, S. L., and Borenstein, J., "Dynamic Path Planning for Mobile Robot Real-time Navigation." _Twelfth IASTED International Symposium on Robotics and Manufacturing_ , Santa Barbara, California, November 13-15, 1989. 

## **Footnotes** 

1. We use the term " _probability_ " in the literal sense of " _likelihood_ ." 

2. For symmetrically shaped vehicles, the VCP is easily defined as the geometric center of the vehicle. For rectangular vehicles, it is possible to chose two VCPs, e.g., one each at the center-point of the front and rear axles. 

```
Page 25
```


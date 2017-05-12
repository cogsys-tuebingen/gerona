# Navigation: Path Follower
A package containing various path following control algorithms (controllers) for wheeled mobile robots. Each controller is thoroughly tested both in simulation and on real mobile robots.

Each controller has one or more implementations, depending on its compatibility with different kinematic configurations, where this compatibility is described as a prefix 'ackermann', 'omnidrive', etc. For instance, the orthogonal-exponential controller ('orthexp') has 3 different implementations:
* `ackermann_orthexp` (this implementation is used for car-like and bi-steerable vehicles),
* `differential_orthexp` (differential and skid-steered drives),
* `omnidrive_orthexp` (omnidirectional vehicles).

The HBZ controller has two implementations:
* `HBZ` (used for regular path following),
* `HBZ_TT` (used for target tracking),

where "TT" is short for "target tracking". This target tracking feature can be used for tracking of any dynamic object (e.g. another robot). We used it to follow a human jogger outdoors.

The controllers and their compatibilities with different kinematic configurations and special features are described in the next table.


## Kinematic compatibilities and features
|                    | ackermann     | bi-steerable   | omnidrive | differential | skid-steer | target tracking | reverse driving |
| ------------------ | ------------- | -------------- | --------- | ------------ | ---------- | --------------- | --------------- | 
| `input_scaling`    | &#10004;      | &#10004;       | &#10005;  | &#10004;     | &#10004;   | &#10005;        | &#10004;        |
| `pure_pursuit`     | &#10004;      | &#10004;       | &#10005;  | &#10004;     | &#10004;   | &#10005;        | &#10004;        |
| `stanley`          | &#10004;      | &#10004;       | &#10005;  | &#10005;     | &#10005;   | &#10005;        | &#10004;        |
| `orthexp`          | &#10004;      | &#10004;       | &#10004;  | &#10004;     | &#10004;   | &#10005;        | &#10004;        |
| `HBZ`              | &#10005;      | &#10005;       | &#10005;  | &#10004;     | &#10004;   | &#10004;        | &#10004;        |
| `PBR`              | &#10005;      | &#10005;       | &#10005;  | &#10004;     | &#10004;   | &#10005;        | &#10004;        |
| `SLP`              | &#10005;      | &#10005;       | &#10005;  | &#10004;     | &#10004;   | &#10005;        | &#10004;        |
| `OFC`              | &#10004;      | &#10004;       | &#10004;  | &#10004;     | &#10004;   | &#10004;        | &#10005;        |
| `potential_field`  | &#10004;      | &#10004;       | &#10004;  | &#10004;     | &#10004;   | &#10004;        | &#10005;        |
| `DWA`              | &#10004;      | &#10004;       | &#10004;  | &#10004;     | &#10004;   | &#10004;        | &#10005;        |
| `PID`              | &#10004;      | &#10005;       | &#10005;  | &#10005;     | &#10005;   | &#10005;        | &#10004;        |


Top collumns represent kinematic configurations: 
* **ackermann** (car-like, i.e. Ackermann steering vehicles), 
* **bi-steerable** (vehicles with front- and rear-wheels steering),
* **omnidrive** (omnidirectional vehicles), 
* **differential** (differential drives, i.e. unicycles), 
* **skid-steer** (skid steering), 

as well as two features:
* **target tracking** (a special feature for tracking a dynamic target, e.g. following a person),
* **reverse driving** (cabability of path following while driving backwards).

The left border rows represent different controllers, and they are listed together with the references as follows:

* `input_scaling`: A controller using a certain mathematical scaling of the input in its derivation, hence the self-forged name "input-scaling". Except for the well-known implementation for car-like vehicles, we also offer a self-derived implementation for unicycles (tested on a differential and a skid-steered platform), and a self-derived implementation for bi-steerable vehicles.
      * References:
        1. De Luca, Alessandro, Giuseppe Oriolo, and Claude Samson. "Feedback control of a nonholonomic car-like robot." Robot motion planning and control. Springer Berlin Heidelberg, 1998. 171-253.
* `pure_pursuit`: The well known pure pursuit controller. The implementation variety is the same as for the input-scaling controller.
      * References:
        1. Coulter, R. Craig. Implementation of the pure pursuit path tracking algorithm. No. CMU-RI-TR-92-01. CARNEGIE- MELLON UNIV PITTSBURGH PA ROBOTICS INST, 1992.
* `stanley`: The DARPA-Challenge-winning Stanley controller. We offer the standard implementation for car-like vehicles, as well as a self-derived implementation for bi-steerable vehicles.
      * References:
        1. Thrun, Sebastian, et al. "Stanley: The robot that won the DARPA Grand Challenge." Journal of field Robotics 23.9 (2006): 661-692.
        2. Snider, Jarrod M. "Automatic steering methods for autonomous automobile path tracking." Robotics Institute, Pittsburgh, PA, Tech. Rep. CMU-RITR-09-08 (2009).
* `orthexp`: A self-made controller from our department, based on the work of several colleagues.The algorithm is using an orthogonal projection and an exponential law for control, hence the name "orthogonal-exponential (orthexp)"
      - References:
        1. Goran Huskić, Sebastian Buck, and Andreas Zell. A simple and efficient path following algorithm for wheeled mobile robots. In Intelligent Autonomous Systems (IAS), The 14th International Conference on, Shanghai, CN, July 2016.
        2. Sebastian Buck, Richard Hanten, Goran Huskić, Gerald Rauscher, Alina Kloss, Jan Leininger, Eugen Ruff, Felix Widmaier, and Andreas Zell. Conclusions from an object-delivery robotic competition: Sick robot day 2014. In Advanced Robotics (ICAR), The 17th International Conference on, pages 137-143, Istanbul, TR, July 2015.
        3. Mojaev, A., Zell, A.: Tracking control and adaptive local navigation for nonholonomic mobile robot. In: Intelligent Autonomous Systems (IAS-8). pp. 521{528. IOS Press, Amsterdam, Netherlands (Mar 2004)
        4. Li, X., Wang, M., Zell, A.: Dribbling control of omnidirectional soccer robots. In: Robotics and Automation, 2007 IEEE International Conference on. pp. 2623{2628. IEEE (2007)
* `HBZ`: A self-made controller from our department. The name comes from the authors' surname initials.
      - References:
        1. Goran Huskić, Sebastian Buck, and Andreas Zell. Path following control of skid-steered wheeled mobile robots at higher speeds on different terrain types. In IEEE International Conference on Robotics and Automation (ICRA), Singapore, 2017. (accepted for publication).
        2. Goran Huskić, Sebastian Buck, Luis Azareel Ibargüen González, and Andreas Zell. Outdoor Person Following at Higher Speeds using a Skid-Steered Mobile Robot. In Intelligent Robots and Systems (IROS), The International Conference on, Singapore, 2017. (submitted)
* `PBR`: A controller for skid-steered vehicles and differential drives. The name comes from the authors' surname initials..
      - References:
        1. J. Pentzer, S. Brennan, and K. Reichard, “Model-based prediction of skid-steer robot kinematics using online estimation of track instantaneous centers of rotation,” Journal of Field Robotics, vol. 31, no. 3, pp. 455–476, 2014.
        2. J. Pentzer, S. Brennan, and K. Reichard, “The use of unicycle robot control strategies for skid-steer robots through the icr kinematic mapping,” in Intelligent Robots and Systems (IROS 2014), 2014 IEEE/RSJ International Conference on. IEEE, 2014, pp. 3201–3206.
* `SLP`: A kinematic controller for unicycles. The name comes from the authors' surname initials..
      - References:
        1. Soetanto, D., L. Lapierre, and A. Pascoal. "Adaptive, non-singular path-following control of dynamic wheeled robots." Decision and Control, 2003. Proceedings. 42nd IEEE Conference on. Vol. 2. IEEE, 2003.
        2. Indiveri, Giovanni, Andreas Nüchter, and Kai Lingemann. "High speed differential drive mobile robot path following control with bounded wheel speed commands."Robotics and Automation, 2007 IEEE International Conference on. IEEE, 2007.
* `OFC`: A PID-based reactive controller for object tracking. The authors call it "Object Following Controller (OFC)".
      - References:
        1. A. Leigh, J. Pineau, N. Olmedo, and H. Zhang, “Person tracking and following with 2d laser scanners,” in Robotics and Automation (ICRA), 2015 IEEE International Conference on. IEEE, 2015, pp. 726–733.
* `potential_field`: The well known obstacle avoidance algorithm "Potential Field Method". We offer its implementation both for path following and target tracking,
      - References:
        1. Y. Koren and J. Borenstein, “Potential field methods and their inherent limitations for mobile robot navigation,” in Robotics and Automation, 1991. Proceedings., 1991 IEEE International Conference on. IEEE, 1991, pp. 1398–1404.
* `DWA`: The well known obstacle avoidance algorithm "Dynamic Window Approach (DWA)". We offer its implementation both for path following and target tracking,
      - References:
        1. D. Fox, W. Burgard, and S. Thrun, “The dynamic window approach to collision avoidance,” IEEE Robotics & Automation Magazine, vol. 4, no. 1, pp. 23–33, 1997.
* `PID`: A self-made PID-based path following algorithm for car-like vehicles.


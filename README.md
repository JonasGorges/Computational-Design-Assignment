# Computational Design Assignment
 ITECH - Final Assignment in Computational Design Class

## Methods & Development

### Code organization
Our aim is to design an agent-based system, where particles interact within defined boundaries, following predetermined rules and behaviors. These particles, representing prey and predators (fishes), undergo calculations to determine forces influencing their movement. Subsequently, these forces are applied to update particle positions over time steps, simulating dynamic behaviors such as schooling, alignment, pursuit, and evasion. The system encapsulates these behaviors through a combination of class attributes, methods, and interactions, resulting in emergent patterns akin to real-world phenomena. Within our system framework, we've delineated distinct methods: one for calculating the forces influencing the fishes, and the other for updating the particle positions based on the applied forces. In order to organize our code better, the particles were separated into parent (Fish) and child classes (Prey/ Predator) where they inherit, as many attributes  and methods are similar.

### Implementation of behaviors
The particles, representing both prey and predator entities, undergo thorough computations to determine the forces directing their motions. These forces are subsequently employed to modify particle positions over successive time intervals, replicating dynamic behaviors such as schooling, alignment, pursuit, and evasion. Initially, each fish, whether prey or predator, is endowed with a random velocity and vector for wandering. Subsequently, the prey vectors are collectively oriented towards an attractor point to foster schooling behavior. This process entails implementing behaviors concerning neighboring fishes, starting with cohesion, separation, and alignment, where vectors are computed to regulate distances between particles. To enhance movement fluidity, a spiral behavior is introduced alongside flight and pursuit, adjusting fish speeds either towards capturing prey or fleeing to avoid predation.

### Improved performance with diagonal checking of distances
In our first implementation of the agent based system distances from each agent to all other agents in the system were calculated for each behavior. This significantly slows down the code execution if the count of fishes in the system is increased. However we needed a high amount of agents, to achieve the impression of a swarm of fish. To be able to also simulate high amounts of fish, we separated the calculation of distances from the behaviors. This is possible, because agent positions or changes to the list of agents (i.e. if a predator is added or prey is killed) are not updated immediately for each behavior but only after all behaviors are calculated for all agents and the update method of the particle system is called. 

The performance of distance calculation is further increased through diagonal checking. Instead of calculating the distance to all other agents for each agent only unique distances are calculated. I.e. given an agent A, we only calculate the distance from agent A to agent B. Since distance from agent B to agent A is equal to distance A to B, this calculation is obsolete. This method can be best implemented by constructing a 2D distance matrix. We can see that this matrix is symmetric to its main diagonal. Hence, only one triangle of distances has to be calculated.

### Generation of 3D-printable geometry
The output of our schooling simulation is limited to lists or points that represent the position of each fish for each iteration. For better visualization these points have been connected to a polyline for each fish, that represents its path. The history of positions for our final simulation was set to 200 points for each fish. While shorter histories have been helpful during development of the algorithm, since the immediate behavior of each fish is more readable with shorter curves, longer histories help to trace the movement of the fish when only one time frame can be shown, like it is the case for a 3D printed static object. Furthermore, we increased the amount of fish to 300, to achieve a higher density of curves. 

After setting length of history and amount of fish we ran the simulation, looking for timeframes that show a variety of behaviors at once. We identified the event the predators attack for the first time as especially interesting. Here the first part of the fish’s history still shows aligned schooling and spiral movement. However the most recent positions of the history show the flight behavior, caused by the predator pursuing its prey.

Once an aesthetically appealing configuration appeared, we stopped the simulation. To get a smoother appearance of the paths, we resampled the polylines with 100 samples each and rebuilt them as splines. In the next step we created pipes with varying radius along the curves. Here we decided to slightly increase the pipe radius from the oldest to the newest part of the history and to decrease it again for the latest part of the history. This way we achieved a more dynamic appearance of the paths. These steps were performed with standard Rhino Grasshopper components.

However the outcome of the previous steps results in unconnected geometries, which would make additional connections necessary for 3D printing. To achieve one connected volume we looked into the generation of isosurfaces. Because of the improved performance compared to Grasshopper and thus the possibility to work with much higher mesh resolutions, we chose the special effects software Houdini VFX for this step. Inside Houdini we generated a value field from our previously generated geometry. Through different blur and erosion algorithms applied to this value field we were able to achieve a fluid like appearance of the generated isosurface, which we think is suitable for representing aquatic dwellers like fish.

### “Floating” 3D-print
Our subsequent goal was to present the developed geometry in a novel way, illustrating the fishes swimming in the ocean. To achieve this, we explored the concept of floating 3D prints, which involved considering the geometry's location, bridge dimensions, and volume density. After numerous iterations, we attained an optimal balance between transparent printed bridges and white geometry to ensure clear visibility. To streamline this process, we devised a parametric model capable of generating iterations at different scales and swiftly creating openings in the geometry for the bridges.

## References
[1]	Vetemaa, E. M. (2020). Simulating the Collective Movement of Fish Schools. 
	University of Tartu, Institute of Computer Science.
[2]	Di Santo, V. (2023). Schooling in fishes. Reference Module in Life 
	Sciences. Elsevier.
[3]	Jeon, W., Kang, S.-H., Leem, J.-B., & Lee, S.-H. (2013). Characterization of fish 
	schooling behavior with different numbers of Medaka (Oryzias latipes) and goldfish 
	(Carassius auratus) using a Hidden Markov Model. Physica A: Statistical Mechanics 
	and its Applications, 392(10), 2426-2433.
[4]	Hartono, A. D., Nguyen, L. T. H., & Tạ, T. V. (2024). A stochastic differential equation 
	model for predator-avoidance fish schooling. Mathematical Biosciences, 367.
[5]	Takagi, T., Nashimoto, K., Yamamoto, K., & Hiraishi, T. (1993). Bull. Japan. Soc. Sci. 
	Fish, 59, 1279.
[6]	Huth, A., & Wissel, C. (1994). Ecol. Model., 75, 135.
[7]	Niwa, H.-S. (1994). J. Theoret. Biol., 171, 123

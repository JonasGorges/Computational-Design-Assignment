# Computational Design Assignment
 ITECH - Final Assignment in Computational Design Class

## Methods & Development

Code organization
Our aim is to design an agent-based system, where particles interact within defined boundaries, following predetermined rules and behaviors. These particles, representing prey and predators (fishes), undergo calculations to determine forces influencing their movement. Subsequently, these forces are applied to update particle positions over time steps, simulating dynamic behaviors such as schooling, alignment, pursuit, and evasion. The system encapsulates these behaviors through a combination of class attributes, methods, and interactions, resulting in emergent patterns akin to real-world phenomena. Within our system framework, we've delineated distinct methods: one for calculating the forces influencing the fishes, and the other for updating the particle positions based on the applied forces. In order to organize our code better, the particles were separated into parent (Fish) and child classes (Prey/ Predator) where they inherit, as many attributes  and methods are similar.

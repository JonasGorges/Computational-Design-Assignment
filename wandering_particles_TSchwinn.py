import Rhino.Geometry as rg
import random
import math

boundarySize = 20

class ParticleSystem:
    
    def __init__(self, particleCount):
        self.Particles = []
        for i in range(0, particleCount):
            self.Particles.append(Particle())
            
    def Update(self):
        for particle in self.Particles:
            particle.Update()

class Particle:

    def __init__(self):
        
        # Initilize the Position to a random point
        self.Position = rg.Point3d(random.uniform(0, boundarySize), random.uniform(0, boundarySize), 0.0)
        
        # Initilize the Velocity to a random 2D Vector of length 0.1
        alpha = random.uniform(0, 6.28)
        self.Velocity = 0.1 * rg.Vector3d(math.cos(alpha), math.sin(alpha), 0.0)
        
        self.History = [self.Position]
    
    
    def Update(self):
        self.Velocity.Rotate(random.uniform(-0.2, 0.2), rg.Vector3d.ZAxis)
        self.Position += self.Velocity
        
        self.ProcessAttractor()
        self.ProcessBoundary()
        
        self.History.append(self.Position)
        
        if len(self.History) > 20:
            del self.History[0]
        
        
    def ProcessAttractor(self):
        toAttractor = iAttractor - self.Position
        toAttractor *= 0.01 / toAttractor.Length
        self.Velocity += toAttractor
        self.Velocity *= 0.1 / self.Velocity.Length
        
        
    def ProcessBoundary(self):
        if self.Position.X < 0.0:
            self.Position.X = 0
            self.Velocity.X = -self.Velocity.X
        elif self.Position.X > boundarySize:
            self.Position.X = boundarySize
            self.Velocity.X = -self.Velocity.X
            
        if self.Position.Y < 0.0:
            self.Position.Y = 0
            self.Velocity.Y = -self.Velocity.Y
        elif self.Position.Y > boundarySize:
            self.Position.Y = boundarySize
            self.Velocity.Y = -self.Velocity.Y


# Main Script:

if iReset or not("myParticleSystem" in globals()):
    myParticleSystem = ParticleSystem(100)
else:
    myParticleSystem.Update()
    
# Output the visualization geometry
paths = [] 
for particle in myParticleSystem.Particles:
    paths.append(rg.PolylineCurve(particle.History))
oGeometry = paths


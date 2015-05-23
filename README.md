# PhysicsProject

Fabio Cogliati, Manuele Nerucci

Physics Programming Exam 

University of Verona, Italy - Computer Game Development Master

Development of a basic physics engine.


--Implemented Features--

RigidBody movement: RigidBody class models a rigidBody and calculates the variation of rotation and position of the object caused by the applied forces to it. It calculates how Gravity, collision friction, air friction and other forces changes rotation and position of the defined object. Wolrd class models a world that contains some rigid bodies.

Collision Detection: Implementation of BoxCollider, SphereCollider and static PlaneCollider (defined by plane equation function. IntersectOperation class implements all the intersection method we need to define to check the collision between all this types of colliders.

Utils: Implementation of Vector3, Quaternion, Matrix to help all the physics operations we need to calculate

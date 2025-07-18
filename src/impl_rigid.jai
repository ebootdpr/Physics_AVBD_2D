/*
* Copyright (c) 2025 Chris Giles
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Chris Giles makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

rigid_instantiate :: ( solver: *Solver,  size: Vector2,  density: float,  friction: float,  position: Vector3,  velocity: Vector3 = .{}) -> *Rigid
{
    rigi:= New(Rigid);
    // : solver(solver), forces(0), next(0), position(position), velocity(velocity), prevVelocity(velocity), size(size), friction(friction)
    rigi.solver = solver;
    rigi.forces = null;
    rigi.next = null;
    rigi.position = position;
    rigi.velocity = velocity;
    rigi.prevVelocity = velocity;
    rigi.size = size;
    rigi.friction = friction;
    //

    //attach procs:
    rigi.draw = rigid_draw;
    rigi.constrainedTo = rigid_constrainedTo;
    //

    
    //add to linked list:
    rigi.next = solver.bodies;
    rigi.solver.bodies = rigi;
    //

    //compute mass properties and bounding radius:
    {
        rigi.mass = size.x * size.y * density;
        rigi.moment = rigi.mass * dot(size, size) / 12.0;
        rigi.radius = length(size * 0.5);
    }
    //
    return rigi;
}


rigid_delete :: (using body: *Rigid)
{
    defer free(body);
    // Remove from linked list
    p: **Rigid = *solver.bodies;
    while (p.* != body)
        p = *(p.*).next;
    p.* = next;
}



rigid_constrainedTo :: (using rigid: *Rigid, other: *Rigid) -> bool
{
    // Check if "rigid" body is constrained to the other body
    f := forces;
    while f != null {
        c1 := (f.bodyA == rigid && f.bodyB == other) ;
        c2 := (f.bodyA == other && f.bodyB == rigid) ;
        if c1 || c2 return true;
        f = f.next;
    }
    return false;
}


default_rigid_draw :: (using rigid: *Rigid)
{
    //gather points from rigid:
    R := rotation(position.z);
    v0 : Vector2= R * Vector2.{ -size.x * 0.5, -size.y * 0.5 } + .{position.x, position.y};
    v1 : Vector2= R * Vector2.{ size.x * 0.5, -size.y * 0.5 } + .{position.x, position.y};
    v2 : Vector2= R * Vector2.{ size.x * 0.5, size.y * 0.5 } + .{position.x, position.y};
    v3 : Vector2= R * Vector2.{ -size.x * 0.5, size.y * 0.5 } + .{position.x, position.y};

    //implement your own draw:
}

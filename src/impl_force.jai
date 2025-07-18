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

//this is only is used by derived classes.
force_instantiate :: (mem: *Force, solver: *Solver,  bodyA: *Rigid,  bodyB: *Rigid) -> *Force
{   
    force: *Force;

    //idk how to do constructor/destructor magic in jai, and I dont want to know because I will replace everything... probably.

    //this is, Manifold Spring nd Joint are structs that at the begginning of their memory, they have attached all Force fields (they inherit from force)
    //so we are safe believing that mem is of type Force.
    if mem force = mem; //else segfault;
    //else force = New(Force); // We never instantiate a force directly, so this is commented out, Is this class "Force" called an abstract class in oops?
    //

    //: solver(solver), bodyA(bodyA), bodyB(bodyB), nextA(0), nextB(0):
    force.solver = solver;
    force.bodyA = bodyA;
    force.bodyB = bodyB;
    force.nextA = null;
    force.nextB = null;
    //
    
    //attach procs:
    force.disable = force_disable;
    //dont attach below procs because in Manifold,Spring, Joint and Ignorecollison, we replace all procedures except disable and the destructor is also never replaced, we use force_delete() for all of them. 
    // force.rows = null;
    // force.initialize = null;//(force: *Force ) -> bool{return false;};
    // force.computeConstraint = null;
    // force.computeDerivatives = null;
    // force.draw = (p: *Force) {}; //I dont trust initializers with structs that have fields like: "#as using some: type"
    //

    //add to solver linked list:
    force.next = solver.forces;
    solver.forces = force;
    //

    //add to body linked lists if they exist:
    if force.bodyA
    {
        force.nextA = bodyA.forces;
        bodyA.forces = force;
    }
    if force.bodyB
    {
        force.nextB = bodyB.forces;
        bodyB.forces = force;
    }
    //

    //set some reasonable defaults:
    for i: 0.. MAX_ROWS-1
    {
        force.J[i] = .{ 0, 0, 0 };
        force.H[i] = .{ 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        force.C[i] = 0.0;
        force.motor[i] = 0.0;
        force.stiffness[i] = FLOAT32_INFINITY;
        force.fmax[i] = FLOAT32_INFINITY;
        force.fmin[i] = -FLOAT32_INFINITY;
        force.fracture[i] = FLOAT32_INFINITY;

        force.penalty[i] = 0.0;
        force.lambda[i] = 0.0;
    }
    //

    return force;
}


force_delete :: (using force: *Force) {
    defer free(force);
    // Remove from solver linked list
     p := *solver.forces;
    while p.* != force {
        p = *((p.*).next);
    }

    p.* = next;

    // Remove from body linked lists
    if bodyA
    {
        p = *(bodyA.forces);
        while p.* != force {
            p = ifx (p.*).bodyA == bodyA then *((p.*).nextA) else *((p.*).nextB);
        }
        p.* = nextA;
    }

    if bodyB
    {
        p = *(bodyB.forces);
        while (p.* != force){
            p = ifx (p.*).bodyA == bodyB then *((p.*).nextA) else *((p.*).nextB);
        }
       p.* = nextB;
    }
    
};


force_disable::(using force: *Force)
{
    // Disable this force by clearing the relavent fields
    for i: 0 .. MAX_ROWS-1
    {
        stiffness[i] = 0;
        penalty[i] = 0;
        lambda[i] = 0;
    }
}


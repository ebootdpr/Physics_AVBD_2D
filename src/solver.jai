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

// Core solver class which holds all the rigid bodies and forces, and has logic to step the simulation forward in time

MAX_ROWS :: 4;                    // Most number of rows an individual constraint can have
PENALTY_MIN :: 1000.0;           // Minimum penalty parameter
PENALTY_MAX :: 1000000000.0;     // Maximum penalty parameter
COLLISION_MARGIN :: 0.01;        // Margin for collision detection to avoid flickering contacts
STICK_THRESH :: 0.01;            // Position threshold for sticking contacts (ie static friction)
SHOW_CONTACTS :: true;           // Whether to show contacts in the debug draw

#load "maths.jai";
#load "impl_solver.jai";
#load "impl_force.jai";
#load "impl_rigid.jai";
#load "impl_manifold.jai";
#load "impl_spring.jai";
#load "impl_joint.jai";
#load "impl_collide.jai";

Solver :: struct {
    dt: float32 = 1.0 / 60.0;           // Timestep
    gravity: float32 = -10.0;      // Gravity
    iterations: s32  = 10;     // Solver iterations

    alpha: float32 = 0.99;        // Stabilization parameter
    beta: float32 = 100000.0;         // Penalty ramping parameter
    gamma: float32 =  0.99;        // Warmstarting decay parameter

    bodies: *Rigid;
    forces: *Force; 

    // Procedure Pointers
    pick : (solver: *Solver, at: Vector2, local: *Vector2) -> *Rigid;
    clear: (solver: *Solver);
    defaultParams: (solver: *Solver);
    step: (solver: *Solver);
    draw: (solver: *Solver);
   
};

// Holds all the state for a single rigid body that is needed by AVBD
Rigid :: struct
{
    solver: *Solver;
    forces: *Force;
    next: *Rigid;
    position: Vector3 ;
    initial: Vector3 ;
    inertial: Vector3 ;
    velocity: Vector3 ;
    prevVelocity: Vector3 ;
    size: Vector2 ;
    mass: float32 ;
    moment: float32 ;
    friction: float32 ;
    radius: float32 ;

    //Procedure Pointers
    constrainedTo : (this: *Rigid, other: *Rigid) -> bool;
    draw : (rigid: *Rigid);
};


// Holds all user defined and derived constraint parameters, and provides a common interface for all forces.
Force :: struct 
{
    solver: *Solver;
    bodyA: *Rigid;
    bodyB: *Rigid;
    nextA: *Force;
    nextB: *Force;
    next: *Force;

    J: [MAX_ROWS]Vector3 ;
    H: [MAX_ROWS]Matrix3 ;
    C: [MAX_ROWS]float32 ;
    fmin: [MAX_ROWS]float32 ;
    fmax: [MAX_ROWS]float32 ;
    stiffness: [MAX_ROWS]float32 ;
    motor: [MAX_ROWS]float32 ;
    fracture: [MAX_ROWS]float32 ;
    penalty: [MAX_ROWS]float64 ;
    lambda: [MAX_ROWS]float32 ;

    discriminate: int = 9;
    disable: PhysicsFPT.Disable_Proc;

    // Procedure Pointers to be replaced by their derived ones. (manifold, joint, spring and ignorecollision)
    rows: PhysicsFPT.Rows_Proc = null;  //virtual
    initialize: PhysicsFPT.Initialize_Proc = null; //virtual
    computeConstraint: PhysicsFPT.Compute_Constraint_Proc = null; //virtual
    computeDerivatives: PhysicsFPT.Compute_Derivatives_Proc = null; //virtual
    draw: PhysicsFPT.Force_Draw_Proc = (p: *Force) {}; //virtual
};

// (FPT) Stands for Function Pointer Types
PhysicsFPT :: struct { 
    Rows_Proc :: #type (p: *Force) -> s32;
    Disable_Proc :: #type (p: *Force);
    Initialize_Proc :: #type (p: *Force) -> bool;
    Compute_Constraint_Proc :: #type (p: *Force, alpha: float32);
    Compute_Derivatives_Proc :: #type (p: *Force, body: *Rigid);
    Force_Draw_Proc :: #type (p: *Force);
    Rigid_Draw_Proc :: #type (p: *Rigid);
}


// Revolute joint + angle constraint between two rigid bodies, with optional motor and fracture
Joint :: struct
{
    #as using base: Force;
    rA, rB: Vector2;
    C0: Vector3;
    torqueArm: float32;
    restAngle: float32;
};


// Standard spring force
Spring :: struct
{
    #as using base: Force;

    rA, rB: Vector2;
    rest: float32;
};

// Force which has no physical effect, but is used to ignore collisions between two bodies
 IgnoreCollision :: struct
{
    #as using base: Force; //inheritance, Joint inherits Force memory layout
};

ignorecollision_instantiate :: (solver: *Solver, bodyA: *Rigid, bodyB: *Rigid) -> *IgnoreCollision {
        igcol:= New(IgnoreCollision);
        force_instantiate(igcol, solver, bodyA, bodyB);
        igcol.discriminate = 4;

        //attach procs:
        igcol.rows = (p: *Force )->s32 {return 0;};
        igcol.initialize = (p: *Force )->bool{return true;};
        igcol.computeConstraint = (p: *Force, alpha: float){};
        igcol.computeDerivatives = (p: *Force, body: *Rigid ){};
        igcol.draw = (p: *Force ){};
        //

        return igcol;
    } ;

// Collision manifold between two rigid bodies, which contains up to two frictional contact points
 Manifold :: struct
{
     #as using base: Force; //inheritance, Joint inherits Force memory layout

    // Used to track contact features between frames
    FeaturePair:: union {
        e:  Edges;
        value: s32;
    };

    Edges :: struct {
         inEdge1: EdgeNumbers; //char
         outEdge1: EdgeNumbers; //char
         inEdge2: EdgeNumbers; //char
         outEdge2: EdgeNumbers; //char
    };
    
    // Contact point information for a single contact
    Contact :: struct
    {
        feature: FeaturePair;
        rA: Vector2;
        rB: Vector2;
        normal: Vector2;

        JAn, JBn, JAt, JBt: Vector3;
        C0: Vector2;
        stick: bool;
    };


     contacts: [2]Contact;
    numContacts: s32;
    friction: float32;

    // Manifold(Solver* solver, Rigid* bodyA, Rigid* bodyB);

    // int rows() const override { return numContacts * 2; }

    // bool initialize() override;
    // void computeConstraint(float alpha) override;
    // void computeDerivatives(Rigid* body) override;
    // void draw() const override;

    // collide: ( bodyA: *Rigid,  bodyB: *Rigid,  contacts: *Contact)-> s32; //static=global encapsulated
};



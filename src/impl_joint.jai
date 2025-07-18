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


joint_instantiate :: ( solver: *Solver,  bodyA: *Rigid,  bodyB: *Rigid,  rA: Vector2,  rB: Vector2,  stiffness: Vector3 = .{FLOAT32_INFINITY,FLOAT32_INFINITY,FLOAT32_INFINITY},  motor: float = 0.0,  fracture: float = FLOAT32_INFINITY) -> *Joint
{
    // : Force(solver, bodyA, bodyB), rA(rA), rB(rB)
    joint := New(Joint);
    force_instantiate(joint, solver, bodyA, bodyB);
    joint.rA = rA;
    joint.rB = rB;
    joint.stiffness[0] = stiffness.x;
    joint.stiffness[1] = stiffness.y;
    joint.stiffness[2] = stiffness.z;
    joint.motor[2] = motor;
    joint.fmax[2] = fracture;
    joint.fmin[2] = -fracture;
    joint.fracture[2] = fracture;

    //attach procs:
    joint.rows =  (p: *Force) -> s32 {return 3;};
    joint.discriminate = 1;
    joint.initialize =  joint_initialize;
    joint.computeConstraint =  joint_computeConstraint;
    joint.computeDerivatives =  joint_computeDerivatives;
    joint.draw =  joint_draw;
    //

    joint.restAngle = (ifx bodyA then bodyA.position.z else 0.0) - bodyB.position.z;
    joint.torqueArm = length_squared((ifx bodyA then bodyA.size else Vector2.{ 0, 0 }) + bodyB.size);
    return joint;
}

joint_initialize :: (joint_f: *Force) -> bool
{

    joint := cast(*Joint)joint_f;
    using joint;
    // Store constraint function at beginnning of timestep C(x-)
    // Note: if bodyA is null, it is assumed that the joint connects a body to the world space position rA
    ConstraintXY := (ifx bodyA then transform(bodyA.position, rA) else rA) - transform(bodyB.position, rB);
    C0.x , C0.y = ConstraintXY.x, ConstraintXY.y;
    C0.z = ((ifx bodyA then bodyA.position.z else 0) - bodyB.position.z - restAngle) * torqueArm;
    return stiffness[0] != 0 || stiffness[1] != 0 || stiffness[2] != 0;
}

joint_computeConstraint :: (joint_f: *Force,  alpha: float)
{
    joint := cast(*Joint)joint_f;
    using joint;
    // Compute constraint function at current state C(x)
    Cn: Vector3;
    ConstraintXY := (ifx bodyA then transform(bodyA.position, rA) else rA) - transform(bodyB.position, rB);
    Cn.x , Cn.y = ConstraintXY.x, ConstraintXY.y;
    Cn.z = ((ifx bodyA then bodyA.position.z else 0) - bodyB.position.z - restAngle) * torqueArm;

    for  i: 0.. joint.rows(joint) - 1
    {
        // Store stabilized constraint function, if a hard constraint (Eq. 18)
        if (isinf(stiffness[i]))
            C[i] = Cn.component[i] - C0.component[i] * alpha;
        else
            C[i] = Cn.component[i];
    }
}

joint_computeDerivatives :: (joint_f: *Force, body: *Rigid)
{
    joint := cast(*Joint)joint_f;
    using joint;
    // Compute the first and second derivatives for the desired body
    if (body == bodyA)
    {
         r := rotate(rA, bodyA.position.z);
        J[0] = .{ 1.0, 0.0, -r.y };
        J[1] = .{ 0.0, 1.0, r.x };
        J[2] = .{ 0.0, 0.0, torqueArm };
        H[0] = .{ 0, 0, 0, 0, 0, 0, 0, 0, -r.x };
        H[1] = .{ 0, 0, 0, 0, 0, 0, 0, 0, -r.y };
        H[2] = .{ 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    }
    else
    {
         r := rotate(rB, bodyB.position.z);
        J[0] = .{ -1.0, 0.0, r.y };
        J[1] = .{ 0.0, -1.0, -r.x };
        J[2] = .{ 0.0, 0.0, -torqueArm };
        H[0] = .{ 0, 0, 0, 0, 0, 0, 0, 0, r.x };
        H[1] = .{ 0, 0, 0, 0, 0, 0, 0, 0, r.y };
        H[2] = .{ 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    }
}


default_joint_draw :: (joint_f: *Force) 
{
    joint := cast(*Joint)joint_f;
    using joint;
    v0 := ifx bodyA then transform(bodyA.position, rA) else rA;
    v1 := transform(bodyB.position, rB);
    
    /*
    glColor3f(0.75, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex2f(v0.x, v0.y);
    glVertex2f(v1.x, v1.y);
    glEnd();
    */
}


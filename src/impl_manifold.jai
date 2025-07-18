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

manifold_instantiate :: (solver: *Solver, bodyA: *Rigid, bodyB: *Rigid) -> *Manifold
{
    mani:= New(Manifold);
    
    // : Force(solver, bodyA, bodyB), numContacts(0)
    force_instantiate(mani, solver, bodyA, bodyB);
    mani.numContacts = 0;
    //

    //attach procs:
    mani.discriminate = 2;
    mani.initialize = manifold_initialize;
    mani.rows = manifold_rows; //from line 181 of solver.h
    mani.computeConstraint = manifold_computeConstraint;
    mani.computeDerivatives =  manifold_computeDerivatives;
    mani.draw = manifold_draw;
    //


    //compute initialization:
    // using mani;
    mani.fmax[0] , mani.fmax[2] = 0.0;
    mani.fmin[0] , mani.fmin[2] = -FLOAT32_INFINITY;
    //

    return mani;
}

manifold_initialize :: ( manifold_f: *Force) -> bool
{   
    manifold :=cast(*Manifold) manifold_f;
    using manifold;
    // Compute friction
    friction = sqrt(bodyA.friction * bodyB.friction);

    // Store previous contact state
     oldContacts: [2]Contact = .[ contacts[0], contacts[1] ];
     oldPenalty: [4]float64 = .[ penalty[0], penalty[1], penalty[2], penalty[3] ];
     oldLambda: [4]float = .[ lambda[0], lambda[1], lambda[2], lambda[3] ];
     oldStick: [2]bool = .[ contacts[0].stick, contacts[1].stick ];
     oldNumContacts: s32 = numContacts;

    // Compute new contacts
    manifold.numContacts = manifold_collide(bodyA, bodyB, contacts);
    // Merge old contact data with new contacts
    for i: 0 .. numContacts-1
    {
        penalty[i * 2 + 0] , penalty[i * 2 + 1] = 0.0;
        lambda[i * 2 + 0] , lambda[i * 2 + 1] = 0.0;

        for j: 0.. oldNumContacts -1
        {
            if (contacts[i].feature.value == oldContacts[j].feature.value)
            {
                penalty[i * 2 + 0] = oldPenalty[j * 2 + 0];
                penalty[i * 2 + 1] = oldPenalty[j * 2 + 1];
                lambda[i * 2 + 0] = oldLambda[j * 2 + 0];
                lambda[i * 2 + 1] = oldLambda[j * 2 + 1];
                contacts[i].stick = oldStick[j];

                // If static friction in last frame, use the old contact points
                if (oldStick[j])
                {
                    contacts[i].rA = oldContacts[j].rA;
                    contacts[i].rB = oldContacts[j].rB;
                }
            }
        }
    }

    for i: 0 .. numContacts-1
    {
        // Compute the contact basis (Eq. 15)
        normal: Vector2 = contacts[i].normal;
        tangent: Vector2 = .{ normal.y, -normal.x };
        basis: Matrix2 = .{
            normal.x, normal.y,
            tangent.x, tangent.y
        };

        rAW: Vector2 = rotate(contacts[i].rA, bodyA.position.z);
        rBW: Vector2 = rotate(contacts[i].rB, bodyB.position.z);

        // Precompute the constraint and derivatives at C(x-), since we use a truncated Taylor series for contacts (Sec 4).
        // Note that we discard the second order term, since it is insignificant for contacts
        contacts[i].JAn = .{ basis.coef[0][0], basis.coef[0][1], cross(rAW, normal) };
        contacts[i].JBn = .{ -basis.coef[0][0], -basis.coef[0][1], -cross(rBW, normal) };
        contacts[i].JAt = .{ basis.coef[1][0], basis.coef[1][1], cross(rAW, tangent) };
        contacts[i].JBt = .{ -basis.coef[1][0], -basis.coef[1][1], -cross(rBW, tangent) };

        contacts[i].C0 = basis * (xy(bodyA.position) + rAW - xy(bodyB.position) - rBW) + Vector2.{ COLLISION_MARGIN, 0 };
    }

    return numContacts > 0;
}



manifold_rows :: ( manifold_f: *Force) -> s32 {    
    manifold :=cast(*Manifold) manifold_f;
    using manifold;
    return numContacts * 2;
}

manifold_computeConstraint :: ( manifold_f: *Force, alpha: float)
{
    manifold :=cast(*Manifold) manifold_f;
    using manifold;
    for i: 0..numContacts -1
    {
        // Compute the Taylor series approximation of the constraint function C(x) (Sec 4)
         dpA : Vector3 = bodyA.position - bodyA.initial;
         dpB : Vector3 = bodyB.position - bodyB.initial;
        
        C[i * 2 + 0] = contacts[i].C0.x * (1 - alpha) + dot(contacts[i].JAn, dpA) + dot(contacts[i].JBn, dpB);
        C[i * 2 + 1] = contacts[i].C0.y * (1 - alpha) + dot(contacts[i].JAt, dpA) + dot(contacts[i].JBt, dpB);

        // Update the friction bounds using the latest lambda values
         frictionBound :float = abs(lambda[i * 2 + 0]) * friction;
        fmax[i * 2 + 1] = frictionBound;
        fmin[i * 2 + 1] = -frictionBound;

        // Check if the contact is sticking, so that on the next frame we can use the old contact points for better static friction handling
        contacts[i].stick = abs(lambda[i * 2 + 1]) < frictionBound && abs(contacts[i].C0.y) < STICK_THRESH;
    }
}

manifold_computeDerivatives :: ( manifold_f: *Force, body: *Rigid)
{
    manifold :=cast(*Manifold) manifold_f;
    using manifold;
    // Just store precomputed derivatives in J for the desired body
    for i: 0..numContacts -1
    {
        if (body == bodyA)
        {
            J[i * 2 + 0] = contacts[i].JAn;
            J[i * 2 + 1] = contacts[i].JAt;
        }
        else
        {
            J[i * 2 + 0] = contacts[i].JBn;
            J[i * 2 + 1] = contacts[i].JBt;
        }
    }
}


default_manifold_draw :: ( manifold_f: *Force) 
{   
    manifold :=cast(*Manifold) manifold_f;
    using manifold;

    if !SHOW_CONTACTS
        return;

    for i: 0..numContacts -1
    {
         v0: Vector2= transform(bodyA.position, contacts[i].rA);
         v1: Vector2= transform(bodyB.position, contacts[i].rB);

    /*        glColor3f(0.75, 0.0, 0.0);
            glBegin(GL_POINTS);
            glVertex2f(v0.x, v0.y);
            glVertex2f(v1.x, v1.y);
            glEnd();
        */
    }
}


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


solver_instantiate :: () -> *Solver {
    //: bodies(0), forces(0)
    solver:= New(Solver);
    solver.bodies = null;
    solver.forces = null;
    //attach procs:
    solver.pick = solver_pick;
    solver.clear = solver_clear;
    solver.defaultParams = solver_defaultParams;
    solver.step = solver_step;
    solver.draw = solver_draw;
    //

    solver_defaultParams(solver);
    return solver;
}


solver_defaultParams :: (using solver: *Solver)
{
    dt = 1.0 / 60.0;
    gravity = -10.0;
    iterations = 10;

    // Note: in the paper, beta is suggested to be [1, 1000]. Technically, the best choice will
    // depend on the length, mass, and constraint function scales (ie units) of your simulation,
    // along with your strategy for incrementing the penalty parameters.
    // If the value is not in the right range, you may see slower convergance for complex scenes.
    beta = 100000.0;

    // Alpha controls how much stabilization is applied. Higher values give slower and smoother
    // error correction, and lower values are more responsive and energetic. Tune this depending
    // on your desired constraint error response.
    alpha = 0.99;

    // Gamma controls how much the penalty and lambda values are decayed each step during warmstarting.
    // This should always be < 1 so that the penalty values can decrease (unless you use a different
    // penalty parameter strategy which does not require decay).
    gamma = 0.99;
}


solver_clear :: (using solver: *Solver) {

        while forces {
            force_delete(forces);
        }

        while bodies {
            rigid_delete (bodies);
        }


}

solver_pick :: (using solver: *Solver, at: Vector2,  local: *Vector2) -> *Rigid
{
    // Find which body is at the given point
    body :=  bodies;
    while body
    {
        defer body = body.next;
        Rt : Matrix2 = rotation(-body.position.z);
        local.* = Rt * (at - .{body.position.x,body.position.y});
        c1 := local.x >= -body.size.x * 0.5 && local.x <= body.size.x * 0.5;
        c2 := local.y >= -body.size.y * 0.5 && local.y <= body.size.y * 0.5;
        if c1 && c2 return body;
       
    }

    return null;
}


solver_draw :: (solver: *Solver)
{   
    body := solver.bodies;
    while body {
        defer body = body.next;
        body.draw(body);
    }


    force := solver.forces;
    while force {
        defer force = force.next;
        force.draw(force);

    }

}



solver_step :: (using solver: *Solver)
{
    // Perform broadphase collision detection
    // This is a naive O(n^2) approach, but it is sufficient for small numbers of bodies in this sample.

    
    bodyA :=  bodies;
    while bodyA != null 
    {
        defer bodyA = bodyA.next;
        bodyB :=  bodyA.next;
        while bodyB != null 
        {
            defer bodyB = bodyB.next;
            dp: Vector2= .{bodyA.position.x, bodyA.position.y} - .{bodyB.position.x, bodyB.position.y};
             r := bodyA.radius + bodyB.radius;
            if (dot(dp, dp) <= r * r && !bodyA.constrainedTo(bodyA, bodyB)) {
                manifold_instantiate(solver, bodyA, bodyB); // Manifold(this, bodyA, bodyB);
            }
           
        }
        
    }

    // Initialize and warmstart forces
    

    force := forces;
    while force != null 
    {
        // Initialization can including caching anything that is constant over the step
        dd := force.discriminate;
        if (!force.initialize(force))
        {
            // Force has returned false meaning it is inactive, so remove it from the solver
            next := force.next;
            force_delete(force);
            force = next;
        }
        else
        {
            for i: 0..force.rows(force) - 1
            {
                // Warmstart the dual variables and penalty parameters (Eq. 19)
                // Penalty is safely clamped to a minimum and maximum value
                force.lambda[i] = force.lambda[i] * alpha * gamma;
                force.penalty[i] = clamp(force.penalty[i] * gamma, PENALTY_MIN, PENALTY_MAX);

                // If it's not a hard constraint, we don't let the penalty exceed the material stiffness
                force.penalty[i] = min(force.penalty[i], force.stiffness[i]);
            }

            force = force.next;
        }
    }
    
    
    
    #if 1 {
    // Initialize and warmstart bodies (ie primal variables)
    body := solver.bodies;
    while body {
        defer body = body.next;
        // Don't let bodies rotate too fast
        body.velocity.z = clamp(body.velocity.z, -10.0, 10.0);

        // Compute inertial position (Eq 2)
        body.inertial = body.position + body.velocity * dt;
        if (body.mass > 0)
            body.inertial += Vector3.{ 0, gravity, 0 } * (dt * dt);

        // Adaptive warmstart (See original VBD paper)
        accel: Vector3 = (body.velocity - body.prevVelocity) / dt;
         accelExt: float = accel.y * sign(gravity);
         accelWeight: float = clamp(accelExt / abs(gravity), 0.0, 1.0);
        if (!is_finite(accelWeight)) accelWeight = 0.0;

        // Save initial position (x-) and compute warmstarted position (See original VBD paper)
        body.initial = body.position;
        body.position = body.position + body.velocity * dt + Vector3.{ 0, gravity, 0 } * (accelWeight * dt * dt);
    }
    }
    #if 1 {
    // Main solver loop
    for it: 0..iterations - 1
    {
        // Primal update
        body := solver.bodies;
        while body {
            defer body = body.next;
            // Skip static / kinematic bodies
            if (body.mass <= 0)
                continue;

            // Initialize left and right hand sides of the linear system (Eqs. 5, 6)
            M: Matrix3 = diagonal(body.mass, body.mass, body.moment);
            lhs: Matrix3 = M * (1/ (dt * dt));
            rhs: Vector3 = M * (1/ (dt * dt)) * (body.position - body.inertial);

            // Iterate over all forces acting on the body
            force := body.forces;
            while force {
                defer force =  ifx (force.bodyA == body) then force.nextA else force.nextB;
                // Compute constraint and its derivatives
                force.computeConstraint(force, alpha);
                force.computeDerivatives(force, body);

                for i: 0..force.rows(force) - 1
                {
                    // Use lambda as 0 if it's not a hard constraint
                     lambda: float = ifx isinf(force.stiffness[i]) then force.lambda[i] else 0.0;

                    // Compute the clamped force magnitude (Sec 3.2)
                     f: float64 = clamp(force.penalty[i] * force.C[i] + lambda + force.motor[i], force.fmin[i], force.fmax[i]);

                    // Compute the diagonally lumped geometric stiffness term (Sec 3.5)
                    G: Matrix3 = diagonal(length(column(force.H[i], 0)), length(column(force.H[i], 1)), length(column(force.H[i], 2))) * abs(cast(float32)f);

                    // Accumulate force (Eq. 13) and hessian (Eq. 17)
                    rhs += force.J[i] * f;
                    lhs += outer(force.J[i], force.J[i] * force.penalty[i]) + G;
                }
            }

            // Solve the SPD linear system using LDL and apply the update (Eq. 4)
            body.position -= solve(lhs, rhs);
            // a := solve(lhs, rhs);
            // if a != .{}
            // log("solve(lhs, rhs); %", a);
        }

        // Dual update
        force := solver.forces;
        while force {
            defer force = force.next;

            // Compute constraint
            force.computeConstraint(force,alpha);

            for i: 0..force.rows(force) - 1 
            {
                // Use lambda as 0 if it's not a hard constraint
                lambda: float = ifx isinf(force.stiffness[i]) then force.lambda[i] else 0.0;

                // Update lambda (Eq 11)
                // Note that we don't include non-conservative forces (ie motors) in the lambda update, as they are not part of the dual problem.
                force.lambda[i] = cast(float32)clamp(force.penalty[i] * force.C[i] + lambda, force.fmin[i], force.fmax[i]);

                // Disable the force if it has exceeded its fracture threshold
                if (abs(force.lambda[i]) >= force.fracture[i])
                    force.disable(force);

                // Update the penalty parameter and clamp to material stiffness if we are within the force bounds (Eq. 16)
                if (force.lambda[i] > force.fmin[i] && force.lambda[i] < force.fmax[i])
                    force.penalty[i] = min(force.penalty[i] + beta * abs(force.C[i]), min(PENALTY_MAX, force.stiffness[i]));
            }
        }
    }

    // Compute velocities (BDF1)
    {
    body := solver.bodies;
    while body {
        defer body = body.next;
        body.prevVelocity = body.velocity;
        if (body.mass > 0)
            body.velocity = (body.position - body.initial) / dt;
    }
    }
    }
    
}

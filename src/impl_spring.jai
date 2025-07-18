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

spring_instantiate :: (solver: *Solver, bodyA: *Rigid, bodyB:  *Rigid,  rA: Vector2,  rB: Vector2,  stiffness: float,  rest: float = -1) -> *Spring
{
    spring := New(Spring);
    
    // : Force(solver, bodyA, bodyB), rA(rA), rB(rB), rest(rest)
    force_instantiate(spring, solver, bodyA, bodyB);
    spring.rest = rest;
    spring.rA = rA;
    spring.rB = rB;
    //

    //attach procs:
    spring.rows = spring_rows;
    spring.discriminate = 3;
    spring.initialize = spring_initialize;
    spring.computeConstraint = spring_computeConstraint;
    spring.computeDerivatives = spring_computeDerivatives;
    spring.draw = spring_draw;
    //

    //compute initialization:
    spring.stiffness[0] = stiffness;
    if (spring.rest < 0)
        spring.rest = length(transform(bodyA.position, rA) - transform(bodyB.position, rB));
    //

    return spring;
}

spring_rows :: (p: *Force) -> s32 {return 1;};
spring_initialize :: (p: *Force) -> bool {return true;};

spring_computeConstraint :: (spring_f: *Force,  alpha: float)
{
    spring :=cast(*Spring) spring_f;
    using spring;
    // Compute constraint function at current state C(x)
    C[0] = length(transform(bodyA.position, rA) - transform(bodyB.position, rB)) - rest;
}


default_spring_draw :: (spring_f: *Force) 
{
    spring :=cast(*Spring) spring_f;
    using spring;
     v0 : Vector2= transform(bodyA.position, rA);
     v1 : Vector2= transform(bodyB.position, rB);

/*    glColor3f(0.75, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex2f(v0.x, v0.y);
    glVertex2f(v1.x, v1.y);
    glEnd();
    */
}



spring_computeDerivatives :: (spring_f: *Force, body: *Rigid)
{
    spring :=cast(*Spring) spring_f;
    using spring;
    // Compute the first and second derivatives for the desired body
     S: Matrix2 = .{ 0, -1, 1, 0 };
     I: Matrix2 = .{ 1, 0, 0, 1 };

     d: Vector2 = transform(bodyA.position, rA) - transform(bodyB.position, rB);
     dlen2: float = dot(d, d);
    if (dlen2 == 0)
        return;
     dlen: float = sqrt(dlen2);
     n: Vector2 = d / dlen;
     k_dlen := 1/dlen;
     k_dlen2 := 1/dlen2;
     dxx: Matrix2 = (I - outer(n, n) * k_dlen2) * k_dlen;

    if (body == bodyA)
    {
         Sr: Vector2 = rotate( S * rA, bodyA.position.z);
         r: Vector2 = rotate( rA, bodyA.position.z);
         dxr: Vector2 = dxx * Sr;
         drr: float = dot(Sr, dxr) - dot(n, r);

        J[0].x = n.x;
        J[0].y = n.y;
        J[0].z = dot(n, Sr);
        H[0] = .{
            dxx.v[0].x, dxx.v[0].y, dxr.x,
            dxx.v[1].x, dxx.v[1].y, dxr.y,
            dxr.x,          dxr.y,        drr
        };
    }
    else
    {
         Sr: Vector2 = rotate( S * rB, bodyB.position.z);
         r: Vector2 = rotate( rB, bodyB.position.z);
         dxr: Vector2 = dxx * -Sr;
         drr: float = dot(Sr, dxr) + dot(n, r);

        J[0].x = -n.x;
        J[0].y = -n.y;
        J[0].z = dot(n, -Sr);
        H[0] = .{
            dxx.v[0].x, dxx.v[0].y, dxr.x,
            dxx.v[1].x, dxx.v[1].y, dxr.y,
            dxr.x,          dxr.y,        drr
        };
    }
}



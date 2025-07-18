
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

Scene_Proc :: #type (solver: *Solver);
sceneEmpty :: (solver: *Solver)
{
    solver.clear(solver);
}



sceneGround :: (solver: *Solver)
{
    solver.clear(solver);
    rigid_instantiate(solver, .{ 100, 1 }, 0.0, 0.5, .{ 0, 0, 0 }, .{ 0, 0, 0 });
}



sceneDynamicFriction :: (solver: *Solver)
{
    solver.clear(solver);
    rigid_instantiate(solver, .{ 100, 1 }, 0.0, 0.5, .{ 0, 0, 0 }, .{ 0, 0, 0 });
    for  x: 0 .. 10
        rigid_instantiate(solver, .{ 1, 0.5 }, 1.0, 5.0 - (x / 10.0 * 5.0), .{ -30.0 + x * 2.0, 0.75, 0 }, .{ 10.0, 0, 0 });
}

sceneStaticFriction :: (solver: *Solver)
{
    solver.clear(solver);
    rigid_instantiate(solver, .{ 100, 1 }, 0.0, 1.0, .{ 0, 0, 3.14159 / 6.0 });
    for  y: 0 .. 10
        rigid_instantiate(solver, .{ 5, 0.5 }, 1.0, 1.0, .{ 0.0, y * 1.0 + 1.0, 3.14159 / 6.0 });
}

scenePyramid :: (solver: *Solver)
{
    SIZE: s32 : 20;
    solver.clear(solver);
    rigid_instantiate(solver, .{ 100, 0.5 }, 0.0, 0.5, .{ 0.0, -2.0, 0.0 });
    for y: 0 .. SIZE -1
        for x: 0 .. SIZE - y - 1
            rigid_instantiate(solver, .{ 1, 0.5 }, 1.0, 0.5, .{ x * 1.1 + y * 0.5 - SIZE / 2.0, y * 0.85, 0.0 });
}

sceneRope :: (solver: *Solver)
{
    solver.clear(solver);
    prev: *Rigid = null;
    for  i: 0.. 20 - 1
    {
        curr: *Rigid = rigid_instantiate(solver, .{ 1, 0.5 }, ifx  i == 0 then 0.0 else 1.0, 0.5, .{ cast(float32)i, 10.0, 0.0 });
        if (prev)
            joint_instantiate(solver, prev, curr, .{ 0.5, 0 }, .{ -0.5, 0 }, .{FLOAT32_INFINITY,FLOAT32_INFINITY, 0.0 });
        prev = curr;
    }
}

sceneHeavyRope :: (solver: *Solver)
{
    N: s32: 20;
    SIZE: float32: 30;
    solver.clear(solver);
    prev: *Rigid = null;
    for  i: 0..N - 1
    {
        curr: *Rigid = rigid_instantiate(solver, ifx i == N - 1 then Vector2.{ SIZE, SIZE } else Vector2.{ 1, 0.5 }, ifx i == 0 then 0.0 else 1.0, 0.5, .{ cast(float32)i + (ifx i == N - 1 then SIZE / 2 else 0), 10.0, 0.0});
        if (prev)
            joint_instantiate(solver, prev, curr, .{ 0.5, 0 },ifx i == N - 1 then Vector2.{ -SIZE / 2, 0 } else Vector2.{ -0.5, 0 }, .{FLOAT32_INFINITY,FLOAT32_INFINITY, 0.0 });
        prev = curr;
    }
}

sceneHangingRope :: (solver: *Solver)
{
    N: s32: 50;
    SIZE: float32: 10;
    solver.clear(solver);
    prev: *Rigid = null;
    
    for  i: 0..N - 1
    {
        curr: *Rigid = rigid_instantiate(solver,ifx i == N - 1 then Vector2.{ SIZE, SIZE } else Vector2.{ 0.5, 1.0 },ifx i == 0 then 0.0 else 1.0, 0.5, .{ 0.0, 10.0 - (cast(float32)i + (ifx i == N - 1 then SIZE / 2 else 0)), 0.0 });
        if (prev)
            joint_instantiate(solver, prev, curr, .{ 0, -0.5 },ifx i == N - 1 then Vector2.{ 0, SIZE / 2} else Vector2.{ 0, 0.5 }, .{FLOAT32_INFINITY,FLOAT32_INFINITY, 0.0 });
        prev = curr;
    }
}

sceneSpring :: (solver: *Solver)
{
    solver.clear(solver);
    anchor: *Rigid = rigid_instantiate(solver, .{ 1, 1 }, 0.0, 0.5, .{ 0.0, 0.0, 0.0 });
    block: *Rigid = rigid_instantiate(solver, .{ 4, 4 }, 1.0, 0.5, .{ 0.0, -8.0, 0.0 });
    spring_instantiate(solver, anchor, block, .{ 0, 0 }, .{ 0, 0 }, 100.0, 4.0);
}

sceneSpringsRatio :: (solver: *Solver)
{
    N: s32: 8;
    solver.clear(solver);
    prev: *Rigid = null;
    
    for  i: 0..N - 1
    {
        curr: *Rigid = rigid_instantiate(solver, .{ 1, 0.5 }, ifx (i == 0 ||  i == N - 1) then 0.0 else 1.0, 0.5, .{ cast(float32)i * 4, 10.0, 0.0 });
        if (prev)
            spring_instantiate(solver, prev, curr, .{ 0.5, 0 }, .{ -0.5, 0 }, ifx i % 2 == 0 then 1000.0 else 1000000.0, 0.1);
        prev = curr;
    }
}

sceneStack :: (solver: *Solver)
{
    solver.clear(solver);
    rigid_instantiate(solver, .{ 100, 1 }, 0.0, 0.5, .{ 0, 0, 0 });
    for i: 0..20 - 1
        rigid_instantiate(solver, .{ 1, 1 }, 1.0, 0.5, .{ 0, i * 2.0 + 1.0, 0 });
}

sceneStackRatio :: (solver: *Solver)
{
    solver.clear(solver);
    rigid_instantiate(solver, .{ 100, 1 }, 0.0, 0.5, .{ 0, 0, 0 });
    y := 1;
    s := 1;
    for i: 0..6 - 1
    {
        rigid_instantiate(solver, .{ cast(float32)s, cast(float32)s }, 1.0, 0.5, .{ 0, cast(float32)y, 0 });
        y += s * 3 / 2;
        s *= 2;
    }
}

sceneRod :: (solver: *Solver)
{
    solver.clear(solver);
    prev: *Rigid = null;
    for i: 0..20 - 1
    {
        curr: *Rigid = rigid_instantiate(solver, .{ 1, 0.5 }, ifx i == 0 then 0.0 else 1.0, 0.5, .{ cast(float32)i, 10.0, 0.0 });
        if (prev)
            joint_instantiate(solver, prev, curr, .{ 0.5, 0 }, .{ -0.5, 0 }, .{FLOAT32_INFINITY,FLOAT32_INFINITY,FLOAT32_INFINITY });
        prev = curr;
    }
}



sceneSoftBody :: (solver: *Solver)
{
    solver.clear(solver);
    //we add ground, density 0 means fixed on spacetime.
    rigid_instantiate(solver, .{ 100, 0.5 }, 0.0, 0.5, .{ 0.0, 0.0, 0.0 });

    //jiggle physics constants
    Klin := 1000.0;
    Kang := 100.0;
    
    //cell dimensions
    W: int : 15; 
    H: int : 5;
    N : s32 : 2;
    
    for  i: 0..N - 1
    {
        grid: [W][H]*Rigid;
        for x: 0..W-1
            for y: 0..H-1
                grid[x][y] = rigid_instantiate(solver, .{ 1, 1 }, 1.0, 0.5, .{ cast(float32)x, cast(float32)y + H * i * 2.0 + 5.0, 0.0 });

        for x: 1..W-1
            for y: 0..H-1
                joint_instantiate(solver, grid[x - 1][y], grid[x][y], .{ 0.5, 0 }, .{ -0.5, 0 }, .{ Klin, Klin, Kang });

        for x: 0..W-1
            for y: 1..H-1
                joint_instantiate(solver, grid[x][y - 1], grid[x][y], .{ 0, 0.5 }, .{ 0, -0.5 }, .{ Klin, Klin, Kang });

        for x: 1..W-1
        {
            for y: 1..H-1
            {   
                //the reason why I decided that, the way to instantiate IngoreCollision is different that the others instantiate, is because there is no reason.
                // IgnoreCollision.ignorecollision_instantiate(solver)(solver, grid[x - 1][y - 1], grid[x][y]);
                ignorecollision_instantiate(solver, grid[x][y - 1], grid[x - 1][y]);
            }
        }
    }
}



 sceneJointGrid :: (solver: *Solver)
{
    solver.clear(solver);
        //I reducd the numbers W and H because my laptop is slow.
    // W :: 50;
    // H :: 50;
    W :: 10;
    H :: 25;
    N :: 2;

    grid: [W][H]*Rigid;
    for x: 0..W-1
        for y: 0..H-1
            grid[x][y] = rigid_instantiate(solver, .{ 1, 1 }, ifx y == H - 1 && (x == 0 || x == W - 1) then 0.0 else 1.0, 0.5, .{ cast(float32)x, cast(float32)y, 0.0 });

    for x: 1..W-1
        for y: 0..H-1
            joint_instantiate(solver, grid[x - 1][y], grid[x][y], .{ 0.5, 0 }, .{ -0.5, 0 });

    for x: 0..W-1
        for y: 1..H-1
            joint_instantiate(solver, grid[x][y - 1], grid[x][y], .{ 0, 0.5 }, .{ 0, -0.5 });

    for x: 1..W-1
    {
        for y: 1..H-1
        {
            ignorecollision_instantiate(solver, grid[x - 1][y - 1], grid[x][y]);
            ignorecollision_instantiate(solver, grid[x][y - 1], grid[x - 1][y]);
        }
    }
}

 sceneNet :: (solver: *Solver)
{
    N : s32 : 40;
    
    solver.clear(solver);
    floor_anlge:= .2; //in radians
    rigid_instantiate(solver, .{ 100, 0.5 }, 0.0, 0.5, .{ 0.0, 0.0, floor_anlge});

    prev: *Rigid = null;
    
    for  i: 0..N - 1
    {
        curr: *Rigid = rigid_instantiate(solver, .{ 1, 0.5 },ifx i == 0 || i == N - 1 then 0.0 else 1.0, 0.5, .{ cast(float32)i - N / 2.0, 10.0, -floor_anlge });
        if (prev)
            joint_instantiate(solver, prev, curr, .{ 0.5, 0 }, .{ -0.5, 0 }, .{FLOAT32_INFINITY,FLOAT32_INFINITY, 0.0 });
        prev = curr;
    }

    //number of  stacked garbagios, for num_columns {for num_rows}
    for x: 0..N-1
        for y: 0.. N / 8
            rigid_instantiate(solver, .{ 1, 1 }, 1.0, 0.5, .{ cast(float32)x - N / 8.0, cast(float32)y + 15.0, 0.0 });
}

 sceneMotor :: (solver: *Solver)
{
    solver.clear(solver);
    floor_anlge:= 0.0; //in radians
    rigid_instantiate(solver, .{ 100, 0.5 }, 0.0, 0.5, .{ 0.0, -10.0, floor_anlge });

    a0 := rigid_instantiate(solver, .{ 5, 0.5 }, 1.0, 0.5, .{ 0.0, 0.0, 0.0 });
    a1 := rigid_instantiate(solver, .{ 5, 0.5 }, 1.0, 0.5, .{ 0.0, 0.0, 3.14159 / 2.0 });
    joint_instantiate(solver, a0, a1, .{ 0, 0 }, .{ 0, 0 }, .{FLOAT32_INFINITY,FLOAT32_INFINITY,FLOAT32_INFINITY });
    joint_instantiate(solver, null, a0, .{ 0, 0 }, .{ 0, 0 }, .{FLOAT32_INFINITY,FLOAT32_INFINITY, 0.0 }, -25.0);
}

 sceneFracture :: (solver: *Solver)
{
    N : s32 : 10;
    M :: 10;

    solver.clear(solver);
    floor_anlge:= 0.0; //in radians
    rigid_instantiate(solver, .{ 100, 0.5 }, 0.0, 0.5, .{ 0.0, 0.0, floor_anlge });

    prev: *Rigid = null;
    for i: 0..N //Inclusive!
    {
        curr: *Rigid = rigid_instantiate(solver, .{ 1, 0.5 }, 1.0, 0.5, .{ cast(float32)i - N / 2.0, 6.0, 0.0 });
        if (prev)
            joint_instantiate(solver, prev, curr, .{ 0.5, 0 }, .{ -0.5, 0 }, .{FLOAT32_INFINITY,FLOAT32_INFINITY,FLOAT32_INFINITY }, 0.0, 500.0);
        prev = curr;
    }

    rigid_instantiate(solver, .{ 1, 5 }, 1.0, 0.5, .{ -N / 2.0, 2.5, 0 });
    rigid_instantiate(solver, .{ 1, 5 }, 1.0, 0.5, .{ N / 2.0, 2.5, 0 });

    for i: 0..M -1
        rigid_instantiate(solver, .{ 2, 1 }, 1.0, 0.5, .{ 0, i * 2.0 + 8.0, 0 });
}




aacpp :: #string CPP


#pragma once

#include "maths.h"
#include "solver.h"


static void sceneSoftBody(Solver* solver)
{
    solver.clear(solver);
    rigid_instantiate(solver, .{ 100, 0.5 }, 0.0, 0.5, .{ 0.0, 0.0 });

    const float Klin = 1000.0;
    const float Kang = 100.0;
    const int W = 15, H = 5;
    N : s32 : 2;
    
    for  i: 0..N - 1
    {
        Rigid* grid[W][H];
        for x: 0..W-1
            for y: 0..H-1
                grid[x][y] = rigid_instantiate(solver, .{ 1, 1 }, 1.0, 0.5, .{ cast(float32)x, cast(float32)y + H * i * 2.0 + 5.0, 0.0 });

        for x: 1..W-1
            for y: 0..H-1
                joint_instantiate(solver, grid[x - 1][y], grid[x][y], .{ 0.5, 0 }, .{ -0.5, 0 }, .{ Klin, Klin, Kang });

        for x: 0..W-1
            for y: 1..H-1
                joint_instantiate(solver, grid[x][y - 1], grid[x][y], .{ 0, 0.5 }, .{ 0, -0.5 }, .{ Klin, Klin, Kang });

        for x: 1..W-1
        {
            for y: 1..H-1
            {
                new IgnoreCollision(solver, grid[x - 1][y - 1], grid[x][y]);
                new IgnoreCollision(solver, grid[x][y - 1], grid[x - 1][y]);
            }
        }
    }
}

static void sceneJointGrid(Solver* solver)
{
    solver.clear(solver);

    const int W = 50, H = 50;
    N : s32 : 2;

    Rigid* grid[W][H];
    for (int x = 0; x < W; x++)
        for (int y = 0; y < H; y++)
            grid[x][y] = rigid_instantiate(solver, .{ 1, 1 }, y == H - 1 && ifx (x == 0 || x == W - 1) then 0.0 else 1.0, 0.5, .{ cast(float32)x, cast(float32)y, 0.0 });

    for (int x = 1; x < W; x++)
        for (int y = 0; y < H; y++)
            joint_instantiate(solver, grid[x - 1][y], grid[x][y], .{ 0.5, 0 }, .{ -0.5, 0 });

    for (int x = 0; x < W; x++)
        for (int y = 1; y < H; y++)
            joint_instantiate(solver, grid[x][y - 1], grid[x][y], .{ 0, 0.5 }, .{ 0, -0.5 });

    for (int x = 1; x < W; x++)
    {
        for (int y = 1; y < H; y++)
        {
            new IgnoreCollision(solver, grid[x - 1][y - 1], grid[x][y]);
            new IgnoreCollision(solver, grid[x][y - 1], grid[x - 1][y]);
        }
    }
}

static void sceneNet(Solver* solver)
{
    N : s32 : 40;

    solver.clear(solver);
    rigid_instantiate(solver, .{ 100, 0.5 }, 0.0, 0.5, .{ 0.0, 0.0 });

    prev: *Rigid = null;
    
    for  i: 0..N - 1
    {
        curr: *Rigid = rigid_instantiate(solver, .{ 1, 0.5 },ifx i == 0 || i == N - 1 then 0.0 else 1.0, 0.5, .{ cast(float32)i - N / 2.0, 10.0, 0.0 });
        if (prev)
            joint_instantiate(solver, prev, curr, .{ 0.5, 0 }, .{ -0.5, 0 }, .{FLOAT32_INFINITY,FLOAT32_INFINITY, 0.0 });
        prev = curr;
    }

    for (int x = 0; x < N / 4; x++)
        for (int y = 0; y < N / 8; y++)
            rigid_instantiate(solver, .{ 1, 1 }, 1.0, 0.5, .{ cast(float32)x - N / 8.0, cast(float32)y + 15.0, 0.0 });
}

static void sceneMotor(Solver* solver)
{
    solver.clear(solver);
    rigid_instantiate(solver, .{ 100, 0.5 }, 0.0, 0.5, .{ 0.0, -10.0 });

    Rigid* a0 = rigid_instantiate(solver, .{ 5, 0.5 }, 1.0, 0.5, .{ 0.0, 0.0, 0.0 });
    Rigid* a1 = rigid_instantiate(solver, .{ 5, 0.5 }, 1.0, 0.5, .{ 0.0, 0.0, 3.14159f / 2.0 });
    joint_instantiate(solver, a0, a1, .{ 0, 0 }, .{ 0, 0 }, .{FLOAT32_INFINITY,FLOAT32_INFINITY,FLOAT32_INFINITY });
    joint_instantiate(solver, 0, a0, .{ 0, 0 }, .{ 0, 0 }, .{FLOAT32_INFINITY,FLOAT32_INFINITY, 0.0 }, -25.0);
}

static void sceneFracture(Solver* solver)
{
    N : s32 : 10;
    const int M = 10;

    solver.clear(solver);
    rigid_instantiate(solver, .{ 100, 0.5 }, 0.0, 0.5, .{ 0.0, 0.0 });

    prev: *Rigid = null;
    for (int i = 0; i <= N; i++)
    {
        curr: *Rigid = rigid_instantiate(solver, .{ 1, 0.5 }, 1.0, 0.5, .{ cast(float32)i - N / 2.0, 6.0, 0.0 });
        if (prev)
            joint_instantiate(solver, prev, curr, .{ 0.5, 0 }, .{ -0.5, 0 }, .{FLOAT32_INFINITY,FLOAT32_INFINITY,FLOAT32_INFINITY }, 0.0, 500.0);
        prev = curr;
    }

    rigid_instantiate(solver, .{ 1, 5 }, 1.0, 0.5, .{ -N / 2.0, 2.5, 0 });
    rigid_instantiate(solver, .{ 1, 5 }, 1.0, 0.5, .{ N / 2.0, 2.5, 0 });

    for (int i = 0; i < M; i++)
        rigid_instantiate(solver, .{ 2, 1 }, 1.0, 0.5, .{ 0, i * 2.0 + 8.0, 0 });
}



CPP
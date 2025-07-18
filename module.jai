#load "src/solver.jai";

//you must replace these in your program if you want to draw the physics
rigid_draw: PhysicsFPT.Rigid_Draw_Proc = default_rigid_draw;
spring_draw: PhysicsFPT.Force_Draw_Proc = default_spring_draw;
manifold_draw: PhysicsFPT.Force_Draw_Proc = default_manifold_draw;
joint_draw: PhysicsFPT.Force_Draw_Proc = default_joint_draw;

#scope_module

#import "Math";
#import "Basic";

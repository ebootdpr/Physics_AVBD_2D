
repalce_physics2d_avbd_drawing_procs :: (solver: *Solver) {
    assert(solver != null,"You need to instantiate solver using solver_instantiate()");
    // These are gloval variables from the physics module:
    //   rigid_draw: PhysicsFPT.Rigid_Draw_Proc = default_rigid_draw;
    //   spring_draw: PhysicsFPT.Force_Draw_Proc = default_spring_draw;
    //   manifold_draw: PhysicsFPT.Force_Draw_Proc = default_manifold_draw;
    //   joint_draw: PhysicsFPT.Force_Draw_Proc = default_joint_draw;
    // we replace them with the ones on this file:

    //this is not type checked, this is procedure name checked, be careful human.
    rigid_draw = My_rigid_draw;
    spring_draw = My_spring_draw;
    manifold_draw = My_manifold_draw;
    joint_draw = My_joint_draw;

}

//
// xD: I dont even care anymore. Im drawing with imgui.
//
xD_world_to_screen :: (p: Vector2) -> Vector2 {
    using context.example_ctx;
    xD_io := ImGui.GetIO();
    fb_width := (xD_io.DisplaySize.x * xD_io.DisplayFramebufferScale.x);
    fb_height := (xD_io.DisplaySize.y * xD_io.DisplayFramebufferScale.y);

    half := Vector2.{ fb_width * 0.5, fb_height * 0.5 };

    screen_p := (p - camPos) * camZoom;
    return Vector2.{ half.x + screen_p.x, half.y - screen_p.y }; // Y flipped
}



My_rigid_draw :: (using rigid: *Rigid)
{
    
    R := rotation(position.z);
    v0 : Vector2= R * Vector2.{ -size.x * 0.5, -size.y * 0.5 } + .{position.x, position.y};
    v1 : Vector2= R * Vector2.{ size.x * 0.5, -size.y * 0.5 } + .{position.x, position.y};
    v2 : Vector2= R * Vector2.{ size.x * 0.5, size.y * 0.5 } + .{position.x, position.y};
    v3 : Vector2= R * Vector2.{ -size.x * 0.5, size.y * 0.5 } + .{position.x, position.y};

    //
    // xD: Hack to get the drawings in the right position on the screen
    //
    v0 = xD_world_to_screen(v0);
    v1 = xD_world_to_screen(v1);
    v2 = xD_world_to_screen(v2);
    v3 = xD_world_to_screen(v3);


    //this doesnt work for me:
    // glColor3f(0.6, 0.6, 0.6);
    // glBegin(GL_QUADS);
    // glVertex2f(v0.x, v0.y);
    // glVertex2f(v1.x, v1.y);
    // glVertex2f(v2.x, v2.y);
    // glVertex2f(v3.x, v3.y);
    // glEnd();

    // glColor3f(0, 0, 0);
    // glBegin(GL_LINE_LOOP);
    // glVertex2f(v0.x, v0.y);
    // glVertex2f(v1.x, v1.y);
    // glVertex2f(v2.x, v2.y);
    // glVertex2f(v3.x, v3.y);
    // glEnd();

    //
    // xD: Here we use Imgui to draw the rigids
    //
    draw_list := ImGui.GetBackgroundDrawList();

    //strategy, we draw 2 quads, and we activate oultine on one of them, wait, we can just use 1 quad and activate outline... //no we cannot, idk how to use imgui.
    draw_list.AddQuadFilled(draw_list,
        Vector2.{v0.x, v0.y},
        Vector2.{v1.x, v1.y},
        Vector2.{v2.x, v2.y},
        Vector2.{v3.x, v3.y},
        ImGui.GetColorU32(Vector4.{0.6, 0.6, 0.6, 1.0})
    );

    // Outline
    draw_list.AddQuad(draw_list,
        Vector2.{v0.x, v0.y},
        Vector2.{v1.x, v1.y},
        Vector2.{v2.x, v2.y},
        Vector2.{v3.x, v3.y},
        ImGui.GetColorU32(Vector4.{0.0, 0.0, 0.0, 1.0}),
        1.0 // thickness
    );
    
}



My_spring_draw :: (spring_f: *Force) 
{
    spring :=cast(*Spring) spring_f;
    using spring;
     v0 : Vector2= transform(bodyA.position, rA);
     v1 : Vector2= transform(bodyB.position, rB);

    /*
    //this doesnt work for me:
    glColor3f(0.75, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex2f(v0.x, v0.y);
    glVertex2f(v1.x, v1.y);
    glEnd();
    */
    

    // Convert world coordinates to screen coordinates
    screen_v0 := xD_world_to_screen(v0);
    screen_v1 := xD_world_to_screen(v1);
    
    // Draw line using ImGui
    draw_list := ImGui.GetBackgroundDrawList();
    draw_list.AddLine(draw_list,
        Vector2.{screen_v0.x, screen_v0.y},
        Vector2.{screen_v1.x, screen_v1.y},
        ImGui.GetColorU32(Vector4.{0.75, 0.0, 0.0, 1.0}), // Red color
        2.0 // thickness
    );
}


My_manifold_draw :: ( manifold_f: *Force) 
{   
    manifold :=cast(*Manifold) manifold_f;
    using manifold;

    if !SHOW_CONTACTS
        return;

    draw_list := ImGui.GetBackgroundDrawList();

    for i: 0..numContacts -1
    {
         v0: Vector2= transform(bodyA.position, contacts[i].rA);
         v1: Vector2= transform(bodyB.position, contacts[i].rB);

        /* 
        //this doesnt work for me:
        glColor3f(0.75, 0.0, 0.0);
        glBegin(GL_POINTS);
        glVertex2f(v0.x, v0.y);
        glVertex2f(v1.x, v1.y);
        glEnd();
        */

        
        // Convert world coordinates to screen coordinates
        screen_v0 := xD_world_to_screen(v0);
        screen_v1 := xD_world_to_screen(v1);
        
        // Draw contact points as small circles
        draw_list.AddCircleFilled(draw_list,
            Vector2.{screen_v0.x, screen_v0.y},
            3.0, // radius
            ImGui.GetColorU32(Vector4.{0.75, 0.0, 0.0, 1.0}) // Red color
        );
        
        draw_list.AddCircleFilled(draw_list,
            Vector2.{screen_v1.x, screen_v1.y},
            3.0, // radius
            ImGui.GetColorU32(Vector4.{0.75, 0.0, 0.0, 1.0}) // Red color
        );

    }
}

My_joint_draw :: (joint_f: *Force) 
{
    joint := cast(*Joint)joint_f;
    using joint;
    v0 := ifx bodyA then transform(bodyA.position, rA) else rA;
    v1 := transform(bodyB.position, rB);
    
    /*
    //this doesnt work for me:
    glColor3f(0.75, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex2f(v0.x, v0.y);
    glVertex2f(v1.x, v1.y);
    glEnd();
    */
    

    // Convert world coordinates to screen coordinates
    screen_v0 := xD_world_to_screen(v0);
    screen_v1 := xD_world_to_screen(v1);
    
    // Draw line using ImGui
    draw_list := ImGui.GetBackgroundDrawList();
    draw_list.AddLine(draw_list,
        Vector2.{screen_v0.x, screen_v0.y},
        Vector2.{screen_v1.x, screen_v1.y},
        ImGui.GetColorU32(Vector4.{0.75, 0.0, 0.0, 1.0}), // Red color
        2.0 // thickness
    );
}

SHOULD_ENABLE_MEMORY_DEBUGGER_FOR_CODEX_VIEWER :: false;

#load "imgui_sdl_gl.jai";

#load "scenes_awareness.jai";
#load "scenes_impl.jai"; //if you create a new scene here, you must add it to the scene_awareness.jai lists of names and procedure names in order.
#import,dir "../../../../Physics_AVBD_2D";
#load "physics_custom_draw_procs.jai";
//#import "Debug";


main :: () {

    SDL_Init(SDL_INIT_VIDEO);

    window := SDL_CreateWindow("A-VBD Physics Solution", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1280, 720, SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE);

    // Check that the window was successfully created
    if window == null {
        // In the case that the window could not be made...
        log_error("Could not create window: %", to_string(SDL_GetError()));
        exit(1);
    }

    log("%", to_string(ImGui.GetVersion()));

    //SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    //SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, xx SDL_GL_CONTEXT_PROFILE_CORE);
    // SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, xx SDL_GL_CONTEXT_PROFILE_ES);

    //ASDJKKASASDDASASDASDKHASDK Idk how to fix the error Unsupported Function used for immediate mod opengl procs
    // SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, xx SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);
    // SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    // SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);

    gl_context := SDL_GL_CreateContext(window);
    if !gl_context {
        log_error("Could not create OpenGL context: %", to_string(SDL_GetError()));
        exit(1);
    }

    gl_load(*gl, SDL_GL_GetProcAddress);

    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(0); // Enable vsync

    imgui_context := ImGui.CreateContext();



    // Setup ImGui binding
    ImGui_ImplSdl_Init(window);

    // Setup style
    ImGui.StyleColorsClassic();
    //ImGui.StyleColorsLight();

    io := ImGui.GetIO();

    show_demo_window := true;
    show_another_window := false;
    clear_color := Vector4.{0.45, 0.55, 0.60, 1.00};

    f: float = 0.0;

    // Initialize one phyics solver to be used in the example:
    maybe_initialize_example_context();

    // Load default scene
    scenes[context.example_ctx.currScene](context.example_ctx.solver);

    // Main loop
    done := false;
    g_timer_current = seconds_since_init();
    while !done {
        g_timer_previous = g_timer_current;
        g_timer_current = seconds_since_init();
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        event: SDL_Event;
        while SDL_PollEvent(*event) {
            ImGui_ImplSdl_ProcessEvent(*event);
            if event.type == SDL_QUIT   done = true;
            if event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window) {
                done = true;
            }
        }
        ImGui_ImplSdl_NewFrame(window);
        // Start the frame. This call will update the io.WantCaptureMouse, io.WantCaptureKeyboard flag that you can use to dispatch inputs (or not) to your application.
        ImGui.NewFrame();

      
        input();
        ui();

        if ImGui.GetMouseCursor() == {
            case .Arrow;         SDL_SetCursor(cursor_arrow);
            case .TextInput;     SDL_SetCursor(cursor_text_input);
            case .Hand;          SDL_SetCursor(cursor_move);
            case .ResizeNS;      SDL_SetCursor(cursor_resize_ns);
            case .ResizeEW;      SDL_SetCursor(cursor_resize_ew);
            case .ResizeNESW;    SDL_SetCursor(cursor_resize_nesw);
            case .ResizeNWSE;    SDL_SetCursor(cursor_resize_nwse);
        }
        
        //
        // Should Step the physics this frame?
        //
        should_step_physics_now: bool;
        accumulator += g_timer_current - g_timer_previous;  //when accumulator reaches physics timestep needs or beyond, we can step.
        physics_timestep = context.example_ctx.solver.dt;               
        while accumulator >= physics_timestep {
            should_step_physics_now = true;
            accumulator -= physics_timestep;
        }

        //
        // Step the physics!
        //
        if should_step_physics_now 
            context.example_ctx.solver.step(context.example_ctx.solver);

        //
        // Rendering the physics! using IMGUI to draw the physics on this example.
        //
        context.example_ctx.solver.draw(context.example_ctx.solver);
        
        ImGui.Render();
        glViewport(0, 0, xx io.DisplaySize.x, xx io.DisplaySize.y);
        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        // ImGui_ImplOpenGL3_RenderDrawData(ImGui.GetDrawData());
        ImGui_ImplSdl_RenderDrawLists(ImGui.GetDrawData());
        SDL_GL_SwapWindow(window);
        #if SHOULD_ENABLE_MEMORY_DEBUGGER_FOR_CODEX_VIEWER memory_visualizer_per_frame_update();
        reset_temporary_storage();
    }

    // Cleanup
    ImGui_ImplSdl_Shutdown();
    ImGui.DestroyContext(imgui_context);
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
g_timer_current: float64;
g_timer_previous: float64;



accumulator:float64;
physics_timestep := 1.0/60.0;
renderalpha :float64;


Physics2D_Example_Context :: struct {
    solver: *Solver     = null;
    drag : *Joint       = null;
    camZoom: float      = 25.0;
    camPos: Vector2     = Vector2.{ 0, 5 };
    currScene: int     = 4;
    boxSize: Vector2    = Vector2.{ 1, 1 };
    boxVelocity: Vector2 = Vector2.{ 0, 0 };
    boxFriction: float  = 0.5;
    boxDensity: float   = 1.0;
}

#add_context example_ctx: *Physics2D_Example_Context;

maybe_initialize_example_context :: () {
    if context.example_ctx != null then return;
    context.example_ctx = New(Physics2D_Example_Context);
    
    context.example_ctx.solver = solver_instantiate();
    
    repalce_physics2d_avbd_drawing_procs(context.example_ctx.solver);
    

}

input :: ()
{
    io := ImGui.GetIO();
    using context.example_ctx;
    // Convert mouse position to world coordinates
    mousePos := camPos + (Vector2.{ io.MousePos.x, io.DisplaySize.y - io.MousePos.y } -
        Vector2.{ io.DisplaySize.x, io.DisplaySize.y } *0.5) / camZoom;

    // Camera keyboard controls
    if (ImGui.IsKeyDown(ImGui.Key.ImGuiKey_D))
        camPos.x += 10 / camZoom;
    if (ImGui.IsKeyDown(ImGui.Key.ImGuiKey_A))
        camPos.x -= 10 / camZoom;
    if (ImGui.IsKeyDown(ImGui.Key.ImGuiKey_W))
        camPos.y += 10 / camZoom;
    if (ImGui.IsKeyDown(ImGui.Key.ImGuiKey_S))
        camPos.y -= 10 / camZoom;
    if (ImGui.IsKeyDown(ImGui.Key.ImGuiKey_E))
        camZoom *= 1.025;
    if (ImGui.IsKeyDown(ImGui.Key.ImGuiKey_Q))
        camZoom /= 1.025;

    // Camera mouse controls
    if (ImGui.IsMouseDown(ImGui.MouseButton.ImGuiMouseButton_Middle))
        camPos -= Vector2.{ io.MouseDelta.x, -io.MouseDelta.y } / camZoom;
    camZoom *= pow(1.1, io.MouseWheel);

    // Draw box
    if (io.MouseDown[ImGui.MouseButton.ImGuiMouseButton_Left])
    {
        if (!drag)
        {
            local: Vector2;
            b: *Rigid;
            b = solver.pick(solver, mousePos, *local);
            if b
                drag = joint_instantiate(solver, null, b, mousePos, local, Vector3.{ 1000.0, 1000.0, 0.0 });
        }
        else
            drag.rA = mousePos;
    }
    else if (drag)
    {
        force_delete(drag);
        drag = null;
    }

    // Create box
    if (ImGui.IsMouseClicked(ImGui.MouseButton.ImGuiMouseButton_Right))
    {
        rigid_instantiate(solver, boxSize, boxDensity, boxFriction, Vector3.{ mousePos.x, mousePos.y, 0.0 },
            Vector3.{ boxVelocity.x, boxVelocity.y, 0.0 });
    }
}



ui :: ()
{
    using context.example_ctx;
    // Draw the ImGui UI
    ImGui.Begin("Controls");
    ImGui.Text("Move Cam: W,A,S,D / Middle Mouse");
    ImGui.Text("Zoom Cam: Q,E / Mouse Wheel");
    ImGui.Text("Make Box: Right Mouse");
    ImGui.Text("Drag Box: Left Mouse");

    ImGui.Spacing();
    ImGui.Separator();
    ImGui.Spacing();

    scene := currScene;
    if ImGui.BeginCombo("Scene", temp_c_string(sceneNames[scene]))
    {
        for i: 0..sceneCount-1
        {
            selected := scene == i;
            if ImGui.Selectable(temp_c_string(sceneNames[i]), selected) && i != currScene
            {
                currScene = i;
                scenes[currScene](solver);
            }
            if selected
                ImGui.SetItemDefaultFocus();
        }
        ImGui.EndCombo();
    }

    if ImGui.Button(" Reset ")
        scenes[currScene](solver);
    ImGui.SameLine();
    if ImGui.Button("Default")
        solver.defaultParams(solver);

    ImGui.Spacing();
    ImGui.SliderFloat("Box Friction", *boxFriction, 0.0, 2.0);
    ImGui.SliderFloat2("Box Size", xx *boxSize, 0.1, 10.0);
    ImGui.SliderFloat2("Box Velocity", xx *boxVelocity, -20.0, 20.0);

    ImGui.Spacing();
    ImGui.Separator();
    ImGui.Spacing();

    ImGui.SliderFloat("Gravity", *solver.gravity, -20.0, 20.0);
    ImGui.SliderFloat("Dt", *solver.dt, 0.001, 0.1);
    ImGui.SliderInt("Iterations", *solver.iterations, 1, 50);
    ImGui.SliderFloat("Alpha", *solver.alpha, 0.0, 1.0);
    ImGui.SliderFloat("Beta", *solver.beta, 0.0, 1000000.0, "%.f", .Logarithmic);
    ImGui.SliderFloat("Gamma", *solver.gamma, 0.0, 1.0);

    ImGui.End();
}

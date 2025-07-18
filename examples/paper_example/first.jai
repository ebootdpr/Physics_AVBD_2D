#import "Compiler";
#import "Basic";
#import "Process";
#import "File";

#import "Metaprogram_Plugins"; 

OUTPUT_EXECUTABLE_NAME :: "avbd_demo2d";
OUTPUT_BUILD_DIR :: "";

#run {
    set_build_options_dc(.{do_output = false});
    w := compiler_create_workspace("Target Programm");
    // w := get_current_workspace();
    options := get_build_options(w);
    args := options.compile_time_command_line;
   


    copy_commonly_propagated_fields(get_build_options(), *options); //If you dont do this, then -x64 doesnt work
    options.output_executable_name = OUTPUT_EXECUTABLE_NAME;
    options.output_path = OUTPUT_BUILD_DIR;
    set_build_options(options, w);
    compiler_begin_intercept(w); //* not needed, but, for android and linking interceptions, it is needed. @july 2025: it is needed for plugins too!



    add_build_file("src/main.jai",w); //compilin starts here on another thread process

    compiler_end_intercept(w);

}
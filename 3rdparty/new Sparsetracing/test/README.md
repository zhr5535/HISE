# Generating Executable Test Files or Dynamic Libraries  
  
To generate executable test files or dynamic libraries in the current folder, please follow the instructions below. 

1. **Navigate to the Current Directory**:  
   Ensure you are in the directory that contains the necessary source files and the Makefile.  

2. **Generate the Test Executable**:  
  
    We have already provided dynamic library files for the x86 platform (located in the `lib` folder). You can directly use the command `make x86_exe` to generate an executable file for the x86 platform.  
  
    If you choose to recompile the dynamic library files, please ensure that the `lib` folder exists and then enter the command `make x86_lib` to generate the dynamic library for the x86 platform.  
    
    ```bash
    make x86_lib
    make x86_exe
    ```
  
    For the ARM platform, you will need to recompile the dynamic library first. Use the command `make arm_lib` to generate the dynamic library for the ARM platform. After that, you can use the command `make arm_exe` to generate the executable file for the ARM platform.  

      ```bash
    make arm_lib
    make arm_exe
    ```
3. **Run the Test Executable**:  
    Once the executable is generated, you can run it by executing the following command:

    ```bash
    LD_LIBRARY_PATH=../lib ./test
    ```
    In the above command, `LD_LIBRARY_PATH=../lib` specifies the search path for dynamic libraries. Here, we are telling the system to look for the compiled dynamic library in the parent directory (`../lib`) of the current directory. If you prefer to place the dynamic library in a system-default library location, that is also an option. Typically, system-default library locations are `/usr/lib` or `/usr/local/lib`, but this often requires administrative privileges to place files. If you place the dynamic library in one of these locations, you won't need to set the `LD_LIBRARY_PATH` environment variable when running the executable.

4. **Clean Up**:  
   After you have finished testing and no longer need the executable or any intermediate files generated during the build process, you can delete them by running the following command:
   
   ```bash
   make clean
   ```
   This command will invoke the `clean` target in the Makefile, which is typically configured to remove the executable and any temporary files created during the build.    


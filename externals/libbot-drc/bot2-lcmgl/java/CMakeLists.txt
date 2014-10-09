find_package(Java)
if(JAVA_COMPILE STREQUAL JAVA_COMPILE-NOTFOUND OR
       JAVA_ARCHIVE STREQUAL JAVA_ARCHIVE-NOTFOUND OR
 JAVA_COMPILE STREQUAL Java_JAVAC_EXECUTABLE-NOTFOUND OR
 JAVA_ARCHIVE STREQUAL Java_JAR_EXECUTABLE-NOTFOUND)
   message(STATUS "Java not found... not building lcmgl java bindings")
   return()
endif()

set(java_dir ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(class_dir ${CMAKE_CURRENT_BINARY_DIR}/java-build)
set(jar_fname bot2-lcmgl.jar)

# where is lcm.jar?
execute_process(COMMAND pkg-config --variable=classpath lcm-java 
    OUTPUT_VARIABLE LCM_JAR_FILE)
if (NOT LCM_JAR_FILE)
	message(STATUS "\n\n----- ERROR: lcm-java not found. Not building bot-spy plugins")
	return()
endif()
string(STRIP ${LCM_JAR_FILE} LCM_JAR_FILE)

set(classpath ${java_dir}:${LCM_JAR_FILE})


# ============= shouldn't need to modify anything below this line =============

# get a list of all .java files
file(GLOB_RECURSE java_files ${java_dir}/*.java)

# convert the list of .java filenames to a list of .class filenames
foreach(javafile ${java_files})
    string(REPLACE .java .class tmp_fname ${javafile})
    string(REPLACE ${java_dir} ${class_dir} tmp_fname ${tmp_fname})
    list(APPEND class_files ${tmp_fname})
endforeach()

# create the java build directory if needed
if(NOT IS_DIRECTORY ${class_dir})
    file(MAKE_DIRECTORY ${class_dir})
endif()

# add a rule to build the .class files from from the .java files
add_custom_command(OUTPUT ${class_files} COMMAND 
    ${JAVA_COMPILE} -source 6 -target 6 -cp ${classpath} -d ${class_dir} ${java_files} 
    DEPENDS ${java_files} VERBATIM)

# add a rule to build a .jar file from the .class files
add_custom_command(OUTPUT ${jar_fname} COMMAND
    ${JAVA_ARCHIVE} cf ${jar_fname} -C ${class_dir} . 
    DEPENDS ${class_files} VERBATIM)
add_custom_target(jar ALL DEPENDS ${jar_fname})

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${jar_fname} DESTINATION share/java)

#### Building the models

Generate kindsl files from a urdf by running urdf2robcogen

```commandline
roslaunch ocs2_anymal_models urdf2rocogen.launch robot_name:=camel
```

The kindsl files are placed in ocs2_anymal_models/kindsl/<robot_name>

Afterward, use robocogen to generate the files

```commandline
./robcogen.sh /path/robot_name.kindsl /path/robot_name_minimal.dtdsl
```
In the robcogen menu you want to select (1) -> (4) -> (27)

We use the <robot_name>_minimal.dtdsl to only generate the model for a subset of the frames found in the URDF.

Look for the generated robocogen cpp files and copy them to the <robot_name>/generated folder in /include. Look for the differences in miscellaneous.h, miscellaneous.cpp, and inertia_properties.h. Those have been manually adapted after the code generation to fix a code-gen bug.
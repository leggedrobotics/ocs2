OCS2 Documentation Tools
========================

Consists of the following parts:

- **Doxygen** to introspect the C++ code and generate XML output
- **Breathe** to convert Doxygen's XML output to a Sphinx readable format
- **Sphinx** to do the actual generation of the documentation

Updating the Docker documentation environment
---------------------------------------------

GitHub actions pulls the Docker documentation environment from the RSL Harbor _hub_ (https://registry.leggedrobotics.com/). 

```
# in the ocs2_doc directory, where `Dockerfile` resides. 
docker build -t registry.leggedrobotics.com/ocs2/doc . 
# you need push access to `https://registry.leggedrobotics.com/`
docker push registry.leggedrobotics.com/ocs2/doc 
```

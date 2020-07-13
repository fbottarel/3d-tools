# prepare_pc

Simple script to center pointclouds and fix the viewpoint (given the camera ref frame is supposed to be at least 30cm away from objects, as is usual for real RGBD cameras). 

Point clouds are centered according to their center of mass. 
The "fixed" point of view here is according to the [convention](https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/lookat-function) of setting the Z axis facing away from the scene. 

Change the input and output directories in the first lines of the .py file. Point clouds are supposed to be in `.pcd` format. 

### Example

```
pip install --user -r requirements.txt
python prepare_pc.py
```

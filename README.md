# RobotArmProject
## Abstract
This is our implementation of our Project A in the Technion regarding the integration of a Robotic system for the Pick & Place problem and data collection.

Our system is able to take in a 2D camera input and from it create the bounding box of the system and perform perspective transform.
It later is able to geometrically reconstruct the 3D cube and predict the ideal pickup position of the blocks.

<img src="https://github.com/doraviv02/RobotArmProject/blob/main/Prediction.png" width="800">


## Instructions
After cloning and before exectution, add the U2net pretrained model weights to /Segmentation/U_2_Net/saved_models/u2net
The model is from https://drive.google.com/file/d/1ao1ovG1Qtx4b7EoskHXmi2E9rp5CHLcZ/view taken from the U2Net Github

To run the code, execute the main function. There are multiple parameters that can be modified regarding the pickup position noise, block margins etc.

<img src="https://github.com/doraviv02/RobotArmProject/blob/main/Architecture.png" width="800">

## Citation 
U2Net Github

https://github.com/xuebinqin/U-2-Net

xArm-Python-SDK

https://github.com/xArm-Developer/xArm-Python-SDK

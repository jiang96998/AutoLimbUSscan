****************************
2022.06.12 By Yuan_Gao
****************************

This project is about the generation of vascular scan path using OpenPose and Quasi-Newton nonrigid registration.

OpenPose:https: https://github.com/CMU-Perceptual-Computing-Lab/openpose.git
Quasi-Newton: https://github.com/Juyong/Fast_RNRR.git

The related code is all inside the "openposeWithKinect/examples/user_code/vessel/src"

detectBody: capture the whole scene using Kinect and detect joint pints using OpenPose, then segment the arm;
clipArm: clip arm into upper arm and lower arm;
NonRigidreg: do non-rigid registration between atlas and target based on Fast_RNRR ;
workutils: generate path based on the correspondence between atlas and target.

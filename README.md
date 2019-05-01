# Optical Flow and Stitching using RANSAC

## Optical Flow
The Lucas-Kanade optical flow algorithm was implemented using MATLAB to determine the optical flow for 3 distinct sets of images. The code for the algorithm published in HTML is as follows. The file is also attached for reference.  A threshold of 0.01 was used for filtering out certain eigenvalues. It should be noted that this algorithm works best for images which have an object moving within a stationary plane rather than all pixels of an image moving at once. This is most noticeable in the sphere, as the sphere spins the algorithm shows majority of the motion on the sphere. However, in the hallway, as all the pixels move, the optical flow is all over the place with no distinct motion. The synth photo is not very descriptive, as it is hard to determine what is going on. However, the optical flow algorithm seems to produce an output which is certain. 

All six quiverplots and their corresponding window sizes are shown below. 

![img 1](https://raw.githubusercontent.com/somogysm/OpticalFlow-and-Stitching/master/imgs/1.PNG)

![img 2](https://raw.githubusercontent.com/somogysm/OpticalFlow-and-Stitching/master/imgs/2.PNG)

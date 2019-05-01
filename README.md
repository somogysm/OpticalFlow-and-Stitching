# Optical Flow and Stitching using RANSAC

## Optical Flow
The Lucas-Kanade optical flow algorithm was implemented using MATLAB to determine the optical flow for 3 distinct sets of images. The code for the algorithm published in HTML is as follows. The file is also attached for reference.  A threshold of 0.01 was used for filtering out certain eigenvalues. It should be noted that this algorithm works best for images which have an object moving within a stationary plane rather than all pixels of an image moving at once. This is most noticeable in the sphere, as the sphere spins the algorithm shows majority of the motion on the sphere. However, in the hallway, as all the pixels move, the optical flow is all over the place with no distinct motion. The synth photo is not very descriptive, as it is hard to determine what is going on. However, the optical flow algorithm seems to produce an output which is certain. 

All six quiverplots and their corresponding window sizes are shown below. 
![img 1](https://raw.githubusercontent.com/somogysm/OpticalFlow-and-Stitching/master/imgs/1.PNG)

![img 2](https://raw.githubusercontent.com/somogysm/OpticalFlow-and-Stitching/master/imgs/2.PNG)

## RANSAC Stitching

A RANSAC algorithm was generated in order to determine a good match of interesting feature points between two images. Feature points were detected using a Harris Feature detecting algorithm, while the SIFT algorithm was used to generate descriptors around each interesting point. The Euclidean distance was then used to determine which descriptors offered a match between images. A ratio of the distance between nearest points was compared to a threshold of 0.6, if the ratio was less than the threshold, a match was determined.  From here RANSAC was used to generate a projective matrix, M, given four random matching points.  

![img 3](https://raw.githubusercontent.com/somogysm/OpticalFlow-and-Stitching/master/imgs/3.PNG)

The RANSAC algorithm ran for 250 iterations, for each iteration a distinct projective matrix was formed. This matrix was used to project the points in one image onto the other.  The distance between the actual points and projected points (residue) was then calculated and compared against a threshold of 2.6.  The projective matrix which produced the maximum number of inliers was then considered. Each inlier was then extracted, and a new projective matrix was formulated using the same method as above. This new projective matrix was used with the maketform and imtransform functions in matlab to produce a transformed image. Knowing an appropriate transformed image, the two images were then stitched on top of each other, with shared pixel indices being averaged out. 
 
The produced code, stitched image, feature points, and matched points are shown as follows. The best performing iteration produced 96 inliers and 78 outliers with an average residual of 12755.  

![img 4](https://raw.githubusercontent.com/somogysm/OpticalFlow-and-Stitching/master/imgs/4.PNG)
![img 5](https://raw.githubusercontent.com/somogysm/OpticalFlow-and-Stitching/master/imgs/5.PNG)
![img 6](https://raw.githubusercontent.com/somogysm/OpticalFlow-and-Stitching/master/imgs/6.PNG)

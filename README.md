# Blob-Pushing
Source code of a Blue blob tracking/pushing algo using OpenCV & Moveit!.

This code will be improved with the future use of services to declare arm states "push/dont push" or by using Actions to move the arm instead of constantly polling the states topic. 
The code is capable of detecting and following a blob but if the blob is not in-view at all times, errors arise.

HSV color filtering is used then a mask is applied which produces a frame showing the area of designated color.
The HoughCircle function is then called to detect the circular shape of the blobs and their radii.  
Check out this project by following this link: https://drive.google.com/file/d/1zA-VS63BS4WcPmu5KEgP0OXq7WAOBd-O/view?usp=share_link

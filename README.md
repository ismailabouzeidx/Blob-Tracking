# Blob-Tracking
Source code of a Blue blob tracking algo using OpenCV.

The code is bareboned with some bugs to squash.
The code is capable of detecting and following a blob but if the blob is not in-view at all times, errors arise.

HSV color filtering is used then a mask is applied which produces a frame showing the area of designated color.
The HoughCircle function is then called to detect the circular shape of the blobs and their radii.  

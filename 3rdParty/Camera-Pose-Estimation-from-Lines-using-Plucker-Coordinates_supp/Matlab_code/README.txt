The proposed method is implemented in the function "linePoseEstim.m".
The proposed method with Algebraic Outlier Rejection is implemented in the function "linePoseEstimAOR.m".

To see how to call it, please see the script "test_simple9.m".

* test_simple9.m
  - The method is run on predefined 9 lines.

* test_random.m
  - The method is run on N randomly generated lines.

* test_VGG.m
  - The method can be run on real-world image sequences from the VGG multiview dataset.
  - Please uncomment the dataset of your choice at the beginning of the script.
  - As a result, you will see images with reprojected 3D lines using both ground-truth and estimated camera pose.
  
* test_AOR.m
  - The method is run on N randomly generated lines perturbed with slight image noise.
    M out of N lines are outliers which are perturbed with strong image noise.

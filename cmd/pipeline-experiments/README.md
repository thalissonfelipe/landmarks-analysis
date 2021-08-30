# pipeline-experiments

The experiments used on my undergraduate thesis.

For executing the algorithm, you need to provide this set of parameters:

**filename** (required): filename where the results will be saved.

**pointType** (required): name of the fiducial point (the same name as the folder where the fiducial points are located) you want to perform the experiments.

**expression** (optional): if set to true, only expression clouds will be used.

**neutral** (optional): if set to true, only neutral clouds will be used.

**Obs**: choose only expression or neutral at a time.

To run the experiments, it is necessary to change some parameters in the code, this is a `limitation` that can be improved in the future.

Change the following variables with the filters you want to apply the experiments to:

```c
std::vector<std::string> filters{"gaussianCurvature", "shapeIndex"};
std::vector<std::string> kdtreeMethods{"radius", "radius"};
std::vector<float> kdtreeValues{10, 10};
std::vector<float> minThresholds{0.002, -1};
std::vector<float> maxThresholds{0.009, -0.6};
```

The _i_ position of each array corresponds to the settings of an unique filter. In the example above, we have the following filters applied:

- Gaussian curvature with neighborhood calculated from a radius of 10 mm and minimum and maximum thresholds of 0.002 and 0.009, respectively.
- Shape index with neighborhood calculated from a radius of 10 mm and minimum and maximum thresholds of -1 and -0.6, respectively.

Change the following variables with the path where the bosphorus clouds are located:

**landmarksPath**: path where the landmarks clouds are located (note the `pointType` parameter).

**cloudsPath** path where the original clouds are located.

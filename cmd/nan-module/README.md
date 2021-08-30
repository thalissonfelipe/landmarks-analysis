# nan-module

## Install dependencies and compile

```
npm install
npm run compile
```

To use this module inside a nodejs application, pass the parameters like a regular function call:

```js
const fiducialPointFinder = require('[path to fiducial_point_finder.node]');

const response = fiducialPointFinder.findFiducialPoint(
    filename,
    flexibilizeThresholds,
    flexibilizeCrop,
    computationRadiusOrKSize,
    computationMethod,
    minGaussianCurvature,
    shapeIndexLimit,
    minCropSize,
    maxCropSize,
    minPointsToContinue,
    removeIsolatedPointsRadius,
    removeIsolatedPointsThreshold,
    nosetipSearchRadius,
    gfSearchRadius, // *DEPRECATED
    features, // *DEPRECATED
    featuresThreshold, // *DEPRECATED
    fiducialPoint
);
```

***DEPRECATED**: These parameters were used in older versions but have not yet been removed, just pass null in the correct order of parameters until this is fixed.

## Parameters

**filename**: path where the file is located.

**flexibilizeThresholds**: if after filtering, the cloud does not have enough points, set `true` to continue or `false` to stop the algorithm.

**flexibilizeCrop**: if after filtering, the cloud does not have enough points, set `true` to continue or `false` to stop the algorithm.

**computationRadiusOrKSize**: method to calculate the neighborhood: `radius` or `number k of neighbors`;

**computationMethod**: value to calculate the neighborhood.

The parameters below serve as thresholds for the algorithm.

**minGaussianCurvature**

**shapeIndexLimit**

**minCropSize**

**maxCropSize**

**minPointsToContinue**

**removeIsolatedPointsRadius**

**removeIsolatedPointsThreshold**

**nosetipSearchRadius**

**gfSearchRadius**: set this parameter to null

**features**: set this parameter to null

**featuresThreshold**: set this parameter to null

**fiducialPoint**: the fiducial point you want to find, currently only supports `nosetip` option.

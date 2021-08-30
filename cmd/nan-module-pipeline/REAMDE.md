# nan-module-pipeline

## Install dependencies and compile

```
npm install
npm run compile
```

To use this module inside a nodejs application, pass the parameters like a regular function call:

```js
const pipeline = require('[path to pipeline.node]');

const response = pipeline.applyFilters(
    filename,
    filepath,
    outputFilename,
    filters,
    saveResults
);
```

## Parameters

**filename**: name of the file.

****filepath**** path where the file is located.

****outputFilename****: if provided, the name of the file were the results will be saved.

****filters****: an object containing information about each filter. Example:

```json
 {
    "filters": [
        {
            "filterName": "shapeIndex",
            "kdtreeMethod": "radius",
            "kdtreeValue": "10",
            "minThreshold": "-1",
            "maxThreshold": "1"
        }
    ]
}
```

**saveResults**: flag indicating whether the result should be saved or not.

# regular

## Parameters

For executing the algorithm, you need to provide this set of parameters **(in this order)**:

- 1 - Input filename - `STRING`
- 2 - Flexibilize thresholds - `BOOLEAN`
- 3 - Flexibilize crop - `BOOLEAN`
- 4 - Normal estimation size (radius size or k value) - `INTEGER`
- 5 - Normal estimation method - `radius` or `k` (as string)
- 6 - Minimum threshold of Gaussian Curvature - `DOUBLE`
- 7 - Maximum threshold of Shape Index - `DOUBLE` (between `-1` and `+1`)
- 8 - Minimum crop size - `FLOAT`
- 9 - Maximum crop size - `FLOAT`
- 10 - Minimum points to continue - `INTEGER`
- 11 - Removal radius of isolated points - `FLOAT`
- 12 - Threshold of points count of isolated points - `INTEGER`
- 13 - Search radius of nose tip - `INTEGER`

If using regular C++ build, pass this parameters inline, like: 

```
[project_path]/cmd/regular/build/nosetip_finder parameter1 parameter2 parameter3 [...]
```

## Regular C++ output exclusive parameters

If you are using the regular C++ build, you can use some other parameters. Note that, considering the order, these parameters should be set after the parameters listed on the last section.

**Note that the only optional parameter is the 16. All the others are required.**

Exclusive parameters for C++ regular build **(in this order)**:
- 14 - Ouput file to save the nosetip as a `.pcd` file - `STRING`
- 15 - Definition if you want to visualize the final and the intermediary results - Set this parameter as `visualizar` if you want to visualize or set as any other string if you do not want to visualize the results.
- 16 - A file with a known nosetip to verify if the algorithm result is a good nosetip - `STRING`
- 17 - A CSV file to save the processing results _(analyzed cloud, if the result is a good nosetip, execution time, known nosetip and the nosetip found)_ - `STRING`
- 18 - A CSV file where eventual errors will be written - `STRING`

Obs: If you don't pass the parameter 16 and parameter 15 is set as `visualizar`, you will be prompted by the terminal to input if the result is a good nosetip or not.

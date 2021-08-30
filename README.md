# Landmark Analysis - Find nosetip and apply filtering (Pipeline)

This project is a fork from https://github.com/MarcusVLMA/nosetip_finder and was developed during my undergraduate thesis.

## Dependencies

To run this project you will need:

- [Point Cloud Library](https://pointclouds.org/)
- [CMake](https://cmake.org/) to build the output.

If you intend to generate the `.node` file, you will also need:

- Node 10 (I recommend using [nvm](https://github.com/nvm-sh/nvm) since this is a old node version).
- [CMake.js](https://github.com/cmake-js/cmake-js) to build using cmake.

## Building

This project is divided into four parts.

- [nan-module](cmd/nan-module/README.md): adapted to include algorithms other than nose tip location.
- [nan-module-pipeline](cmd/nan-module-pipeline/README.md): responsible for compiling the algorithm that performs filtering in c++ and that will be used in nodejs.
- [pipeline-experiments](cmd/pipeline-experiments/README.md): experiments used on my undergraduate thesis.
- [regular](cmd/regular/README.md): command line to execute the nose tip algorithm.

Each folder has a README explaining how to build and run the project. In general, projects can be compiled as follows:
### Regular C++ output

Once you have installed Point Cloud Library and CMake, just get into `cmd/regular` or `cmd/pipeline-experiments` and run:

```
cmake .
make
```

The executable output file  can be found inside of `cmd/regular/build` or `cmd/pipeline-experiments/build` folders.

### To use with Node.js

You may want to use this algorithm with a web application, like I did in this [repository](https://github.com/thalisson/latin-pcd-viewer). A way to do this is generating the `.node` build, to use as a dependency in Node.js code.

As mencioned earlier, you will need Node.js 10. If you are using `nvm`, you can set Node version to 10 by running:

```
nvm install 10.23.0
nvm use 10
```

_Note that this will set your Node version only in the current terminal._

After that, install `cmake-js` globally with:

```
npm install -g cmake-js
```

Now, go into `cmd/nan-module` or `cmd/nan-module-pipeline` and run:

```
npm run compile
```

This will install any necessary dependencies and compile the code with `cmake-js`.

The output file can be found inside of `cmd/nan-module/build/Release` or `cmd/nan-module-pipeline/build/Release` folders.

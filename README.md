# GTSAM Vendordep

A wrapper for [GTSAM](https://gtsam.org/), built as a WPILib
[third party library](https://github.com/wpilibsuite/thirdparty-gtsam/),
which is referenced in config.gradle, in nativeUtils.nativeDependencyContainer.gtsam.

Classes are wrapped individually, e.g. src/main/driver/cpp/Point2.cpp:

```C++
extern "C" {
gtsam::Point2* Point2(double x, double y) {
    return new gtsam::Point2(x, y);
}
...
```

Each set of symbols is referenced in src/main/driver/symbols.txt, using wildcards:

```
Point2*
...
```

The third-party GTSAM build uses TBB, so be sure to install libtbb-dev before building.

```
sudo apt update
sudo apt install libtbb-dev
```

The GTSAM build produces a lot of warnings,
so I commented out the relevant lines in config.gradle:
```
// nativeUtils.wpi.addWarnings()
// nativeUtils.wpi.addWarningsAsErrors()
```

To run the build:

```
./gradlew build
```

To publish the output to build/repos

```
./gradlew publish
```

Look in the library to see what's visible:

```
$ nm -g libgtsamwrapper.so | grep ' T '
00000000000023f0 T Point2
0000000000002440 T Point2_delete
0000000000002480 T Point2_print
0000000000002460 T Point2_x
0000000000002470 T Point2_y
```

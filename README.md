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

## Warnings

The GTSAM build produces a lot of warnings,
so I commented out the relevant lines in config.gradle:
```
// nativeUtils.wpi.addWarnings()
// nativeUtils.wpi.addWarningsAsErrors()
```

## Building

If you ran the thirdparty gtsam build on the same machine, the artifacts
should be published in $HOME/releases/maven, where the gtsam vendordep build
should find them.

Make sure you're using JDK 22+ for FFM.

Gradle keeps daemons around that can get confused if you change the java version;
kill them with `./gradlew --stop`

To run the build:

```
./gradlew build
```
or
```
./gradlew build -PreleaseMode=true
```

To publish the output to build/repos

```
./gradlew publish
```

or

```
./gradlew publish -PreleaseMode=true
```



## Symbols

Look in the library to see what's visible:

```
$ nm -g libgtsamwrapper.so | grep ' T '
00000000000023f0 T Point2
0000000000002440 T Point2_delete
0000000000002480 T Point2_print
0000000000002460 T Point2_x
0000000000002470 T Point2_y
```

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

Make sure you update `symbols.txt` when you add new cpp code!

The Java bindings are sugared a little via the enum pattern, using
the return type and argument types, just to make the whole thing more terse:

```
public enum FF {
    Rot2(ADDRESS, JAVA_DOUBLE),
    Rot2_delete(null, ADDRESS);
    ...
}
```

Pass null as the return type if you want a void binding.

The name of the enum value must match the name of the C extern exactly.

There are many instances of optional Jacobians; these are supported with
distinct C extern functions.  I tried making the Jacobian arguments
optional, in the way Java can do it, e.g. passing MemorySegment.NULL,
but I couldn't find a way to sort out the null in the C bindings, and
it would have been a bit opaque anyway.  So the C bindings are a bit more
verbose, but more straightforward.

## Types

The GTSAM types, and C++ type system in general, can only be approximated
with the (simpler) Java type system.  Here are some of the highlights:

* Tangent vector types (`TangentVector` in Lie.h) are fixed-size, e.g. `Vector3`.
* Jacobian types are dynamic size (`Matrix`).
* In C++, geometry requirements (e.g. LieGroup) are expressed with static
  duck typing; in Java there is an interface describing the duck-type expectations (e.g. `p.retract(v)`), with a nested interface for the static elements (e.g. `Class::Retract(p,v)`)
* The C++ code uses operator overloading for things like `compose`; in Java
  there are only methods.
* Geometry traits specify static methods; these are replaced with instance
  methods.
* Some of the geometry requirements (not traits) are also static, and
  these also use instance methods.
* The unit tests seem to mostly only use traits where they're required, e.g.
  for types like Point3 which are typedefs of something else.  Traits are also
  explicitly used by numeric differentiation.

## Lifecycle

Everything passed between Java and C++ is heap-allocated. Values returned
to Java are generally pointers to these heap-allocated objects, and the
same pointers are passed back to C++.

There are two ways to manage the lifecycle of these heap objects.

The most common method uses ForeignObject, which manages "owned" objects,
mirroring the JVM. When the wrapper becomes unreachable, the "cleaner" is
called to delete the corresponding C++ object.  So shared_ptr itself is a
ForeignObject: deleting it doesn't actually delete the referent, it just
decrements the reference count.

Some objects are merely "observed", so there is no deleter, just pass null.

The shared_ptr object's lifecycle is managed by Java, but the
objects it returns with get() are not managed -- they have null deleters.


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

## Java tests

Once you publish (see above) then the java tests should work,
including through the vscode test extension.


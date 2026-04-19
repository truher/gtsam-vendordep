package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_INT;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/** Actually a shared_ptr */
public class SharedNoiseModel {
    private static final MethodHandle Sigmas1 = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("Sigmas1"),
            FunctionDescriptor.of(ADDRESS, ADDRESS));
    private static final MethodHandle Sigmas2 = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("Sigmas2"),
            FunctionDescriptor.of(ADDRESS, ADDRESS));
    private static final MethodHandle Sigmas3 = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("Sigmas3"),
            FunctionDescriptor.of(ADDRESS, ADDRESS));
    private static final MethodHandle Unit = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("Unit"),
            FunctionDescriptor.of(ADDRESS, JAVA_INT));
    private static final MethodHandle use_count = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("use_count"),
            FunctionDescriptor.of(JAVA_LONG, ADDRESS));

    /** Pointer to the shared pointer. */
    public final MemorySegment ptr;

    private SharedNoiseModel(MemorySegment p) {
        ptr = p;
    }

      public static SharedNoiseModel Sigmas(Vector1 v) throws Throwable {
        return new SharedNoiseModel((MemorySegment) Sigmas1.invokeExact(v.ptr));
    }
    public static SharedNoiseModel Sigmas(Vector2 v) throws Throwable {
        return new SharedNoiseModel((MemorySegment) Sigmas2.invokeExact(v.ptr));
    }

    public static SharedNoiseModel Sigmas(Vector3 v) throws Throwable {
        return new SharedNoiseModel((MemorySegment) Sigmas3.invokeExact(v.ptr));
    }

    public static SharedNoiseModel Unit(int dim) throws Throwable {
        return new SharedNoiseModel((MemorySegment) Unit.invokeExact(dim));
    }

    public long use_count() throws Throwable {
        return (long) use_count.invokeExact(ptr);
    }

}

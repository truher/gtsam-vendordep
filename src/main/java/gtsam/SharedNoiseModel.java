package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_INT;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/**
 * Actually a shared_ptr
 * TODO: maybe actually use the shared_ptr thing in java, instead of doing it
 * differently here.
 */
public class SharedNoiseModel {
    public enum FF {
        SharedNoiseModel_Sigmas(ADDRESS, ADDRESS),
        SharedNoiseModel_Sigmas1(ADDRESS, ADDRESS),
        SharedNoiseModel_Sigmas2(ADDRESS, ADDRESS),
        SharedNoiseModel_Sigmas3(ADDRESS, ADDRESS),
        SharedNoiseModel_Unit(ADDRESS, JAVA_INT),
        SharedNoiseModel_use_count(JAVA_LONG, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    /** Pointer to the shared pointer. */
    public final MemorySegment ptr;

    private SharedNoiseModel(MemorySegment p) {
        ptr = p;
    }

    public static SharedNoiseModel Sigmas(Vector v) throws Throwable {
        return new SharedNoiseModel((MemorySegment) FF.SharedNoiseModel_Sigmas.h.invokeExact(v.ptr));
    }

    public static SharedNoiseModel Sigmas(Vector1 v) throws Throwable {
        return new SharedNoiseModel((MemorySegment) FF.SharedNoiseModel_Sigmas1.h.invokeExact(v.ptr));
    }

    public static SharedNoiseModel Sigmas(Vector2 v) throws Throwable {
        return new SharedNoiseModel((MemorySegment) FF.SharedNoiseModel_Sigmas2.h.invokeExact(v.ptr));
    }

    public static SharedNoiseModel Sigmas(Vector3 v) throws Throwable {
        return new SharedNoiseModel((MemorySegment) FF.SharedNoiseModel_Sigmas3.h.invokeExact(v.ptr));
    }

    public static SharedNoiseModel Unit(int dim) throws Throwable {
        return new SharedNoiseModel((MemorySegment) FF.SharedNoiseModel_Unit.h.invokeExact(dim));
    }

    public long use_count() throws Throwable {
        return (long) FF.SharedNoiseModel_use_count.h.invokeExact(ptr);
    }

}

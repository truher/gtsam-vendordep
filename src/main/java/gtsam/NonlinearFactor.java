package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class NonlinearFactor {
    private static final MethodHandle NonlinearFactor_linearize = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("NonlinearFactor_linearize"),
            FunctionDescriptor.of(ADDRESS, ADDRESS, ADDRESS));

    /** Pointer to the factor, the result of shared_ptr.get(). */
    final MemorySegment ptr;

    /** @param p pointer to the factor itself, not the shared_ptr. */
    NonlinearFactor(MemorySegment p) {
        ptr = p;
    }

    /** returns shared_ptr<GaussianFactor> */
    public GaussianFactor linearize(Values v) throws Throwable {
        return new GaussianFactor((MemorySegment) NonlinearFactor_linearize.invokeExact(
                ptr, v.ptr));
    }

}

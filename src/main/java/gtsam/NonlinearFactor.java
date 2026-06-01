package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class NonlinearFactor {
    public enum FF {
        NonlinearFactor_linearize(ADDRESS, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    /** Pointer to the factor, the result of shared_ptr.get(). */
    final MemorySegment ptr;

    /** @param p pointer to the factor itself, not the shared_ptr. */
    NonlinearFactor(MemorySegment p) {
        ptr = p;
    }

    /** returns shared_ptr<GaussianFactor> */
    public GaussianFactor linearize(Values v) throws Throwable {
        return new GaussianFactor((MemorySegment) FF.NonlinearFactor_linearize.h.invokeExact(
                ptr, v.ptr));
    }

}

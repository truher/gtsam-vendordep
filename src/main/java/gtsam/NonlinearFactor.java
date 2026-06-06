package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * Factors are always passed around inside shared_ptr, so this does not manage
 * its own lifecycle, passes null to the parent ForeignObject deleter.
 */
public class NonlinearFactor extends ForeignObject {
    public enum FF {
        NonlinearFactor_linearize(ADDRESS, ADDRESS, ADDRESS),
        NonlinearFactor_print(null, ADDRESS),
        NonlinearFactor_equals(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        NonlinearFactor_weight(JAVA_DOUBLE, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    /** @param p pointer to the factor itself, not the shared_ptr. */
    NonlinearFactor(MemorySegment p) {
        super(p, null);
    }

    /**
     * Linearize this factor.
     * 
     * Returns shared_ptr<GaussianFactor>
     */
    public GaussianFactor linearize(Values v) throws Throwable {
        return new GaussianFactor((MemorySegment) FF.NonlinearFactor_linearize.h.invokeExact(
                ptr, v.ptr));
    }

    public void print() throws Throwable {
        FF.NonlinearFactor_print.h.invokeExact(ptr);
    }

    public boolean equals(NonlinearFactor g) throws Throwable {
        return (boolean) FF.NonlinearFactor_equals.h.invokeExact(ptr, g.ptr);
    }

    public double weight(Values v) throws Throwable {
        return (double) FF.NonlinearFactor_weight.h.invokeExact(ptr, v.ptr);
    }

}

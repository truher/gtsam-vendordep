package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/**
 * Factors are always passed around inside shared_ptr, so this does not manage
 * its own lifecycle, passes null to the parent ForeignObject deleter.
 */
public class NonlinearFactor extends Factor {
    public enum FF {
        NonlinearFactor_error(JAVA_DOUBLE, ADDRESS, ADDRESS),
        NonlinearFactor_linearize(ADDRESS, ADDRESS, ADDRESS),
        NonlinearFactor_print(null, ADDRESS),
        NonlinearFactor_equals(JAVA_BOOLEAN, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    /** @param p pointer to the factor itself, not the shared_ptr. */
    NonlinearFactor(MemorySegment p) {
        super(p);
    }

    public double error(Values v) throws Throwable {
        return (double) FF.NonlinearFactor_error.h.invokeExact(ptr, v.ptr);
    }

    /**
     * Linearize this factor.
     * 
     * Returns shared_ptr<GaussianFactor>
     */
    public shared_ptr<GaussianFactor> linearize(Values v) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.NonlinearFactor_linearize.h.invokeExact(
                        ptr, v.ptr),
                GaussianFactor::new);
    }

    public void print() throws Throwable {
        FF.NonlinearFactor_print.h.invokeExact(ptr);
    }

    public boolean equals(NonlinearFactor g) throws Throwable {
        return (boolean) FF.NonlinearFactor_equals.h.invokeExact(ptr, g.ptr);
    }

}

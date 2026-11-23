package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public abstract class NonlinearOptimizer extends ForeignObject {
    public enum FF {
        NonlinearOptimizer_optimize(ADDRESS, ADDRESS),
        NonlinearOptimizer_error(JAVA_DOUBLE, ADDRESS),
        NonlinearOptimizer_iterations(JAVA_LONG, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    NonlinearOptimizer(MemorySegment pointer, MethodHandle deleter) {
        // abstract, deleted by subclass
        super(pointer, deleter);
    }

    public Values optimize() {
        return Values.owned((MemorySegment) FF.NonlinearOptimizer_optimize.invoke(ptr));
    }

    public double error() {
        return (double) FF.NonlinearOptimizer_error.invoke(ptr);
    }

    public long iterations() {
        return (long) FF.NonlinearOptimizer_iterations.invoke(ptr);
    }

}

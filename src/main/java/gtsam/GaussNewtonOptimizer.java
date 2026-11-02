package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class GaussNewtonOptimizer extends NonlinearOptimizer {
    public enum FF {
        GaussNewtonOptimizer(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        GaussNewtonOptimizer_delete(null, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    GaussNewtonOptimizer(MemorySegment pointer) {
        super(pointer, FF.GaussNewtonOptimizer_delete.h);
    }

    public GaussNewtonOptimizer(
            NonlinearFactorGraph graph,
            Values initialValues,
            GaussNewtonParams params) {
        this((MemorySegment) FF.GaussNewtonOptimizer.invoke(
                graph.ptr, initialValues.ptr, params.ptr));
    }
}

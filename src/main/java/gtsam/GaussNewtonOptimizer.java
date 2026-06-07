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

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    GaussNewtonOptimizer(MemorySegment pointer) {
        super(pointer, FF.GaussNewtonOptimizer_delete.h);
    }

    public GaussNewtonOptimizer(
            NonlinearFactorGraph graph,
            Values initialValues,
            GaussNewtonParams params) throws Throwable {
        this((MemorySegment) FF.GaussNewtonOptimizer.h.invokeExact(
                graph.ptr, initialValues.ptr, params.ptr));
    }
}

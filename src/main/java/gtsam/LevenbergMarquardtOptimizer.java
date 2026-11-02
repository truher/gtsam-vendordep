package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class LevenbergMarquardtOptimizer extends NonlinearOptimizer {
    public enum FF {
        LevenbergMarquardtOptimizer(ADDRESS, ADDRESS, ADDRESS),
        LevenbergMarquardtOptimizer_delete(null, ADDRESS),
        LevenbergMarquardtOptimizer3(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        LevenbergMarquardtOptimizer_values(ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public LevenbergMarquardtOptimizer(MemorySegment p) {
        super(p, FF.LevenbergMarquardtOptimizer_delete.h);
    }

    public LevenbergMarquardtOptimizer(
            NonlinearFactorGraph graph,
            Values initialValues) {
        this((MemorySegment) FF.LevenbergMarquardtOptimizer.invoke(
                graph.ptr, initialValues.ptr));
    }

    public LevenbergMarquardtOptimizer(
            NonlinearFactorGraph graph,
            Values initialValues,
            LevenbergMarquardtParams params) {
        this((MemorySegment) FF.LevenbergMarquardtOptimizer3.invoke(
                graph.ptr, initialValues.ptr, params.ptr));
    }

    /** Returned Values are owned. */
    public Values values() {
        return Values.owned(
                (MemorySegment) FF.LevenbergMarquardtOptimizer_values.invoke(ptr));
    }

}

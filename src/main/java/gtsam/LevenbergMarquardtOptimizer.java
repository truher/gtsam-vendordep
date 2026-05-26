package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class LevenbergMarquardtOptimizer extends ForeignObject {
    public enum FF {
        LevenbergMarquardtOptimizer(ADDRESS, ADDRESS, ADDRESS),
        LevenbergMarquardtOptimizer_delete(null, ADDRESS),
        LevenbergMarquardtOptimizer3(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        LevenbergMarquardtOptimizer_optimize(ADDRESS, ADDRESS),
        LevenbergMarquardtOptimizer_values(ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public LevenbergMarquardtOptimizer(MemorySegment p) {
        super(p, FF.LevenbergMarquardtOptimizer_delete.h);
    }

    public LevenbergMarquardtOptimizer(
            NonlinearFactorGraph graph,
            Values initialValues) throws Throwable {
        this((MemorySegment) FF.LevenbergMarquardtOptimizer.h.invokeExact(
                graph.ptr, initialValues.ptr));
    }

    public LevenbergMarquardtOptimizer(
            NonlinearFactorGraph graph,
            Values initialValues,
            LevenbergMarquardtParams params) throws Throwable {
        this((MemorySegment) FF.LevenbergMarquardtOptimizer3.h.invokeExact(
                graph.ptr, initialValues.ptr, params.ptr));
    }

    /** Returned Values are owned. */
    public Values optimize() throws Throwable {
        return Values.owned(
                (MemorySegment) FF.LevenbergMarquardtOptimizer_optimize.h.invokeExact(ptr));
    }

    /** Returned Values are owned. */
    public Values values() throws Throwable {
        return Values.owned(
                (MemorySegment) FF.LevenbergMarquardtOptimizer_values.h.invokeExact(ptr));
    }

}

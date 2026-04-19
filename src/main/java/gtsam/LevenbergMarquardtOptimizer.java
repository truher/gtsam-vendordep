package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class LevenbergMarquardtOptimizer extends ForeignObject {
    private static final MethodHandle LevenbergMarquardtOptimizer = Lib.down(
            "LevenbergMarquardtOptimizer", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle LevenbergMarquardtOptimizer_delete = Lib.downVoid(
            "LevenbergMarquardtOptimizer_delete", ADDRESS);
    private static final MethodHandle LevenbergMarquardtOptimizer3 = Lib.down(
            "LevenbergMarquardtOptimizer3", ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle LevenbergMarquardtOptimizer_optimize = Lib.down(
            "LevenbergMarquardtOptimizer_optimize", ADDRESS, ADDRESS);
    private static final MethodHandle LevenbergMarquardtOptimizer_values = Lib.down(
            "LevenbergMarquardtOptimizer_values", ADDRESS, ADDRESS);

    public LevenbergMarquardtOptimizer(MemorySegment p) {
        super(p, LevenbergMarquardtOptimizer_delete);
    }

    public LevenbergMarquardtOptimizer(
            NonlinearFactorGraph graph,
            Values initialValues) throws Throwable {
        this((MemorySegment) LevenbergMarquardtOptimizer.invokeExact(
                graph.ptr, initialValues.ptr));
    }

    public LevenbergMarquardtOptimizer(
            NonlinearFactorGraph graph,
            Values initialValues,
            LevenbergMarquardtParams params) throws Throwable {
        this((MemorySegment) LevenbergMarquardtOptimizer3.invokeExact(
                graph.ptr, initialValues.ptr, params.ptr));
    }

    /** Returned Values are owned. */
    public Values optimize() throws Throwable {
        return Values.owned(
                (MemorySegment) LevenbergMarquardtOptimizer_optimize.invokeExact(ptr));
    }

    /** Returned Values are owned. */
    public Values values() throws Throwable {
        return Values.owned(
                (MemorySegment) LevenbergMarquardtOptimizer_values.invokeExact(ptr));
    }

}

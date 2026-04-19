package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class BatchFixedLagSmoother extends ForeignObject {
    private static final MethodHandle BatchFixedLagSmoother = Lib.down(
            "BatchFixedLagSmoother", ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle BatchFixedLagSmoother_delete = Lib.downVoid(
            "BatchFixedLagSmoother_delete", ADDRESS);
    private static final MethodHandle BatchFixedLagSmoother_update = Lib.down(
            "BatchFixedLagSmoother_update", ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle BatchFixedLagSmoother_calculateEstimate = Lib.down(
            "BatchFixedLagSmoother_calculateEstimate", ADDRESS, ADDRESS);
    private static final MethodHandle BatchFixedLagSmoother_getFactors = Lib.down(
            "BatchFixedLagSmoother_getFactors", ADDRESS, ADDRESS);

    public BatchFixedLagSmoother(MemorySegment p) {
        super(p, BatchFixedLagSmoother_delete);
    }

    public BatchFixedLagSmoother(double lag) throws Throwable {
        this((MemorySegment) BatchFixedLagSmoother.invokeExact(lag));
    }

    public FixedLagSmoother.Result update(
            NonlinearFactorGraph newFactors,
            Values newTheta,
            FixedLagSmoother.KeyTimestampMap timestamps) throws Throwable {
        return new FixedLagSmoother.Result(
                (MemorySegment) BatchFixedLagSmoother_update.invokeExact(
                        ptr, newFactors.ptr, newTheta.ptr, timestamps.ptr));
    }

    /** Returned Values are owned. */
    public Values calculateEstimate() throws Throwable {
        return Values.owned((MemorySegment) BatchFixedLagSmoother_calculateEstimate.invokeExact(ptr));
    }

    public NonlinearFactorGraph getFactors() throws Throwable {
        return new NonlinearFactorGraph((MemorySegment) BatchFixedLagSmoother_getFactors.invokeExact(ptr));
    }

}

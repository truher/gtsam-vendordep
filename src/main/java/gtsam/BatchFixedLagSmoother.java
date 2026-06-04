package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class BatchFixedLagSmoother extends ForeignObject {
    public enum FF {
        BatchFixedLagSmoother(ADDRESS, JAVA_DOUBLE),
        BatchFixedLagSmoother2(ADDRESS, JAVA_DOUBLE, ADDRESS, JAVA_BOOLEAN),
        BatchFixedLagSmoother_delete(null, ADDRESS),
        BatchFixedLagSmoother_update(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        BatchFixedLagSmoother_calculateEstimate(ADDRESS, ADDRESS),
        BatchFixedLagSmoother_getFactors(ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public BatchFixedLagSmoother(MemorySegment p) {
        super(p, FF.BatchFixedLagSmoother_delete.h);
    }

    public BatchFixedLagSmoother(double lag) throws Throwable {
        this((MemorySegment) FF.BatchFixedLagSmoother.h.invokeExact(lag));
    }

    public BatchFixedLagSmoother(double lag,
            LevenbergMarquardtParams params,
            boolean consistent) throws Throwable {
        this((MemorySegment) FF.BatchFixedLagSmoother2.h.invokeExact(
                lag, params.ptr, consistent));
    }

    public FixedLagSmoother.Result update(
            NonlinearFactorGraph newFactors,
            Values newTheta,
            FixedLagSmoother.KeyTimestampMap timestamps) throws Throwable {
        return new FixedLagSmoother.Result(
                (MemorySegment) FF.BatchFixedLagSmoother_update.h.invokeExact(
                        ptr, newFactors.ptr, newTheta.ptr, timestamps.ptr));
    }

    /** Returned Values are owned. */
    public Values calculateEstimate() throws Throwable {
        return Values.owned((MemorySegment) FF.BatchFixedLagSmoother_calculateEstimate.h.invokeExact(ptr));
    }

    public NonlinearFactorGraph getFactors() throws Throwable {
        return new NonlinearFactorGraph((MemorySegment) FF.BatchFixedLagSmoother_getFactors.h.invokeExact(ptr));
    }

}

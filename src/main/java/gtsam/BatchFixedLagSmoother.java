package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

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
        BatchFixedLagSmoother_updateFactorIndices(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        BatchFixedLagSmoother_calculateEstimate(ADDRESS, ADDRESS),
        BatchFixedLagSmoother_calculateEstimatePoint2(ADDRESS, ADDRESS, JAVA_LONG),
        BatchFixedLagSmoother_getFactors(ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public BatchFixedLagSmoother(MemorySegment p) {
        super(p, FF.BatchFixedLagSmoother_delete.h);
    }

    public BatchFixedLagSmoother(double lag) {
        this((MemorySegment) FF.BatchFixedLagSmoother.invoke(lag));
    }

    public BatchFixedLagSmoother(double lag,
            LevenbergMarquardtParams params) {
        this((MemorySegment) FF.BatchFixedLagSmoother2.invoke(
                lag, params.ptr, true));
    }

    public BatchFixedLagSmoother(double lag,
            LevenbergMarquardtParams params,
            boolean consistent) {
        this((MemorySegment) FF.BatchFixedLagSmoother2.invoke(
                lag, params.ptr, consistent));
    }

    public FixedLagSmoother.Result update(
            NonlinearFactorGraph newFactors,
            Values newTheta,
            FixedLagSmoother.KeyTimestampMap timestamps) {
        MemorySegment p = (MemorySegment) FF.BatchFixedLagSmoother_update.invoke(
                ptr, newFactors.ptr, newTheta.ptr, timestamps.ptr);
        if (p.equals(MemorySegment.NULL)) {
            throw new IndeterminateSystemException();
        }
        return new FixedLagSmoother.Result(p);
    }

    public FixedLagSmoother.Result update(
            NonlinearFactorGraph newFactors,
            Values newTheta,
            FixedLagSmoother.KeyTimestampMap timestamps,
            FactorIndices indices) {
        MemorySegment p = (MemorySegment) FF.BatchFixedLagSmoother_updateFactorIndices.invoke(
                ptr, newFactors.ptr, newTheta.ptr, timestamps.ptr, indices.ptr);
        if (p.equals(MemorySegment.NULL)) {
            throw new IndeterminateSystemException();
        }
        return new FixedLagSmoother.Result(p);
    }

    /** Returned Values are owned. */
    public Values calculateEstimate() {
        return Values.owned((MemorySegment) FF.BatchFixedLagSmoother_calculateEstimate.invoke(ptr));
    }

    public Point2 calculateEstimatePoint2(Key key) {
        return new Point2((MemorySegment) FF.BatchFixedLagSmoother_calculateEstimatePoint2.invoke(ptr, key.j));
    }

    public NonlinearFactorGraph getFactors() {
        return new NonlinearFactorGraph((MemorySegment) FF.BatchFixedLagSmoother_getFactors.invoke(ptr));
    }

}

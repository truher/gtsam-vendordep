package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class IncrementalFixedLagSmoother extends ForeignObject {
    public enum FF {
        IncrementalFixedLagSmoother_delete(null, ADDRESS),
        IncrementalFixedLagSmoother(ADDRESS, JAVA_DOUBLE, ADDRESS),
        IncrementalFixedLagSmoother_update(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        IncrementalFixedLagSmoother_updateFactorIndices(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        IncrementalFixedLagSmoother_calculateEstimate(ADDRESS, ADDRESS),
        IncrementalFixedLagSmoother_calculateEstimatePoint2(ADDRESS, ADDRESS, JAVA_LONG),
        IncrementalFixedLagSmoother_getFactors(ADDRESS, ADDRESS),
        IncrementalFixedLagSmoother_getISAM2Result(ADDRESS, ADDRESS),
        IncrementalFixedLagSmoother_getLinearizationPoint(ADDRESS, ADDRESS),
        IncrementalFixedLagSmoother_getISAM2(ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    IncrementalFixedLagSmoother(MemorySegment pointer) {
        super(pointer, FF.IncrementalFixedLagSmoother_delete.h);
    }

    public IncrementalFixedLagSmoother(double lag,
            ISAM2Params params) {
        this((MemorySegment) FF.IncrementalFixedLagSmoother.invoke(
                lag, params.ptr));
    }

    public FixedLagSmoother.Result update(
            NonlinearFactorGraph newFactors,
            Values newTheta,
            FixedLagSmoother.KeyTimestampMap timestamps) {
        return new FixedLagSmoother.Result(
                (MemorySegment) FF.IncrementalFixedLagSmoother_update.invoke(
                        ptr, newFactors.ptr, newTheta.ptr, timestamps.ptr));
    }

    public FixedLagSmoother.Result update(
            NonlinearFactorGraph newFactors,
            Values newTheta,
            FixedLagSmoother.KeyTimestampMap timestamps,
            FactorIndices indices) {
        return new FixedLagSmoother.Result(
                (MemorySegment) FF.IncrementalFixedLagSmoother_updateFactorIndices.invoke(
                        ptr, newFactors.ptr, newTheta.ptr, timestamps.ptr, indices.ptr));
    }

    public Values calculateEstimate() {
        return Values.owned((MemorySegment) FF.IncrementalFixedLagSmoother_calculateEstimate.invoke(ptr));
    }

    public Point2 calculateEstimatePoint2(Key key) {
        return new Point2(
                (MemorySegment) FF.IncrementalFixedLagSmoother_calculateEstimatePoint2.invoke(ptr, key.j));
    }

    public NonlinearFactorGraph getFactors() {
        return new NonlinearFactorGraph((MemorySegment) FF.IncrementalFixedLagSmoother_getFactors.invoke(ptr));
    }

    public ISAM2Result getISAM2Result() {
        return new ISAM2Result((MemorySegment) FF.IncrementalFixedLagSmoother_getISAM2Result.invoke(ptr));
    }

    public Values getLinearizationPoint() {
        return Values.observed((MemorySegment) FF.IncrementalFixedLagSmoother_getLinearizationPoint.invoke(ptr));
    }

    public ISAM2 getISAM2() {
        return new ISAM2((MemorySegment) FF.IncrementalFixedLagSmoother_getISAM2.invoke(ptr));

    }

}

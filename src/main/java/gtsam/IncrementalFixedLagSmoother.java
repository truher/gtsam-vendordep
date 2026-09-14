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

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    IncrementalFixedLagSmoother(MemorySegment pointer) {
        super(pointer, FF.IncrementalFixedLagSmoother_delete.h);
    }

    public IncrementalFixedLagSmoother(double lag,
            ISAM2Params params) throws Throwable {
        this((MemorySegment) FF.IncrementalFixedLagSmoother.h.invokeExact(
                lag, params.ptr));
    }

    public FixedLagSmoother.Result update(
            NonlinearFactorGraph newFactors,
            Values newTheta,
            FixedLagSmoother.KeyTimestampMap timestamps) throws Throwable {
        return new FixedLagSmoother.Result(
                (MemorySegment) FF.IncrementalFixedLagSmoother_update.h.invokeExact(
                        ptr, newFactors.ptr, newTheta.ptr, timestamps.ptr));
    }

    public FixedLagSmoother.Result update(
            NonlinearFactorGraph newFactors,
            Values newTheta,
            FixedLagSmoother.KeyTimestampMap timestamps,
            FactorIndices indices) throws Throwable {
        return new FixedLagSmoother.Result(
                (MemorySegment) FF.IncrementalFixedLagSmoother_updateFactorIndices.h.invokeExact(
                        ptr, newFactors.ptr, newTheta.ptr, timestamps.ptr, indices.ptr));
    }

    public Values calculateEstimate() throws Throwable {
        return Values.owned((MemorySegment) FF.IncrementalFixedLagSmoother_calculateEstimate.h.invokeExact(ptr));
    }

    public Point2 calculateEstimatePoint2(Key key) throws Throwable {
        return new Point2(
                (MemorySegment) FF.IncrementalFixedLagSmoother_calculateEstimatePoint2.h.invokeExact(ptr, key.j));
    }

    public NonlinearFactorGraph getFactors() throws Throwable {
        return new NonlinearFactorGraph((MemorySegment) FF.IncrementalFixedLagSmoother_getFactors.h.invokeExact(ptr));
    }

    public ISAM2Result getISAM2Result() throws Throwable {
        return new ISAM2Result((MemorySegment) FF.IncrementalFixedLagSmoother_getISAM2Result.h.invokeExact(ptr));
    }

    public Values getLinearizationPoint() throws Throwable {
        return Values.observed((MemorySegment) FF.IncrementalFixedLagSmoother_getLinearizationPoint.h.invokeExact(ptr));
    }

    public ISAM2 getISAM2() throws Throwable {
        return new ISAM2((MemorySegment) FF.IncrementalFixedLagSmoother_getISAM2.h.invokeExact(ptr));

    }

}

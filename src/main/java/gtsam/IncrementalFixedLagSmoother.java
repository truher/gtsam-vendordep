package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class IncrementalFixedLagSmoother extends FixedLagSmoother {
    public enum FF {
        IncrementalFixedLagSmoother_delete(null, ADDRESS),
        IncrementalFixedLagSmoother(ADDRESS, JAVA_DOUBLE, ADDRESS),
        IncrementalFixedLagSmoother_calculateEstimatePoint2(ADDRESS, ADDRESS, JAVA_LONG),
        IncrementalFixedLagSmoother_calculateEstimatePose2(ADDRESS, ADDRESS, JAVA_LONG),
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

    public Point2 calculateEstimatePoint2(Key key) throws Throwable {
        return new Point2(
                (MemorySegment) FF.IncrementalFixedLagSmoother_calculateEstimatePoint2.h.invokeExact(ptr, key.j));
    }

    public Pose2 calculateEstimatePose2(Key key) throws Throwable {
        return new Pose2(
                (MemorySegment) FF.IncrementalFixedLagSmoother_calculateEstimatePose2.h.invokeExact(ptr, key.j));
    }

    // This method is not an override in C++ but it probably should be here.
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

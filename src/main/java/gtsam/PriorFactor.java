package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class PriorFactor<T> extends NonlinearFactor {
    public enum FF {
        PriorFactorDouble(ADDRESS, JAVA_LONG, JAVA_DOUBLE, ADDRESS),
        PriorFactorPose2(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        PriorFactorPose3(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        PriorFactorCal3DS2(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        PriorFactorDouble_delete(null, ADDRESS),
        PriorFactorPose2_delete(null, ADDRESS),
        PriorFactorPose3_delete(null, ADDRESS),
        PriorFactorCal3DS2_delete(null, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    /** @param p pointer to the factor itself, not the shared_ptr. */
    private PriorFactor(MemorySegment p) {
        super(p);
    }

      public static shared_ptr<PriorFactor<Double>> PriorFactorDouble(
            Key poseKey, double prior, SharedNoiseModel model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.PriorFactorPose2.h.invokeExact(
                poseKey.j, prior, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, PriorFactor::new, FF.PriorFactorDouble_delete.h);
    }

    /** @param prior is copied, ok to delete. */
    public static shared_ptr<PriorFactor<Pose2>> PriorFactorPose2(
            Key poseKey, Pose2 prior, SharedNoiseModel model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.PriorFactorPose2.h.invokeExact(
                poseKey.j, prior.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, PriorFactor::new, FF.PriorFactorPose2_delete.h);
    }

    /** @param prior is copied, ok to delete. */
    public static shared_ptr<PriorFactor<Pose3>> PriorFactorPose3(
            Key key, Pose3 prior, SharedNoiseModel model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.PriorFactorPose3.h.invokeExact(
                key.j, prior.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, PriorFactor::new, FF.PriorFactorPose3_delete.h);

    }

    /** @param prior is copied, ok to delete. */
    public static shared_ptr<PriorFactor<Cal3DS2>> PriorFactorCal3DS2(
            Key key, Cal3DS2 prior, SharedNoiseModel model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.PriorFactorCal3DS2.h.invokeExact(
                key.j, prior.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, PriorFactor::new, FF.PriorFactorCal3DS2_delete.h);
    }
}

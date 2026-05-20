package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class PriorFactor<T> extends NonlinearFactor {
    private static final MethodHandle PriorFactorPose2 = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("PriorFactorPose2"),
            FunctionDescriptor.of(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS));
    private static final MethodHandle PriorFactorPose3 = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("PriorFactorPose3"),
            FunctionDescriptor.of(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS));
    private static final MethodHandle PriorFactorCal3DS2 = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("PriorFactorCal3DS2"),
            FunctionDescriptor.of(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS));

    /** @param p pointer to the factor itself, not the shared_ptr. */
    private PriorFactor(MemorySegment p) {
        super(p);
    }

    /** @param prior is copied, ok to delete. */
    public static shared_ptr<PriorFactor<Pose2>> PriorFactorPose2(
            Key poseKey, Pose2 prior, SharedNoiseModel model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) PriorFactorPose2.invokeExact(
                poseKey.j, prior.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, PriorFactor::new);
    }

    /** @param prior is copied, ok to delete. */
    public static shared_ptr<PriorFactor<Pose3>> PriorFactorPose3(
            Key key, Pose3 prior, SharedNoiseModel model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) PriorFactorPose3.invokeExact(
                key.j, prior.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, PriorFactor::new);

    }

    /** @param prior is copied, ok to delete. */
    public static shared_ptr<PriorFactor<Cal3DS2>> PriorFactorCal3DS2(
            Key key, Cal3DS2 prior, SharedNoiseModel model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) PriorFactorCal3DS2.invokeExact(
                key.j, prior.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, PriorFactor::new);
    }
}

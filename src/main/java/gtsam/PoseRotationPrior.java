package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class PoseRotationPrior<T> extends NonlinearFactor {
    private static final MethodHandle PoseRotationPriorPose2 = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("PoseRotationPriorPose2"),
            FunctionDescriptor.of(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS));

    /** @param p pointer to the factor itself, not the shared_ptr. */
    private PoseRotationPrior(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<PoseRotationPrior<Pose2>> PoseRotationPriorPose2(
            Key k, Pose2 p, SharedNoiseModel model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) PoseRotationPriorPose2.invokeExact(
                k.j, p.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, PoseRotationPrior::new);
    }

}

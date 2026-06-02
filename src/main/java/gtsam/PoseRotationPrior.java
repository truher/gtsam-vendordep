package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class PoseRotationPrior<T> extends NonlinearFactor {
    public enum FF {
        PoseRotationPriorPose2(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        PoseRotationPriorPose2_delete(null, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    /** @param p pointer to the factor itself, not the shared_ptr. */
    private PoseRotationPrior(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<PoseRotationPrior<Pose2>> PoseRotationPriorPose2(
            Key k,
            Pose2 p,
            shared_ptr<? extends gtsam.noiseModel.Base> model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.PoseRotationPriorPose2.h.invokeExact(
                k.j, p.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, PoseRotationPrior::new, FF.PoseRotationPriorPose2_delete.h);
    }

}

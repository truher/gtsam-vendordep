package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class PoseRotationPrior<T> extends NonlinearFactor {
    public enum FF {
        PoseRotationPriorPose2(ADDRESS, JAVA_LONG, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    /** @param p pointer to the factor itself, not the shared_ptr. */
    private PoseRotationPrior(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<PoseRotationPrior<Pose2>> PoseRotationPriorPose2(
            Key k,
            Pose2 p,
            shared_ptr<? extends gtsam.noiseModel.Base> model) {
        return new shared_ptr<>(
                (MemorySegment) FF.PoseRotationPriorPose2.invoke(k.j, p.ptr, model.ptr),
                PoseRotationPrior::new);
    }

}

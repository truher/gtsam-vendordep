package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class BetweenFactorPose2 extends NoiseModelFactor {

    public enum FF {
        BetweenFactorPose2(ADDRESS, JAVA_LONG, JAVA_LONG, ADDRESS, ADDRESS),
        BetweenFactorPose2_error(JAVA_DOUBLE, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    BetweenFactorPose2(MemorySegment p) {
        super(p);
    }

    /** @param measured is copied, ok to delete */
    public static shared_ptr<BetweenFactorPose2> newBetweenFactorPose2(
            Key key1,
            Key key2,
            Pose2 measured,
            shared_ptr<? extends gtsam.noiseModel.Base> model) {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.BetweenFactorPose2.invoke(
                key1.j, key2.j, measured.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, BetweenFactorPose2::new);
    }

    public double error(Values v) {
        return (double) FF.BetweenFactorPose2_error.invoke(ptr, v.ptr);
    }
}

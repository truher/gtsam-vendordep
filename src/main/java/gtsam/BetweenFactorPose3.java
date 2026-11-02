package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class BetweenFactorPose3 extends NoiseModelFactor {

    public enum FF {
        BetweenFactorPose3(ADDRESS, JAVA_LONG, JAVA_LONG, ADDRESS, ADDRESS),
        BetweenFactorPose3_error(JAVA_DOUBLE, ADDRESS, ADDRESS),
        BetweenFactorPose3_evaluateError(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        BetweenFactorPose3_evaluateErrorH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    BetweenFactorPose3(MemorySegment p) {
        super(p);
    }

    /** @param measured is copied, ok to delete */
    public static shared_ptr<BetweenFactorPose3> newBetweenFactorPose3(
            Key key1,
            Key key2,
            Pose3 measured,
            shared_ptr<? extends gtsam.noiseModel.Base> model) {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.BetweenFactorPose3.invoke(
                key1.j, key2.j, measured.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, BetweenFactorPose3::new);
    }

    public double error(Values v) {
        return (double) FF.BetweenFactorPose3_error.invoke(ptr, v.ptr);
    }

    public Vector6 evaluateError(Pose3 R1, Pose3 R2) {
        return new Vector6(
                (MemorySegment) FF.BetweenFactorPose3_evaluateError.invoke(ptr, R1.ptr, R2.ptr));
    }

    public Vector6 evaluateError(Pose3 R1, Pose3 R2, Matrix H1, Matrix H2) {
        return new Vector6(
                (MemorySegment) FF.BetweenFactorPose3_evaluateErrorH.invoke(
                        ptr, R1.ptr, R2.ptr, H1.ptr, H2.ptr));
    }
}

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


        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
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
            shared_ptr<? extends gtsam.noiseModel.Base> model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.BetweenFactorPose3.h.invokeExact(
                key1.j, key2.j, measured.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, BetweenFactorPose3::new);
    }

    public double error(Values v) throws Throwable {
        return (double) FF.BetweenFactorPose3_error.h.invokeExact(ptr, v.ptr);
    }

    public Vector6 evaluateError(Pose3 R1, Pose3 R2) throws Throwable {
        return new Vector6(
                (MemorySegment) FF.BetweenFactorPose3_evaluateError.h.invokeExact(ptr, R1.ptr, R2.ptr));
    }

    public Vector6 evaluateError(Pose3 R1, Pose3 R2, Matrix H1, Matrix H2) throws Throwable {
        return new Vector6(
                (MemorySegment) FF.BetweenFactorPose3_evaluateErrorH.h.invokeExact(
                        ptr, R1.ptr, R2.ptr, H1.ptr, H2.ptr));
    }
}

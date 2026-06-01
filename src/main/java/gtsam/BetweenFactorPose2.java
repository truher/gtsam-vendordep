package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

// TODO: make this generic
// TODO: make a superclass, NoiseModelFactor, for error().
public class BetweenFactorPose2 extends NonlinearFactor {

    public enum FF {
        BetweenFactorPose2(ADDRESS, JAVA_LONG, JAVA_LONG, ADDRESS, ADDRESS),
        BetweenFactorPose2_delete(null, ADDRESS),
        BetweenFactorPose2_error(JAVA_DOUBLE, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    BetweenFactorPose2(MemorySegment p) {
        super(p);
    }

    /** @param measured is copied, ok to delete */
    public static shared_ptr<BetweenFactorPose2> newBetweenFactorPose2(
            Key key1, Key key2, Pose2 measured, SharedNoiseModel model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.BetweenFactorPose2.h.invokeExact(
                key1.j, key2.j, measured.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, BetweenFactorPose2::new, FF.BetweenFactorPose2_delete.h);
    }

    public double error(Values v) throws Throwable {
        return (double) FF.BetweenFactorPose2_error.h.invokeExact(ptr, v.ptr);
    }
}

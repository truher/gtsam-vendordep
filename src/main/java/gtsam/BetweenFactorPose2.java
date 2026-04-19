package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

// TODO: make this generic
public class BetweenFactorPose2 extends NonlinearFactor {
    private static final MethodHandle BetweenFactorPose2 = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("BetweenFactorPose2"),
            FunctionDescriptor.of(ADDRESS, JAVA_LONG, JAVA_LONG, ADDRESS, ADDRESS));
    // TODO: make a superclass, NoiseModelFactor, for error().
    private static final MethodHandle BetweenFactorPose2_error = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("BetweenFactorPose2_error"),
            FunctionDescriptor.of(JAVA_DOUBLE, ADDRESS, ADDRESS));

    BetweenFactorPose2(MemorySegment p) {
        super(p);
    }

    /** @param measured is copied, ok to delete */
    public static shared_ptr<BetweenFactorPose2> newBetweenFactorPose2(
            Key key1, Key key2, Pose2 measured, SharedNoiseModel model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) BetweenFactorPose2.invokeExact(
                key1.j, key2.j, measured.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, BetweenFactorPose2::new);
    }

    public double error(Values v) throws Throwable {
        return (double) BetweenFactorPose2_error.invokeExact(ptr, v.ptr);
    }
}

package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class BearingRangeFactorPose2Point2 extends NoiseModelFactor {
    public enum FF {
        BearingRangeFactorPose2Point2(ADDRESS, JAVA_LONG, JAVA_LONG, ADDRESS, JAVA_DOUBLE, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    BearingRangeFactorPose2Point2(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<BearingRangeFactorPose2Point2> newBearingRangeFactorPose2Point2(
            Key key1,
            Key key2,
            Rot2 bearing,
            double range,
            shared_ptr<? extends gtsam.noiseModel.Base> model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.BearingRangeFactorPose2Point2.h.invokeExact(
                key1.j, key2.j, bearing.ptr, range, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, BearingRangeFactorPose2Point2::new);
    }

}

package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class BetweenFactorPoint2 extends NoiseModelFactor {

    public enum FF {
        BetweenFactorPoint2(ADDRESS, JAVA_LONG, JAVA_LONG, ADDRESS, ADDRESS),
        BetweenFactorPoint2_evaluateError(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        BetweenFactorPoint2_evaluateErrorH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    BetweenFactorPoint2(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<BetweenFactorPoint2> newBetweenFactorPoint2(
            Key key1,
            Key key2,
            Point2 measured,
            shared_ptr<? extends gtsam.noiseModel.Base> model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.BetweenFactorPoint2.h.invokeExact(
                key1.j, key2.j, measured.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, BetweenFactorPoint2::new);
    }

    public Vector2 evaluateError(Point2 R1, Point2 R2) throws Throwable {
        return new Vector2(
                (MemorySegment) FF.BetweenFactorPoint2_evaluateError.h.invokeExact(ptr, R1.ptr, R2.ptr));
    }

    public Vector2 evaluateError(Point2 R1, Point2 R2, Matrix H1, Matrix H2) throws Throwable {
        return new Vector2(
                (MemorySegment) FF.BetweenFactorPoint2_evaluateErrorH.h.invokeExact(
                        ptr, R1.ptr, R2.ptr, H1.ptr, H2.ptr));
    }
}

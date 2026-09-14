package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class BetweenFactorPoint3 extends NoiseModelFactor {

    public enum FF {
        BetweenFactorPoint3(ADDRESS, JAVA_LONG, JAVA_LONG, ADDRESS, ADDRESS),
        BetweenFactorPoint3_evaluateError(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        BetweenFactorPoint3_evaluateErrorH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    BetweenFactorPoint3(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<BetweenFactorPoint3> newBetweenFactorPoint3(
            Key key1,
            Key key2,
            Point3 measured,
            shared_ptr<? extends gtsam.noiseModel.Base> model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.BetweenFactorPoint3.h.invokeExact(
                key1.j, key2.j, measured.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, BetweenFactorPoint3::new);
    }

    public Vector3 evaluateError(Point3 R1, Point3 R2) throws Throwable {
        return new Vector3(
                (MemorySegment) FF.BetweenFactorPoint3_evaluateError.h.invokeExact(ptr, R1.ptr, R2.ptr));
    }

    public Vector3 evaluateError(Point3 R1, Point3 R2, Matrix H1, Matrix H2) throws Throwable {
        return new Vector3(
                (MemorySegment) FF.BetweenFactorPoint3_evaluateErrorH.h.invokeExact(
                        ptr, R1.ptr, R2.ptr, H1.ptr, H2.ptr));
    }
}

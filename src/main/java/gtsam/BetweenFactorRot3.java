package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class BetweenFactorRot3 extends NoiseModelFactor {

    public enum FF {
        BetweenFactorRot3(ADDRESS, JAVA_LONG, JAVA_LONG, ADDRESS, ADDRESS),
        BetweenFactorRot3_evaluateError(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        BetweenFactorRot3_evaluateErrorH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    BetweenFactorRot3(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<BetweenFactorRot3> newBetweenFactorRot3(
            Key key1,
            Key key2,
            Rot3 measured,
            shared_ptr<? extends gtsam.noiseModel.Base> model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.BetweenFactorRot3.h.invokeExact(
                key1.j, key2.j, measured.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, BetweenFactorRot3::new);
    }

    public Vector3 evaluateError(Rot3 R1, Rot3 R2) throws Throwable {
        return new Vector3(
                (MemorySegment) FF.BetweenFactorRot3_evaluateError.h.invokeExact(ptr, R1.ptr, R2.ptr));
    }

    public Vector3 evaluateError(Rot3 R1, Rot3 R2, Matrix H1, Matrix H2) throws Throwable {
        return new Vector3(
                (MemorySegment) FF.BetweenFactorRot3_evaluateErrorH.h.invokeExact(
                        ptr, R1.ptr, R2.ptr, H1.ptr, H2.ptr));
    }
}

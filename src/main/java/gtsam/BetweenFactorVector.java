package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class BetweenFactorVector extends NoiseModelFactor {

    public enum FF {
        BetweenFactorVector(ADDRESS, JAVA_LONG, JAVA_LONG, ADDRESS, ADDRESS),
        BetweenFactorVector_evaluateError(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        BetweenFactorVector_evaluateErrorH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    BetweenFactorVector(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<BetweenFactorVector> newBetweenFactorVector(
            Key key1,
            Key key2,
            Vector measured,
            shared_ptr<? extends gtsam.noiseModel.Base> model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.BetweenFactorVector.h.invokeExact(
                key1.j, key2.j, measured.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, BetweenFactorVector::new);
    }

    public Vector evaluateError(Vector R1, Vector R2) throws Throwable {
        return new Vector(
                (MemorySegment) FF.BetweenFactorVector_evaluateError.h.invokeExact(ptr, R1.ptr, R2.ptr));
    }

    public Vector evaluateError(Vector R1, Vector R2, Matrix H1, Matrix H2) throws Throwable {
        return new Vector(
                (MemorySegment) FF.BetweenFactorVector_evaluateErrorH.h.invokeExact(
                        ptr, R1.ptr, R2.ptr, H1.ptr, H2.ptr));
    }
}

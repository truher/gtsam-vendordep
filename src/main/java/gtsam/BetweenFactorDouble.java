package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class BetweenFactorDouble extends NoiseModelFactor {

    public enum FF {
        BetweenFactorDouble(ADDRESS, JAVA_LONG, JAVA_LONG, JAVA_DOUBLE, ADDRESS),
        BetweenFactorDouble_evaluateError(ADDRESS, ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE),
        BetweenFactorDouble_evaluateErrorH(ADDRESS, ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    BetweenFactorDouble(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<BetweenFactorDouble> newBetweenFactorDouble(
            Key key1,
            Key key2,
            double measured,
            shared_ptr<? extends gtsam.noiseModel.Base> model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.BetweenFactorDouble.h.invokeExact(
                key1.j, key2.j, measured, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, BetweenFactorDouble::new);
    }

    public Vector1 evaluateError(double R1, double R2) throws Throwable {
        return new Vector1(
                (MemorySegment) FF.BetweenFactorDouble_evaluateError.h.invokeExact(ptr, R1, R2));
    }

    public Vector1 evaluateError(double R1, double R2, Matrix H1, Matrix H2) throws Throwable {
        return new Vector1(
                (MemorySegment) FF.BetweenFactorDouble_evaluateErrorH.h.invokeExact(
                        ptr, R1, R2, H1.ptr, H2.ptr));
    }
}

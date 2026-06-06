package gtsam.noiseModel.mEstimator;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Base extends ForeignObject {
    public enum FF {
        noiseModel_mEstimator_Base_weight(JAVA_DOUBLE, ADDRESS, JAVA_DOUBLE),
        noiseModel_mEstimator_Base_loss(JAVA_DOUBLE, ADDRESS, JAVA_DOUBLE);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Base(MemorySegment pointer) {
        super(pointer, null);
    }

    public double weight(double distance) throws Throwable {
        return (double) FF.noiseModel_mEstimator_Base_weight.h.invokeExact(ptr, distance);
    }

    public double loss(double distance) throws Throwable {
        return (double) FF.noiseModel_mEstimator_Base_loss.h.invokeExact(ptr, distance);
    }

}

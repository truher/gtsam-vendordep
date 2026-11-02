package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class NoiseModelFactor extends NonlinearFactor {

    public enum FF {
        NoiseModelFactor_weight(JAVA_DOUBLE, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    NoiseModelFactor(MemorySegment p) {
        super(p);
    }

    public double weight(Values v) {
        return (double) FF.NoiseModelFactor_weight.invoke(ptr, v.ptr);
    }

}

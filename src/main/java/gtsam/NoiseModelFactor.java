package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class NoiseModelFactor extends NonlinearFactor {

       public enum FF {
        NoiseModelFactor_weight(JAVA_DOUBLE, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    NoiseModelFactor(MemorySegment p) {
        super(p);
    }

      public double weight(Values v) throws Throwable {
        return (double) FF.NoiseModelFactor_weight.h.invokeExact(ptr, v.ptr);
    }
    
}

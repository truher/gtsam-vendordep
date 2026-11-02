package gtsam.noiseModel.mEstimator;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.shared_ptr;

public class Tukey extends Base {

    public enum FF {
        noiseModel_mEstimator_Tukey_Create(ADDRESS, JAVA_DOUBLE);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Tukey(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Tukey> Create(double k) {
        return new shared_ptr<Tukey>(
                (MemorySegment) FF.noiseModel_mEstimator_Tukey_Create.invoke(k),
                Tukey::new);
    }
}

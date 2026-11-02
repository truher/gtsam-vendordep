package gtsam.noiseModel.mEstimator;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.shared_ptr;

public class Welsch extends Base {

    public enum FF {
        noiseModel_mEstimator_Welsch_Create(ADDRESS, JAVA_DOUBLE);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Welsch(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Welsch> Create(double k) {
        return new shared_ptr<Welsch>(
                (MemorySegment) FF.noiseModel_mEstimator_Welsch_Create.invoke(k),
                Welsch::new);
    }
}

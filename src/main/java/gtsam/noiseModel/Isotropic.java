package gtsam.noiseModel;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.shared_ptr;

public class Isotropic extends Diagonal {
    public enum FF {
        noiseModel_Isotropic_Sigma(ADDRESS, JAVA_INT, JAVA_DOUBLE, JAVA_BOOLEAN),
        noiseModel_Isotropic_Variance(ADDRESS, JAVA_INT, JAVA_DOUBLE, JAVA_BOOLEAN),
        noiseModel_Isotropic_Precision(ADDRESS, JAVA_INT, JAVA_DOUBLE, JAVA_BOOLEAN);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Isotropic(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Isotropic> Sigma(int dim, double sigma, boolean smart) {
        MemorySegment p = (MemorySegment) FF.noiseModel_Isotropic_Sigma.invoke(dim, sigma, smart);
        if (p.equals(MemorySegment.NULL))
            throw new IllegalArgumentException();
        return new shared_ptr<Isotropic>(p, Isotropic::new);
    }

    public static shared_ptr<Isotropic> Sigma(int dim, double sigma) {
        MemorySegment p = (MemorySegment) FF.noiseModel_Isotropic_Sigma.invoke(dim, sigma, true);
        if (p.equals(MemorySegment.NULL))
            throw new IllegalArgumentException();
        return new shared_ptr<Isotropic>(p, Isotropic::new);
    }

    public static shared_ptr<Isotropic> Variance(int dim, double variance, boolean smart) {
        MemorySegment p = (MemorySegment) FF.noiseModel_Isotropic_Variance.invoke(dim, variance, smart);
        if (p.equals(MemorySegment.NULL))
            throw new IllegalArgumentException();
        return new shared_ptr<Isotropic>(p, Isotropic::new);
    }

    public static shared_ptr<Isotropic> Precision(int dim, double precision, boolean smart) {
        return new shared_ptr<Isotropic>(
                (MemorySegment) FF.noiseModel_Isotropic_Precision.invoke(dim, precision, smart),
                Isotropic::new);
    }

}

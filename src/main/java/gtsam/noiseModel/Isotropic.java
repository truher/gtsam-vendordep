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

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Isotropic(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Isotropic> Sigma(int dim, double sigma, boolean smart) throws Throwable {
        MemorySegment p = (MemorySegment) FF.noiseModel_Isotropic_Sigma.h.invokeExact(dim, sigma, smart);
        if (p.equals(MemorySegment.NULL))
            throw new IllegalArgumentException();
        return new shared_ptr<Isotropic>(p, Isotropic::new);
    }

    public static shared_ptr<Isotropic> Variance(int dim, double variance, boolean smart) throws Throwable {
        MemorySegment p = (MemorySegment) FF.noiseModel_Isotropic_Variance.h.invokeExact(dim, variance, smart);
        if (p.equals(MemorySegment.NULL))
            throw new IllegalArgumentException();
        return new shared_ptr<Isotropic>(p, Isotropic::new);
    }

    public static shared_ptr<Isotropic> Precision(int dim, double precision, boolean smart) throws Throwable {
        return new shared_ptr<Isotropic>(
                (MemorySegment) FF.noiseModel_Isotropic_Precision.h.invokeExact(dim, precision, smart),
                Isotropic::new);
    }

}

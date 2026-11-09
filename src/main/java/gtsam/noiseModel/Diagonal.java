package gtsam.noiseModel;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.Vector;
import gtsam.Vector1;
import gtsam.Vector2;
import gtsam.Vector3;
import gtsam.shared_ptr;

/**
 * "smart" means to return diagonal or isotropic or constrained depending on the
 * arguments.
 */
public class Diagonal extends Gaussian {
    public enum FF {
        noiseModel_Diagonal_SigmasVector(ADDRESS, ADDRESS, JAVA_BOOLEAN),
        noiseModel_Diagonal_SigmasVector1(ADDRESS, ADDRESS, JAVA_BOOLEAN),
        noiseModel_Diagonal_SigmasVector2(ADDRESS, ADDRESS, JAVA_BOOLEAN),
        noiseModel_Diagonal_SigmasVector3(ADDRESS, ADDRESS, JAVA_BOOLEAN),
        noiseModel_Diagonal_VariancesVector3(ADDRESS, ADDRESS, JAVA_BOOLEAN),
        noiseModel_Diagonal_PrecisionsVector3(ADDRESS, ADDRESS, JAVA_BOOLEAN),
        noiseModel_Diagonal_invsigmas(ADDRESS, ADDRESS),
        noiseModel_Diagonal_precisions(ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Diagonal(MemorySegment p) {
        super(p);
    }

    /** @throws IllegalArgumentException if any sigmas are negative. */
    public static shared_ptr<Diagonal> Sigmas(Vector sigmas) {
        return Sigmas(sigmas, true);
    }

    /** @throws IllegalArgumentException if any sigmas are negative. */
    public static shared_ptr<Diagonal> Sigmas(Vector sigmas, boolean smart) {
        MemorySegment p = (MemorySegment) FF.noiseModel_Diagonal_SigmasVector.invoke(sigmas.ptr, smart);
        if (p.equals(MemorySegment.NULL))
            throw new IllegalArgumentException();
        return new shared_ptr<Diagonal>(p, Diagonal::new);
    }

    /** @throws IllegalArgumentException if any sigmas are negative. */
    public static shared_ptr<Diagonal> Sigmas(Vector1 sigmas, boolean smart) {
        MemorySegment p = (MemorySegment) FF.noiseModel_Diagonal_SigmasVector1.invoke(sigmas.ptr, smart);
        if (p.equals(MemorySegment.NULL))
            throw new IllegalArgumentException();
        return new shared_ptr<>(p, Diagonal::new);
    }

    /** @throws IllegalArgumentException if any sigmas are negative. */
    public static shared_ptr<Diagonal> Sigmas(Vector1 sigmas) {
        return Sigmas(sigmas, true);
    }

    /** @throws IllegalArgumentException if any sigmas are negative. */
    public static shared_ptr<Diagonal> Sigmas(Vector2 sigmas, boolean smart) {
        MemorySegment p = (MemorySegment) FF.noiseModel_Diagonal_SigmasVector2.invoke(sigmas.ptr, smart);
        if (p.equals(MemorySegment.NULL))
            throw new IllegalArgumentException();
        return new shared_ptr<>(p, Diagonal::new);
    }

    /** @throws IllegalArgumentException if any sigmas are negative. */
    public static shared_ptr<Diagonal> Sigmas(Vector2 sigmas) {
        return Sigmas(sigmas, true);
    }

    /** @throws IllegalArgumentException if any sigmas are negative. */
    public static shared_ptr<Diagonal> Sigmas(Vector3 sigmas, boolean smart) {
        MemorySegment p = (MemorySegment) FF.noiseModel_Diagonal_SigmasVector3.invoke(sigmas.ptr, smart);
        if (p.equals(MemorySegment.NULL))
            throw new IllegalArgumentException();
        return new shared_ptr<>(p, Diagonal::new);
    }

    /** @throws IllegalArgumentException if any sigmas are negative. */
    public static shared_ptr<Diagonal> Sigmas(Vector3 sigmas) {
        return Sigmas(sigmas, true);
    }

    /** @throws IllegalArgumentException if any sigmas are negative. */
    public static shared_ptr<Diagonal> Variances(Vector3 variances, boolean smart) {
        MemorySegment p = (MemorySegment) FF.noiseModel_Diagonal_VariancesVector3.invoke(variances.ptr, smart);
        if (p.equals(MemorySegment.NULL))
            throw new IllegalArgumentException();
        return new shared_ptr<>(p, Diagonal::new);
    }

    /** @throws IllegalArgumentException if any sigmas are negative. */
    public static shared_ptr<Diagonal> Precisions(Vector3 precisions, boolean smart) {
        MemorySegment p = (MemorySegment) FF.noiseModel_Diagonal_PrecisionsVector3.invoke(precisions.ptr, smart);
        if (p.equals(MemorySegment.NULL))
            throw new IllegalArgumentException();
        return new shared_ptr<>(p, Diagonal::new);
    }

    public Vector invsigmas() {
        return new Vector((MemorySegment) FF.noiseModel_Diagonal_invsigmas.invoke(ptr));
    }

    public Vector precisions() {
        return new Vector((MemorySegment) FF.noiseModel_Diagonal_precisions.invoke(ptr));
    }
}

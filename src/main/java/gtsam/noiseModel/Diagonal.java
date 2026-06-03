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
        noiseModel_Diagonal_PrecisionsVector3(ADDRESS, ADDRESS, JAVA_BOOLEAN);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Diagonal(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Diagonal> Sigmas(Vector sigmas) throws Throwable {
        return Sigmas(sigmas, true);
    }

    public static shared_ptr<Diagonal> Sigmas(Vector sigmas, boolean smart) throws Throwable {
        return new shared_ptr<Diagonal>(
                (MemorySegment) FF.noiseModel_Diagonal_SigmasVector.h.invokeExact(sigmas.ptr, smart),
                Diagonal::new);
    }

    public static shared_ptr<Diagonal> Sigmas(Vector1 sigmas, boolean smart) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.noiseModel_Diagonal_SigmasVector1.h.invokeExact(sigmas.ptr, smart),
                Diagonal::new);
    }

    public static shared_ptr<Diagonal> Sigmas(Vector1 sigmas) throws Throwable {
        return Sigmas(sigmas, true);
    }

    public static shared_ptr<Diagonal> Sigmas(Vector2 sigmas, boolean smart) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.noiseModel_Diagonal_SigmasVector2.h.invokeExact(sigmas.ptr, smart),
                Diagonal::new);
    }

    public static shared_ptr<Diagonal> Sigmas(Vector2 sigmas) throws Throwable {
        return Sigmas(sigmas, true);
    }

    public static shared_ptr<Diagonal> Sigmas(Vector3 sigmas, boolean smart) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.noiseModel_Diagonal_SigmasVector3.h.invokeExact(sigmas.ptr, smart),
                Diagonal::new);
    }

    public static shared_ptr<Diagonal> Sigmas(Vector3 sigmas) throws Throwable {
        return Sigmas(sigmas, true);
    }

    public static shared_ptr<Diagonal> Variances(Vector3 variances, boolean smart) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.noiseModel_Diagonal_VariancesVector3.h.invokeExact(variances.ptr, smart),
                Diagonal::new);
    }

    public static shared_ptr<Diagonal> Precisions(Vector3 precisions, boolean smart) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.noiseModel_Diagonal_PrecisionsVector3.h.invokeExact(precisions.ptr, smart),
                Diagonal::new);
    }
}

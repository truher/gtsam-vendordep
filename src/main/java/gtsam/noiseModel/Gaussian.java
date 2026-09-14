package gtsam.noiseModel;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.Matrix;
import gtsam.Vector;
import gtsam.shared_ptr;

public class Gaussian extends Base {
    public enum FF {
        noiseModel_Gaussian_SqrtInformation(ADDRESS, ADDRESS, JAVA_BOOLEAN),
        noiseModel_Gaussian_Covariance(ADDRESS, ADDRESS, JAVA_BOOLEAN),
        noiseModel_Gaussian_Information(ADDRESS, ADDRESS, JAVA_BOOLEAN),
        noiseModel_Gaussian_sigmas(ADDRESS, ADDRESS),
        noiseModel_Gaussian_whiten(ADDRESS, ADDRESS, ADDRESS),
        noiseModel_Gaussian_unwhiten(ADDRESS, ADDRESS, ADDRESS),
        noiseModel_Gaussian_R(ADDRESS, ADDRESS),
        noiseModel_Gaussian_covariance(ADDRESS, ADDRESS),
        noiseModel_Gaussian_information(ADDRESS, ADDRESS),
        noiseModel_Gaussian_Whiten(ADDRESS, ADDRESS, ADDRESS),
        noiseModel_Gaussian_WhitenInPlace(null, ADDRESS, ADDRESS),
        noiseModel_Gaussian_QR(ADDRESS, ADDRESS, ADDRESS),
        noiseModel_Gaussian_negLogConstant(JAVA_DOUBLE, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Gaussian(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Gaussian> SqrtInformation(Matrix R, boolean smart) throws Throwable {
        return new shared_ptr<Gaussian>(
                (MemorySegment) FF.noiseModel_Gaussian_SqrtInformation.h.invokeExact(R.ptr, smart),
                Gaussian::new);
    }

    public static shared_ptr<Gaussian> Covariance(Matrix covariance, boolean smart) throws Throwable {
        return new shared_ptr<Gaussian>(
                (MemorySegment) FF.noiseModel_Gaussian_Covariance.h.invokeExact(covariance.ptr, smart),
                Gaussian::new);
    }

    public static shared_ptr<Gaussian> Information(Matrix M, boolean smart) throws Throwable {
        return new shared_ptr<Gaussian>(
                (MemorySegment) FF.noiseModel_Gaussian_Information.h.invokeExact(M.ptr, smart),
                Gaussian::new);
    }

    @Override
    public Vector sigmas() throws Throwable {
        return new Vector((MemorySegment) FF.noiseModel_Gaussian_sigmas.h.invokeExact(ptr));
    }

    @Override
    public Vector whiten(Vector v) throws Throwable {
        return new Vector((MemorySegment) FF.noiseModel_Gaussian_whiten.h.invokeExact(ptr, v.ptr));
    }

    @Override
    public Vector unwhiten(Vector v) throws Throwable {
        return new Vector((MemorySegment) FF.noiseModel_Gaussian_unwhiten.h.invokeExact(ptr, v.ptr));
    }

 
    public Matrix R() throws Throwable {
        return new Matrix((MemorySegment) FF.noiseModel_Gaussian_R.h.invokeExact(ptr));
    }

    public Matrix covariance() throws Throwable {
        return new Matrix((MemorySegment) FF.noiseModel_Gaussian_covariance.h.invokeExact(ptr));
    }

    public Matrix information() throws Throwable {
        return new Matrix((MemorySegment) FF.noiseModel_Gaussian_information.h.invokeExact(ptr));
    }

    @Override
    public Matrix Whiten(Matrix H) throws Throwable {
        return new Matrix((MemorySegment) FF.noiseModel_Gaussian_Whiten.h.invokeExact(ptr, H.ptr));
    }

    public void WhitenInPlace(Matrix H) throws Throwable {
        FF.noiseModel_Gaussian_WhitenInPlace.h.invokeExact(ptr, H.ptr);
    }

    public shared_ptr<Diagonal> QR(Matrix Ab) throws Throwable {
        return new shared_ptr<Diagonal>(
                (MemorySegment) FF.noiseModel_Gaussian_QR.h.invokeExact(ptr, Ab.ptr),
                Diagonal::new);
    }

    public double negLogConstant() throws Throwable {
        return (double) FF.noiseModel_Gaussian_negLogConstant.h.invokeExact(ptr);
    }
}

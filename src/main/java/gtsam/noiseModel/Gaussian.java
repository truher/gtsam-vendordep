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

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Gaussian(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Gaussian> SqrtInformation(Matrix R, boolean smart) {
        return new shared_ptr<Gaussian>(
                (MemorySegment) FF.noiseModel_Gaussian_SqrtInformation.invoke(R.ptr, smart),
                Gaussian::new);
    }

    public static shared_ptr<Gaussian> Covariance(Matrix covariance, boolean smart) {
        return new shared_ptr<Gaussian>(
                (MemorySegment) FF.noiseModel_Gaussian_Covariance.invoke(covariance.ptr, smart),
                Gaussian::new);
    }

    public static shared_ptr<Gaussian> Information(Matrix M, boolean smart) {
        return new shared_ptr<Gaussian>(
                (MemorySegment) FF.noiseModel_Gaussian_Information.invoke(M.ptr, smart),
                Gaussian::new);
    }

    @Override
    public Vector sigmas() {
        return new Vector((MemorySegment) FF.noiseModel_Gaussian_sigmas.invoke(ptr));
    }

    @Override
    public Vector whiten(Vector v) {
        return new Vector((MemorySegment) FF.noiseModel_Gaussian_whiten.invoke(ptr, v.ptr));
    }

    @Override
    public Vector unwhiten(Vector v) {
        return new Vector((MemorySegment) FF.noiseModel_Gaussian_unwhiten.invoke(ptr, v.ptr));
    }

    public Matrix R() {
        return new Matrix((MemorySegment) FF.noiseModel_Gaussian_R.invoke(ptr));
    }

    public Matrix covariance() {
        return new Matrix((MemorySegment) FF.noiseModel_Gaussian_covariance.invoke(ptr));
    }

    public Matrix information() {
        return new Matrix((MemorySegment) FF.noiseModel_Gaussian_information.invoke(ptr));
    }

    @Override
    public Matrix Whiten(Matrix H) {
        return new Matrix((MemorySegment) FF.noiseModel_Gaussian_Whiten.invoke(ptr, H.ptr));
    }

    public void WhitenInPlace(Matrix H) {
        FF.noiseModel_Gaussian_WhitenInPlace.invoke(ptr, H.ptr);
    }

    public shared_ptr<Diagonal> QR(Matrix Ab) {
        return new shared_ptr<Diagonal>(
                (MemorySegment) FF.noiseModel_Gaussian_QR.invoke(ptr, Ab.ptr),
                Diagonal::new);
    }

    public double negLogConstant() {
        return (double) FF.noiseModel_Gaussian_negLogConstant.invoke(ptr);
    }
}

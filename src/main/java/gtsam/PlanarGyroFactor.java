package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * See gtsam/navigation/PlanarGyroFactor.h
 */
public class PlanarGyroFactor extends ForeignObject {
    public static class PlanarGyroParams extends ForeignObject {
        public enum FF {
            PlanarGyroParams(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE),
            PlanarGyroParams_delete(null, ADDRESS),
            PlanarGyroParams_arwSigma(JAVA_DOUBLE, ADDRESS, JAVA_DOUBLE);

            public final MethodHandle h;

            FF(ValueLayout returnType, ValueLayout... parameterTypes) {
                h = Lib.ff(this, returnType, parameterTypes);
            }
        }

        /** @param p pointer to the factor itself, not the shared_ptr. */
        private PlanarGyroParams(MemorySegment p) {
            super(p, null);
        }

        public static shared_ptr<PlanarGyroParams> makeSharedPlanarGyroParams(
                double arw, double biasInstability)
                throws Throwable {
            MemorySegment sharedPtrPtr = (MemorySegment) FF.PlanarGyroParams.h.invokeExact(arw, biasInstability);
            return new shared_ptr<>(sharedPtrPtr, PlanarGyroParams::new, FF.PlanarGyroParams_delete.h);
        }

        public double arwSigma(double dt) throws Throwable {
            return (double) FF.PlanarGyroParams_arwSigma.h.invokeExact(ptr, dt);
        }

    }

    public enum FF {
        PlanarGyroFactor_delete(null, ADDRESS),
        PlanarGyroFactor_FromRotation(
                ADDRESS, JAVA_LONG, JAVA_LONG, JAVA_LONG, ADDRESS, ADDRESS, JAVA_DOUBLE),
        PlanarGyroFactor_FromRate(
                ADDRESS, JAVA_LONG, JAVA_LONG, JAVA_LONG, ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE),
        PlanarGyroFactor_deltaR(ADDRESS, ADDRESS, JAVA_DOUBLE, ADDRESS),
        PlanarGyroFactor_predict(ADDRESS, ADDRESS, ADDRESS, JAVA_DOUBLE, ADDRESS, ADDRESS),
        PlanarGyroFactor_computeError(JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS, JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS),
        PlanarGyroFactor_evaluateError(ADDRESS, ADDRESS, ADDRESS, ADDRESS, JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    /** @param p pointer to the factor itself, not the shared_ptr. */
    private PlanarGyroFactor(MemorySegment p) {
        super(p, null);
    }

    public static shared_ptr<PlanarGyroFactor> FromRotation(
            Key pose_i, Key pose_j, Key bias, shared_ptr<PlanarGyroParams> p, Rot2 dr, double dt) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.PlanarGyroFactor_FromRotation.h.invokeExact(
                pose_i.j, pose_j.j, bias.j, p.sharedPtrPtr, dr.ptr, dt);
        return new shared_ptr<>(sharedPtrPtr, PlanarGyroFactor::new, FF.PlanarGyroFactor_delete.h);
    }

    public static shared_ptr<PlanarGyroFactor> FromRate(
            Key pose_i, Key pose_j, Key bias, shared_ptr<PlanarGyroParams> p, double omega, double dt)
            throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.PlanarGyroFactor_FromRate.h.invokeExact(
                pose_i.j, pose_j.j, bias.j, p.sharedPtrPtr, omega, dt);
        return new shared_ptr<>(sharedPtrPtr, PlanarGyroFactor::new, FF.PlanarGyroFactor_delete.h);
    }

    public Rot2 deltaR(double bias, Matrix H) throws Throwable {
        return new Rot2((MemorySegment) FF.PlanarGyroFactor_deltaR.h.invokeExact(ptr, bias, H.ptr));
    }

    public Rot2 predict(Rot2 Ri, double bias, Matrix H1, Matrix H2) throws Throwable {
        return new Rot2((MemorySegment) FF.PlanarGyroFactor_predict.h.invokeExact(ptr, Ri.ptr, bias, H1.ptr, H2.ptr));
    }

    public double computeError(Rot2 Ri, Rot2 Rj, double bias, Matrix H1, Matrix H2, Matrix H3) throws Throwable {
        return (double) FF.PlanarGyroFactor_computeError.h.invokeExact(//
                ptr, Ri.ptr, Rj.ptr, bias, H1.ptr, H2.ptr, H3.ptr);
    }

    public Vector evaluateError(Pose2 Pi, Pose2 Pj, double bias, Matrix H1, Matrix H2, Matrix H3) throws Throwable {
        return new Vector((MemorySegment) FF.PlanarGyroFactor_evaluateError.h.invokeExact(//
                ptr, Pi.ptr, Pj.ptr, bias, H1.ptr, H2.ptr, H3.ptr));
    }
}

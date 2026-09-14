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
 * Gyro factor that functions like "Between" for just the yaw, and treats bias
 * as slowly changing.
 * 
 * Example:
 * 
 * {@snippet :
 * 
 * // Std dev of "angle random walk" noise
 * // The usual published measurement is stddev (σ), rad/√s.
 * // Typical value is 1e-4 rad/sqrt(s)
 * double arwSigma = 1e-4;
 * 
 * // Std dev of bias instability
 * // The usual published measurement is stddev (σ), rad/s.
 * // typical value is 3e-5 rad/s
 * double biasInstabilitySigma = 3e-5;
 * 
 * shared_ptr<PlanarGyroParams> params = PlanarGyroParams.makeSharedPlanarGyroParams( //
 *         arwSigma, biasInstabilitySigma);
 * 
 * // Measurement period in seconds
 * double dt = 0.5;
 * 
 * // Rotation between poses
 * Rot2 dr = new Rot2(0.05);
 * 
 * // Factor representing the difference in yaw of each pose, Key.X(n)
 * shared_ptr<PlanarGyroFactor> x = PlanarGyroFactor.FromRotation( //
 *         Key.X(0), Key.X(1), Key.B(0), params, dr, dt);
 * 
 * // Factor representing the difference in bias, Key.B(n)
 * shared_ptr<PlanarGyroBiasFactor> b = PlanarGyroBiasFactor.makeSharedPlanarGyroBiasFactor( //
 *         Key.B(0), Key.B(1), params);
 * 
 * graph.add(x);
 * graph.add(b);
 * 
 * }
 * 
 * See gtsam/navigation/PlanarGyroFactor.h
 */
public class PlanarGyroFactor extends NonlinearFactor {
    public static class PlanarGyroParams extends ForeignObject {
        public enum FF {
            PlanarGyroParams(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE),
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
            return new shared_ptr<>(
                    (MemorySegment) FF.PlanarGyroParams.h.invokeExact(arw, biasInstability),
                    PlanarGyroParams::new);
        }

        public double arwSigma(double dt) throws Throwable {
            return (double) FF.PlanarGyroParams_arwSigma.h.invokeExact(ptr, dt);
        }

    }

    public static class PlanarGyroBiasFactor extends NonlinearFactor {
        public enum FF {
            PlanarGyroBiasFactor(ADDRESS, JAVA_LONG, JAVA_LONG, ADDRESS);

            public final MethodHandle h;

            FF(ValueLayout returnType, ValueLayout... parameterTypes) {
                h = Lib.ff(this, returnType, parameterTypes);
            }
        }

        /** @param p pointer to the factor itself, not the shared_ptr. */
        private PlanarGyroBiasFactor(MemorySegment p) {
            super(p);
        }

        public static shared_ptr<PlanarGyroBiasFactor> makeSharedPlanarGyroBiasFactor(
                Key bias_i, Key bias_j, shared_ptr<PlanarGyroParams> p)
                throws Throwable {
            return new shared_ptr<>(
                    (MemorySegment) FF.PlanarGyroBiasFactor.h.invokeExact(bias_i.j, bias_j.j, p.ptr),
                    PlanarGyroBiasFactor::new);
        }
    }

    public enum FF {
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
        super(p);
    }

    public static shared_ptr<PlanarGyroFactor> FromRotation(
            Key pose_i, Key pose_j, Key bias, shared_ptr<PlanarGyroParams> p, Rot2 dr, double dt) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.PlanarGyroFactor_FromRotation.h.invokeExact(
                        pose_i.j, pose_j.j, bias.j, p.ptr, dr.ptr, dt),
                PlanarGyroFactor::new);
    }

    public static shared_ptr<PlanarGyroFactor> FromRate(
            Key pose_i, Key pose_j, Key bias, shared_ptr<PlanarGyroParams> p, double omega, double dt)
            throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.PlanarGyroFactor_FromRate.h.invokeExact(
                        pose_i.j, pose_j.j, bias.j, p.ptr, omega, dt),
                PlanarGyroFactor::new);
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

package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/**
 * ALERT!
 * 
 * PlanarProjectionFactor projects landmarks through a simulated camera, and as
 * the solver wanders around the solution space, the camera can sometimes be in
 * a position such that the landmarks are "behind" the camera. GTSAM, by
 * default, throws an exception if that happens, and we depend on it: without
 * that exception, we would have to duplicate the chirality checking logic.
 * So use the flag as shown here:
 * 
 * {@snippet :
 * cmake -S . -B build -DGTSAM_THROW_CHEIRALITY_EXCEPTION=ON -DGTSAM_BUILD_UNSTABLE=OFF
 * cmake --build build --target check
 * }
 * 
 * The flag about "unstable" is not necessary but we don't use any unstable parts
 * anyway.
 */
public class PlanarProjectionFactor1 extends NonlinearFactor {
    public enum FF {
        PlanarProjectionFactor1(
                ADDRESS, JAVA_LONG, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        PlanarProjectionFactor1_evaluateError(
                ADDRESS, ADDRESS, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    /** @param p pointer to the factor itself, not the shared_ptr. */
    private PlanarProjectionFactor1(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<PlanarProjectionFactor1> newPlanarProjectionFactor1(
            Key poseKey,
            Point3 landmark,
            Point2 measured,
            Pose3 bTc,
            Cal3DS2 calib,
            shared_ptr<? extends gtsam.noiseModel.Base> model) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.PlanarProjectionFactor1.h.invokeExact(
                        poseKey.j, landmark.ptr, measured.ptr, bTc.ptr, calib.ptr, model.ptr),
                PlanarProjectionFactor1::new);
    }

    public Vector2 evaluateError(Pose2 pose, Matrix H) throws Throwable {
        // TODO: maybe this should be Vector instead of Vector2
        return new Vector2(
                (MemorySegment) FF.PlanarProjectionFactor1_evaluateError.h.invokeExact(
                        ptr, pose.ptr, H.ptr));

    }

}

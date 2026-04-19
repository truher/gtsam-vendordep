package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/**
 * ALERT!
 * 
 * PlanarProjectionFactor projects landmarks through a simulated camera, and as
 * the solver wanders around the solution space, the camera can sometimes be in
 * a position such that the landmarks are "behind" the camera. GTSAM, by
 * default, throws an exception if that happens, but we don't want that. So the
 * GTSAM library MUST be built with the correct flags.
 * 
 * <pre>
 * cmake -S . -B build -DGTSAM_THROW_CHEIRALITY_EXCEPTION=OFF -DGTSAM_BUILD_UNSTABLE=OFF
 * cmake --build build --target check
 * </pre>
 * 
 * The flag about "unstable" is necessary because the unstable part doesn't work
 * without the cheirality exception (!?)
 */
public class PlanarProjectionFactor1 extends NonlinearFactor {
    private static final MethodHandle PlanarProjectionFactor1 = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("PlanarProjectionFactor1"),
            FunctionDescriptor.of(
                    ADDRESS,
                    JAVA_LONG,
                    ADDRESS,
                    ADDRESS,
                    ADDRESS,
                    ADDRESS,
                    ADDRESS));
    private static final MethodHandle evaluateError = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("PlanarProjectionFactor1_evaluateError"),
            FunctionDescriptor.of(ADDRESS, ADDRESS, ADDRESS, ADDRESS));

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
            SharedNoiseModel model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) PlanarProjectionFactor1.invokeExact(
                poseKey.j, landmark.ptr, measured.ptr, bTc.ptr, calib.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, PlanarProjectionFactor1::new);
    }

    public Vector2 evaluateError(Pose2 pose, Matrix H) throws Throwable {
        // TODO: maybe this should be Vector instead of Vector2
        return new Vector2(
                (MemorySegment) evaluateError.invokeExact(
                        ptr, pose.ptr, H.ptr));

    }

}

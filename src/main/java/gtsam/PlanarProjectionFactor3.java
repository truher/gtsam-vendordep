package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class PlanarProjectionFactor3 extends NonlinearFactor {
    private static final MethodHandle PlanarProjectionFactor3 = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("PlanarProjectionFactor3"),
            FunctionDescriptor.of(
                    ADDRESS,
                    JAVA_LONG,
                    JAVA_LONG,
                    JAVA_LONG,
                    ADDRESS,
                    ADDRESS,
                    ADDRESS));
    private static final MethodHandle evaluateError = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("PlanarProjectionFactor3_evaluateError"),
            FunctionDescriptor.of(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS));

    /** @param p pointer to the factor itself, not the shared_ptr. */
    private PlanarProjectionFactor3(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<PlanarProjectionFactor3> newPlanarProjectionFactor3(
            Key poseKey,
            Key offsetKey,
            Key calibKey,
            Point3 landmark,
            Point2 measured,
            SharedNoiseModel model) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) PlanarProjectionFactor3.invokeExact(
                poseKey.j, offsetKey.j, calibKey.j, landmark.ptr, measured.ptr, model.ptr);
        return new shared_ptr<>(sharedPtrPtr, PlanarProjectionFactor3::new);
    }

    public Vector evaluateError(Pose2 wTb, Pose3 bTc, Cal3DS2 calib, Matrix HwTb, Matrix HbTc, Matrix Hcalib)
            throws Throwable {
        return new Vector(
                (MemorySegment) evaluateError.invokeExact(
                        ptr, wTb.ptr, bTc.ptr, calib.ptr, HwTb.ptr, HbTc.ptr, Hcalib.ptr));

    }

}

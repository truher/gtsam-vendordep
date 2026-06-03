package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class PlanarProjectionFactor3 extends NonlinearFactor {
    public enum FF {
        PlanarProjectionFactor3(
                ADDRESS, JAVA_LONG, JAVA_LONG, JAVA_LONG, ADDRESS, ADDRESS, ADDRESS),
        PlanarProjectionFactor3_evaluateError(
                ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

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
            shared_ptr<? extends gtsam.noiseModel.Base> model) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.PlanarProjectionFactor3.h.invokeExact(
                        poseKey.j, offsetKey.j, calibKey.j, landmark.ptr, measured.ptr, model.ptr),
                PlanarProjectionFactor3::new);
    }

    public Vector evaluateError(Pose2 wTb, Pose3 bTc, Cal3DS2 calib, Matrix HwTb, Matrix HbTc, Matrix Hcalib)
            throws Throwable {
        return new Vector(
                (MemorySegment) FF.PlanarProjectionFactor3_evaluateError.h.invokeExact(
                        ptr, wTb.ptr, bTc.ptr, calib.ptr, HwTb.ptr, HbTc.ptr, Hcalib.ptr));

    }

}

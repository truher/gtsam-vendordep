package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class GeneralSFMFactor3<T> extends NonlinearFactor {
    public enum FF {
        GeneralSFMFactor3Cal3DS2(
                ADDRESS, ADDRESS, ADDRESS, ADDRESS, JAVA_LONG, JAVA_LONG, JAVA_BOOLEAN),
        GeneralSFMFactor3Cal3DS2_evaluateError(
                ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }

    }

    private GeneralSFMFactor3(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<GeneralSFMFactor3<Cal3DS2>> GeneralSFMFactor3Cal3DS2(
            Point3 landmark,
            Point2 measured,
            shared_ptr<? extends gtsam.noiseModel.Base> model,
            Key poseKey,
            Key calibKey,
            boolean verboseCheirality) {
        return new shared_ptr<>(
                (MemorySegment) FF.GeneralSFMFactor3Cal3DS2.invoke(
                        landmark.ptr, measured.ptr, model.ptr, poseKey.j, calibKey.j, verboseCheirality),
                GeneralSFMFactor3::new);
    }

    public Vector2 evaluateError(Pose3 pose3, Cal3DS2 calib, Matrix H1, Matrix H3) {
        return new Vector2((MemorySegment) FF.GeneralSFMFactor3Cal3DS2_evaluateError.invoke(
                ptr, pose3.ptr, calib.ptr, H1.ptr, H3.ptr));
    }

}

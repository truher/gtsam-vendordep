package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class GeneralSFMFactor2<T> extends NonlinearFactor {
    public enum FF {
        GeneralSFMFactor2Cal3DS2(
                ADDRESS, ADDRESS, ADDRESS, JAVA_LONG, JAVA_LONG, JAVA_LONG),
        GeneralSFMFactor2Cal3DS2_evaluateError(
                ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }

    }

    private GeneralSFMFactor2(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<GeneralSFMFactor2<Cal3DS2>> GeneralSFMFactor2Cal3DS2(
            Point2 measured, //
            shared_ptr<? extends gtsam.noiseModel.Base> model, //
            Key poseKey, //
            Key landmarkKey, //
            Key calibKey) {
        return new shared_ptr<>(
                (MemorySegment) FF.GeneralSFMFactor2Cal3DS2.invoke(
                        measured.ptr, model.ptr, poseKey.j, landmarkKey.j, calibKey.j),
                GeneralSFMFactor2::new);
    }

    public Vector2 evaluateError(Pose3 pose3, Point3 point, Cal3DS2 calib, Matrix H1, Matrix H2, Matrix H3) {
        return new Vector2((MemorySegment) FF.GeneralSFMFactor2Cal3DS2_evaluateError.invoke(
                ptr, pose3.ptr, point.ptr, calib.ptr, H1.ptr, H2.ptr, H3.ptr));
    }

}

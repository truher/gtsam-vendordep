package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/**
 * A projection factor using pixel measurements and fixed landmarks
 * (apriltags), and with variable pose (so you can move the target around), and
 * variable calibration (the thing to be solved).
 * 
 * The purpose of this factor is camera calibration: you use a target with
 * many landmarks (tags), and allow the relative pose of target and camera
 * to vary (i.e. the "chessboard dance").  Each camera frame uses a different
 * pose variable.  All frames use the same calibration variable.
 * 
 * @param <T> camera calibration type
 */
public class GenericProjectionFactor2<T> extends NonlinearFactor {
    public enum FF {
        GenericProjectionFactor2Cal3DS2(
                ADDRESS, JAVA_LONG, JAVA_LONG, ADDRESS, ADDRESS, ADDRESS, JAVA_BOOLEAN),
        GenericProjectionFactor2Cal3DS2_evaluateError(
                ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    private GenericProjectionFactor2(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<GenericProjectionFactor2<Cal3DS2>> GenericProjectionFactor2Cal3DS2(
            Key poseKey,
            Key calibKey,
            Point3 landmark,
            Point2 measured,
            shared_ptr<? extends gtsam.noiseModel.Base> model,
            boolean verboseCheirality) {
        return new shared_ptr<>(
                (MemorySegment) FF.GenericProjectionFactor2Cal3DS2.invoke(
                        poseKey.j, calibKey.j, landmark.ptr, measured.ptr, model.ptr, verboseCheirality),
                GenericProjectionFactor2::new);
    }

    public Vector2 evaluateError(Pose3 pose3, Cal3DS2 calib, Matrix Dpose, Matrix Dcal) {
        return new Vector2((MemorySegment) FF.GenericProjectionFactor2Cal3DS2_evaluateError.invoke(
                ptr, pose3.ptr, calib.ptr, Dpose.ptr, Dcal.ptr));
    }

}

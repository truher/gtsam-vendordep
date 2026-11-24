package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/**
 * A projection factor using pixel measurements and fixed landmarks, fixed
 * calibration, and variable pose.
 * 
 * The purpose of this factor is to determine the "body transform," or what we
 * usually call "camera offset," using a tag in a known position relative to the
 * robot center.
 * 
 * @param <T> camera calibration type
 */
public class GenericProjectionFactor3<T> extends NonlinearFactor {
    public enum FF {
        GenericProjectionFactor3Cal3DS2(
                ADDRESS, JAVA_LONG, ADDRESS, ADDRESS, ADDRESS, ADDRESS, JAVA_BOOLEAN),
        GenericProjectionFactor3Cal3DS2_evaluateError(
                ADDRESS, ADDRESS, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    private GenericProjectionFactor3(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<GenericProjectionFactor3<Cal3DS2>> GenericProjectionFactor3Cal3DS2(
            Key poseKey,
            Point3 landmark,
            Point2 measured,
            shared_ptr<Cal3DS2> K,
            shared_ptr<? extends gtsam.noiseModel.Base> model,
            boolean verboseCheirality) {
        return new shared_ptr<>(
                (MemorySegment) FF.GenericProjectionFactor3Cal3DS2.invoke(
                        poseKey.j, landmark.ptr, measured.ptr, K.ptr, model.ptr, verboseCheirality),
                GenericProjectionFactor3::new);
    }

    public Vector2 evaluateError(Pose3 pose3, Matrix Dpose) {
        return new Vector2((MemorySegment) FF.GenericProjectionFactor3Cal3DS2_evaluateError.invoke(
                ptr, pose3.ptr, Dpose.ptr));
    }
}

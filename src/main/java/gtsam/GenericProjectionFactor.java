package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/**
 * A projection factor uxing pixel measurements and fixed calibration,
 * and with variable pose and landmarks.
 * 
 * The purpose of this factor is determining the "map" of landmarks while
 * "localizing" the camera within it: Simultaneous Localization and Mapping
 * ("SLAM").
 * 
 * @param <T> camera calibration type
 */
public class GenericProjectionFactor<T> extends NonlinearFactor {
    public enum FF {
        GenericProjectionFactorCal3DS2(
                ADDRESS, ADDRESS, ADDRESS, JAVA_LONG, JAVA_LONG, ADDRESS, JAVA_BOOLEAN, JAVA_BOOLEAN),
        GenericProjectionFactorCal3DS2_evaluateError(
                ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    private GenericProjectionFactor(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<GenericProjectionFactor<Cal3DS2>> GenericProjectionFactorCal3DS2(
            Point2 measured,
            shared_ptr<? extends gtsam.noiseModel.Base> model,
            Key poseKey,
            Key pointKey,
            shared_ptr<Cal3DS2> K,
            boolean throwCheirality,
            boolean verboseCheirality) {
        return new shared_ptr<>(
                (MemorySegment) FF.GenericProjectionFactorCal3DS2.invoke(
                        measured.ptr, model.ptr, poseKey.j, pointKey.j, K.ptr, throwCheirality, verboseCheirality),
                GenericProjectionFactor::new);
    }

    public Vector2 evaluateError(Pose3 pose3, Point3 point, Matrix H1, Matrix H2) {
        return new Vector2((MemorySegment) FF.GenericProjectionFactorCal3DS2_evaluateError.invoke(
                ptr, pose3.ptr, point.ptr, H1.ptr, H2.ptr));
    }
}

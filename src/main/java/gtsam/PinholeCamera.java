package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class PinholeCamera<T> extends ForeignObject {

    public enum FF {
        PinholeCameraCal3DS2(ADDRESS, ADDRESS, ADDRESS),
        PinholeCameraCal3DS2_delete(null, ADDRESS),
        PinholeCameraCal3DS2_project(ADDRESS, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    private PinholeCamera(MemorySegment p, MethodHandle deleter) {
        super(p, deleter);
    }

    public static PinholeCamera<Cal3DS2> PinholeCameraCal3DS2(
            Pose3 pose, Cal3DS2 K) {
        return new PinholeCamera<>(
                (MemorySegment) FF.PinholeCameraCal3DS2.invoke(pose.ptr, K.ptr),
                FF.PinholeCameraCal3DS2_delete.h);
    }

    public Point2 project(Point3 pw) {
        return new Point2((MemorySegment) FF.PinholeCameraCal3DS2_project.invoke(ptr, pw.ptr));
    }

}
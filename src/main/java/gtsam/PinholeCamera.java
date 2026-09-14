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

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    private PinholeCamera(MemorySegment p, MethodHandle deleter) {
        super(p, deleter);
    }

    public static PinholeCamera<Cal3DS2> PinholeCameraCal3DS2(
            Pose3 pose, Cal3DS2 K) throws Throwable {
        return new PinholeCamera<>(
                (MemorySegment) FF.PinholeCameraCal3DS2.h.invokeExact(pose.ptr, K.ptr),
                FF.PinholeCameraCal3DS2_delete.h);
    }

    public Point2 project(Point3 pw) throws Throwable {
        return new Point2((MemorySegment) FF.PinholeCameraCal3DS2_project.h.invokeExact(ptr, pw.ptr));
    }

}
package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class PinholeCamera<T> extends ForeignObject {
    private static final MethodHandle PinholeCameraCal3DS2 = Lib.down(
            "PinholeCameraCal3DS2", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle PinholeCameraCal3DS2_delete = Lib.downVoid(
            "PinholeCameraCal3DS2_delete", ADDRESS);
    private static final MethodHandle PinholeCameraCal3DS2_project = Lib.down(
            "PinholeCameraCal3DS2_project", ADDRESS, ADDRESS, ADDRESS);

    public PinholeCamera(MemorySegment p) {
        // TODO: this calls the Cal3DS2 deleter which is wrong for other parameter types
        super(p, PinholeCameraCal3DS2_delete);
    }

    public static PinholeCamera<Cal3DS2> PinholeCameraCal3DS2(
            Pose3 pose, Cal3DS2 K) throws Throwable {
        return new PinholeCamera<>(
                (MemorySegment) PinholeCameraCal3DS2.invokeExact(pose.ptr, K.ptr));
    }

    public Point2 project(Point3 pw) throws Throwable {
        return new Point2((MemorySegment) PinholeCameraCal3DS2_project.invokeExact(ptr, pw.ptr));
    }

}

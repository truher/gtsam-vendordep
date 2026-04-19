package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Pose3 extends ForeignObject {
    private static final MethodHandle Pose3 = Lib.down(
            "Pose3", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose3_delete = Lib.downVoid(
            "Pose2_delete", ADDRESS);
    private static final MethodHandle Pose3_Pose2 = Lib.down(
            "Pose3_Pose2", ADDRESS, ADDRESS);
    private static final MethodHandle Pose3_compose = Lib.down(
            "Pose3_compose", ADDRESS, ADDRESS, ADDRESS);

    public Pose3(MemorySegment p) {
        super(p, Pose3_delete);
    }

    /** Copies the arguments. */
    public Pose3(Rot3 r, Point3 t) throws Throwable {
        this((MemorySegment) Pose3.invokeExact(r.ptr, t.ptr));
    }

    public Pose3(Pose2 p) throws Throwable {
        this((MemorySegment) Pose3_Pose2.invokeExact(p.ptr));
    }

    public Pose3 compose(Pose3 p2) throws Throwable {
        return new Pose3((MemorySegment) Pose3_compose.invokeExact(ptr, p2.ptr));
    }
}

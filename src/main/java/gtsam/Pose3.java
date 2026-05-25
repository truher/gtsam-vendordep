package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Pose3 extends ForeignObject implements LieGroup<Pose3, Vector6> {
    private static final MethodHandle Pose3 = Lib.down(
            "Pose3", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose3_delete = Lib.downVoid(
            "Pose2_delete", ADDRESS);
    private static final MethodHandle Pose3_Pose2 = Lib.down(
            "Pose3_Pose2", ADDRESS, ADDRESS);
    private static final MethodHandle Pose3_localCoordinates = Lib.down(
            "Pose3_localCoordinates", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose3_compose = Lib.down(
            "Pose3_compose", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose3_retract = Lib.down(
            "Pose3_retract", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose3_between = Lib.down(
            "Pose3_between", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose3_inverse = Lib.down(
            "Pose3_inverse", ADDRESS, ADDRESS);
    private static final MethodHandle Pose3_AdjointMap = Lib.down(
            "Pose3_AdjointMap", ADDRESS, ADDRESS);
    private static final MethodHandle Pose3_Expmap = Lib.down(
            "Pose3_Expmap", ADDRESS, ADDRESS);
    private static final MethodHandle Pose3_Logmap = Lib.down(
            "Pose3_Logmap", ADDRESS, ADDRESS);

    public static class Traits implements LieGroup.Traits<Pose3, Vector6> {
        @Override
        public Pose3 Identity() throws Throwable {
            return new Pose3();
        }

        @Override
        public Pose3 Expmap(Vector6 xi) throws Throwable {
            return new Pose3((MemorySegment) Pose3_Expmap.invokeExact(xi.ptr));
        }

        @Override
        public Vector6 Logmap(Pose3 g) throws Throwable {
            return new Vector6((MemorySegment) Pose3_Logmap.invokeExact(g.ptr));
        }
    }

    public static final Traits traits = new Traits();

    @Override
    public Traits traits() {
        return traits;
    }

    @Override
    public Vector6 dxZero() throws Throwable {
        return new Vector6(0, 0, 0, 0, 0, 0);
    }

    public Pose3(MemorySegment p) {
        super(p, Pose3_delete);
    }

    public Pose3() throws Throwable {
        // TODO: avoid these
        this(new Rot3(), new Point3(0,0,0));
    }

    /** Copies the arguments. */
    public Pose3(Rot3 r, Point3 t) throws Throwable {
        this((MemorySegment) Pose3.invokeExact(r.ptr, t.ptr));
    }

    public Pose3(Pose2 p) throws Throwable {
        this((MemorySegment) Pose3_Pose2.invokeExact(p.ptr));
    }

    public Vector6 localCoordinates(Pose3 g) throws Throwable {
        return new Vector6(
                (MemorySegment) Pose3_localCoordinates.invokeExact(ptr, g.ptr));
    }

    @Override
    public Vector6 local(Pose3 other) throws Throwable {
        return localCoordinates(other);
    }

    public Pose3 compose(Pose3 p2) throws Throwable {
        return new Pose3((MemorySegment) Pose3_compose.invokeExact(ptr, p2.ptr));
    }

    @Override
    public int dimension() {
        return 6;
    }

    @Override
    public Pose3 retract(Vector6 v) throws Throwable {
        return new Pose3((MemorySegment) Pose3_retract.invokeExact(ptr, v.ptr()));
    }

    @Override
    public Pose3 inverse() throws Throwable {
        return new Pose3((MemorySegment) Pose3_inverse.invokeExact(ptr));
    }

    @Override
    public Pose3 between(Pose3 other) throws Throwable {
        return new Pose3((MemorySegment) Pose3_between.invokeExact(ptr, other.ptr));
    }

    /** underlying AdjointMap returns Matrix3 but we coerce to dynamic. */
    public Matrix AdjointMap() throws Throwable {
        return new Matrix((MemorySegment) Pose3_AdjointMap.invokeExact(ptr));
    }
}

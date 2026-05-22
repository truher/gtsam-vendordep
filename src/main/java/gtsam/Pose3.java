package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Pose3 extends ForeignObject implements Manifold<Pose3> {
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
    private static final MethodHandle Pose3_print = Lib.downVoid(
            "Pose3_print", ADDRESS);
    private static final MethodHandle Pose3_equals = Lib.down(
            "Pose3_equals", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Pose3_inverse = Lib.down(
            "Pose3_inverse", ADDRESS, ADDRESS);
    private static final MethodHandle Pose3_AdjointMap = Lib.down(
            "Pose3_AdjointMap", ADDRESS, ADDRESS);

    public static class Traits implements Manifold.Traits<Pose3> {

    }

    public static final Traits traits = new Traits();

    @Override
    public Traits traits() {
        return traits;
    }

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

    public Vector localCoordinates(Pose3 g) throws Throwable {
        return new Vector(
                (MemorySegment) Pose3_localCoordinates.invokeExact(ptr, g.ptr));
    }

    @Override
    public Vector local(Pose3 other) throws Throwable {
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
    public Pose3 retract(Vector v) throws Throwable {
        return new Pose3((MemorySegment) Pose3_retract.invokeExact(ptr, v.ptr()));
    }

    public void print() throws Throwable {
        Pose3_print.invokeExact(ptr);
    }

    public boolean equals(Pose3 other, double tol) throws Throwable {
        return (boolean) Pose3_equals.invokeExact(ptr, other.ptr, tol);
    }

    public Pose3 inverse() throws Throwable {
        return new Pose3((MemorySegment) Pose3_inverse.invokeExact(ptr));
    }

    public Matrix AdjointMap() throws Throwable {
        return new Matrix((MemorySegment) Pose3_AdjointMap.invokeExact(ptr));
    }
}

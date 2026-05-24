package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Rot2 extends ForeignObject {
    private static final MethodHandle Rot2 = Lib.down(
            "Rot2", ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Rot2_delete = Lib.downVoid(
            "Rot2_delete", ADDRESS);
    private static final MethodHandle Rot2_theta = Lib.down(
            "Rot2_theta", JAVA_DOUBLE, ADDRESS);
    private static final MethodHandle Rot2_c = Lib.down(
            "Rot2_c", JAVA_DOUBLE, ADDRESS);
    private static final MethodHandle Rot2_s = Lib.down(
            "Rot2_s", JAVA_DOUBLE, ADDRESS);
    private static final MethodHandle Rot2_matrix = Lib.down(
            "Rot2_matrix", ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_compose = Lib.down(
            "Rot2_compose", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_rotate = Lib.down(
            "Rot2_rotate", ADDRESS, ADDRESS, ADDRESS);

    public Rot2(MemorySegment p) {
        super(p, Rot2_delete);
    }

    public Rot2(double theta) throws Throwable {
        this((MemorySegment) Rot2.invokeExact(theta));
    }

    public static Rot2 fromAngle(double theta) throws Throwable {
        return new Rot2(theta);
    }

    public double theta() throws Throwable {
        return (double) Rot2_theta.invokeExact(ptr);
    }

    public double c() throws Throwable {
        return (double) Rot2_c.invokeExact(ptr);
    }

    public double s() throws Throwable {
        return (double) Rot2_s.invokeExact(ptr);
    }

    public Matrix2 matrix() throws Throwable {
        return new Matrix2((MemorySegment) Rot2_matrix.invokeExact(ptr));
    }

    public Rot2 compose(Rot2 other) throws Throwable {
        return new Rot2((MemorySegment) Rot2_compose.invokeExact(ptr, other.ptr));
    }

    public Point2 rotate(Point2 p) throws Throwable {
        return new Point2((MemorySegment) Rot2_rotate.invokeExact(ptr, p.ptr));
    }

}

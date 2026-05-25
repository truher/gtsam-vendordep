package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Rot2 extends ForeignObject implements LieGroup<Rot2, Vector1> {
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
    private static final MethodHandle Rot2_composeH = Lib.down(
            "Rot2_composeH", ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_rotate = Lib.down(
            "Rot2_rotate", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_rotateH = Lib.down(
            "Rot2_rotateH", ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_unrotate = Lib.down(
            "Rot2_unrotate", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_unrotateH = Lib.down(
            "Rot2_unrotateH", ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_fromCosSin = Lib.down(
            "Rot2_fromCosSin", ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Rot2_atan2 = Lib.down(
            "Rot2_atan2", ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Rot2_relativeBearing = Lib.down(
            "Rot2_relativeBearing", ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_relativeBearingH = Lib.down(
            "Rot2_relativeBearingH", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_unit = Lib.down(
            "Rot2_unit", ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_inverse = Lib.down(
            "Rot2_inverse", ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_inverseH = Lib.down(
            "Rot2_inverseH", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_expmap = Lib.down(
            "Rot2_expmap", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_expmapH = Lib.down(
            "Rot2_expmapH", ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_logmap = Lib.down(
            "Rot2_logmap", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_logmapH = Lib.down(
            "Rot2_logmapH", ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_transpose = Lib.down(
            "Rot2_transpose", ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_between = Lib.down(
            "Rot2_between", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_betweenH = Lib.down(
            "Rot2_betweenH", ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_retract = Lib.down(
            "Rot2_retract", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_retractH = Lib.down(
            "Rot2_retractH", ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_localCoordinates = Lib.down(
            "Rot2_localCoordinates", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_localCoordinatesH = Lib.down(
            "Rot2_localCoordinatesH", ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_Expmap = Lib.down(
            "Rot2_Expmap", ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_Logmap = Lib.down(
            "Rot2_Logmap", ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_check_group_invariants = Lib.down(
            "Rot2_check_group_invariants", JAVA_BOOLEAN, ADDRESS, ADDRESS);
    private static final MethodHandle Rot2_check_manifold_invariants = Lib.down(
            "Rot2_check_manifold_invariants", JAVA_BOOLEAN, ADDRESS, ADDRESS);

    public static class Rot2Traits implements LieGroup.Traits<Rot2, Vector1> {

        @Override
        public Rot2 Identity() throws Throwable {
            return new Rot2();
        }

        @Override
        public Rot2 Expmap(Vector1 v) throws Throwable {
            return new Rot2((MemorySegment) Rot2_Expmap.invokeExact(v.ptr));
        }

        @Override
        public Vector1 Logmap(Rot2 g) throws Throwable {
            return new Vector1((MemorySegment) Rot2_Logmap.invokeExact(g.ptr));
        }
    }

    public static class Rot2Statics implements LieGroup.Statics<Rot2, Vector1> {
        @Override
        public Rot2 Identity() throws Throwable {
            return new Rot2();
        }
    }

    public static final Rot2Traits traits = new Rot2Traits();
    public static final Rot2ChartAtOrigin chartAtOrigin = new Rot2ChartAtOrigin();

    @Override
    public Traits<Rot2, Vector1> traits() {
        return traits;
    }

    @Override
    public ChartAtOrigin<Rot2, Vector1> chartAtOrigin() {
        return chartAtOrigin;
    }

    public Rot2(MemorySegment p) {
        super(p, Rot2_delete);
    }

    public Rot2() throws Throwable {
        this(0);
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

    @Override
    public Rot2 compose(Rot2 other) throws Throwable {
        return new Rot2((MemorySegment) Rot2_compose.invokeExact(ptr, other.ptr));
    }

    public Rot2 compose(Rot2 other, Matrix H1, Matrix H2) throws Throwable {
        return new Rot2((MemorySegment) Rot2_composeH.invokeExact(ptr, other.ptr, H1.ptr, H2.ptr));
    }

    public Point2 rotate(Point2 p) throws Throwable {
        return new Point2((MemorySegment) Rot2_rotate.invokeExact(ptr, p.ptr));
    }

    public Point2 rotate(Point2 p, Matrix H1, Matrix H2) throws Throwable {
        return new Point2((MemorySegment) Rot2_rotateH.invokeExact(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public Point2 unrotate(Point2 p) throws Throwable {
        return new Point2((MemorySegment) Rot2_unrotate.invokeExact(ptr, p.ptr));
    }

    public Point2 unrotate(Point2 p, Matrix H1, Matrix H2) throws Throwable {
        return new Point2((MemorySegment) Rot2_unrotateH.invokeExact(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public static Rot2 fromCosSin(double c, double s) throws Throwable {
        return new Rot2((MemorySegment) Rot2_fromCosSin.invokeExact(c, s));
    }

    /** Note order of arguments, y first, x second. */
    public static Rot2 atan2(double y, double x) throws Throwable {
        return new Rot2((MemorySegment) Rot2_atan2.invokeExact(y, x));
    }

    public static Rot2 relativeBearing(Point2 d) throws Throwable {
        return new Rot2((MemorySegment) Rot2_relativeBearing.invokeExact(d.ptr));
    }

    public static Rot2 relativeBearing(Point2 d, Matrix H) throws Throwable {
        return new Rot2((MemorySegment) Rot2_relativeBearingH.invokeExact(d.ptr, H.ptr));
    }

    public Point2 unit() throws Throwable {
        return new Point2((MemorySegment) Rot2_unit.invokeExact(ptr));
    }

    @Override
    public Rot2 inverse() throws Throwable {
        return new Rot2((MemorySegment) Rot2_inverse.invokeExact(ptr));
    }

    @Override
    public Rot2 inverse(Matrix H) throws Throwable {
        return new Rot2((MemorySegment) Rot2_inverseH.invokeExact(ptr, H.ptr));
    }

    @Override
    public Rot2 expmap(Vector1 v) throws Throwable {
        return new Rot2((MemorySegment) Rot2_expmap.invokeExact(ptr, v.ptr));
    }

    @Override
    public Vector1 logmap(Rot2 g) throws Throwable {
        return new Vector1((MemorySegment) Rot2_logmap.invokeExact(ptr, v.ptr));
    }

    @Override
    public Rot2 expmap(Vector1 v, Matrix H1, Matrix H2) throws Throwable {
        return new Rot2((MemorySegment) Rot2_expmapH.invokeExact(ptr, v.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Vector1 logmap(Rot2 g, Matrix H1, Matrix H2) throws Throwable {
        return new Vector1((MemorySegment) Rot2_logmapH.invokeExact(ptr, g.ptr, H1.ptr, H2.ptr));
    }

    public Matrix2 transpose() throws Throwable {
        return new Matrix2((MemorySegment) Rot2_transpose.invokeExact(ptr));
    }

    @Override
    public Rot2 between(Rot2 g) throws Throwable {
        return new Rot2((MemorySegment) Rot2_between.invokeExact(ptr, g.ptr));
    }

    @Override
    public Rot2 between(Rot2 g, Matrix H1, Matrix H2) throws Throwable {
        return new Rot2((MemorySegment) Rot2_betweenH.invokeExact(ptr, g.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Rot2 retract(Vector1 v) throws Throwable {
        return new Rot2((MemorySegment) Rot2_retract.invokeExact(ptr, v.ptr));
    }

    @Override
    public Vector1 localCoordinates(Rot2 g) throws Throwable {
        return new Vector1((MemorySegment) Rot2_localCoordinates.invokeExact(ptr, g.ptr));
    }

    @Override
    public Rot2 retract(Vector1 v, Matrix H1, Matrix H2) throws Throwable {
        return new Rot2((MemorySegment) Rot2_retractH.invokeExact(ptr, v.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Vector1 localCoordinates(Rot2 g, Matrix H1, Matrix H2) throws Throwable {
        return new Vector1((MemorySegment) Rot2_localCoordinatesH.invokeExact(ptr, g.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Vector1 dxZero() throws Throwable {
        return new Vector1(0);
    }

    @Override
    public int dimension() throws Throwable {
        return 1;
    }

    public static boolean check_group_invariants(Rot2 a, Rot2 b) throws Throwable {
        return (boolean) Rot2_check_group_invariants.invokeExact(a.ptr, b.ptr);
    }

    public static boolean check_manifold_invariants(Rot2 a, Rot2 b) throws Throwable {
        return (boolean) Rot2_check_manifold_invariants.invokeExact(a.ptr, b.ptr);
    }
}

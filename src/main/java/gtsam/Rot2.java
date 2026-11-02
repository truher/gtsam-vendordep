package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Rot2 extends ForeignObject implements LieGroup<Rot2, Vector1> {
    public enum FF {
        Rot2(ADDRESS, JAVA_DOUBLE),
        Rot2_delete(null, ADDRESS),
        Rot2_theta(JAVA_DOUBLE, ADDRESS),
        Rot2_c(JAVA_DOUBLE, ADDRESS),
        Rot2_s(JAVA_DOUBLE, ADDRESS),
        Rot2_matrix(ADDRESS, ADDRESS),
        Rot2_compose(ADDRESS, ADDRESS, ADDRESS),
        Rot2_composeH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_rotate(ADDRESS, ADDRESS, ADDRESS),
        Rot2_rotateH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_unrotate(ADDRESS, ADDRESS, ADDRESS),
        Rot2_unrotateH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_fromCosSin(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE),
        Rot2_fromDegrees(ADDRESS, JAVA_DOUBLE),
        Rot2_atan2(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE),
        Rot2_relativeBearing(ADDRESS, ADDRESS),
        Rot2_relativeBearingH(ADDRESS, ADDRESS, ADDRESS),
        Rot2_unit(ADDRESS, ADDRESS),
        Rot2_inverse(ADDRESS, ADDRESS),
        Rot2_inverseH(ADDRESS, ADDRESS, ADDRESS),
        Rot2_AdjointMap(ADDRESS, ADDRESS),
        Rot2_expmap(ADDRESS, ADDRESS, ADDRESS),
        Rot2_expmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_logmap(ADDRESS, ADDRESS, ADDRESS),
        Rot2_logmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_transpose(ADDRESS, ADDRESS),
        Rot2_between(ADDRESS, ADDRESS, ADDRESS),
        Rot2_betweenH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_retract(ADDRESS, ADDRESS, ADDRESS),
        Rot2_retractH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_localCoordinates(ADDRESS, ADDRESS, ADDRESS),
        Rot2_localCoordinatesH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_check_group_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Rot2_check_manifold_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Rot2_print(null, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Rot2(MemorySegment p) {
        super(p, FF.Rot2_delete.h);
    }

    public Rot2() {
        this(0);
    }

    public Rot2(double theta) {
        this((MemorySegment) FF.Rot2.invoke(theta));
    }

    public static Rot2 fromAngle(double theta) {
        return new Rot2(theta);
    }

    public double theta() {
        return (double) FF.Rot2_theta.invoke(ptr);
    }

    public double c() {
        return (double) FF.Rot2_c.invoke(ptr);
    }

    public double s() {
        return (double) FF.Rot2_s.invoke(ptr);
    }

    public Matrix2 matrix() {
        return new Matrix2((MemorySegment) FF.Rot2_matrix.invoke(ptr));
    }

    @Override
    public Rot2 compose(Rot2 other) {
        return new Rot2((MemorySegment) FF.Rot2_compose.invoke(ptr, other.ptr));
    }

    public Rot2 compose(Rot2 other, Matrix H1, Matrix H2) {
        return new Rot2((MemorySegment) FF.Rot2_composeH.invoke(ptr, other.ptr, H1.ptr, H2.ptr));
    }

    public Point2 rotate(Point2 p) {
        return new Point2((MemorySegment) FF.Rot2_rotate.invoke(ptr, p.ptr));
    }

    public Point2 rotate(Point2 p, Matrix H1, Matrix H2) {
        return new Point2((MemorySegment) FF.Rot2_rotateH.invoke(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public Point2 unrotate(Point2 p) {
        return new Point2((MemorySegment) FF.Rot2_unrotate.invoke(ptr, p.ptr));
    }

    public Point2 unrotate(Point2 p, Matrix H1, Matrix H2) {
        return new Point2((MemorySegment) FF.Rot2_unrotateH.invoke(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public static Rot2 fromCosSin(double c, double s) {
        return new Rot2((MemorySegment) FF.Rot2_fromCosSin.invoke(c, s));
    }

    public static Rot2 fromDegrees(double degrees) {
        return new Rot2((MemorySegment) FF.Rot2_fromDegrees.invoke(degrees));
    }

    /** Note order of arguments, y first, x second. */
    public static Rot2 atan2(double y, double x) {
        return new Rot2((MemorySegment) FF.Rot2_atan2.invoke(y, x));
    }

    public static Rot2 relativeBearing(Point2 d) {
        return new Rot2((MemorySegment) FF.Rot2_relativeBearing.invoke(d.ptr));
    }

    public static Rot2 relativeBearing(Point2 d, Matrix H) {
        return new Rot2((MemorySegment) FF.Rot2_relativeBearingH.invoke(d.ptr, H.ptr));
    }

    public Point2 unit() {
        return new Point2((MemorySegment) FF.Rot2_unit.invoke(ptr));
    }

    @Override
    public Rot2 inverse() {
        return new Rot2((MemorySegment) FF.Rot2_inverse.invoke(ptr));
    }

    @Override
    public Rot2 inverse(Matrix H) {
        return new Rot2((MemorySegment) FF.Rot2_inverseH.invoke(ptr, H.ptr));
    }

    @Override
    public Matrix AdjointMap() {
        return new Matrix((MemorySegment) FF.Rot2_AdjointMap.invoke(ptr));
    }

    @Override
    public Rot2 expmap(Vector1 v) {
        return new Rot2((MemorySegment) FF.Rot2_expmap.invoke(ptr, v.ptr));
    }

    @Override
    public Rot2 expmap(Vector1 v, Matrix H1, Matrix H2) {
        return new Rot2((MemorySegment) FF.Rot2_expmapH.invoke(ptr, v.ptr, H1.ptr,
                H2.ptr));
    }

    @Override
    public Vector1 logmap(Rot2 g) {
        return new Vector1((MemorySegment) FF.Rot2_logmap.invoke(ptr, g.ptr));
    }

    @Override
    public Vector1 logmap(Rot2 g, Matrix H1, Matrix H2) {
        return new Vector1((MemorySegment) FF.Rot2_logmapH.invoke(ptr, g.ptr,
                H1.ptr, H2.ptr));
    }

    public Matrix2 transpose() {
        return new Matrix2((MemorySegment) FF.Rot2_transpose.invoke(ptr));
    }

    @Override
    public Rot2 between(Rot2 g) {
        return new Rot2((MemorySegment) FF.Rot2_between.invoke(ptr, g.ptr));
    }

    // @Override
    public Rot2 between(Rot2 g, Matrix H1, Matrix H2) {
        return new Rot2((MemorySegment) FF.Rot2_betweenH.invoke(
                ptr, g.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Rot2 retract(Vector1 v) {
        return new Rot2((MemorySegment) FF.Rot2_retract.invoke(ptr, v.ptr));
    }

    @Override
    public Rot2 retract(Vector1 v, Matrix H1, Matrix H2) {
        return new Rot2((MemorySegment) FF.Rot2_retractH.invoke(ptr, v.ptr, H1.ptr,
                H2.ptr));
    }

    @Override
    public Vector1 local(Rot2 g) {
        return new Vector1((MemorySegment) FF.Rot2_localCoordinates.invoke(ptr, g.ptr));
    }

    @Override
    public Vector1 local(Rot2 g, Matrix H1, Matrix H2) {
        return new Vector1((MemorySegment) FF.Rot2_localCoordinatesH.invoke(ptr,
                g.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Vector1 dxZero() {
        return new Vector1(0);
    }

    @Override
    public int dimension() {
        return 1;
    }

    public static boolean check_group_invariants(Rot2 a, Rot2 b) {
        return (boolean) FF.Rot2_check_group_invariants.invoke(a.ptr, b.ptr);
    }

    public static boolean check_manifold_invariants(Rot2 a, Rot2 b) {
        return (boolean) FF.Rot2_check_manifold_invariants.invoke(a.ptr, b.ptr);
    }

    public void print(String label) {
        System.out.println(label);
        FF.Rot2_print.invoke(ptr);
    }
}

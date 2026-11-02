package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.util.Random;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * Unit vector is locally isomorphic to vector 2, not 3.
 */
public class Unit3 extends ForeignObject
        implements Manifold<Unit3, Vector2> {
    public enum FF {
        Unit3(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Unit3Point3(ADDRESS, ADDRESS),
        Unit3_delete(null, ADDRESS),
        Unit3_point3(ADDRESS, ADDRESS),
        Unit3_point3H(ADDRESS, ADDRESS, ADDRESS),
        Unit3_errorVector(ADDRESS, ADDRESS, ADDRESS),
        Unit3_errorVectorH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Unit3_retract(ADDRESS, ADDRESS, ADDRESS),
        Unit3_retractH(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Unit3_localCoordinates(ADDRESS, ADDRESS, ADDRESS),
        Unit3_dot(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Unit3_dotH(JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Unit3_distance(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Unit3_distanceH(JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS),
        Unit3_basis(ADDRESS, ADDRESS),
        Unit3_basisH(ADDRESS, ADDRESS, ADDRESS),
        Unit3_unitVector(ADDRESS, ADDRESS),
        Unit3_FromPoint(ADDRESS, ADDRESS),
        Unit3_FromPointH(ADDRESS, ADDRESS, ADDRESS),
        Unit3_crossUnit3Unit3(ADDRESS, ADDRESS, ADDRESS),
        Unit3_crossUnit3Unit3H(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Unit3_crossUnit3Point3H(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Unit3_crossPoint3Unit3H(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Unit3_crossUnit3(ADDRESS, ADDRESS, ADDRESS),
        Unit3_crossPoint3(ADDRESS, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    Unit3(MemorySegment p) {
        super(p, FF.Unit3_delete.h);
    }

    public Unit3() {
        // default unit vector is pointing along the x axis.
        this(1, 0, 0);
    }

    public Unit3(double x, double y, double z) {
        this((MemorySegment) FF.Unit3.invoke(x, y, z));
    }

    public Unit3(Point3 p) {
        this((MemorySegment) FF.Unit3Point3.invoke(p.ptr));
    }

    public Unit3(Vector3 v) {
        this(new Point3(v));
    }

    public Point3 point3() {
        return new Point3((MemorySegment) FF.Unit3_point3.invoke(ptr));
    }

    public Point3 point3(Matrix H) {
        return new Point3((MemorySegment) FF.Unit3_point3H.invoke(ptr, H.ptr));
    }

    @Override
    public Vector2 dxZero() {
        return new Vector2(0, 0);
    }

    @Override
    public int dimension() {
        return 2;
    }

    @Override
    public Vector2 local(Unit3 other) {
        return new Vector2((MemorySegment) FF.Unit3_localCoordinates.invoke(ptr, other.ptr));
    }

    @Override
    public Unit3 retract(Vector2 v) {
        return new Unit3((MemorySegment) FF.Unit3_retract.invoke(ptr, v.ptr));
    }

    public Unit3 retract(Vector2 v, Matrix H) {
        return new Unit3((MemorySegment) FF.Unit3_retractH.invoke(ptr, v.ptr, H.ptr));
    }

    public Vector2 errorVector(Unit3 q) {
        return new Vector2((MemorySegment) FF.Unit3_errorVector.invoke(ptr, q.ptr));
    }

    public Vector2 errorVector(Unit3 q, Matrix H1, Matrix H2) {
        return new Vector2((MemorySegment) FF.Unit3_errorVectorH.invoke(ptr, q.ptr, H1.ptr, H2.ptr));
    }

    public double dot(Unit3 q) {
        return (double) FF.Unit3_dot.invoke(ptr, q.ptr);
    }

    public double dot(Unit3 q, Matrix H1, Matrix H2) {
        return (double) FF.Unit3_dotH.invoke(ptr, q.ptr, H1.ptr, H2.ptr);
    }

    public double distance(Unit3 q) {
        return (double) FF.Unit3_distance.invoke(ptr, q.ptr);
    }

    public double distance(Unit3 q, Matrix H) {
        return (double) FF.Unit3_distanceH.invoke(ptr, q.ptr, H.ptr);
    }

    public Matrix basis() {
        return new Matrix((MemorySegment) FF.Unit3_basis.invoke(ptr));
    }

    public Matrix basis(Matrix H) {
        return new Matrix((MemorySegment) FF.Unit3_basisH.invoke(ptr, H.ptr));
    }

    /**
     * Copied from Unit3.cpp to avoid dealing with C++ random.
     */
    public static Unit3 Random(Random rng) {
        double sqsum;
        double x, y;
        do {
            x = rng.nextDouble(-1.0, 1.0);
            y = rng.nextDouble(-1.0, 1.0);
            sqsum = x * x + y * y;
        } while (sqsum > 1);
        double mult = 2 * Math.sqrt(1 - sqsum);
        return new Unit3(x * mult, y * mult, 2 * sqsum - 1);
    }

    public Vector3 unitVector() {
        return new Vector3((MemorySegment) FF.Unit3_unitVector.invoke(ptr));
    }

    public static Unit3 FromPoint3(Point3 p) {
        return new Unit3((MemorySegment) FF.Unit3_FromPoint.invoke(p.ptr));
    }

    public static Unit3 FromPoint3(Point3 p, Matrix H) {
        return new Unit3((MemorySegment) FF.Unit3_FromPointH.invoke(p.ptr, H.ptr));
    }

    public static Unit3 cross(Unit3 p, Unit3 q, Matrix H1, Matrix H2) {
        return new Unit3((MemorySegment) FF.Unit3_crossUnit3Unit3H.invoke(
                p.ptr, q.ptr, H1.ptr, H2.ptr));
    }

    public static Point3 cross(Unit3 p, Point3 q, Matrix H1, Matrix H2) {
        return new Point3((MemorySegment) FF.Unit3_crossUnit3Point3H.invoke(
                p.ptr, q.ptr, H1.ptr, H2.ptr));
    }

    public static Point3 cross(Point3 p, Unit3 q, Matrix H1, Matrix H2) {
        return new Point3((MemorySegment) FF.Unit3_crossPoint3Unit3H.invoke(
                p.ptr, q.ptr, H1.ptr, H2.ptr));
    }

    public static Unit3 cross(Unit3 p, Unit3 q) {
        return new Unit3((MemorySegment) FF.Unit3_crossUnit3Unit3.invoke(
                p.ptr, q.ptr));
    }

    public Unit3 cross(Unit3 q) {
        return new Unit3((MemorySegment) FF.Unit3_crossUnit3.invoke(ptr, q.ptr));
    }

    public Point3 cross(Point3 q) {
        return new Point3((MemorySegment) FF.Unit3_crossPoint3.invoke(ptr, q.ptr));
    }

    @Override
    public Vector2 local(Unit3 g, Matrix H1, Matrix H2) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'localCoordinates'");
    }

    @Override
    public Unit3 retract(Vector2 v, Matrix H1, Matrix H2) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'retract'");
    }
}

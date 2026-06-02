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

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    Unit3(MemorySegment p) {
        super(p, FF.Unit3_delete.h);
    }

    public Unit3() throws Throwable {
        this(0, 0, 0);
    }

    public Unit3(double x, double y, double z) throws Throwable {
        this((MemorySegment) FF.Unit3.h.invokeExact(x, y, z));
    }

    public Unit3(Point3 p) throws Throwable {
        this((MemorySegment) FF.Unit3Point3.h.invokeExact(p.ptr));
    }

    public Unit3(Vector3 v) throws Throwable {
        this(new Point3(v));
    }

    public Point3 point3() throws Throwable {
        return new Point3((MemorySegment) FF.Unit3_point3.h.invokeExact(ptr));
    }

    public Point3 point3(Matrix H) throws Throwable {
        return new Point3((MemorySegment) FF.Unit3_point3H.h.invokeExact(ptr, H.ptr));
    }

    public static class Traits implements Manifold.Traits<Unit3, Vector2> {

    }

    public static final Traits traits = new Traits();

    @Override
    public Traits traits() {
        return traits;
    }

    @Override
    public Vector2 dxZero() throws Throwable {
        return new Vector2(0, 0);
    }

    @Override
    public int dimension() throws Throwable {
        return 2;
    }

    @Override
    public Vector2 localCoordinates(Unit3 other) throws Throwable {
        return new Vector2((MemorySegment) FF.Unit3_localCoordinates.h.invokeExact(ptr, other.ptr));
    }

    @Override
    public Unit3 retract(Vector2 v) throws Throwable {
        return new Unit3((MemorySegment) FF.Unit3_retract.h.invokeExact(ptr, v.ptr));
    }

    public Unit3 retract(Vector2 v, Matrix H) throws Throwable {
        return new Unit3((MemorySegment) FF.Unit3_retractH.h.invokeExact(ptr, v.ptr, H.ptr));
    }

    public Vector2 errorVector(Unit3 q) throws Throwable {
        return new Vector2((MemorySegment) FF.Unit3_errorVector.h.invokeExact(ptr, q.ptr));
    }

    public Vector2 errorVector(Unit3 q, Matrix H1, Matrix H2) throws Throwable {
        return new Vector2((MemorySegment) FF.Unit3_errorVectorH.h.invokeExact(ptr, q.ptr, H1.ptr, H2.ptr));
    }

    public double dot(Unit3 q) throws Throwable {
        return (double) FF.Unit3_dot.h.invokeExact(ptr, q.ptr);
    }

    public double dot(Unit3 q, Matrix H1, Matrix H2) throws Throwable {
        return (double) FF.Unit3_dotH.h.invokeExact(ptr, q.ptr, H1.ptr, H2.ptr);
    }

    public double distance(Unit3 q) throws Throwable {
        return (double) FF.Unit3_distance.h.invokeExact(ptr, q.ptr);
    }

    public double distance(Unit3 q, Matrix H) throws Throwable {
        return (double) FF.Unit3_distanceH.h.invokeExact(ptr, q.ptr, H.ptr);
    }

    public Matrix basis() throws Throwable {
        return new Matrix((MemorySegment) FF.Unit3_basis.h.invokeExact(ptr));
    }

    public Matrix basis(Matrix H) throws Throwable {
        return new Matrix((MemorySegment) FF.Unit3_basisH.h.invokeExact(ptr, H.ptr));
    }

    /**
     * Copied from Unit3.cpp to avoid dealing with C++ random.
     */
    public static Unit3 Random(Random rng) throws Throwable {
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

    public Vector3 unitVector() throws Throwable {
        return new Vector3((MemorySegment) FF.Unit3_unitVector.h.invokeExact(ptr));
    }

    public static Unit3 FromPoint3(Point3 p) throws Throwable {
        return new Unit3((MemorySegment) FF.Unit3_FromPoint.h.invokeExact(p.ptr));
    }

    public static Unit3 FromPoint3(Point3 p, Matrix H) throws Throwable {
        return new Unit3((MemorySegment) FF.Unit3_FromPointH.h.invokeExact(p.ptr, H.ptr));
    }

    public static Unit3 cross(Unit3 p, Unit3 q, Matrix H1, Matrix H2) throws Throwable {
        return new Unit3((MemorySegment) FF.Unit3_crossUnit3Unit3H.h.invokeExact(
                p.ptr, q.ptr, H1.ptr, H2.ptr));
    }

    public static Point3 cross(Unit3 p, Point3 q, Matrix H1, Matrix H2) throws Throwable {
        return new Point3((MemorySegment) FF.Unit3_crossUnit3Point3H.h.invokeExact(
                p.ptr, q.ptr, H1.ptr, H2.ptr));
    }

    public static Point3 cross(Point3 p, Unit3 q, Matrix H1, Matrix H2) throws Throwable {
        return new Point3((MemorySegment) FF.Unit3_crossPoint3Unit3H.h.invokeExact(
                p.ptr, q.ptr, H1.ptr, H2.ptr));
    }

    public static Unit3 cross(Unit3 p, Unit3 q) throws Throwable {
        return new Unit3((MemorySegment) FF.Unit3_crossUnit3Unit3.h.invokeExact(
                p.ptr, q.ptr));
    }

    public Unit3 cross(Unit3 q) throws Throwable {
        return new Unit3((MemorySegment) FF.Unit3_crossUnit3.h.invokeExact(ptr, q.ptr));
    }

    public Point3 cross(Point3 q) throws Throwable {
        return new Point3((MemorySegment) FF.Unit3_crossPoint3.h.invokeExact(ptr, q.ptr));
    }
}

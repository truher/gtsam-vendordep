package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Rot3 extends ForeignObject implements LieGroup<Rot3, Vector3> {
    public enum FF {
        Rot3Point3(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3(ADDRESS,
                JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
                JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
                JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Rot3Matrix3(ADDRESS, ADDRESS),
        Rot3_delete(null, ADDRESS),
        Rot3_Ypr(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Rot3_Rodrigues(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Rot3_AxisAngle(ADDRESS, ADDRESS, JAVA_DOUBLE),
        Rot3_matrix(ADDRESS, ADDRESS),
        Rot3_compose(ADDRESS, ADDRESS, ADDRESS),
        Rot3_composeH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_between(ADDRESS, ADDRESS, ADDRESS),
        Rot3_betweenH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_inverse(ADDRESS, ADDRESS),
        Rot3_inverseH(ADDRESS, ADDRESS, ADDRESS),
        Rot3_AdjointMap(ADDRESS, ADDRESS),
        Rot3_transpose(ADDRESS, ADDRESS),
        Rot3_localCoordinates(ADDRESS, ADDRESS, ADDRESS),
        Rot3_localCoordinatesH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_retract(ADDRESS, ADDRESS, ADDRESS),
        Rot3_retractH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        // Rot3_logmap(ADDRESS, ADDRESS, ADDRESS),
        // Rot3_expmap(ADDRESS, ADDRESS, ADDRESS),
        Rot3_expmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_logmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
             Rot3_OriginRetract(ADDRESS, ADDRESS),
        Rot3_OriginLocalCoordinates(ADDRESS, ADDRESS),
        Rot3_OriginRetractH(ADDRESS, ADDRESS, ADDRESS),
        Rot3_OriginLocalCoordinatesH(ADDRESS, ADDRESS, ADDRESS),
        Rot3_Expmap(ADDRESS, ADDRESS),
        Rot3_ExpmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_Logmap(ADDRESS, ADDRESS),
        Rot3_LogmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_check_group_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Rot3_check_manifold_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public static class Rot3Traits implements LieGroup.Traits<Rot3, Vector3> {

        @Override
        public Rot3 Identity() throws Throwable {
            return statics.Identity();
        }

        @Override
        public Vector3 Logmap(Rot3 g) throws Throwable {
            return statics.Logmap(g);
        }

        @Override
        public Vector3 Logmap(Rot3 g, Matrix H) throws Throwable {
            return statics.Logmap(g, H);
        }

        @Override
        public Rot3 Expmap(Vector3 v) throws Throwable {
            return statics.Expmap(v);
        }

        @Override
        public Rot3 Expmap(Vector3 v, Matrix H) throws Throwable {
            return statics.Expmap(v, H);
        }

    }

    public static final Rot3Traits traits = new Rot3Traits();

    @Override
    public Traits<Rot3, Vector3> traits() {
        return traits;
    }

    public static class Rot3Statics implements LieGroup.Statics<Rot3, Vector3> {
        @Override
        public Rot3 Identity() throws Throwable {
            return new Rot3();
        }

        @Override
        public Vector3 Logmap(Rot3 g) throws Throwable {
            return new Vector3((MemorySegment) FF.Rot3_Logmap.h.invokeExact(g.ptr));
        }

        @Override
        public Vector3 Logmap(Rot3 g, Matrix H) throws Throwable {
            return new Vector3((MemorySegment) FF.Rot3_LogmapH.h.invokeExact(
                    g.ptr, H.ptr));
        }

        @Override
        public Rot3 Expmap(Vector3 v) throws Throwable {
            return new Rot3((MemorySegment) FF.Rot3_Expmap.h.invokeExact(v.ptr));
        }

        @Override
        public Rot3 Expmap(Vector3 v, Matrix H) throws Throwable {
            return new Rot3((MemorySegment) FF.Rot3_ExpmapH.h.invokeExact(
                    v.ptr, H.ptr));
        }

              @Override
        public Rot3 Retract(Vector3 v) throws Throwable {
            return new Rot3((MemorySegment) FF.Rot3_OriginRetract.h.invokeExact(v.ptr));
        }

        @Override
        public Vector3 LocalCoordinates(Rot3 g) throws Throwable {
            return new Vector3((MemorySegment) FF.Rot3_OriginLocalCoordinates.h.invokeExact(g.ptr));
        }

        @Override
        public Rot3 Retract(Vector3 v, Matrix H) throws Throwable {
            return new Rot3((MemorySegment) FF.Rot3_OriginRetractH.h.invokeExact(v.ptr, H.ptr));
        }

        @Override
        public Vector3 LocalCoordinates(Rot3 g, Matrix H) throws Throwable {
            return new Vector3((MemorySegment) FF.Rot3_OriginLocalCoordinatesH.h.invokeExact(g.ptr, H.ptr));
        }
    }

    public static final Rot3Statics statics = new Rot3Statics();

    @Override
    public Rot3Statics statics() {
        return statics;
    }

    public Rot3(MemorySegment p) {
        super(p, FF.Rot3_delete.h);
    }

    public Rot3(Point3 col1, Point3 col2, Point3 col3) throws Throwable {
        this((MemorySegment) FF.Rot3Point3.h.invokeExact(
                col1.ptr, col2.ptr, col3.ptr));
    }

    public Rot3() throws Throwable {
        this(Matrix3.identity());
    }

    public Rot3( //
            double R11, double R12, double R13, //
            double R21, double R22, double R23, //
            double R31, double R32, double R33) throws Throwable {
        this((MemorySegment) FF.Rot3.h.invokeExact(//
                R11, R12, R13, //
                R21, R22, R23, //
                R31, R32, R33));
    }

    public Rot3(Matrix3 R) throws Throwable {
        this((MemorySegment) FF.Rot3Matrix3.h.invokeExact(R.ptr));
    }

    public static Rot3 Ypr(double y, double p, double r) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_Ypr.h.invokeExact(y, p, r));
    }

    public static Rot3 Rodrigues(double wx, double wy, double wz) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_Rodrigues.h.invokeExact(wx, wy, wz));
    }

    public static Rot3 AxisAngle(Point3 axis, double angle) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_AxisAngle.h.invokeExact(axis.ptr, angle));
    }

    @Override
    public Vector3 dxZero() throws Throwable {
        return new Vector3();
    }

    @Override
    public int dimension() throws Throwable {
        return 3;
    }

    @Override
    public Vector3 localCoordinates(Rot3 g) throws Throwable {
        return new Vector3((MemorySegment) FF.Rot3_localCoordinates.h.invokeExact(ptr, g.ptr));
    }

    @Override
    public Vector3 localCoordinates(Rot3 g, Matrix H1, Matrix H2) throws Throwable {
        return new Vector3((MemorySegment) FF.Rot3_localCoordinatesH.h.invokeExact(ptr, g.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Rot3 retract(Vector3 v) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_retract.h.invokeExact(ptr, v.ptr));
    }

    @Override
    public Rot3 retract(Vector3 v, Matrix H1, Matrix H2) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_retract.h.invokeExact(
                ptr, v.ptr, H1.ptr, H2.ptr));
    }

    // public Vector3 logmap(Rot3 g) throws Throwable {
    // return new Vector3((MemorySegment) FF.Rot3_logmap.h.invokeExact(ptr, g.ptr));
    // }

    // public Rot3 expmap(Vector3 v) throws Throwable {
    // return new Rot3((MemorySegment) FF.Rot3_expmap.h.invokeExact(ptr, v.ptr));
    // }

    @Override
    public Rot3 expmap(Vector3 v, Matrix H1, Matrix H2) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_expmapH.h.invokeExact(
                ptr, v.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Vector3 logmap(Rot3 g, Matrix H1, Matrix H2) throws Throwable {
        return new Vector3((MemorySegment) FF.Rot3_logmapH.h.invokeExact(
                ptr, g.ptr, H1.ptr, H2.ptr));
    }

    public Matrix3 matrix() throws Throwable {
        return new Matrix3((MemorySegment) FF.Rot3_matrix.h.invokeExact(ptr));
    }

    @Override
    public Rot3 compose(Rot3 other) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_compose.h.invokeExact(ptr, other.ptr));
    }

    @Override
    public Rot3 compose(Rot3 other, Matrix H1, Matrix H2) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_composeH.h.invokeExact(
                ptr, other.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Rot3 between(Rot3 g) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_between.h.invokeExact(ptr, g.ptr));
    }

    @Override
    public Rot3 between(Rot3 g, Matrix H1, Matrix H2) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_betweenH.h.invokeExact(
                ptr, g.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Rot3 inverse() throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_inverse.h.invokeExact(ptr));
    }

    @Override
    public Rot3 inverse(Matrix H) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_inverseH.h.invokeExact(ptr, H.ptr));
    }

    @Override
    public Matrix AdjointMap() throws Throwable {
        return new Matrix((MemorySegment) FF.Rot3_AdjointMap.h.invokeExact(ptr));
    }

    public Matrix3 transpose() throws Throwable {
        return new Matrix3((MemorySegment) FF.Rot3_transpose.h.invokeExact(ptr));
    }

    public static boolean check_group_invariants(Rot3 a, Rot3 b) throws Throwable {
        return (boolean) FF.Rot3_check_group_invariants.h.invokeExact(a.ptr, b.ptr);
    }

    public static boolean check_manifold_invariants(Rot3 a, Rot3 b) throws Throwable {
        return (boolean) FF.Rot3_check_manifold_invariants.h.invokeExact(a.ptr, b.ptr);
    }

}

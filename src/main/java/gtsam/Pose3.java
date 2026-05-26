package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Pose3 extends ForeignObject implements LieGroup<Pose3, Vector6> {
    public enum FF {
        Pose3(ADDRESS, ADDRESS, ADDRESS),
        Pose3_delete(null, ADDRESS),
        Pose3_Pose2(ADDRESS, ADDRESS),
        Pose3_localCoordinates(ADDRESS, ADDRESS, ADDRESS),
        Pose3_compose(ADDRESS, ADDRESS, ADDRESS),
        Pose3_retract(ADDRESS, ADDRESS, ADDRESS),
        Pose3_between(ADDRESS, ADDRESS, ADDRESS),
        Pose3_inverse(ADDRESS, ADDRESS),
        Pose3_inverseH(ADDRESS, ADDRESS, ADDRESS),
        Pose3_AdjointMap(ADDRESS, ADDRESS),
        Pose3_Expmap(ADDRESS, ADDRESS),
        Pose3_Logmap(ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public static class Traits implements LieGroup.Traits<Pose3, Vector6> {
        @Override
        public Pose3 Identity() throws Throwable {
            return new Pose3();
        }

        @Override
        public Pose3 Expmap(Vector6 xi) throws Throwable {
            return new Pose3((MemorySegment) FF.Pose3_Expmap.h.invokeExact(xi.ptr));
        }

        @Override
        public Vector6 Logmap(Pose3 g) throws Throwable {
            return new Vector6((MemorySegment) FF.Pose3_Logmap.h.invokeExact(g.ptr));
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
        super(p, FF.Pose3_delete.h);
    }

    public Pose3() throws Throwable {
        // TODO: avoid these
        this(new Rot3(), new Point3(0, 0, 0));
    }

    /** Copies the arguments. */
    public Pose3(Rot3 r, Point3 t) throws Throwable {
        this((MemorySegment) FF.Pose3.h.invokeExact(r.ptr, t.ptr));
    }

    public Pose3(Pose2 p) throws Throwable {
        this((MemorySegment) FF.Pose3_Pose2.h.invokeExact(p.ptr));
    }

    @Override
    public Vector6 localCoordinates(Pose3 g) throws Throwable {
        return new Vector6(
                (MemorySegment) FF.Pose3_localCoordinates.h.invokeExact(ptr, g.ptr));
    }

    public Pose3 compose(Pose3 p2) throws Throwable {
        return new Pose3((MemorySegment) FF.Pose3_compose.h.invokeExact(ptr, p2.ptr));
    }

    @Override
    public int dimension() {
        return 6;
    }

    @Override
    public Pose3 retract(Vector6 v) throws Throwable {
        return new Pose3((MemorySegment) FF.Pose3_retract.h.invokeExact(ptr, v.ptr()));
    }

    @Override
    public Pose3 inverse() throws Throwable {
        return new Pose3((MemorySegment) FF.Pose3_inverse.h.invokeExact(ptr));
    }

    public Pose3 inverse(Matrix H) throws Throwable {
        return new Pose3((MemorySegment) FF.Pose3_inverseH.h.invokeExact(ptr, H.ptr));
    }

    @Override
    public Pose3 between(Pose3 other) throws Throwable {
        return new Pose3((MemorySegment) FF.Pose3_between.h.invokeExact(ptr, other.ptr));
    }

    /** underlying AdjointMap returns Matrix3 but we coerce to dynamic. */
    public Matrix AdjointMap() throws Throwable {
        return new Matrix((MemorySegment) FF.Pose3_AdjointMap.h.invokeExact(ptr));
    }
}

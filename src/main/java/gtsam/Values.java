package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * Note that sometimes "Values" should be owned and sometimes
 * only observed, e.g. inside the CustomFactor error function.
 * 
 * All the things "inserted" here are copied; it's ok to delete them after
 * insertion.
 */
public class Values extends ForeignObject {
    public enum FF {
        Values(ADDRESS),
        Values_delete(null, ADDRESS),
        Values_insertValues(null, ADDRESS, ADDRESS),
        Values_insertDouble(null, ADDRESS, JAVA_LONG, JAVA_DOUBLE),
        Values_insertPoint2(null, ADDRESS, JAVA_LONG, ADDRESS),
        Values_insertPoint3(null, ADDRESS, JAVA_LONG, ADDRESS),
        Values_insertPose2(null, ADDRESS, JAVA_LONG, ADDRESS),
        Values_insertPose3(null, ADDRESS, JAVA_LONG, ADDRESS),
        Values_insertRot2(null, ADDRESS, JAVA_LONG, ADDRESS),
        Values_insertRot3(null, ADDRESS, JAVA_LONG, ADDRESS),
        Values_insertCal3DS2(null, ADDRESS, JAVA_LONG, ADDRESS),
        Values_atDouble(JAVA_DOUBLE, ADDRESS, JAVA_LONG),
        Values_atPoint2(ADDRESS, ADDRESS, JAVA_LONG),
        Values_atPoint3(ADDRESS, ADDRESS, JAVA_LONG),
        Values_atPose2(ADDRESS, ADDRESS, JAVA_LONG),
        Values_atPose3(ADDRESS, ADDRESS, JAVA_LONG),
        Values_atCal3DS2(ADDRESS, ADDRESS, JAVA_LONG),
        Values_exists(JAVA_BOOLEAN, ADDRESS, JAVA_LONG),
        Values_clear(null, ADDRESS),
        Values_size(JAVA_LONG, ADDRESS),
        Values_print(null, ADDRESS),
        Values_retract(ADDRESS, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    /**
     * @param p       Pointer to C++ Values object.
     * @param deleter Pass null for observed values.
     */
    private Values(MemorySegment p, MethodHandle deleter) {
        super(p, deleter);
    }

    /**
     * Will be deleted upon GC.
     */
    public Values() throws Throwable {
        this((MemorySegment) FF.Values.h.invokeExact(), FF.Values_delete.h);
    }

    /**
     * Will NOT be deleted upon GC.
     * 
     * For the "observer" case, e.g. in CustomFactor error function.
     */
    public static Values observed(MemorySegment p) {
        return new Values(p, null);
    }

    /**
     * Will be deleted upon GC.
     */
    public static Values owned(MemorySegment p) {
        return new Values(p, FF.Values_delete.h);
    }

    public void print(String label) throws Throwable {
        System.out.println(label);
        FF.Values_print.h.invokeExact(ptr);
    }

    public void insert(Values v) throws Throwable {
        FF.Values_insertValues.h.invokeExact(ptr, v.ptr);
    }

    public void insert(Key j, double p) throws Throwable {
        FF.Values_insertDouble.h.invokeExact(ptr, j.j, p);
    }

    public void insert(Key j, Point2 p) throws Throwable {
        FF.Values_insertPoint2.h.invokeExact(ptr, j.j, p.ptr);
    }

    public void insert(Key j, Point3 p) throws Throwable {
        FF.Values_insertPoint3.h.invokeExact(ptr, j.j, p.ptr);
    }

    public void insert(Key j, Pose2 p) throws Throwable {
        FF.Values_insertPose2.h.invokeExact(ptr, j.j, p.ptr);
    }

    public void insert(Key j, Pose3 p) throws Throwable {
        FF.Values_insertPose3.h.invokeExact(ptr, j.j, p.ptr);
    }

    public void insert(Key j, Rot2 p) throws Throwable {
        FF.Values_insertRot2.h.invokeExact(ptr, j.j, p.ptr);
    }

    public void insert(Key j, Rot3 p) throws Throwable {
        FF.Values_insertRot3.h.invokeExact(ptr, j.j, p.ptr);
    }

    public void insert(Key j, Cal3DS2 p) throws Throwable {
        FF.Values_insertCal3DS2.h.invokeExact(ptr, j.j, p.ptr);
    }

    public double atDouble(Key j) throws Throwable {
        return (double) FF.Values_atDouble.h.invokeExact(ptr, j.j);
    }

    public Point2 atPoint2(Key j) throws Throwable {
        return new Point2((MemorySegment) FF.Values_atPoint2.h.invokeExact(ptr, j.j));
    }

    public Point3 atPoint3(Key j) throws Throwable {
        return new Point3((MemorySegment) FF.Values_atPoint3.h.invokeExact(ptr, j.j));
    }

    public Pose2 atPose2(Key j) throws Throwable {
        return new Pose2((MemorySegment) FF.Values_atPose2.h.invokeExact(ptr, j.j));
    }

    public Pose3 atPose3(Key j) throws Throwable {
        return new Pose3((MemorySegment) FF.Values_atPose3.h.invokeExact(ptr, j.j));
    }

    public Cal3DS2 atCal3DS2(Key j) throws Throwable {
        return new Cal3DS2((MemorySegment) FF.Values_atCal3DS2.h.invokeExact(ptr, j.j));
    }

    public boolean exists(Key j) throws Throwable {
        return (boolean) FF.Values_exists.h.invokeExact(ptr, j.j);
    }

    public void clear() throws Throwable {
        FF.Values_clear.h.invokeExact(ptr);
    }

    public long size() throws Throwable {
        return (long) FF.Values_size.h.invokeExact(ptr);
    }

    public Values retract(VectorValues delta) throws Throwable {
        return Values.owned((MemorySegment) FF.Values_retract.h.invokeExact(
                ptr, delta.ptr));
    }
}

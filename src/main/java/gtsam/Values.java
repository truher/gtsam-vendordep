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
        Values_insertValues(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Values_insertDouble(JAVA_BOOLEAN, ADDRESS, JAVA_LONG, JAVA_DOUBLE),
        Values_insertPoint2(JAVA_BOOLEAN, ADDRESS, JAVA_LONG, ADDRESS),
        Values_insertPoint3(JAVA_BOOLEAN, ADDRESS, JAVA_LONG, ADDRESS),
        Values_insertPose2(JAVA_BOOLEAN, ADDRESS, JAVA_LONG, ADDRESS),
        Values_insertPose3(JAVA_BOOLEAN, ADDRESS, JAVA_LONG, ADDRESS),
        Values_insertRot2(JAVA_BOOLEAN, ADDRESS, JAVA_LONG, ADDRESS),
        Values_insertRot3(JAVA_BOOLEAN, ADDRESS, JAVA_LONG, ADDRESS),
        Values_keys(ADDRESS, ADDRESS),
        Values_insertCal3DS2(JAVA_BOOLEAN, ADDRESS, JAVA_LONG, ADDRESS),
        Values_atDouble(ADDRESS, ADDRESS, JAVA_LONG),
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

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public class ValuesKeyAlreadyExists extends RuntimeException {
        public ValuesKeyAlreadyExists() {
        }

        public ValuesKeyAlreadyExists(Key k) {
            super(String.format("key: %s", k));
        }
    }

    /**
     * Values.at() can throw two different exceptions but we catch
     * them both and return null, so in Java we can't differentiate
     * them. To see which exception was actually thrown, look at
     * stdout.
     */
    public class AtException extends RuntimeException {
        public AtException(Key k) {
            super(String.format("key: %s", k));
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
    public Values() {
        this((MemorySegment) FF.Values.invoke(), FF.Values_delete.h);
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

    public void print(String label) {
        System.out.println(label);
        FF.Values_print.invoke(ptr);
    }

    public void insert(Values v) {
        if (!((boolean) FF.Values_insertValues.invoke(ptr, v.ptr))) {
            v.print("duplicate key somewhere here");
            throw new ValuesKeyAlreadyExists();
        }
    }

    public void insert(Key j, double p) {
        if (!((boolean) FF.Values_insertDouble.invoke(ptr, j.j, p)))
            throw new ValuesKeyAlreadyExists(j);
    }

    public void insert(Key j, Point2 p) {
        if (!((boolean) FF.Values_insertPoint2.invoke(ptr, j.j, p.ptr)))
            throw new ValuesKeyAlreadyExists(j);
    }

    public void insert(Key j, Point3 p) {
        if (!((boolean) FF.Values_insertPoint3.invoke(ptr, j.j, p.ptr)))
            throw new ValuesKeyAlreadyExists(j);
    }

    public void insert(Key j, Pose2 p) {
        if (!((boolean) FF.Values_insertPose2.invoke(ptr, j.j, p.ptr)))
            throw new ValuesKeyAlreadyExists(j);
    }

    public void insert(Key j, Pose3 p) {
        if (!((boolean) FF.Values_insertPose3.invoke(ptr, j.j, p.ptr)))
            throw new ValuesKeyAlreadyExists(j);
    }

    public void insert(Key j, Rot2 p) {
        if (!((boolean) FF.Values_insertRot2.invoke(ptr, j.j, p.ptr)))
            throw new ValuesKeyAlreadyExists(j);
    }

    public void insert(Key j, Rot3 p) {
        if (!((boolean) FF.Values_insertRot3.invoke(ptr, j.j, p.ptr)))
            throw new ValuesKeyAlreadyExists(j);
    }

    public void insert(Key j, Cal3DS2 p) {
        if (!((boolean) FF.Values_insertCal3DS2.invoke(ptr, j.j, p.ptr)))
            throw new ValuesKeyAlreadyExists(j);
    }

    public KeyVector keys() {
        return new KeyVector((MemorySegment) FF.Values_keys.invoke(ptr));
    }

    public double atDouble(Key j) {
        MemorySegment p = (MemorySegment) FF.Values_atDouble.invoke(ptr, j.j);
        if (p.equals(MemorySegment.NULL)) {
            throw new AtException(j);
        }
        return new Vector1(p).at(0);
    }

    public Point2 atPoint2(Key j) {
        MemorySegment p = (MemorySegment) FF.Values_atPoint2.invoke(ptr, j.j);
        if (p.equals(MemorySegment.NULL)) {
            throw new AtException(j);
        }
        return new Point2(p);
    }

    public Point3 atPoint3(Key j) {
        MemorySegment p = (MemorySegment) FF.Values_atPoint3.invoke(ptr, j.j);
        if (p.equals(MemorySegment.NULL)) {
            throw new AtException(j);
        }
        return new Point3(p);
    }

    public Pose2 atPose2(Key j) {
        MemorySegment p = (MemorySegment) FF.Values_atPose2.invoke(ptr, j.j);
        if (p.equals(MemorySegment.NULL)) {
            throw new AtException(j);
        }
        return new Pose2(p);
    }

    public Pose3 atPose3(Key j) {
        MemorySegment p = (MemorySegment) FF.Values_atPose3.invoke(ptr, j.j);
        if (p.equals(MemorySegment.NULL)) {
            throw new AtException(j);
        }
        return new Pose3(p);
    }

    public Cal3DS2 atCal3DS2(Key j) {
        MemorySegment p = (MemorySegment) FF.Values_atCal3DS2.invoke(ptr, j.j);
        if (p.equals(MemorySegment.NULL)) {
            throw new AtException(j);
        }
        return new Cal3DS2(p);
    }

    public boolean exists(Key j) {
        return (boolean) FF.Values_exists.invoke(ptr, j.j);
    }

    public void clear() {
        FF.Values_clear.invoke(ptr);
    }

    public long size() {
        return (long) FF.Values_size.invoke(ptr);
    }

    public Values retract(VectorValues delta) {
        return Values.owned((MemorySegment) FF.Values_retract.invoke(
                ptr, delta.ptr));
    }
}

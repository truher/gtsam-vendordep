package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * Note that sometimes "Values" should be owned and sometimes
 * only observed, e.g. inside the CustomFactor error function.
 */
public class Values extends ForeignObject {
    private static final MethodHandle Values = Lib.down(
            "Values", ADDRESS);
    private static final MethodHandle Values_delete = Lib.downVoid(
            "Values_delete", ADDRESS);
    private static final MethodHandle Values_insertPose2 = Lib.downVoid(
            "Values_insertPose2", ADDRESS, JAVA_LONG, ADDRESS);
    private static final MethodHandle Values_at = Lib.down(
            "Values_atPose2", ADDRESS, ADDRESS, JAVA_LONG);
    private static final MethodHandle Values_exists = Lib.down(
            "Values_exists", JAVA_BOOLEAN, ADDRESS, JAVA_LONG);
    private static final MethodHandle Values_clear = Lib.downVoid(
            "Values_clear", ADDRESS);
    private static final MethodHandle Values_size = Lib.down(
            "Values_size", JAVA_LONG, ADDRESS);
    private static final MethodHandle Values_print = Lib.downVoid(
            "Values_print", ADDRESS);

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
        this((MemorySegment) Values.invokeExact(), Values_delete);
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
        return new Values(p, Values_delete);
    }

    public void print() throws Throwable {
        Values_print.invokeExact(ptr);
    }

    /** Clones p, ok to delete after this. */
    public void insert(Key j, Pose2 p) throws Throwable {
        Values_insertPose2.invokeExact(ptr, j.j, p.ptr);
    }

    public Pose2 atPose2(Key j) throws Throwable {
        return new Pose2((MemorySegment) Values_at.invokeExact(ptr, j.j));
    }

    public boolean exists(Key j) throws Throwable {
        return (boolean) Values_exists.invokeExact(ptr, j.j);
    }

    public void clear() throws Throwable {
        Values_clear.invokeExact(ptr);
    }

    public long size() throws Throwable {
        return (long) Values_size.invokeExact(ptr);
    }
}

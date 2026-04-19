package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * a typedef.
 * see gtsam/inference/Key.h
 * using KeyVector = FastVector<Key>;
 * since we're not using TBB, this is just std::vector<Key>;
 */
public class KeyVector extends ForeignObject {
    private static final MethodHandle KeyVector = Lib.down(
            "KeyVector", ADDRESS);
    private static final MethodHandle KeyVector_delete = Lib.downVoid(
            "KeyVector_delete", ADDRESS);
    private static final MethodHandle KeyVector_push_back = Lib.downVoid(
            "KeyVector_push_back", ADDRESS, JAVA_LONG);
    private static final MethodHandle KeyVector_at = Lib.down(
            "KeyVector_at", JAVA_LONG, ADDRESS, JAVA_LONG);
    private static final MethodHandle KeyVector_size = Lib.down(
            "KeyVector_size", JAVA_LONG, ADDRESS);

    public KeyVector(MemorySegment p) {
        super(p, KeyVector_delete);
    }

    public KeyVector() throws Throwable {
        this((MemorySegment) KeyVector.invokeExact());
    }

    public KeyVector(Key... keys) throws Throwable {
        this();
        for (Key k : keys) {
            push_back(k);
        }
    }

    public void push_back(Key key) throws Throwable {
        KeyVector_push_back.invokeExact(ptr, key.j);
    }

    public Key at(long i) throws Throwable {
        return new Key((long) KeyVector_at.invokeExact(ptr, i));
    }

    public long size() throws Throwable {
        return (long) KeyVector_size.invokeExact(ptr);
    }
}

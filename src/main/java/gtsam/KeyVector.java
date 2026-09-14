package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
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
    public enum FF {
        KeyVector(ADDRESS),
        KeyVector_delete(null, ADDRESS),
        KeyVector_push_back(null, ADDRESS, JAVA_LONG),
        KeyVector_at(JAVA_LONG, ADDRESS, JAVA_LONG),
        KeyVector_size(JAVA_LONG, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public KeyVector(MemorySegment p) {
        super(p, FF.KeyVector_delete.h);
    }

    public KeyVector() throws Throwable {
        this((MemorySegment) FF.KeyVector.h.invokeExact());
    }

    public KeyVector(Key... keys) throws Throwable {
        this();
        for (Key k : keys) {
            push_back(k);
        }
    }

    public void push_back(Key key) throws Throwable {
        FF.KeyVector_push_back.h.invokeExact(ptr, key.j);
    }

    public Key at(long i) throws Throwable {
        return new Key((long) FF.KeyVector_at.h.invokeExact(ptr, i));
    }

    public long size() throws Throwable {
        return (long) FF.KeyVector_size.h.invokeExact(ptr);
    }
}

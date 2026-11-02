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

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public KeyVector(MemorySegment p) {
        super(p, FF.KeyVector_delete.h);
    }

    public KeyVector() {
        this((MemorySegment) FF.KeyVector.invoke());
    }

    public KeyVector(Key... keys) {
        this();
        for (Key k : keys) {
            push_back(k);
        }
    }

    public void push_back(Key key) {
        FF.KeyVector_push_back.invoke(ptr, key.j);
    }

    public Key at(long i) {
        return new Key((long) FF.KeyVector_at.invoke(ptr, i));
    }

    public long size() {
        return (long) FF.KeyVector_size.invoke(ptr);
    }
}

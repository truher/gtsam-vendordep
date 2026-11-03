package gtsam;

/**
 * This is a reimplementation of symbol.cpp, rather than a proxy, because it is
 * used a lot (so calling through FFM many many times seems bad), and it is
 * simple.
 * 
 * The constants here are platform-dependent, watch out.
 */
public class Symbol {
    private static final long keyBits = 64l;
    private static final long chrBits = 8l;
    private static final long indexBits = keyBits - chrBits;
    private static final long chrMask = 255l << indexBits;
    private static final long indexMask = ~chrMask;

    private final char c_;
    private final long j_;

    public Symbol(Key key) {
        c_ = (char) ((key.j & chrMask) >> indexBits);
        j_ = key.j & indexMask;
    }

    public char chr() {
        return c_;
    }

    public long index() {
        return j_;
    }

    @Override
    public String toString() {
        return String.format("%c%d", c_, j_);
    }

}

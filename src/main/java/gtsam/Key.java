package gtsam;

import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/**
 * in gtsam a key is a typedef for uint64.
 * TODO: remove the duplication here.
 */
public class Key {

    private static final MethodHandle symbol_shorthand_A = symbol("A");
    private static final MethodHandle symbol_shorthand_B = symbol("B");
    private static final MethodHandle symbol_shorthand_C = symbol("C");
    private static final MethodHandle symbol_shorthand_D = symbol("D");
    private static final MethodHandle symbol_shorthand_E = symbol("E");
    private static final MethodHandle symbol_shorthand_F = symbol("F");
    private static final MethodHandle symbol_shorthand_G = symbol("G");
    private static final MethodHandle symbol_shorthand_H = symbol("H");
    private static final MethodHandle symbol_shorthand_I = symbol("I");
    private static final MethodHandle symbol_shorthand_J = symbol("J");
    private static final MethodHandle symbol_shorthand_K = symbol("K");
    private static final MethodHandle symbol_shorthand_L = symbol("L");
    private static final MethodHandle symbol_shorthand_M = symbol("M");
    private static final MethodHandle symbol_shorthand_N = symbol("N");
    private static final MethodHandle symbol_shorthand_O = symbol("O");
    private static final MethodHandle symbol_shorthand_P = symbol("P");
    private static final MethodHandle symbol_shorthand_Q = symbol("Q");
    private static final MethodHandle symbol_shorthand_R = symbol("R");
    private static final MethodHandle symbol_shorthand_S = symbol("S");
    private static final MethodHandle symbol_shorthand_T = symbol("T");
    private static final MethodHandle symbol_shorthand_U = symbol("U");
    private static final MethodHandle symbol_shorthand_V = symbol("V");
    private static final MethodHandle symbol_shorthand_W = symbol("W");
    private static final MethodHandle symbol_shorthand_X = symbol("X");
    private static final MethodHandle symbol_shorthand_Y = symbol("Y");
    private static final MethodHandle symbol_shorthand_Z = symbol("Z");

    private static final MethodHandle symbol(String label) {
        return Lib.ff("symbol_shorthand_" + label, JAVA_LONG, JAVA_LONG);
    }

    public final long j;

    public Key(long _j) {
        j = _j;
    }

    public static Key A(long j) throws Throwable {
        return new Key((long) symbol_shorthand_A.invokeExact(j));
    }

    public static Key B(long j) throws Throwable {
        return new Key((long) symbol_shorthand_B.invokeExact(j));
    }

    public static Key C(long j) throws Throwable {
        return new Key((long) symbol_shorthand_C.invokeExact(j));
    }

    public static Key D(long j) throws Throwable {
        return new Key((long) symbol_shorthand_D.invokeExact(j));
    }

    public static Key E(long j) throws Throwable {
        return new Key((long) symbol_shorthand_E.invokeExact(j));
    }

    public static Key F(long j) throws Throwable {
        return new Key((long) symbol_shorthand_F.invokeExact(j));
    }

    public static Key G(long j) throws Throwable {
        return new Key((long) symbol_shorthand_G.invokeExact(j));
    }

    public static Key H(long j) throws Throwable {
        return new Key((long) symbol_shorthand_H.invokeExact(j));
    }

    public static Key I(long j) throws Throwable {
        return new Key((long) symbol_shorthand_I.invokeExact(j));
    }

    public static Key J(long j) throws Throwable {
        return new Key((long) symbol_shorthand_J.invokeExact(j));
    }

    public static Key K(long j) throws Throwable {
        return new Key((long) symbol_shorthand_K.invokeExact(j));
    }

    public static Key L(long j) throws Throwable {
        return new Key((long) symbol_shorthand_L.invokeExact(j));
    }

    public static Key M(long j) throws Throwable {
        return new Key((long) symbol_shorthand_M.invokeExact(j));
    }

    public static Key N(long j) throws Throwable {
        return new Key((long) symbol_shorthand_N.invokeExact(j));
    }

    public static Key O(long j) throws Throwable {
        return new Key((long) symbol_shorthand_O.invokeExact(j));
    }

    public static Key P(long j) throws Throwable {
        return new Key((long) symbol_shorthand_P.invokeExact(j));
    }

    public static Key Q(long j) throws Throwable {
        return new Key((long) symbol_shorthand_Q.invokeExact(j));
    }

    public static Key R(long j) throws Throwable {
        return new Key((long) symbol_shorthand_R.invokeExact(j));
    }

    public static Key S(long j) throws Throwable {
        return new Key((long) symbol_shorthand_S.invokeExact(j));
    }

    public static Key T(long j) throws Throwable {
        return new Key((long) symbol_shorthand_T.invokeExact(j));
    }

    public static Key U(long j) throws Throwable {
        return new Key((long) symbol_shorthand_U.invokeExact(j));
    }

    public static Key V(long j) throws Throwable {
        return new Key((long) symbol_shorthand_V.invokeExact(j));
    }

    public static Key W(long j) throws Throwable {
        return new Key((long) symbol_shorthand_W.invokeExact(j));
    }

    public static Key X(long j) throws Throwable {
        return new Key((long) symbol_shorthand_X.invokeExact(j));
    }

    public static Key Y(long j) throws Throwable {
        return new Key((long) symbol_shorthand_Y.invokeExact(j));
    }

    public static Key Z(long j) throws Throwable {
        return new Key((long) symbol_shorthand_Z.invokeExact(j));
    }

}

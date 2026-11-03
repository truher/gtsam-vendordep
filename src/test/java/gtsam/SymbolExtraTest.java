package gtsam;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class SymbolExtraTest {
    @Test
    void test0() {
        Symbol s0 = new Symbol(Key.X(0));
        assertEquals('x', s0.chr());
        assertEquals(0, s0.index());

        Symbol s1 = new Symbol(Key.B(100));
        assertEquals('b', s1.chr());
        assertEquals(100, s1.index());
    }

}

package org.team100.foreign;

import java.lang.foreign.MemoryLayout;
import java.lang.foreign.StructLayout;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.VarHandle;

public class Pairs {
    public static final StructLayout PtrPair = MemoryLayout.structLayout(
            ValueLayout.ADDRESS.withName("first"),
            ValueLayout.ADDRESS.withName("second"));
    public static final VarHandle PtrPair_first = PtrPair.varHandle(
            MemoryLayout.PathElement.groupElement("first"));
    public static final VarHandle PtrPair_second = PtrPair.varHandle(
            MemoryLayout.PathElement.groupElement("second"));
}

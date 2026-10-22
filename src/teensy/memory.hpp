#include <Arduino.h>
#include <malloc.h>

extern "C" char _ebss;
extern "C" char _estack;
extern "C" char _heap_end;
extern "C" char *__brkval;

namespace memory
{

void printFreeMemory()
{
    // RAM1: Approximate remaining stack space
    char stackMarker;
    uintptr_t stackPtr = (uintptr_t)&stackMarker;
    uintptr_t staticEnd = (uintptr_t)&_ebss;

    uint32_t freeStack =
        stackPtr > staticEnd ? stackPtr - staticEnd : 0;

    // RAM2: Heap allocator statistics
    struct mallinfo mi = mallinfo();
    // fordblks excludes RAM beyond the allocator's current break.
    uintptr_t heapBreak = (uintptr_t)__brkval;
    uintptr_t heapEnd = (uintptr_t)&_heap_end;
    uint32_t unclaimedHeap = heapEnd > heapBreak ? heapEnd - heapBreak : 0;
    uint32_t freeHeap = unclaimedHeap + mi.fordblks;

    Serial.printf("RAM1 free stack: %lu B\n",
                  (unsigned long)freeStack);

    Serial.printf("RAM2 free heap: %lu B\n",
                  (unsigned long)freeHeap);

    Serial.printf("RAM2 heap allocated: %lu B\n",
                  (unsigned long)mi.uordblks);
}

}

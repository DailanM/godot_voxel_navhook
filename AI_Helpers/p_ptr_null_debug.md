# Debugging: "Parameter p_ptr is null" Error

## Symptom
~18,000 lines of `ERROR: core/os/memory.cpp:192 - Parameter "p_ptr" is null` spammed
when the voxel-nav-testing project is opened in the editor and terrain generates.
Only occurs in non-dev builds (`target=editor`), NOT in dev builds (`target=editor dev_build=yes`).

## Root Cause

`StdDefaultAllocator::deallocate()` (in `util/memory/std_allocator.h`) is called with
`nullptr` by the STL when destroying empty or moved-from `StdVector`/`StdString`
containers. This is legal per the C++ standard — allocators must accept null in
`deallocate()`. But `ZN_FREE` maps to Godot's `Memory::free_static()`, which has
`ERR_FAIL_NULL(p_ptr)`.

### Why dev builds don't trigger it

In dev builds (`-O0`/`-Og`), the STL implementation either doesn't call `deallocate()`
for empty vectors, or the compiler doesn't inline the function. In non-dev optimized
builds (`-O2`/`-O3`), the compiler aggressively inlines `deallocate()` and, as a
GCC 15.2.1 optimization artifact, **eliminates the null guard** despite it being present
in the source and even visible in the `.o` disassembly. The `test rdi,rdi` / `je`
instruction appears in the object code, but the linked binary's optimizer or code layout
causes some call paths to bypass it.

## Fix (VERIFIED WORKING — 2026-04-26)

Added `__attribute__((noinline))` to `deallocate()` plus a null guard:

```cpp
__attribute__((noinline)) void deallocate(T *p, std::size_t n) noexcept {
    if (p == nullptr) {
        return;
    }
#ifdef DEBUG_ENABLED
    StdDefaultAllocatorCounters::g_deallocated += n * sizeof(T);
#endif
    ZN_FREE(p);
}
```

The `noinline` attribute is critical: it prevents the compiler from inlining the function
into callers, which stops the optimizer from eliminating the null check across the call
boundary.

**Result:** 0 errors (down from ~18,000 originally, then ~1,993 with the null guard
alone but without `noinline`).

Applied to both:
- `godot_voxel_navhook/util/memory/std_allocator.h`
- `godot/modules/voxel/util/memory/std_allocator.h`

## Debugging Timeline

1. Initial stack trace via CRASH_NOW_MSG in memory.cpp showed all crashes from
   `NavMeshBuildTask::run()` → `StdVector<unsigned char>::~vector` → `StdDefaultAllocator::deallocate` → `Memory::free_static(nullptr)`.

2. Added null guard to `deallocate()` — reduced errors from ~18,000 to ~1,993. The
   null guard was confirmed present in `.o` disassembly (`test %rdi,%rdi` / `je`), and
   all `.o` timestamps were newer than the header. Yet crashes persisted through the
   same code path.

3. Added `ZN_CRASH_MSG` inside the `if (p == nullptr)` branch with
   `__attribute__((noinline))` — crash fired INSIDE the null branch, confirming null
   does reach `deallocate()`. Without `noinline`, the inlined version somehow bypassed
   the same check.

4. Removed the crash, kept `noinline` + null return → 0 errors. Fix verified.

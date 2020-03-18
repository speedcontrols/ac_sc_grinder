#ifndef __YIELD_H__
#define __YIELD_H__

#include <stdbool.h>
#include <setjmp.h>

#ifdef YIELD_USE_JMP

struct _yield_state_t {
    jmp_buf env;
    bool yielded;
};

#define YIELDABLE \
    static struct _yield_state_t _yield_state; \
    if (_yield_state.yielded) longjmp(_yield_state.env, 1); \

#define YIELD(val) \
    do { \
        if (!setjmp(_yield_state.env)) { _yield_state.yielded = true; return val; } \
        else _yield_state.yielded = false; \
    } while (0)

#else

struct _yield_state_t {
    void *yield_ptr;
    bool yielded;
};

#define _yield_label3(prefix, line) prefix ## line
#define _yield_label2(prefix, line) _yield_label3(prefix, line)
#define _yield_label _yield_label2(YIELD_ENTRY_, __LINE__)

#define YIELDABLE \
    static struct _yield_state_t _yield_state; \
    if (_yield_state.yielded) goto *_yield_state.yield_ptr; \

#define YIELD(val) \
    do { \
        _yield_state.yield_ptr = &&_yield_label; \
        _yield_state.yielded = true; \
        return val; \
        _yield_label: \
        _yield_state.yielded = false; \
    } while (0)

#endif

// Helpers

#define YIELD_WHILE(cond, val) while (cond) { YIELD(val); }
#define YIELD_UNTIL(cond, val) YIELD_WHILE(!cond, val)


#endif

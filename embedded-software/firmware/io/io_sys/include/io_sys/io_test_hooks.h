/**
 * @file io_test_hooks.h
 * @brief Test-only setter/getter macros for module-private statics.
 *
 * The IO and DEV layers keep their state in `static` variables (no
 * cross-TU globals). For host-side unit tests we still need to peek
 * and poke that state — but we don't want production builds carrying
 * those access functions in flash.
 *
 * Usage in a driver / impl .c:
 *
 *   static struct foo s_self;
 *   IO_TEST_HOOK_RW(s_self, struct foo, s_self);
 *
 * This expands to nothing in a normal build. When the translation unit
 * is compiled with @c IO_TEST_HOOKS defined, it expands to:
 *
 *   struct foo IO_TEST_get_s_self(void)             { return s_self; }
 *   void       IO_TEST_set_s_self(struct foo v)     { s_self = v;    }
 *   struct foo *IO_TEST_ref_s_self(void)            { return &s_self;}
 *
 * The companion @c IO_TEST_HOOK_DECL macro produces just the
 * declarations for use in a test-only header.
 *
 * Conventions:
 *   - All exposed names are prefixed @c IO_TEST_ so they're impossible
 *     to call by accident from production code.
 *   - The macro is intentionally noisy at call sites so reviewers see
 *     when a private becomes test-visible.
 *
 * UBC Rocket, 2026
 */
#ifndef IO_TEST_HOOKS_H
#define IO_TEST_HOOKS_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(IO_TEST_HOOKS)

#include <stddef.h>

/* Generate get/set/ref functions for a module-private scalar / struct. */
#define IO_TEST_HOOK_RW(var, type, name)                                 \
    type        IO_TEST_get_##name(void)         { return var; }         \
    void        IO_TEST_set_##name(type v)       { var = v;    }         \
    type       *IO_TEST_ref_##name(void)         { return &var; }

/* Read-only variant — no setter. */
#define IO_TEST_HOOK_RO(var, type, name)                                 \
    type        IO_TEST_get_##name(void)         { return var; }         \
    type const *IO_TEST_ref_##name(void)         { return &var; }

/* Array variant: a function can't return a C array, but it can return a
 * pointer to the first element. We also expose the compile-time element
 * count so tests can iterate safely. Use the storage-class type of one
 * element for @p elem_type (e.g. uint8_t for `uint8_t s_buf[8];`). */
#define IO_TEST_HOOK_ARRAY(var, elem_type, name)                         \
    elem_type *IO_TEST_get_##name(void)          { return (var); }       \
    size_t     IO_TEST_size_##name(void) {                               \
        return sizeof(var) / sizeof((var)[0]);                           \
    }

/* Declarations only (for use in a -test.h header). */
#define IO_TEST_HOOK_DECL_RW(type, name)                                 \
    type        IO_TEST_get_##name(void);                                \
    void        IO_TEST_set_##name(type v);                              \
    type       *IO_TEST_ref_##name(void);

#define IO_TEST_HOOK_DECL_RO(type, name)                                 \
    type        IO_TEST_get_##name(void);                                \
    type const *IO_TEST_ref_##name(void);

#define IO_TEST_HOOK_DECL_ARRAY(elem_type, name)                         \
    elem_type  *IO_TEST_get_##name(void);                                \
    size_t      IO_TEST_size_##name(void);

#else  /* !IO_TEST_HOOKS — compiles to nothing in production */

#define IO_TEST_HOOK_RW(var, type, name)
#define IO_TEST_HOOK_RO(var, type, name)
#define IO_TEST_HOOK_ARRAY(var, elem_type, name)
#define IO_TEST_HOOK_DECL_RW(type, name)
#define IO_TEST_HOOK_DECL_RO(type, name)
#define IO_TEST_HOOK_DECL_ARRAY(elem_type, name)

#endif

#ifdef __cplusplus
}
#endif

#endif /* IO_TEST_HOOKS_H */

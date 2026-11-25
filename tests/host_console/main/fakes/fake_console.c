/* SPDX-License-Identifier: GPL-3.0-or-later */
/**
 * @file fake_console.c
 * @brief kodeOS's console registry for the host: the rules its kregistry.c
 *        keeps at the pinned release, so a row kodeOS would refuse - a name
 *        too long, a word it cannot carry, a name a function and a variable
 *        share - fails here before it fails on a Dot.
 */
#include "fake_console.h"

#include <string.h>

static const kode_function_t *s_ftab[KODE_FUNCTION_TABLES];
static size_t                 s_fn[KODE_FUNCTION_TABLES];
static size_t                 s_nft;
static const kode_variable_t *s_vtab[KODE_VARIABLE_TABLES];
static size_t                 s_vn[KODE_VARIABLE_TABLES];
static size_t                 s_nvt;

void console_fake_reset(void)
{
    s_nft = 0U;
    s_nvt = 0U;
}

static size_t count(const size_t *n, size_t tables)
{
    size_t c = 0U;
    for (size_t t = 0U; t < tables; t++) {
        c += n[t];
    }
    return c;
}

size_t console_fake_functions(void)
{
    return count(s_fn, s_nft);
}

size_t console_fake_variables(void)
{
    return count(s_vn, s_nvt);
}

static const kode_function_t *function_named(const char *name)
{
    for (size_t t = 0U; t < s_nft; t++) {
        for (size_t i = 0U; i < s_fn[t]; i++) {
            if (strcmp(s_ftab[t][i].name, name) == 0) {
                return &s_ftab[t][i];
            }
        }
    }
    return NULL;
}

static const kode_variable_t *variable_named(const char *name)
{
    for (size_t t = 0U; t < s_nvt; t++) {
        for (size_t i = 0U; i < s_vn[t]; i++) {
            if (strcmp(s_vtab[t][i].name, name) == 0) {
                return &s_vtab[t][i];
            }
        }
    }
    return NULL;
}

const kode_function_t *console_fake_function(const char *name)
{
    return function_named(name);
}

/** @brief One namespace for both kinds, as kodeOS keeps it. */
static bool taken(const char *name)
{
    return (function_named(name) != NULL) || (variable_named(name) != NULL);
}

/** @brief [a-z0-9_], 1..KODE_ROW_NAME_MAX: a row's name or an enum's word. */
static bool name_ok(const char *s)
{
    if (s == NULL) {
        return false;
    }
    size_t const n = strlen(s);
    if ((n == 0U) || (n > KODE_ROW_NAME_MAX)) {
        return false;
    }
    for (size_t k = 0U; k < n; k++) {
        char const c = s[k];
        if (!(((c >= 'a') && (c <= 'z')) || ((c >= '0') && (c <= '9')) || (c == '_'))) {
            return false;
        }
    }
    return true;
}

static bool desc_ok(const char *d)
{
    return (d == NULL) || (strlen(d) <= KODE_ROW_DESC_MAX);
}

static bool function_ok(const kode_function_t *r)
{
    if (((unsigned)r->arg > (unsigned)KODE_TYPE_STRING) || !name_ok(r->name) || (r->run == NULL)
        || !desc_ok(r->desc) || (r->answer_ms > KODE_FUNCTION_ANSWER_MS_MAX)) {
        return false;
    }
    if (r->arg == KODE_TYPE_ENUM) {
        if ((r->enums == NULL) || (r->n_enums == 0U) || (r->n_enums > KODE_VALUE_ENUMS_MAX)) {
            return false;
        }
        for (uint8_t k = 0U; k < r->n_enums; k++) {
            if (!name_ok(r->enums[k])) {
                return false;
            }
        }
    }
    return (r->arg != KODE_TYPE_INT) || (r->min <= r->max);
}

/** @brief The name checks both kinds make: taken already, or twice in this table. */
static kode_status_t name_free(const char *name, const char *const *before, size_t n_before)
{
    if (taken(name)) {
        return KODE_EXISTS;
    }
    for (size_t j = 0U; j < n_before; j++) {
        if (strcmp(name, before[j]) == 0) {
            return KODE_EXISTS;
        }
    }
    return KODE_OK;
}

kode_status_t kode_functions_add(const kode_function_t *table, size_t n)
{
    if ((table == NULL) || (n == 0U) || (s_nft >= KODE_FUNCTION_TABLES)) {
        return KODE_DENIED;
    }
    if ((console_fake_functions() + n) > KODE_FUNCTIONS_MAX) {
        return KODE_NO_RESOURCES;
    }
    const char *names[KODE_FUNCTIONS_MAX];
    for (size_t i = 0U; i < n; i++) {
        if (!function_ok(&table[i])) {
            return KODE_DENIED;
        }
        kode_status_t const rc = name_free(table[i].name, names, i);
        if (rc != KODE_OK) {
            return rc;
        }
        names[i] = table[i].name;
    }
    s_ftab[s_nft] = table;
    s_fn[s_nft]   = n;
    s_nft++;
    return KODE_OK;
}

kode_status_t kode_variables_add(const kode_variable_t *table, size_t n)
{
    if ((table == NULL) || (n == 0U) || (s_nvt >= KODE_VARIABLE_TABLES)) {
        return KODE_DENIED;
    }
    if ((console_fake_variables() + n) > KODE_VARIABLES_MAX) {
        return KODE_NO_RESOURCES;
    }
    const char *names[KODE_VARIABLES_MAX];
    for (size_t i = 0U; i < n; i++) {
        if (!name_ok(table[i].name) || (table[i].get == NULL) || !desc_ok(table[i].desc)) {
            return KODE_DENIED;
        }
        kode_status_t const rc = name_free(table[i].name, names, i);
        if (rc != KODE_OK) {
            return rc;
        }
        names[i] = table[i].name;
    }
    s_vtab[s_nvt] = table;
    s_vn[s_nvt]   = n;
    s_nvt++;
    return KODE_OK;
}

kode_status_t console_fake_run(const char *name, const kode_value_t *arg)
{
    const kode_function_t *const r = function_named(name);
    if (r == NULL) {
        return KODE_DENIED;
    }
    if (r->arg == KODE_TYPE_NONE) {
        return r->run(NULL);
    }
    if (arg == NULL) {
        return r->run(NULL);   /* the line carried no arg=: the row decides */
    }
    if ((r->arg == KODE_TYPE_INT) && (r->min != r->max)
        && ((arg->i < r->min) || (arg->i > r->max))) {
        return KODE_DENIED;
    }
    if ((r->arg == KODE_TYPE_ENUM) && (arg->e >= r->n_enums)) {
        return KODE_DENIED;
    }
    return r->run(arg);
}

kode_status_t console_fake_read(const char *name, char word[KODE_VALUE_STR_MAX + 1U])
{
    const kode_variable_t *const r = variable_named(name);
    if (r == NULL) {
        return KODE_DENIED;
    }
    char buf[KODE_VALUE_STR_MAX + 2U];   /* room to tell 32 from longer */
    (void)memset(buf, 0, sizeof(buf));
    kode_status_t const rc = r->get(buf, sizeof(buf));
    if (rc != KODE_OK) {
        return rc;
    }
    size_t const n = strlen(buf);
    if ((n == 0U) || (n > KODE_VALUE_STR_MAX)) {
        return KODE_DENIED;
    }
    (void)memcpy(word, buf, n + 1U);
    return KODE_OK;
}

#include "lcmgen.h"

#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <inttypes.h>
#include <limits.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "tokenize.h"

/** The LCM grammar is implemented here with a recursive-descent parser.
    handle_file is the top-level function, which calls parse_struct/parse_enum,
    and so on.

    Every LCM type has an associated "signature", which is a hash of
    various components of its delcaration. If the declaration of a
    signature changes, the hash changes with high probability.
**/

enum { LCM_PARSE_ERROR = -10 };

static int parse_error(const tokenize_t *t, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    fprintf(stderr, "%s:%d: ", t->path, t->token_line);
    vfprintf(stderr, fmt, args);
    fputc('\n', stderr);
    va_end(args);
    return LCM_PARSE_ERROR;
}

// lcm's built-in types. Note that unsigned types are not present
// because there is no safe java implementation. Really, you don't
// want to add unsigned types.
static const char *primitive_types[] = {
    "int8_t", "int16_t", "int32_t", "int64_t", "byte", "float", "double", "string", "boolean", NULL,
};

// which types can be legally used as array dimensions?
static const char *array_dimension_types[] = {
    "int8_t", "int16_t", "int32_t", "int64_t", NULL,
};

// which types can be legally used as const values?
static const char *const_types[] = {
    "int8_t", "int16_t", "int32_t", "int64_t", "float", "double", NULL,
};

// Given NULL-terminated array of strings "ts", does "t" appear in it?
static int string_in_array(const char *t, const char **ts)
{
    for (int i = 0; ts[i] != NULL; i++) {
        if (!strcmp(ts[i], t))
            return 1;
    }

    return 0;
}

int lcm_is_primitive_type(const char *t)
{
    return string_in_array(t, primitive_types);
}

int lcm_is_array_dimension_type(const char *t)
{
    return string_in_array(t, array_dimension_types);
}

int lcm_is_legal_member_name(const char *t)
{
    if (!isalpha((unsigned char) t[0]) && t[0] != '_')
        return 0;
    for (++t; *t; ++t) {
        if (!isalnum((unsigned char) *t) && *t != '_')
            return 0;
    }
    return 1;
}

int lcm_is_legal_const_type(const char *t)
{
    return string_in_array(t, const_types);
}

// Make the hash dependent on the value of the given character. The
// order that hash_update is called in IS important.
static int64_t hash_update(int64_t v, char c)
{
    const uint64_t bits = (((uint64_t) v << 8) ^ (uint64_t) (v >> 55)) + (uint64_t) c;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

// Make the hash dependent on each character in a string.
static int64_t hash_string_update(int64_t v, const char *s)
{
    v = hash_update(v, strlen(s));

    for (; *s != 0; s++)
        v = hash_update(v, *s);

    return v;
}

static void lcm_typename_destroy(lcm_typename_t *type)
{
    g_free(type->lctypename);
    g_free(type->package);
    g_free(type->shortname);
    g_free(type);
}

static void lcm_dimension_destroy(gpointer pointer)
{
    lcm_dimension_t *dim = pointer;
    g_free(dim->size);
    g_free(dim);
}

static void lcm_constant_destroy(gpointer pointer)
{
    lcm_constant_t *constant = pointer;
    g_free(constant->lctypename);
    g_free(constant->membername);
    g_free(constant->val_str);
    g_free(constant->comment);
    g_free(constant);
}

static void lcm_member_destroy(gpointer pointer)
{
    lcm_member_t *member = pointer;
    lcm_typename_destroy(member->type);
    g_free(member->membername);
    g_free(member->comment);
    g_ptr_array_unref(member->dimensions);
    g_free(member);
}

static void lcm_enum_value_destroy(gpointer pointer)
{
    lcm_enum_value_t *value = pointer;
    g_free(value->valuename);
    g_free(value);
}

static void lcm_enum_destroy(gpointer pointer)
{
    lcm_enum_t *enumeration = pointer;
    lcm_typename_destroy(enumeration->enumname);
    g_ptr_array_unref(enumeration->values);
    g_free(enumeration->lcmfile);
    g_free(enumeration);
}

static void lcm_struct_destroy(gpointer pointer)
{
    lcm_struct_t *structure = pointer;
    lcm_typename_destroy(structure->structname);
    g_ptr_array_unref(structure->members);
    g_ptr_array_unref(structure->constants);
    g_ptr_array_unref(structure->enums);
    g_ptr_array_unref(structure->structs);
    g_free(structure->lcmfile);
    g_free(structure->comment);
    g_free(structure->file_comment);
    g_free(structure);
}

lcmgen_t *lcmgen_create(void)
{
    lcmgen_t *lcmgen = g_new0(lcmgen_t, 1);
    lcmgen->structs = g_ptr_array_new_with_free_func(lcm_struct_destroy);
    lcmgen->enums = g_ptr_array_new_with_free_func(lcm_enum_destroy);
    lcmgen->parse_cache.package = g_strdup("");

    return lcmgen;
}

void lcmgen_destroy(lcmgen_t *lcmgen)
{
    if (lcmgen) {
        g_ptr_array_unref(lcmgen->structs);
        g_ptr_array_unref(lcmgen->enums);
        g_free(lcmgen->parse_cache.package);
        g_free(lcmgen->parse_cache.comment_doc);
        g_free(lcmgen->parse_cache.file_comment);
        g_free(lcmgen);
    }
}

// Parse a type into package and class name.  If no package is
// specified, we will try to use the package from the last specified
// "package" directive, like in Java.
static lcm_typename_t *lcm_typename_create(lcmgen_t *lcmgen, const char *name)
{
    lcm_typename_t *type = g_new0(lcm_typename_t, 1);
    const char *separator = strrchr(name, '.');
    if (separator) {
        type->package = g_strndup(name, (gsize) (separator - name));
        type->shortname = g_strdup(separator + 1);
        type->lctypename = g_strdup(name);
    } else {
        type->package = g_strdup(lcm_is_primitive_type(name) ? "" : lcmgen->parse_cache.package);
        type->shortname = g_strdup(name);
        type->lctypename = *type->package
            ? g_strconcat(type->package, ".", name, NULL)
            : g_strdup(name);
    }
    return type;
}

static lcm_struct_t *lcm_struct_create(lcmgen_t *lcmgen, const char *lcmfile, const char *name)
{
    lcm_struct_t *structure = g_new0(lcm_struct_t, 1);
    structure->lcmfile = g_strdup(lcmfile);
    structure->structname = lcm_typename_create(lcmgen, name);
    structure->members = g_ptr_array_new_with_free_func(lcm_member_destroy);
    structure->constants = g_ptr_array_new_with_free_func(lcm_constant_destroy);
    structure->enums = g_ptr_array_new_with_free_func(lcm_enum_destroy);
    structure->structs = g_ptr_array_new_with_free_func(lcm_struct_destroy);
    return structure;
}

static lcm_enum_t *lcm_enum_create(lcmgen_t *lcmgen, const char *lcmfile, const char *name)
{
    lcm_enum_t *enumeration = g_new0(lcm_enum_t, 1);
    enumeration->lcmfile = g_strdup(lcmfile);
    enumeration->enumname = lcm_typename_create(lcmgen, name);
    enumeration->values = g_ptr_array_new_with_free_func(lcm_enum_value_destroy);
    return enumeration;
}

int64_t lcm_struct_hash(lcm_struct_t *lr)
{
    int64_t v = 0x12345678;

    // NO: Purposefully, we do NOT include the structname in the hash.
    // this allows people to rename data types and still have them work.
    //
    // In contrast, we DO hash the types of a structs members (and their names).
    //  v = hash_string_update(v, lr->structname);

    for (unsigned int i = 0; i < g_ptr_array_size(lr->members); i++) {
        lcm_member_t *lm = (lcm_member_t *) g_ptr_array_index(lr->members, i);

        // hash the member name
        v = hash_string_update(v, lm->membername);

        // if the member is a primitive type, include the type
        // signature in the hash. Do not include them for compound
        // members, because their contents will be included, and we
        // don't want a struct's name change to break the hash.
        if (lcm_is_primitive_type(lm->type->lctypename))
            v = hash_string_update(v, lm->type->lctypename);

        // hash the dimensionality information
        int ndim = g_ptr_array_size(lm->dimensions);
        v = hash_update(v, ndim);
        for (int j = 0; j < ndim; j++) {
            lcm_dimension_t *dim = (lcm_dimension_t *) g_ptr_array_index(lm->dimensions, j);
            v = hash_update(v, dim->mode);
            v = hash_string_update(v, dim->size);
        }
    }

    return v;
}

// The hash for LCM enums is defined only by the name of the enum;
// this allows bit declarations to be added over time.
int64_t lcm_enum_hash(lcm_enum_t *le)
{
    int64_t v = 0x87654321;

    v = hash_string_update(v, le->enumname->shortname);
    return v;
}

// The next call to tokenize_next() still consumes the non-comment token.
static void skip_comments(lcmgen_t *lcmgen, tokenize_t *t)
{
    if (lcmgen)
        g_clear_pointer(&lcmgen->parse_cache.comment_doc, g_free);

    while (tokenize_peek(t) >= 0 && t->token_type == LCM_TOK_COMMENT) {
        tokenize_next(t);
        if (!lcmgen)
            continue;
        char *previous = lcmgen->parse_cache.comment_doc;
        lcmgen->parse_cache.comment_doc = previous
            ? g_strconcat(previous, "\n", t->token, NULL)
            : g_strdup(t->token);
        g_free(previous);
    }
}

static int try_consume(tokenize_t *t, const char *token)
{
    skip_comments(NULL, t);
    if (tokenize_peek(t) < 0 || t->token_type != LCM_TOK_OTHER || strcmp(t->token, token))
        return 0;
    tokenize_next(t);
    return 1;
}

static int next_token(tokenize_t *t, const char *description)
{
    skip_comments(NULL, t);
    if (tokenize_next(t) < 0 || t->token_type != LCM_TOK_OTHER)
        return parse_error(t, "Expected %s", description);
    return 0;
}

static int require_token(tokenize_t *t, const char *token)
{
    return try_consume(t, token) ? 0 : parse_error(t, "Expected '%s'", token);
}

static int is_qualified_name(const char *name)
{
    int first = 1;
    for (; *name; ++name) {
        const unsigned char ch = (unsigned char) *name;
        if (first && !(isalpha(ch) || ch == '_'))
            return 0;
        if (!first && !(isalnum(ch) || ch == '_' || ch == '.'))
            return 0;
        first = ch == '.';
    }
    return !first;
}

static int parse_integer(const char *token, long long min, long long max, long long *value)
{
    char *end;
    errno = 0;
    *value = strtoll(token, &end, 0);
    return end != token && *end == '\0' && errno != ERANGE && *value >= min && *value <= max;
}

static int parse_constant_value(lcm_constant_t *constant, tokenize_t *t)
{
    const char *type = constant->lctypename;
    const char *token = t->token;
    long long integer;

    if (!strcmp(type, "float") || !strcmp(type, "double")) {
        char *end;
        double maximum = !strcmp(type, "float") ? FLT_MAX : DBL_MAX;
        double value = strtod(token, &end);
        if (end == token || *end != '\0' || value < -maximum || value > maximum)
            return parse_error(t, "Invalid %s constant '%s'", type, token);
        if (!strcmp(type, "float"))
            constant->val.f = (float) value;
        else
            constant->val.d = value;
    } else if (!strcmp(type, "int8_t") && parse_integer(token, INT8_MIN, INT8_MAX, &integer)) {
        constant->val.i8 = (int8_t) integer;
    } else if (!strcmp(type, "int16_t") && parse_integer(token, INT16_MIN, INT16_MAX, &integer)) {
        constant->val.i16 = (int16_t) integer;
    } else if (!strcmp(type, "int32_t") && parse_integer(token, INT32_MIN, INT32_MAX, &integer)) {
        constant->val.i32 = (int32_t) integer;
    } else if (!strcmp(type, "int64_t") && parse_integer(token, INT64_MIN, INT64_MAX, &integer)) {
        constant->val.i64 = (int64_t) integer;
    } else {
        return parse_error(t, "Invalid %s constant '%s'", type, token);
    }
    constant->val_str = g_strdup(token);
    return 0;
}

static int parse_declaration_name(lcm_struct_t *structure, tokenize_t *t)
{
    if (next_token(t, "a field name") != 0)
        return LCM_PARSE_ERROR;
    if (!lcm_is_legal_member_name(t->token))
        return parse_error(t, "Invalid field name '%s'", t->token);
    if (lcm_find_member(structure, t->token) || lcm_find_const(structure, t->token))
        return parse_error(t, "Duplicate field '%s'", t->token);
    return 0;
}

static int parse_const(lcmgen_t *lcmgen, lcm_struct_t *structure, tokenize_t *t)
{
    if (next_token(t, "a constant type") != 0)
        return LCM_PARSE_ERROR;
    if (!lcm_is_legal_const_type(t->token))
        return parse_error(t, "Invalid type '%s' for a constant", t->token);

    char *type = g_strdup(t->token);
    int result = 0;
    do {
        result = parse_declaration_name(structure, t);
        if (result != 0)
            break;
        lcm_constant_t *constant = g_new0(lcm_constant_t, 1);
        constant->lctypename = g_strdup(type);
        constant->membername = g_strdup(t->token);
        constant->comment = lcmgen->parse_cache.comment_doc;
        lcmgen->parse_cache.comment_doc = NULL;
        g_ptr_array_add(structure->constants, constant);
        result = require_token(t, "=");
        if (result == 0)
            result = next_token(t, "a constant value");
        if (result == 0)
            result = parse_constant_value(constant, t);
    } while (result == 0 && try_consume(t, ","));
    g_free(type);
    return result == 0 ? require_token(t, ";") : result;
}

static int parse_dimension(lcm_struct_t *structure, lcm_member_t *member, tokenize_t *t)
{
    if (next_token(t, "an array dimension") != 0)
        return LCM_PARSE_ERROR;

    const char *size = t->token;
    lcm_dimension_mode_t mode = LCM_CONST;
    if (lcm_is_legal_member_name(size)) {
        const lcm_constant_t *constant = lcm_find_const(structure, size);
        if (constant) {
            if (!lcm_is_array_dimension_type(constant->lctypename))
                return parse_error(t, "Array dimension '%s' must be an integer", size);
            size = constant->val_str;
        } else {
            const lcm_member_t *dimension_member = lcm_find_member(structure, size);
            if (!dimension_member || dimension_member == member
                || dimension_member->dimensions->len != 0
                || !lcm_is_array_dimension_type(dimension_member->type->lctypename))
                return parse_error(t, "Array dimension '%s' must be a preceding integer field", size);
            mode = LCM_VAR;
        }
    }
    long long length;
    if (mode == LCM_CONST && !parse_integer(size, 1, INT_MAX, &length))
        return parse_error(t, "Invalid array size '%s'", size);

    lcm_dimension_t *dimension = g_new0(lcm_dimension_t, 1);
    dimension->mode = mode;
    dimension->size = g_strdup(size);
    g_ptr_array_add(member->dimensions, dimension);
    return require_token(t, "]");
}

static int parse_member(lcmgen_t *lcmgen, lcm_struct_t *structure, tokenize_t *t)
{
    if (try_consume(t, "struct") || try_consume(t, "enum"))
        return parse_error(t, "Inline type declarations are not supported");
    if (try_consume(t, "const"))
        return parse_const(lcmgen, structure, t);
    if (next_token(t, "a field type") != 0)
        return LCM_PARSE_ERROR;
    if (!is_qualified_name(t->token))
        return parse_error(t, "Invalid field type '%s'", t->token);

    char *type = g_strdup(t->token);
    int result = 0;
    do {
        result = parse_declaration_name(structure, t);
        if (result != 0)
            break;
        lcm_member_t *member = g_new0(lcm_member_t, 1);
        member->type = lcm_typename_create(lcmgen, type);
        member->membername = g_strdup(t->token);
        member->dimensions = g_ptr_array_new_with_free_func(lcm_dimension_destroy);
        member->comment = lcmgen->parse_cache.comment_doc;
        lcmgen->parse_cache.comment_doc = NULL;
        g_ptr_array_add(structure->members, member);

        while (result == 0 && try_consume(t, "["))
            result = parse_dimension(structure, member, t);
    } while (result == 0 && try_consume(t, ","));
    g_free(type);
    return result == 0 ? require_token(t, ";") : result;
}

static int parse_enum_value(lcm_enum_t *enumeration, tokenize_t *t)
{
    if (next_token(t, "an enum name") != 0)
        return LCM_PARSE_ERROR;
    if (!lcm_is_legal_member_name(t->token))
        return parse_error(t, "Invalid enum name '%s'", t->token);

    lcm_enum_value_t *value = g_new0(lcm_enum_value_t, 1);
    value->valuename = g_strdup(t->token);
    int result = 0;
    long long integer;
    if (try_consume(t, "=")) {
        result = next_token(t, "an enum value");
        if (result == 0 && !parse_integer(t->token, INT32_MIN, INT32_MAX, &integer))
            result = parse_error(t, "Invalid enum value '%s'", t->token);
    } else {
        integer = 1;
        for (guint i = 0; i < enumeration->values->len; ++i) {
            const lcm_enum_value_t *prior = g_ptr_array_index(enumeration->values, i);
            if (prior->value >= integer)
                integer = (long long) prior->value + 1;
        }
        if (integer > INT32_MAX)
            result = parse_error(t, "Enum value is out of range");
    }

    for (guint i = 0; result == 0 && i < enumeration->values->len; ++i) {
        const lcm_enum_value_t *prior = g_ptr_array_index(enumeration->values, i);
        if (prior->value == integer || !strcmp(prior->valuename, value->valuename))
            result = parse_error(t, "Duplicate enum member '%s' or value %lld", value->valuename, integer);
    }
    if (result != 0) {
        lcm_enum_value_destroy(value);
        return result;
    }

    value->value = (int32_t) integer;
    g_ptr_array_add(enumeration->values, value);
    return 0;
}

static int parse_struct(lcmgen_t *lcmgen, const char *lcmfile, tokenize_t *t)
{
    if (next_token(t, "a structure name") != 0)
        return LCM_PARSE_ERROR;
    if (!is_qualified_name(t->token))
        return parse_error(t, "Invalid structure name '%s'", t->token);

    lcm_struct_t *structure = lcm_struct_create(lcmgen, lcmfile, t->token);
    structure->file_comment = lcmgen->parse_cache.file_comment;
    structure->comment = lcmgen->parse_cache.comment_doc;
    lcmgen->parse_cache.file_comment = NULL;
    lcmgen->parse_cache.comment_doc = NULL;

    int result = require_token(t, "{");
    while (result == 0) {
        skip_comments(lcmgen, t);
        if (try_consume(t, "}"))
            break;
        result = parse_member(lcmgen, structure, t);
    }
    for (guint i = 0; result == 0 && i < lcmgen->structs->len; ++i) {
        const lcm_struct_t *prior = g_ptr_array_index(lcmgen->structs, i);
        if (!strcmp(prior->structname->lctypename, structure->structname->lctypename))
            result = parse_error(t, "Duplicate structure '%s'", structure->structname->lctypename);
    }

    if (result != 0) {
        lcm_struct_destroy(structure);
        return result;
    }
    structure->hash = lcm_struct_hash(structure);
    g_ptr_array_add(lcmgen->structs, structure);
    return 0;
}

static int parse_enum(lcmgen_t *lcmgen, const char *lcmfile, tokenize_t *t)
{
    if (next_token(t, "an enum name") != 0)
        return LCM_PARSE_ERROR;
    if (!is_qualified_name(t->token))
        return parse_error(t, "Invalid enum name '%s'", t->token);

    lcm_enum_t *enumeration = lcm_enum_create(lcmgen, lcmfile, t->token);
    int result = require_token(t, "{");
    while (result == 0 && !try_consume(t, "}")) {
        result = parse_enum_value(enumeration, t);
        if (result == 0) {
            try_consume(t, ",");
            try_consume(t, ";");
        }
    }
    if (result != 0) {
        lcm_enum_destroy(enumeration);
        return result;
    }
    enumeration->hash = lcm_enum_hash(enumeration);
    g_ptr_array_add(lcmgen->enums, enumeration);
    return 0;
}

// Parse one top-level declaration, leaving completed declarations in lcmgen.
int parse_entity(lcmgen_t *lcmgen, const char *lcmfile, tokenize_t *t)
{
    skip_comments(lcmgen, t);
    const int status = tokenize_next(t);
    if (status == EOF && t->token_type == LCM_TOK_EOF && !ferror(t->f))
        return EOF;
    if (status < 0 || t->token_type != LCM_TOK_OTHER)
        return parse_error(t, "Invalid token");

    if (!strcmp(t->token, "package")) {
        if (next_token(t, "a package name") != 0)
            return LCM_PARSE_ERROR;
        if (!is_qualified_name(t->token))
            return parse_error(t, "Invalid package name '%s'", t->token);
        g_free(lcmgen->parse_cache.package);
        lcmgen->parse_cache.package = g_strdup(t->token);
        g_free(lcmgen->parse_cache.file_comment);
        lcmgen->parse_cache.file_comment = lcmgen->parse_cache.comment_doc;
        lcmgen->parse_cache.comment_doc = NULL;
        return require_token(t, ";");
    }
    if (!strcmp(t->token, "struct"))
        return parse_struct(lcmgen, lcmfile, t);
    if (!strcmp(t->token, "enum"))
        return parse_enum(lcmgen, lcmfile, t);
    return parse_error(t, "Unexpected declaration '%s'", t->token);
}

int lcmgen_handle_file(lcmgen_t *lcmgen, const char *path)
{
    struct stat info;
    if (stat(path, &info) != 0 || !S_ISREG(info.st_mode))
        return LCM_PARSE_ERROR;
    tokenize_t *t = tokenize_create(path);
    if (!t)
        return LCM_PARSE_ERROR;

    int result;
    do {
        result = parse_entity(lcmgen, path, t);
    } while (result == 0);

    tokenize_destroy(t);
    return result == EOF ? 0 : result;
}

void lcm_typename_dump(lcm_typename_t *lt)
{
    printf("\t%-20s", lt->lctypename);
}

void lcm_member_dump(lcm_member_t *lm)
{
    lcm_typename_dump(lm->type);

    printf("  ");

    printf("%s", lm->membername);

    int ndim = g_ptr_array_size(lm->dimensions);
    for (int i = 0; i < ndim; i++) {
        lcm_dimension_t *dim = (lcm_dimension_t *) g_ptr_array_index(lm->dimensions, i);
        switch (dim->mode) {
        case LCM_CONST:
            printf(" [ (const) %s ]", dim->size);
            break;
        case LCM_VAR:
            printf(" [ (var) %s ]", dim->size);
            break;
        default:
            // oops! unhandled case
            assert(0);
        }
    }

    printf("\n");
}

void lcm_enum_dump(lcm_enum_t *le)
{
    printf("enum %s\n", le->enumname->lctypename);
    for (unsigned int i = 0; i < g_ptr_array_size(le->values); i++) {
        lcm_enum_value_t *lev = (lcm_enum_value_t *) g_ptr_array_index(le->values, i);
        printf("        %-20s  %i\n", lev->valuename, lev->value);
    }
}

void lcm_struct_dump(lcm_struct_t *lr)
{
    printf("struct %s [hash=0x%16" PRIx64 "]\n", lr->structname->lctypename, lr->hash);
    for (unsigned int i = 0; i < g_ptr_array_size(lr->members); i++) {
        lcm_member_t *lm = (lcm_member_t *) g_ptr_array_index(lr->members, i);
        lcm_member_dump(lm);
    }

    for (unsigned int i = 0; i < g_ptr_array_size(lr->enums); i++) {
        lcm_enum_t *le = (lcm_enum_t *) g_ptr_array_index(lr->enums, i);
        lcm_enum_dump(le);
    }
}

void lcmgen_dump(lcmgen_t *lcmgen)
{
    for (unsigned int i = 0; i < g_ptr_array_size(lcmgen->enums); i++) {
        lcm_enum_t *le = (lcm_enum_t *) g_ptr_array_index(lcmgen->enums, i);
        lcm_enum_dump(le);
    }

    for (unsigned int i = 0; i < g_ptr_array_size(lcmgen->structs); i++) {
        lcm_struct_t *lr = (lcm_struct_t *) g_ptr_array_index(lcmgen->structs, i);
        lcm_struct_dump(lr);
    }
}

/** Find and return the member whose name is name. **/
lcm_member_t *lcm_find_member(lcm_struct_t *lr, const char *name)
{
    for (unsigned int i = 0; i < g_ptr_array_size(lr->members); i++) {
        lcm_member_t *lm = (lcm_member_t *) g_ptr_array_index(lr->members, i);
        if (!strcmp(lm->membername, name))
            return lm;
    }

    return NULL;
}

/** Find and return the const whose name is name. **/
lcm_constant_t *lcm_find_const(lcm_struct_t *lr, const char *name)
{
    for (unsigned int i = 0; i < g_ptr_array_size(lr->constants); i++) {
        lcm_constant_t *lc = (lcm_constant_t *) g_ptr_array_index(lr->constants, i);
        if (!strcmp(lc->membername, name))
            return lc;
    }
    return NULL;
}

int lcm_needs_generation(lcmgen_t *lcmgen, const char *declaringfile, const char *outfile)
{
    struct stat instat, outstat;
    int res;

    res = stat(declaringfile, &instat);
    if (res) {
        printf("Funny error: can't stat the .lcm file");
        perror(declaringfile);
        return 1;
    }

    res = stat(outfile, &outstat);
    if (res)
        return 1;

    return instat.st_mtime > outstat.st_mtime;
}

/** Is the member an array of constant size? If it is not an array, it returns zero. **/
int lcm_is_constant_size_array(lcm_member_t *lm)
{
    int ndim = g_ptr_array_size(lm->dimensions);

    if (ndim == 0)
        return 1;

    for (int i = 0; i < ndim; i++) {
        lcm_dimension_t *dim = (lcm_dimension_t *) g_ptr_array_index(lm->dimensions, i);

        if (dim->mode == LCM_VAR)
            return 0;
    }

    return 1;
}

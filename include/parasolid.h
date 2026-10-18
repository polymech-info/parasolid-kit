#ifndef PARASOLID_KIT_H
#define PARASOLID_KIT_H

/* C ABI for parasolid.dll. Returned strings are NUL-terminated UTF-8, owned by
 * the DLL, and freed with parasolid_string_free.
 *
 * This library inspects and parses Parasolid X_T / X_B transmit files.
 * parasolid_summary returns JSON counts. parasolid_mesh returns a little-endian
 * triangle buffer the native viewer draws:
 *
 *   u32 magic 'PSM1' (0x314D5350), u32 version (2), u32 flags, u32 body_count
 *   flags bit 0 is set when the mapped model is complete
 *   each body: u32 ordinal, u32 kind (1 solid, 2 wire, 3 sheet, 6 general),
 *              i32 node_id (-1 if absent), u32 triangle_count,
 *              f32 min[3], f32 max[3],
 *              triangle_count times 3 corners of f32 position[3] + normal[3]
 *   min > max means the body has no bounds
 *   then u32 instance_count, and each instance: u32 body ordinal, i32 node_id,
 *   16 f32 Coin matrix (axis columns, translation in the last row)
 */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) && !defined(PARASOLID_STATIC)
#    define PARASOLID_API __declspec(dllimport)
#else
#    define PARASOLID_API
#endif

enum {
    PARASOLID_OK = 0,
    PARASOLID_UNSUPPORTED = 1,
    PARASOLID_MALFORMED = 2,
    PARASOLID_LIMIT = 3,
    PARASOLID_IO = 4,
    PARASOLID_PANIC = 5,
    PARASOLID_INVALID = 6
};

/* 0 is "detect" for the bytes/path readers, and "not recognized" for the
 * format_from_* queries. */
enum {
    PARASOLID_FORMAT_DETECT = 0,
    PARASOLID_FORMAT_XB = 1,
    PARASOLID_FORMAT_XT = 2
};

PARASOLID_API const char* parasolid_version(void);

PARASOLID_API void parasolid_string_free(char* s);

PARASOLID_API int parasolid_format_from_bytes(const uint8_t* bytes, size_t len);
PARASOLID_API int parasolid_format_from_extension(const char* ext);
PARASOLID_API int parasolid_format_from_path(const char* path);

/* On success, *out_json is set and *out_err is NULL. On failure, *out_json is
 * NULL and *out_err is a message when out_err is non-NULL. Either out pointer
 * may be NULL to discard that string.
 *
 * format is PARASOLID_FORMAT_DETECT, PARASOLID_FORMAT_XB, or PARASOLID_FORMAT_XT.
 * Inspect reads the header only. Summary parses with a built-in schema profile
 * and reports body, face, edge, and vertex counts. */
PARASOLID_API int parasolid_inspect(
    const char* path, char** out_json, char** out_err);
PARASOLID_API int parasolid_inspect_bytes(
    const uint8_t* bytes, size_t len, int format, char** out_json, char** out_err);
PARASOLID_API int parasolid_summary(
    const char* path, char** out_json, char** out_err);
PARASOLID_API int parasolid_summary_bytes(
    const uint8_t* bytes, size_t len, int format, char** out_json, char** out_err);

/* Mesh bytes are freed with parasolid_bytes_free. out_size is required.
 * out_bytes and out_err follow the same null-on-failure rule as the JSON calls. */
PARASOLID_API int parasolid_mesh(
    const char* path, uint8_t** out_bytes, size_t* out_size, char** out_err);
PARASOLID_API int parasolid_mesh_bytes(
    const uint8_t* bytes, size_t len, int format, uint8_t** out_bytes, size_t* out_size, char** out_err);
PARASOLID_API void parasolid_bytes_free(uint8_t* bytes, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* PARASOLID_KIT_H */

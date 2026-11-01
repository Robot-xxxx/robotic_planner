#pragma once

#ifdef __cplusplus
extern "C" {
#endif

const char* rp_version(void);

/* Returns 0 on success. On failure returns 1 and, if error_message is non-null,
 * stores a newly allocated UTF-8 string that must be released with rp_free_string. */
int rp_plan_file(const char* config_path, const char* output_path, char** error_message);

/* Returns 0 on success and stores a newly allocated JSON string in plan_json.
 * The caller must release it with rp_free_string. */
int rp_plan_json(const char* config_json, char** plan_json, char** error_message);

void rp_free_string(char* text);

#ifdef __cplusplus
}
#endif

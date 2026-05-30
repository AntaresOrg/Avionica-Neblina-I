#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

bool lora_init(void);

bool lora_available(void);

int lora_read(char *buffer, size_t max_len);

void lora_send(const char *msg);

// Helper: sends one line framed as "<text>\r\n" (truncates to fit).
void lora_send_line(const char *text);

#ifdef __cplusplus
}
#endif
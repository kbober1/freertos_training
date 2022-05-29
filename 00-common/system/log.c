/*
MIT License

Copyright (c) 2022 Marcin Borowicz <marcinbor85@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "log.h"
#include "shell.h"
#include "system.h"

#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "semphr.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define TASK_NAME                       "logs"
#define TASK_PRIORITY                   (tskIDLE_PRIORITY + 1)
#define TASK_STACK_SIZE                 (configMINIMAL_STACK_SIZE)
#define MESSAGE_TIMEOUT_TICKS           100

static MessageBufferHandle_t message_buffer_handle = NULL;
static char log_buffer_in[1024] = {0};
static char log_buffer_out[1024] = {0};
static SemaphoreHandle_t xMutexHandle = NULL;

static void log_service(void *pvParameters) {
    while (message_buffer_handle) {
        while (xMessageBufferReceive(message_buffer_handle, log_buffer_out, sizeof(log_buffer_out), MESSAGE_TIMEOUT_TICKS) > 0) {
            printf("%s", log_buffer_out);
            memset(log_buffer_out, 0, sizeof(log_buffer_out));
        }
    }
}

int log_init(size_t logs_buf_size) {
        message_buffer_handle = xMessageBufferCreate(logs_buf_size);

        xMutexHandle = xSemaphoreCreateMutex();
        BaseType_t s;
        s = xTaskCreate(log_service, TASK_NAME, TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL);
        configASSERT(s != pdFALSE);

    return 0;
}

void log_printf(int level, const char *file, const char *func, unsigned long line, const char *fmt, ... )
{
        char level_name[8] = {0};
        char prefix[16] = {0};

        static unsigned long cntr = 0;

        system_stdout_lock();

        if (level <= LOGGING_LEVEL_FATAL) {
                strcpy(level_name, "FATAL");
                strcpy(prefix, SHELL_FONT_LIGHTGRAY SHELL_FONT_BACK_RED);
        } else if (level <= LOGGING_LEVEL_ERROR) {
                strcpy(level_name, "ERROR");
                strcpy(prefix, SHELL_FONT_RED);
        } else if (level <= LOGGING_LEVEL_WARNING) {
                strcpy(level_name, "WARNING");
                strcpy(prefix, SHELL_FONT_YELLOW);
        } else if (level <= LOGGING_LEVEL_INFO) {
                strcpy(level_name, "INFO");
                strcpy(prefix, SHELL_FONT_GREEN);
        } else if (level <= LOGGING_LEVEL_DEBUG) {
                strcpy(level_name, "DEBUG");
                strcpy(prefix, SHELL_FONT_RESET);
        } else {
                strcpy(level_name, "TRACE");
                strcpy(prefix, SHELL_FONT_CYAN);
        }

        TickType_t ticks = xTaskGetTickCount();
        xSemaphoreTake(xMutexHandle, MESSAGE_TIMEOUT_TICKS);
        int data_size = snprintf(log_buffer_in, sizeof(log_buffer_in), "%s%04lu %08lu %s %s %s:%lu > ", prefix, cntr, ticks, level_name, func, file, line);
        ++cntr;

        va_list arg;
        va_start( arg, fmt );
        char *posInBuff = log_buffer_in + data_size;
        data_size += vsnprintf(posInBuff, sizeof(log_buffer_in) - data_size, fmt, arg);
        va_end( arg );

        strcat(log_buffer_in, SHELL_FONT_RESET);
        strcat(log_buffer_in, SHELL_NEW_LINE);
        data_size += strlen(SHELL_FONT_RESET) + strlen(SHELL_NEW_LINE);

        if (message_buffer_handle) {
            xMessageBufferSend(message_buffer_handle, log_buffer_in, data_size, MESSAGE_TIMEOUT_TICKS);
        } else {
            printf("%s", log_buffer_in);
        }
        xSemaphoreGive(xMutexHandle);

        system_stdout_unlock();
}

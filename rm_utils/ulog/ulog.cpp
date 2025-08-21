/**
MIT License

Copyright (c) 2019 R. Dunbar Poor <rdpoor@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * \file ulog.c
 *
 * \brief uLog: lightweight logging for embedded systems
 *
 * See ulog.h for sparse documentation.
 */

#include "ulog.hpp"
#include <iostream>

#ifdef ULOG_ENABLED  // whole file...

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>


// =============================================================================
// types and definitions

typedef struct {
  ulog_function_t fn;
  ulog_level_t threshold;
} subscriber_t;

static ulog_level_t ulog_lowest_log_level();

// =============================================================================
// local storage

static subscriber_t s_subscribers[ULOG_MAX_SUBSCRIBERS];
static char s_message[ULOG_MAX_MESSAGE_LENGTH];
static ulog_level_t s_lowest_log_level;
static FILE *s_log_file = NULL;

// =============================================================================
// console logger with color support

void ulog_console_logger(ulog_level_t level, const char *msg) {
    time_t t = time(NULL);
    struct tm *tm_info = localtime(&t);
    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm_info);
    
    const char *color = ulog_level_color(level);
    const char *level_name = ulog_level_name(level);
    
    std::cout << color << timestamp << " [" << level_name << "]: " << color << msg << ULOG_COLOR_RESET << std::endl;
    //printf("%s%s [%s]: %s%s%s\n", color, timestamp, level_name, color, msg, ULOG_COLOR_RESET);
}

// =============================================================================
// user-visible code

void ulog_init(void) {
  // 如果已经有打开的日志文件，先关闭它
  if (s_log_file) {
    fclose(s_log_file);
    s_log_file = NULL;
  }
  
  memset(s_subscribers, 0, sizeof(s_subscribers));
  s_lowest_log_level = ulog_lowest_log_level();
  
  // Create log directory if it doesn't exist
  struct stat st = {0};
  if (stat("log", &st) == -1) {
#ifdef _WIN32
    mkdir("log");
#else
    mkdir("log", 0755);
#endif
  }
  
  // Open log file
  time_t t = time(NULL);
  struct tm *tm_info = localtime(&t);
  char filename[100];
  strftime(filename, sizeof(filename), "log/log_%Y-%m-%d_%H-%M-%S.log", tm_info);
  s_log_file = fopen(filename, "w");
}

void ulog_init_console_and_file(ulog_level_t console_level, ulog_level_t file_level) {
  ulog_init();
  ulog_subscribe(ulog_console_logger, console_level);
  // 确保最低日志级别能够允许文件记录所需的级别
  if (file_level < s_lowest_log_level) {
    s_lowest_log_level = file_level;
  }
}

// search the s_subscribers table to install or update fn
ulog_err_t ulog_subscribe(ulog_function_t fn, ulog_level_t threshold) {
  int available_slot = -1;
  int i;
  for (i=0; i<ULOG_MAX_SUBSCRIBERS; i++) {
    if (s_subscribers[i].fn == fn) {
      // already subscribed: update threshold and return immediately.
      s_subscribers[i].threshold = threshold;
      return ULOG_ERR_NONE;

    } else if (s_subscribers[i].fn == NULL) {
      // found a free slot
      available_slot = i;
    }
  }
  // fn is not yet a subscriber.  assign if possible.
  if (available_slot == -1) {
    return ULOG_ERR_SUBSCRIBERS_EXCEEDED;
  }
  s_subscribers[available_slot].fn = fn;
  s_subscribers[available_slot].threshold = threshold;
  s_lowest_log_level = ulog_lowest_log_level(); // Update lowest log level

  return ULOG_ERR_NONE;
}

// search the s_subscribers table to remove
ulog_err_t ulog_unsubscribe(ulog_function_t fn) {
  int i;
  for (i=0; i<ULOG_MAX_SUBSCRIBERS; i++) {
    if (s_subscribers[i].fn == fn) {
      s_subscribers[i].fn = NULL;    // mark as empty
      s_lowest_log_level = ulog_lowest_log_level(); // Update lowest log level
      return ULOG_ERR_NONE;
    }
  }
  return ULOG_ERR_NOT_SUBSCRIBED;
}

const char *ulog_level_name(ulog_level_t severity) {
  switch(severity) {
   case ULOG_TRACE_LEVEL: return "TRACE";
   case ULOG_DEBUG_LEVEL: return "DEBUG";
   case ULOG_INFO_LEVEL: return "INFO";
   case ULOG_WARNING_LEVEL: return "WARNING";
   case ULOG_ERROR_LEVEL: return "ERROR";
   case ULOG_CRITICAL_LEVEL: return "CRITICAL";
   case ULOG_ALWAYS_LEVEL: return "ALWAYS";
   default: return "UNKNOWN";
  }
}

const char *ulog_level_color(ulog_level_t severity) {
  switch(severity) {
   case ULOG_TRACE_LEVEL: return ULOG_COLOR_WHITE;
   case ULOG_DEBUG_LEVEL: return ULOG_COLOR_CYAN;
   case ULOG_INFO_LEVEL: return ULOG_COLOR_GREEN;
   case ULOG_WARNING_LEVEL: return ULOG_COLOR_YELLOW;
   case ULOG_ERROR_LEVEL: return ULOG_COLOR_RED;
   case ULOG_CRITICAL_LEVEL: return ULOG_COLOR_BOLD ULOG_COLOR_RED;
   case ULOG_ALWAYS_LEVEL: return ULOG_COLOR_MAGENTA;
   default: return ULOG_COLOR_RESET;
  }
}

void ulog_message(ulog_level_t severity, const char *fmt, ...) {
  // Do not evaluate the log message if it will never be logged
  if (severity < s_lowest_log_level){
    return;
  }

  va_list ap;
  int i;
  va_start(ap, fmt);
  vsnprintf(s_message, ULOG_MAX_MESSAGE_LENGTH, fmt, ap);
  va_end(ap);

  // Write to file if open
  if (s_log_file) {
    time_t t = time(NULL);
    struct tm *tm_info = localtime(&t);
    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm_info);
    
    fprintf(s_log_file, "%s [%s]: %s\n", timestamp, ulog_level_name(severity), s_message);
    fflush(s_log_file);
  }

  for (i=0; i<ULOG_MAX_SUBSCRIBERS; i++) {
    if (s_subscribers[i].fn != NULL) {
      if (severity >= s_subscribers[i].threshold) {
        s_subscribers[i].fn(severity, s_message);
      }
    }
  }
}

void ulog_message_tag(const char *tag, ulog_level_t severity, const char *fmt, ...) {
  // Do not evaluate the log message if it will never be logged
  if (severity < s_lowest_log_level){
    return;
  }

  char tagged_message[ULOG_MAX_MESSAGE_LENGTH];
  va_list ap;
  int i;
  
  // Format the tag and message
  va_start(ap, fmt);
  int tag_len = snprintf(tagged_message, ULOG_MAX_MESSAGE_LENGTH, "[%s] ", tag);
  if (tag_len < ULOG_MAX_MESSAGE_LENGTH) {
    vsnprintf(tagged_message + tag_len, ULOG_MAX_MESSAGE_LENGTH - tag_len, fmt, ap);
  }
  va_end(ap);

  // Write to file if open
  if (s_log_file) {
    time_t t = time(NULL);
    struct tm *tm_info = localtime(&t);
    char timestamp[32];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm_info);
    
    fprintf(s_log_file, "%s [%s]: %s\n", timestamp, ulog_level_name(severity), tagged_message);
    fflush(s_log_file);
  }

  for (i=0; i<ULOG_MAX_SUBSCRIBERS; i++) {
    if (s_subscribers[i].fn != NULL) {
      if (severity >= s_subscribers[i].threshold) {
        s_subscribers[i].fn(severity, tagged_message);
      }
    }
  }
}

void ulog_deinit(void) {
    if (s_log_file) {
        fclose(s_log_file);
        s_log_file = NULL;
    }
}

// =============================================================================
// private code

static ulog_level_t ulog_lowest_log_level(){
  ulog_level_t lowest_log_level = ULOG_ALWAYS_LEVEL;
  int i;
  for (i=0; i<ULOG_MAX_SUBSCRIBERS; i++) {
    if (s_subscribers[i].fn != NULL){
      if (s_subscribers[i].threshold < lowest_log_level) {
        lowest_log_level = s_subscribers[i].threshold;
      }
    }
  }
  return lowest_log_level;
}

#endif  // #ifdef ULOG_ENABLED
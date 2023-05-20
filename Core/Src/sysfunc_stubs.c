/*
 * Stub functions for system functions that are not implemented in the HAL.
*/

#include <sys/stat.h>

int _close(int file) {
    return -1;
}

int _fstat(int file, struct stat *st) {
    return -1;
}

int _isatty(int file) {
    return 1;
}

int _lseek(int file, int ptr, int dir) {
    return 0;
}

int _read(int file, char *ptr, int len) {
    return 0;
}

int _getpid() {
    return 1;
}

int _kill(int pid, int sig) {
    return(-1);
}

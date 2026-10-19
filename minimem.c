/* Tiny replacements for the three string functions the kernel calls (newlib's memcpy/memset/memcmp are 244 bytes of
 * alignment-optimised code; these are 70). Built with -fno-builtin so the loops are not turned back into calls to
 * themselves. Plain byte loops: the kernel only moves small buffers. */

#include <stddef.h>

void *memset(void *d, int c, size_t n)
{
	unsigned char *p = d;
	while (n--)
		*p++ = (unsigned char) c;
	return d;
}

void *memcpy(void *d, const void *s, size_t n)
{
	unsigned char *p = d;
	const unsigned char *q = s;
	while (n--)
		*p++ = *q++;
	return d;
}

int memcmp(const void *a, const void *b, size_t n)
{
	const unsigned char *p = a, *q = b;
	while (n--) {
		if (*p != *q)
			return *p - *q;
		p++;
		q++;
	}
	return 0;
}

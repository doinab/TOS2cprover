#ifndef __LIBC_STRING_H
#define __LIBC_STRING_H

typedef unsigned int size_t;

/* write a byte to a byte string */
void *memset(void *dst, int val, size_t count)
{
  size_t i=0;

//  assert(0);

  while (i < count) {
    ((unsigned char *)dst)[i] = (unsigned char)val;
    i++;
  }

  return dst;
}
/*
void *memset(void *dst, int val, size_t count)
{
  void *temp = dst;

  while (count) // != 0, unsigned
  {
    *(char *)temp = (char)val;
    temp = (char *)temp + 1;
    --count;
  }

  return dst;
}
 */
/*
void *memset(void *dst, int val, size_t count)
{
  void *start = dst;

  while (count) 
  {
    *(char *)dst = (char)val;
    dst = (char *)dst + 1;
    --count;
  }

  return start;
}
*/
#endif
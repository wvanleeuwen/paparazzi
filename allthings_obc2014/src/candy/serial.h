

extern int fd;



int serial_init(char* port_name);

static inline bool ttyUSB0ChAvailable(void)
{
  return FALSE;
}

#define ttyUSB0Transmit(_char)     \
{                                  \
  char c = _char;                  \
  int __attribute__((unused)) ret = write(fd,&c,1);        \
}

#define ttyUSB0Getch() ({char c;int ret=read(fd, &c,1);c;})

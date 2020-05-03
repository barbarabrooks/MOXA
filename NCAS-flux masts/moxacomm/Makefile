# uncomment the compiler details required for desired MOXA model

# --  UC-7420 compiler --
#PREFIXPATH=/usr/local/mxscaleb/bin
#CC=$(PREFIXPATH)/mxscaleb-gcc
#STRIP=$(PREFIXPATH)/mxscaleb-strip

# -- UC-7112 PLUS compiler --
PREFIXPATH=/usr/local/arm-linux/bin
CC=$(PREFIXPATH)/arm-linux-gcc
STRIP=$(PREFIXPATH)/arm-linux-strip

# this bit common to all MOXA versions
NAME=sercomm

all: release debug

clean:
	rm -f $(NAME) $(NAME)-debug

release:	$(NAME).c
	$(CC) -Wall -o $(NAME) $(NAME).c
	$(STRIP) -s $(NAME)

debug:		$(NAME).c
	$(CC) -ggdb -o $(NAME)-debug $(NAME).c

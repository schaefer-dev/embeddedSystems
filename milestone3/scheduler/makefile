CC=gcc
CFLAGS=-I.
DEPS = taskList.h

build/%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

scheduler: build/taskList.o build/main.o
	gcc -o scheduler build/taskList.o build/main.o -I.

clean:
	rm -f build/*.o scheduler

route: route.o table.o logging.o
	gcc -pthread -o route route.o table.o logging.o -I ./ ./LatLong-UTMconversion.c -lm
	rm *.o
route.o: route.c
	gcc -c route.c
table.o: table.c
	gcc -c table.c
logging.o: logging.c
	gcc -c logging.c
all:: route
clean:
	rm -f *.o route

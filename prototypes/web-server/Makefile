LDFLAGS =  \
        -lpion-common -lpion-net \
        -lboost_thread -lboost_system -lboost_filesystem -lboost_regex \
        -lboost_date_time -lboost_signals -lboost_iostreams \
        -lssl -lcrypto -lz -lbz2 -ldl

CFLAGS = -Wall

main: main.cpp
	g++ ${LDFLAGS} ${CFLAGS} -o main main.cpp


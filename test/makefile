all:test_feynman

test_feynman:test_feynman.o feynman_sdk.o ring_queue.o feynman_upgrade_module.o
	g++ -g $^ -o $@ -pthread -lusb-1.0

test_feynman.o:test_feynman.cpp
	g++ -g -c $^ -o $@ -pthread

feynman_sdk.o:../src/feynman_sdk.cpp
	g++ -g -c $^ -o $@ -pthread

ring_queue.o:../src/ring_queue.cpp
	g++ -g -c $^ -o $@ -pthread


feynman_upgrade_module.o:../src/feynman_upgrade_module.cpp
	g++ -g -c $^ -o $@ -pthread
	
clean:
	rm -f test_feynman.o feynman_sdk.o ring_queue.o feynman_upgrade_module.o test_feynman

all: installed

downloaded: Makefile
	make clean
	darcs get http://common-lisp.net/~trittweiler/darcs/synchronization-tools/
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	mkdir -p asdf
	ln -s ../synchronization-tools/synchronization-tools.asd asdf/
	touch installed

clean:
	rm -rf synchronization-tools asdf
	rm -f downloaded
	rm -f rospack_nosubdirs
	rm -f installed


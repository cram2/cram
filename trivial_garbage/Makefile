
all: installed

downloaded: Makefile
	make clean
	darcs get http://common-lisp.net/~loliveira/darcs/trivial-garbage/
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	mkdir -p asdf
	ln -s ../trivial-garbage/trivial-garbage.asd asdf
	touch installed

clean:
	rm -rf trivial-garbage asdf
	rm -f downloaded
	rm -f rospack_nosubdirs
	rm -f installed


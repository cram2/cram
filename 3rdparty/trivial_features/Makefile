
all: installed

downloaded: Makefile
	make clean
	darcs get http://common-lisp.net/~loliveira/darcs/trivial-features
	touch rospack_nosubdirs
	touch downloaded

installed: downloaded
	mkdir -p asdf
	ln -s ../trivial-features/trivial-features.asd asdf/
	touch installed

clean:
	rm -rf trivial-features asdf
	rm -f downloaded
	rm -f rospack_nosubdirs
	rm -f installed

